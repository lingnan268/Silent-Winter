#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "threadsafe_queue.h"

// 传感器数据类型
enum class SensorType { CAMERA, IMU };

// 传感器数据包
struct SensorData {
    SensorType type;
    int sequence;
    long long timestamp_us; // 时间戳（微秒）
};

// 全局队列容量
constexpr size_t QUEUE_CAPACITY = 100;

// 全局线程安全队列
ThreadSafeQueue<SensorData> g_queue(QUEUE_CAPACITY);

// 生产者完成计数（用原子变量保证线程安全）
std::atomic<int> g_producers_finished{0};
constexpr int TOTAL_PRODUCERS = 2;

// ==========================================
// 生产者：读取相机数据文件
// ==========================================
void camera_producer(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Camera] 无法打开文件: " << filename << std::endl;
        g_producers_finished.fetch_add(1);
        g_queue.close(); // 失败时直接关闭队列，让消费者退出
        return;
    }

    int seq;
    long long ts;
    while (file >> seq >> ts) {
        SensorData data{SensorType::CAMERA, seq, ts};
        try {
            g_queue.push(std::move(data));
        } catch (const std::runtime_error& e) {
            std::cerr << "[Camera] 队列已关闭，停止生产" << std::endl;
            break;
        }
    }

    file.close();
    int finished = g_producers_finished.fetch_add(1) + 1;
    std::cout << "[Camera] 生产者结束，已完成 " << finished << "/" << TOTAL_PRODUCERS << " 个生产者" << std::endl;

    if (finished >= TOTAL_PRODUCERS) {
        g_queue.close();
        std::cout << "[Camera] 最后一个生产者已结束，队列已关闭" << std::endl;
    }
}

// ==========================================
// 生产者：读取 IMU 数据文件
// ==========================================
void imu_producer(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[IMU] 无法打开文件: " << filename << std::endl;
        g_producers_finished.fetch_add(1);
        g_queue.close();
        return;
    }

    int seq;
    long long ts;
    while (file >> seq >> ts) {
        SensorData data{SensorType::IMU, seq, ts};
        try {
            g_queue.push(std::move(data));
        } catch (const std::runtime_error& e) {
            std::cerr << "[IMU] 队列已关闭，停止生产" << std::endl;
            break;
        }
    }

    file.close();
    int finished = g_producers_finished.fetch_add(1) + 1;
    std::cout << "[IMU] 生产者结束，已完成 " << finished << "/" << TOTAL_PRODUCERS << " 个生产者" << std::endl;

    if (finished >= TOTAL_PRODUCERS) {
        g_queue.close();
        std::cout << "[IMU] 最后一个生产者已结束，队列已关闭" << std::endl;
    }
}

// ==========================================
// 时间戳匹配：在 IMU 数据中找离 camera_ts 最近的
// 返回匹配的 IMU 序号和时间差（绝对值）
// ==========================================
std::pair<int, long long> match_imu_timestamp(
    long long camera_ts,
    const std::vector<SensorData>& imu_data)
{
    if (imu_data.empty()) {
        return {-1, 0};
    }

    int n = static_cast<int>(imu_data.size());

    // 二分查找第一个 >= camera_ts 的 IMU 数据
    int left = 0;
    int right = n - 1;
    int result_idx = -1;

    while (left <= right) {
        int mid = (left + right) / 2;
        if (imu_data[mid].timestamp_us >= camera_ts) {
            result_idx = mid;
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }

    long long min_delta = 0;
    int best_idx = -1;

    // 情况1：所有 IMU 都比相机早 → 匹配最后一个 IMU
    if (result_idx == -1) {
        best_idx = n - 1;
        min_delta = std::abs(imu_data[n - 1].timestamp_us - camera_ts);
        return {imu_data[best_idx].sequence, min_delta};
    }

    // 情况2：result_idx 本身是候选（第一个 >= camera_ts 的 IMU）
    if (result_idx < n) {
        best_idx = result_idx;
        min_delta = std::abs(imu_data[result_idx].timestamp_us - camera_ts);
    }

    // 情况3：result_idx-1 也是候选（最后一个 < camera_ts 的 IMU）
    if (result_idx - 1 >= 0) {
        long long delta = std::abs(imu_data[result_idx - 1].timestamp_us - camera_ts);
        if (delta < min_delta) {
            best_idx = result_idx - 1;
            min_delta = delta;
        } else if (delta == min_delta) {
            // 时间差完全相同，取序号更小的
            if (result_idx - 1 < best_idx) {
                best_idx = result_idx - 1;
            }
        }
    }

    return {imu_data[best_idx].sequence, min_delta};
}

// ==========================================
// 消费者：从队列取数据，分别保存，最后做时间同步
// ==========================================
void consumer(const std::string& output_filename) {
    std::vector<SensorData> camera_data_list;
    std::vector<SensorData> imu_data_list;

    // 第一步：不断从队列取数据，直到队列关闭且为空
    SensorData data;
    while (g_queue.wait_and_pop(data)) {
        if (data.type == SensorType::CAMERA) {
            camera_data_list.push_back(data);
        } else if (data.type == SensorType::IMU) {
            imu_data_list.push_back(data);
        }
    }

    std::cout << "[Consumer] 数据采集完成，相机数据: " << camera_data_list.size()
              << " 条, IMU数据: " << imu_data_list.size() << " 条" << std::endl;

    // 第二步：按序号/时间戳排序
    std::sort(camera_data_list.begin(), camera_data_list.end(),
              [](const SensorData& a, const SensorData& b) {
                  return a.sequence < b.sequence;
              });
    std::sort(imu_data_list.begin(), imu_data_list.end(),
              [](const SensorData& a, const SensorData& b) {
                  return a.timestamp_us < b.timestamp_us;
              });

    // 第三步：对每个相机数据匹配最近的 IMU 数据，并写入输出文件
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "[Consumer] 无法打开输出文件: " << output_filename << std::endl;
        return;
    }

    for (const auto& cam : camera_data_list) {
        auto [imu_seq, delta] = match_imu_timestamp(cam.timestamp_us, imu_data_list);
        if (imu_seq >= 0) {
            outfile << "CAMERA " << cam.sequence
                    << " IMU " << imu_seq
                    << " DELTA " << delta
                    << std::endl;
        } else {
            outfile << "CAMERA " << cam.sequence << " UNMATCHED" << std::endl;
        }
    }

    outfile.close();
    std::cout << "[Consumer] 匹配结果已写入: " << output_filename << std::endl;
}

// ==========================================
// 主函数
// ==========================================
int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cout << "用法: " << argv[0]
                  << " <相机数据文件> <IMU数据文件> <输出文件>" << std::endl;
        std::cout << "示例: " << argv[0]
                  << " camera.txt imu.txt result.txt" << std::endl;
        return 1;
    }

    std::string camera_file = argv[1];
    std::string imu_file = argv[2];
    std::string output_file = argv[3];

    std::cout << "=== 传感器时间同步程序 ===" << std::endl;
    std::cout << "相机数据文件: " << camera_file << std::endl;
    std::cout << "IMU数据文件: " << imu_file << std::endl;
    std::cout << "输出文件: " << output_file << std::endl;
    std::cout << "队列容量: " << QUEUE_CAPACITY << std::endl;
    std::cout << std::endl;

    // 创建并启动线程
    std::thread camera_thread(camera_producer, camera_file);
    std::thread imu_thread(imu_producer, imu_file);
    std::thread consumer_thread(consumer, output_file);

    // 等待所有线程结束
    camera_thread.join();
    imu_thread.join();
    consumer_thread.join();

    std::cout << std::endl;
    std::cout << "=== 所有线程已正常退出 ===" << std::endl;
    return 0;
}
