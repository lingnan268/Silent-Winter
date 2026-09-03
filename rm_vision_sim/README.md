# 传感器时间同步程序 (sensor_sync)

## 程序概述

本程序模拟 RoboMaster 比赛中相机与 IMU 两个传感器的数据同步过程。使用多线程生产者-消费者模型，两个生产者线程分别读取相机和 IMU 数据文件，通过有界线程安全队列传递数据，消费者线程收集数据后用二分查找进行时间戳匹配。

## 编译与运行

```bash
cd ~/sensor_sync
mkdir build && cd build
cmake ..
make
./sensor_sync ../data/camera.txt ../data/imu.txt result.txt
cat result.txt
```

## 数据结构

```cpp
enum class SensorType { CAMERA, IMU };

struct SensorData {
    SensorType type;
    int sequence;
    long long timestamp_us;
};

template <typename T>
class ThreadSafeQueue {
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cond_not_full_;
    std::condition_variable cond_not_empty_;
    size_t capacity_;
    bool closed_;
};
```

## 问题回答

### 1. 队列已满或为空时，线程如何进入等待状态？

使用 `std::mutex`（互斥锁）和两个 `std::condition_variable`（条件变量）实现：

- **队列满时，生产者等待**：生产者调用 `push()` 时，先加锁，然后用 `cond_not_full_.wait()` 等待。`wait()` 内部会**自动释放锁**并阻塞线程，避免忙等（busy-waiting），不消耗 CPU。当消费者取走数据后会调用 `cond_not_full_.notify_one()` 唤醒等待的生产者。

- **队列空时，消费者等待**：消费者调用 `wait_and_pop()` 时，先加锁，然后用 `cond_not_empty_.wait()` 等待。当生产者推入数据后会调用 `cond_not_empty_.notify_one()` 唤醒等待的消费者。

- **防虚假唤醒**：`wait()` 使用 Lambda 谓词形式 `wait(lock, [&]{ return 条件; })`，内部用 `while` 循环检查条件，防止操作系统虚假唤醒导致逻辑错误。

```cpp
// 生产者 push()
cond_not_full_.wait(lock, [&] { return queue_.size() < capacity_ || closed_; });

// 消费者 wait_and_pop()
cond_not_empty_.wait(lock, [&] { return !queue_.empty() || closed_; });
```

### 2. 如何判断最后一个生产者已经结束？

使用 `std::atomic<int>` 原子计数器 `g_producers_finished`，初始值为 0，总生产者数 `TOTAL_PRODUCERS = 2`。

每个生产者线程结束时调用 `g_producers_finished.fetch_add(1)`，原子操作保证线程安全。返回值 +1 后等于 `TOTAL_PRODUCERS` 时，说明自己是最后一个结束的生产者，此时调用 `g_queue.close()` 关闭队列：

```cpp
int finished = g_producers_finished.fetch_add(1) + 1;
if (finished >= TOTAL_PRODUCERS) {
    g_queue.close();
}
```

使用 `std::atomic` 而非普通 `int`，保证多线程下的 `++` 操作是原子的，不会出现数据竞争。

### 3. 为什么关闭队列后仍然需要允许消费者读取剩余数据？

队列关闭只是表示**不再有新数据进入**（生产者不再 push），但队列中可能还有之前已推入但尚未被消费的数据。如果不允许继续读取：

- **数据丢失**：队列中剩余的传感器数据会被丢弃，导致匹配结果不完整
- **线程异常退出**：消费者可能在没有处理完所有数据的情况下被强制终止

因此 `wait_and_pop()` 的设计是：关闭后如果队列不为空，仍然返回 `true` 并取出数据；只有当队列**既关闭又为空**时才返回 `false`，让消费者自然退出：

```cpp
cond_not_empty_.wait(lock, [&] { return !queue_.empty() || closed_; });
if (queue_.empty()) return false;  // 关闭且为空，才退出
// 否则继续取数据
```

### 4. 时间戳匹配算法的具体流程和复杂度是什么？

**流程：**

1. 消费者将收集到的 IMU 数据按时间戳升序排序（`std::sort`）
2. 对每条相机数据的时间戳 `camera_ts`，在 IMU 数组中用**二分查找**找到第一个 `>= camera_ts` 的位置 `result_idx`
3. 比较 `result_idx` 和 `result_idx - 1` 两个候选 IMU 数据，取时间差绝对值更小的作为匹配结果

**复杂度分析：**

- 排序：O(M log M)，M 为 IMU 数据条数
- 单次二分查找：O(log M)
- N 条相机数据共查找 N 次：O(N log M)
- **总时间复杂度**：O(M log M + N log M)
- **空间复杂度**：O(N + M)，用于存储两类数据

相比暴力查找 O(N×M)，二分查找大幅降低了匹配开销。

### 5. 如何处理时间差相同、时间戳相同等边界情况？

- **时间差相同**：当 `result_idx` 和 `result_idx - 1` 两个候选 IMU 与相机时间戳的差值绝对值相等时，取序号较小的（即时间更早的）那个，保证匹配结果的确定性：

```cpp
if (delta == min_delta) {
    if (result_idx - 1 < best_idx) {
        best_idx = result_idx - 1;
    }
}
```

- **所有 IMU 时间戳都早于相机**：二分查找返回 `result_idx == -1`，此时匹配最后一条 IMU 数据（虽然时间差较大，但仍是最接近的）：

```cpp
if (result_idx == -1) {
    best_idx = n - 1;
    min_delta = std::abs(imu_data[n - 1].timestamp_us - camera_ts);
}
```

- **IMU 数据为空**：返回 `{-1, 0}`，消费者写入 `UNMATCHED`
- **相机时间戳恰好等于某条 IMU 时间戳**：二分查找找到的 `result_idx` 时间差为 0，直接匹配，无需比较前一个

### 6. 如果某个生产者读取文件失败，如何保证其他线程能够正常退出？

生产者在打开文件失败时执行以下操作：
1. 递增 `g_producers_finished`（即使失败也计数，否则永远不会触发关闭条件）
2. 调用 `g_queue.close()` 关闭队列（唤醒所有等待的线程）

```cpp
if (!file.is_open()) {
    g_producers_finished.fetch_add(1);
    g_queue.close();
    return;
}
```

这样设计的保证：
- **不会死锁**：`close()` 会唤醒所有在 `cond_not_empty_` 和 `cond_not_full_` 上等待的线程
- **其他生产者仍能工作**：另一个生产者可以继续推数据，结束时也会递增计数器
- **消费者能正常退出**：队列关闭后，消费者处理完剩余数据后自然退出
- **异常安全**：使用 `try-catch` 包裹 `push()`，队列已关闭时抛出异常被捕获，生产者安全退出。
