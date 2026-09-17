#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>

namespace fs = std::filesystem;

// ---------- 卡尔曼滤波器（位置预测） ----------
struct KalmanTracker {
    cv::KalmanFilter kf;
    cv::Point2f last_pos;
    bool initialized = false;

    KalmanTracker() : kf(4, 2, 0, CV_32F) {
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    }

    void update(const cv::Point2f& measurement) {
        if (!initialized) {
            kf.statePost.at<float>(0) = measurement.x;
            kf.statePost.at<float>(1) = measurement.y;
            kf.statePost.at<float>(2) = 0;
            kf.statePost.at<float>(3) = 0;
            initialized = true;
        } else {
            cv::Mat measurement_mat = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);
            kf.correct(measurement_mat);
        }
        last_pos = measurement;
    }

    cv::Point2f predict() {
        cv::Mat pred = kf.predict();
        return cv::Point2f(pred.at<float>(0), pred.at<float>(1));
    }
};

// ---------- 检测结果 ----------
struct Detection {
    cv::Rect bbox;
    cv::Point2f center;
    double area;
    std::string label;
    bool is_r_marker = false;
    bool is_energy_rune = false;
};

int main(int argc, char** argv) {
    std::string video_path = "/mnt/f/健身APP/111/333/能量机关视频.mp4";
    std::string output_video = "rune_detect_output.mp4";
    bool save_frames = false;

    if (argc > 1) video_path = argv[1];
    if (argc > 2) output_video = argv[2];

    cv::VideoCapture cap(video_path, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        cap.open(video_path);
    }
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频: " << video_path << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    int total_frames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    int width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    std::cout << "视频信息: " << width << "x" << height
              << " " << fps << "fps " << total_frames << "帧" << std::endl;

    // 缩小处理尺寸（加速处理）
    int target_width = 960;
    double scale = (double)target_width / width;
    int target_height = (int)(height * scale);
    cv::Size target_size(target_width, target_height);

    // 输出视频
    cv::VideoWriter writer(output_video, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                           fps, target_size, true);
    if (!writer.isOpened()) {
        std::cerr << "无法创建输出视频" << std::endl;
        return -1;
    }

    // 卡尔曼跟踪器（能量机关位置预测）
    KalmanTracker rune_tracker;
    KalmanTracker rmarker_tracker;

    int frame_idx = 0;
    int detected_frames = 0;
    int total_detectable = 0;

    cv::namedWindow("Energy Rune Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Energy Rune Detection", target_width, target_height);

    cv::Mat frame;
    while (cap.read(frame)) {
        // 缩小
        cv::Mat small;
        cv::resize(frame, small, target_size);

        // HSV 转换
        cv::Mat hsv;
        cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);

        // 红色阈值分割（红色在HSV中分布在两端）
        cv::Mat mask1, mask2, red_mask;
        cv::inRange(hsv, cv::Scalar(0, 30, 30), cv::Scalar(15, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(165, 30, 30), cv::Scalar(180, 255, 255), mask2);
        red_mask = mask1 | mask2;

        // 形态学操作（去噪 + 连接断裂）
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(red_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 筛选轮廓
        std::vector<Detection> detections;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 30) continue;  // 过滤太小的噪点

            cv::Rect bbox = cv::boundingRect(contour);
            cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);

            double aspect = (double)bbox.width / bbox.height;
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            double hull_area = cv::contourArea(hull);
            double solidity = hull_area > 0 ? area / hull_area : 0;

            Detection det;
            det.bbox = bbox;
            det.center = center;
            det.area = area;

            // 分类规则：
            // 1. R标：面积中等(30~200)，近似正方形(aspect 0.7~1.3)，solidity高
            // 2. 能量机关：面积较大(>200)，或长条形
            if (area > 30 && area < 250 && aspect > 0.6 && aspect < 1.5 && solidity > 0.6) {
                det.is_r_marker = true;
                det.label = "R-Marker";
            } else if (area >= 250) {
                det.is_energy_rune = true;
                det.label = "Energy Rune";
            } else {
                // 小面积长条形也算能量机关的一部分
                if (aspect > 1.5 || aspect < 0.5) {
                    det.is_energy_rune = true;
                    det.label = "Energy Rune";
                } else {
                    det.label = "Unknown";
                }
            }
            detections.push_back(det);
        }

        // 统计识别
        bool found_rune = false;
        bool found_rmarker = false;
        for (const auto& d : detections) {
            if (d.is_energy_rune) found_rune = true;
            if (d.is_r_marker) found_rmarker = true;
        }
        if (found_rune || found_rmarker) detected_frames++;
        total_detectable++;

        // 卡尔曼预测
        for (const auto& d : detections) {
            if (d.is_energy_rune) {
                rune_tracker.update(d.center);
            }
            if (d.is_r_marker) {
                rmarker_tracker.update(d.center);
            }
        }
        cv::Point2f rune_pred = rune_tracker.predict();
        cv::Point2f rmarker_pred = rmarker_tracker.predict();

        // 绘制结果
        cv::Mat result = small.clone();

        for (const auto& d : detections) {
            cv::Scalar color;
            if (d.is_energy_rune) {
                color = cv::Scalar(0, 255, 0);  // 绿色
            } else if (d.is_r_marker) {
                color = cv::Scalar(0, 255, 255);  // 黄色
            } else {
                color = cv::Scalar(128, 128, 128);  // 灰色
            }

            cv::rectangle(result, d.bbox, color, 2);
            cv::putText(result, d.label + " A=" + std::to_string((int)d.area),
                        cv::Point(d.bbox.x, d.bbox.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            cv::circle(result, d.center, 3, color, -1);
        }

        // 绘制卡尔曼预测位置（蓝色十字）
        if (rune_tracker.initialized) {
            cv::drawMarker(result, rune_pred, cv::Scalar(255, 0, 0),
                           cv::MARKER_CROSS, 15, 2);
            cv::putText(result, "Predict", rune_pred + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
        }
        if (rmarker_tracker.initialized) {
            cv::drawMarker(result, rmarker_pred, cv::Scalar(255, 0, 255),
                           cv::MARKER_CROSS, 15, 2);
        }

        // 帧信息
        std::string info = "Frame: " + std::to_string(frame_idx + 1) + "/" + std::to_string(total_frames) +
                          " | Rune: " + (found_rune ? "YES" : "NO") +
                          " | R-Marker: " + (found_rmarker ? "YES" : "NO");
        cv::putText(result, info, cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // 识别率
        double rate = total_detectable > 0 ? (double)detected_frames / total_detectable * 100.0 : 0;
        std::string rate_str = "Rate: " + cv::format("%.1f", rate) + "%";
        cv::putText(result, rate_str, cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

        writer.write(result);
        cv::imshow("Energy Rune Detection", result);

        if (save_frames && frame_idx % 100 == 0) {
            std::string fname = "frame_result_" + std::to_string(frame_idx) + ".png";
            cv::imwrite(fname, result);
        }

        // waitKey(100) 按作业要求
        int key = cv::waitKey(100) & 0xFF;
        if (key == 27) break;

        frame_idx++;
        if (frame_idx % 100 == 0) {
            std::cout << "处理中: " << frame_idx << "/" << total_frames
                      << " 识别率: " << rate << "%" << std::endl;
        }
    }

    cap.release();
    writer.release();
    cv::destroyAllWindows();

    double final_rate = total_detectable > 0 ? (double)detected_frames / total_detectable * 100.0 : 0;
    std::cout << "\n==== 识别结果 ====" << std::endl;
    std::cout << "总帧数: " << total_frames << std::endl;
    std::cout << "有效检测帧: " << detected_frames << "/" << total_detectable << std::endl;
    std::cout << "识别率: " << final_rate << "%" << std::endl;
    std::cout << "输出视频: " << output_video << std::endl;

    return 0;
}
