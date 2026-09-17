#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    const std::string images_dir = "/mnt/c/Users/asus/Pictures/Camera Roll";
    const std::string output_file = "calibration_result.xml";
    const std::string corners_dir = "corners_detection";
    const cv::Size board_size(8, 5);
    const float square_size = 25.0f;

    fs::create_directories(corners_dir);

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;

    std::vector<cv::Point3f> obj_template;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            obj_template.emplace_back(j * square_size, i * square_size, 0.0f);
        }
    }

    int valid_count = 0;
    std::vector<std::string> image_names;

    for (const auto& entry : fs::directory_iterator(images_dir)) {
        std::string ext = entry.path().extension().string();
        if (ext == ".jpg" || ext == ".JPG" || ext == ".png" || ext == ".PNG") {
            image_names.push_back(entry.path().string());
        }
    }
    std::sort(image_names.begin(), image_names.end());

    std::cout << "共找到 " << image_names.size() << " 张照片" << std::endl;
    std::cout << "棋盘格内角点: " << board_size.width << " x " << board_size.height << std::endl;
    std::cout << "开始逐张检测角点...\n" << std::endl;

    for (const auto& path : image_names) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) {
            std::cerr << "无法读取: " << path << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));

            image_size = img.size();
            image_points.push_back(corners);
            object_points.push_back(obj_template);
            valid_count++;

            // 保存角点检测结果图
            cv::Mat display = img.clone();
            cv::drawChessboardCorners(display, board_size, corners, found);
            std::string num_str = std::to_string(valid_count);
            std::string label = num_str + "/" + std::to_string(image_names.size());
            cv::putText(display, label, cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(display, fs::path(path).filename().string(),
                cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 1);

            std::string out_path = corners_dir + "/corners_" +
                (valid_count < 10 ? "0" : "") + num_str + ".png";
            cv::imwrite(out_path, display);

            std::cout << "[OK] " << valid_count << "/" << image_names.size()
                      << " " << fs::path(path).filename().string()
                      << " -> " << out_path << std::endl;
        } else {
            std::cout << "[SKIP] " << fs::path(path).filename().string()
                      << " (未检测到棋盘格)" << std::endl;
        }
    }

    if (valid_count < 5) {
        std::cerr << "\n有效图片仅 " << valid_count << " 张，至少需要 5 张，标定失败！" << std::endl;
        return -1;
    }

    std::cout << "\n==== 开始标定 ====" << std::endl;
    std::cout << "有效图片: " << valid_count << " 张" << std::endl;
    std::cout << "图像尺寸: " << image_size.width << " x " << image_size.height << std::endl;

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(object_points, image_points, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "\n==== 标定结果 ====" << std::endl;
    std::cout << "重投影误差 (RMS): " << rms << std::endl;
    std::cout << "\n相机内参矩阵:" << std::endl;
    std::cout << camera_matrix << std::endl;
    std::cout << "\n畸变系数 (k1, k2, p1, p2, k3):" << std::endl;
    std::cout << dist_coeffs.t() << std::endl;

    if (rms < 0.5) {
        std::cout << "\n标定合格！重投影误差 < 0.5" << std::endl;
    } else {
        std::cout << "\n警告：重投影误差 >= 0.5，建议重新拍摄标定照片" << std::endl;
    }

    cv::FileStorage fs_out(output_file, cv::FileStorage::WRITE);
    fs_out << "image_width" << image_size.width;
    fs_out << "image_height" << image_size.height;
    fs_out << "board_width" << board_size.width;
    fs_out << "board_height" << board_size.height;
    fs_out << "square_size" << square_size;
    fs_out << "num_images" << valid_count;
    fs_out << "reprojection_error" << rms;
    fs_out << "camera_matrix" << camera_matrix;
    fs_out << "distortion_coefficients" << dist_coeffs;
    fs_out.release();

    std::cout << "\n标定结果已保存到: " << output_file << std::endl;

    double total_error = 0;
    double total_points = 0;
    for (size_t i = 0; i < object_points.size(); i++) {
        std::vector<cv::Point2f> proj_points;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i],
            camera_matrix, dist_coeffs, proj_points);
        double err = cv::norm(image_points[i], proj_points, cv::NORM_L2);
        total_error += err * err;
        total_points += object_points[i].size();
    }
    double avg_error = std::sqrt(total_error / total_points);
    std::cout << "平均重投影误差: " << avg_error << " 像素" << std::endl;

    return 0;
}
