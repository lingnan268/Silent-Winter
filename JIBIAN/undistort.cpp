#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    std::string calib_file = "calibration_result.xml";
    std::string images_dir = "/mnt/c/Users/asus/Pictures/Camera Roll";
    std::string output_dir = "undistort_results";

    if (argc > 1) images_dir = argv[1];

    cv::FileStorage fs_in(calib_file, cv::FileStorage::READ);
    if (!fs_in.isOpened()) {
        std::cerr << "无法打开标定文件: " << calib_file << std::endl;
        return -1;
    }

    cv::Mat camera_matrix, dist_coeffs;
    int img_width, img_height;
    fs_in["image_width"] >> img_width;
    fs_in["image_height"] >> img_height;
    fs_in["camera_matrix"] >> camera_matrix;
    fs_in["distortion_coefficients"] >> dist_coeffs;
    fs_in.release();

    std::cout << "已加载标定数据" << std::endl;
    std::cout << "内参矩阵:\n" << camera_matrix << std::endl;
    std::cout << "畸变系数: " << dist_coeffs.t() << std::endl;

    cv::Size image_size(img_width, img_height);

    cv::Mat map1, map2;
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, image_size, 1.0, image_size);
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(),
        new_camera_matrix, image_size, CV_16SC2, map1, map2);

    // 创建输出目录
    fs::create_directories(output_dir);

    // 收集所有标定照片
    std::vector<std::string> image_names;
    for (const auto& entry : fs::directory_iterator(images_dir)) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".jpg" || ext == ".png" || ext == ".jpeg") {
            image_names.push_back(entry.path().string());
        }
    }
    std::sort(image_names.begin(), image_names.end());

    if (image_names.empty()) {
        std::cerr << "目录中未找到图片: " << images_dir << std::endl;
        return -1;
    }

    std::cout << "\n共找到 " << image_names.size() << " 张照片" << std::endl;
    std::cout << "开始生成去畸变对比图...\n" << std::endl;

    for (int idx = 0; idx < (int)image_names.size(); idx++) {
        cv::Mat frame = cv::imread(image_names[idx]);
        if (frame.empty()) continue;
        if (frame.size() != image_size) {
            cv::resize(frame, frame, image_size);
        }

        cv::Mat undistorted;
        cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);

        cv::Mat left = frame.clone();
        cv::Mat right = undistorted.clone();

        cv::putText(left, "Original", cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        cv::putText(right, "Undistorted", cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        cv::Mat combined;
        cv::hconcat(left, right, combined);

        std::string fname = output_dir + "/compare_" +
            std::to_string(idx + 1) + ".png";
        cv::imwrite(fname, combined);
        std::cout << "[" << (idx + 1) << "/" << image_names.size() << "] " << fname << std::endl;
    }

    // 保存第一张的高清对比作为主结果
    if (!image_names.empty()) {
        cv::Mat frame = cv::imread(image_names[0]);
        if (!frame.empty()) {
            if (frame.size() != image_size) {
                cv::resize(frame, frame, image_size);
            }
            cv::Mat undistorted;
            cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);

            cv::Mat left = frame.clone();
            cv::Mat right = undistorted.clone();
            cv::putText(left, "Original", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            cv::putText(right, "Undistorted", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            cv::Mat combined;
            cv::hconcat(left, right, combined);
            cv::imwrite("undistort_result.png", combined);
        }
    }

    std::cout << "\n完成！对比图已保存到 " << output_dir << "/ 目录" << std::endl;
    std::cout << "主结果图: undistort_result.png" << std::endl;
    return 0;
}
