#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class ColorDetector : public rclcpp::Node {
public:
    ColorDetector() : Node("color_detector") {
        cv::namedWindow("Color Detection", cv::WINDOW_NORMAL);
        cv::resizeWindow("Color Detection", 640, 480);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_image", 10, std::bind(&ColorDetector::image_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "色块识别节点已启动，等待图像...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // 蓝色检测：HSV色彩空间
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat blue_mask;
        cv::inRange(hsv, cv::Scalar(100, 50, 50), cv::Scalar(130, 255, 255), blue_mask);

        std::vector<std::vector<cv::Point>> blue_contours;
        cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto & contour : blue_contours) {
            double area = cv::contourArea(contour);
            if (area < 500) continue;
            cv::Rect rect = cv::boundingRect(contour);
            cv::rectangle(frame, rect, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, "Blue", cv::Point(rect.x, rect.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        }

        // 红色检测：inRange (BGR)
        cv::Mat red_mask;
        cv::inRange(frame, cv::Scalar(0, 0, 100), cv::Scalar(50, 50, 255), red_mask);

        std::vector<std::vector<cv::Point>> red_contours;
        cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto & contour : red_contours) {
            double area = cv::contourArea(contour);
            if (area < 500) continue;
            cv::Rect rect = cv::boundingRect(contour);
            cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, "Red", cv::Point(rect.x, rect.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        RCLCPP_INFO(this->get_logger(), "检测到蓝色块: %d 个, 红色块: %d 个",
                    static_cast<int>(blue_contours.size()),
                    static_cast<int>(red_contours.size()));

        cv::imshow("Color Detection", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetector>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
