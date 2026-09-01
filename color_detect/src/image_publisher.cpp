#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher"), count_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&ImagePublisher::timer_callback, this));

        image_ = cv::imread("/home/asus/ros2_ws/src/color_detect/test.jpg");
        if (image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "无法加载图片 /home/asus/ros2_ws/src/color_detect/test.jpg");
        } else {
            RCLCPP_INFO(this->get_logger(), "图片加载成功，尺寸: %dx%d", image_.cols, image_.rows);
        }
    }

private:
    void timer_callback() {
        if (image_.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";
        publisher_->publish(*msg);
        count_++;
        RCLCPP_INFO(this->get_logger(), "发布图像第 %d 帧", count_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat image_;
    int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
