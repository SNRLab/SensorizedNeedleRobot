#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

/**
 * Emulate the needle depth and rotation sensors, use until
 * real sensors have been fully integrated. Also exposes topics
 * to modify virtual needle pose.
 **/
class EmulateSensorsNode : public rclcpp::Node
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Float64 = std_msgs::msg::Float64;

    EmulateSensorsNode()
        : Node("emulate_sensors_node")
    {
        // Needle pose publisher
        pose_publisher = this->create_publisher<PoseStamped>("needle/state/pose", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&EmulateSensorsNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Needle sensor emulators ready.");

        // Needle control subscribers
        y_subscriber = this->create_subscription<Float64>(
            "needle/emulated/y",
            10,
            std::bind(&EmulateSensorsNode::y_command_callback, this, std::placeholders::_1));

        theta_subscriber = this->create_subscription<Float64>(
            "needle/emulated/theta",
            10,
            std::bind(&EmulateSensorsNode::theta_command_callback, this, std::placeholders::_1));

        
        RCLCPP_INFO(this->get_logger(), "Needle control emulators ready.");
    }

private:
    void timer_callback()
    {
        auto message = PoseStamped();
        auto position = geometry_msgs::msg::Point();
        auto orientation = geometry_msgs::msg::Quaternion();

        position.set__x(0);
        position.set__y(y);
        position.set__z(0);

        orientation.set__x(0);
        orientation.set__y(sin(theta / 2));
        orientation.set__z(0);
        orientation.set__w(cos(theta / 2));

        message.pose.set__position(position);
        message.pose.set__orientation(orientation);
        message.header.set__stamp(this->get_clock()->now());

        pose_publisher->publish(message);
    }

    void y_command_callback(const Float64::SharedPtr msg) 
    {
        y = msg->data;
    }

    void theta_command_callback(const Float64::SharedPtr msg)
    {
        theta = msg->data;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<Float64>::SharedPtr y_subscriber;
    rclcpp::Subscription<Float64>::SharedPtr theta_subscriber;
    double y;
    double theta;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmulateSensorsNode>());
    rclcpp::shutdown();
    return 0;
}
