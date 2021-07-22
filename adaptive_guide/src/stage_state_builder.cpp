#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class StageStateBuilderNode : public rclcpp::Node
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    StageStateBuilderNode()
        : Node("stage_state_builder_node")
    {
        // Needle pose publisher
        pose_publisher = this->create_publisher<PoseStamped>("stage/state/needle_pose", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&StageStateBuilderNode::timer_callback, this));

        // Needle control subscribers
        stage_subscriber = this->create_subscription<PoseStamped>(
            "stage/state/pose",
            10,
            std::bind(&StageStateBuilderNode::stage_pose_callback, this, std::placeholders::_1));

        needle_subscriber = this->create_subscription<PoseStamped>(
            "needle/state/pose",
            10,
            std::bind(&StageStateBuilderNode::needle_pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Needle state builder ready.");
    }

private:
    void timer_callback()
    {
        auto message = PoseStamped();
        message.pose.set__position(position);
        message.pose.set__orientation(orientation);
        message.header.set__stamp(this->get_clock()->now());

        pose_publisher->publish(message);
    }

    void stage_pose_callback(const PoseStamped::SharedPtr msg) 
    {
        position.set__x(msg->pose.position.x);
        position.set__z(msg->pose.position.z);
    }

    void needle_pose_callback(const PoseStamped::SharedPtr msg)
    {
        position.set__y(msg->pose.position.y);
        orientation = msg->pose.orientation;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<PoseStamped>::SharedPtr stage_subscriber;
    rclcpp::Subscription<PoseStamped>::SharedPtr needle_subscriber;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StageStateBuilderNode>());
    rclcpp::shutdown();
    return 0;
}
