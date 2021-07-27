#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "stage_control_interfaces/action/move_stage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
//#include "stage_control/visibility_control.h"

using namespace std::chrono_literals;

namespace stage_control
{
    class StageControlNode : public rclcpp::Node
    {
    public:
        using MoveStage = stage_control_interfaces::action::MoveStage;
        using GoalHandleMoveStage = rclcpp_action::ServerGoalHandle<MoveStage>;
        using Float64 = std_msgs::msg::Float64;

        //STAGE_CONTROL_PUBLIC
        explicit StageControlNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("move_stage_node", options)
        {
            using namespace std::placeholders;

            // Subscribe to hardware topics
            hardware_x_subscriber = this->create_subscription<Float64>(
                "stage/joint_states/x",
                10,
                std::bind(&StageControlNode::x_state_callback, this, std::placeholders::_1));

            hardware_z_subscriber = this->create_subscription<Float64>(
                "stage/joint_states/z",
                10,
                std::bind(&StageControlNode::z_state_callback, this, std::placeholders::_1));

            // Publish to hardware joint command topics
            hardware_x_publisher = this->create_publisher<Float64>("stage/x_position_contoller/command", 10);
            hardware_z_publisher = this->create_publisher<Float64>("stage/z_position_contoller/command", 10);

            // Subscribe to virtual topics


            // Publish to virtual joint command topics


            // Start stage position publisher
            RCLCPP_INFO(this->get_logger(), "Starting stage pose publisher...");
            this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/stage/state/pose", 10);
            this->pose_timer_ = this->create_wall_timer(
                50ms, std::bind(&StageControlNode::pose_timer_callback, this));

            // Start action server
            RCLCPP_INFO(this->get_logger(), "Starting stage control action server...");
            this->action_server_ = rclcpp_action::create_server<MoveStage>(
                this,
                "move_stage",
                std::bind(&StageControlNode::handle_goal, this, _1, _2),
                std::bind(&StageControlNode::handle_cancel, this, _1),
                std::bind(&StageControlNode::handle_accepted, this, _1));
            
            RCLCPP_INFO(this->get_logger(), "Stage control ready.");
        }

    private:
        rclcpp::Subscription<Float64>::SharedPtr hardware_x_subscriber;
        rclcpp::Subscription<Float64>::SharedPtr hardware_z_subscriber;
        rclcpp::Publisher<Float64>::SharedPtr hardware_x_publisher;
        rclcpp::Publisher<Float64>::SharedPtr hardware_z_publisher;
        rclcpp_action::Server<MoveStage>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr pose_timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        double current_x;
        double current_z;

        void x_state_callback(const Float64::SharedPtr msg) 
        {
            this->current_x = msg->data;
        }

        void z_state_callback(const Float64::SharedPtr msg) 
        {
            this->current_z = msg->data;
        }

        void pose_timer_callback()
        {
            auto message = geometry_msgs::msg::PoseStamped();
            auto position = geometry_msgs::msg::Point();
            auto orientation = geometry_msgs::msg::Quaternion();

            position.set__x(this->current_x);
            position.set__y(0);
            position.set__z(this->current_z);
            orientation.set__w(1);

            message.pose.set__position(position);
            message.pose.set__orientation(orientation);
            message.header.set__stamp(this->get_clock()->now());

            this->pose_publisher_->publish(message);
        }

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const MoveStage::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with position (%f, %f)", goal->x, goal->z);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleMoveStage> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMoveStage> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&StageControlNode::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleMoveStage> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(50);

            // Set up messages
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<MoveStage::Feedback>();
            auto result = std::make_shared<MoveStage::Result>();

            // Send command to motors
            MoveStageHardware(goal->x, goal->z);

            // Get current time
            double start = this->now().seconds();

            while (calculate_error(goal->x, goal->z) > goal->eps)
            {
                // Check if there is a cancel request
                if (goal_handle->is_canceling())
                {
                    result->error_code = 1;
                    result->x = this->current_x;
                    result->z = this->current_z;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                // Publish feedback
                feedback->x = this->current_x;
                feedback->z = this->current_z;
                feedback->time = this->now().seconds() - start;
                feedback->error = calculate_error(goal->x, goal->z);

                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            // Check if goal is done
            if (rclcpp::ok())
            {
                result->x = this->current_x;
                result->z = this->current_z;
                result->time = this->now().seconds() - start;
                result->error = calculate_error(goal->x, goal->z);
                result->error_code = 0;

                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        void MoveStageHardware(double x, double z)
        {
            RCLCPP_INFO(this->get_logger(), "Moving stage hardware from (%f, %f) to (%f, %f)", 
                this->current_x, this->current_z, x, z);

            auto x_command = Float64();
            auto z_command = Float64();

            x_command.data = x;
            z_command.data = z;

            this->hardware_x_publisher->publish(x_command);
            this->hardware_z_publisher->publish(z_command);
        }

        double calculate_error(double x, double z)
        {
            return sqrt(pow(x - this->current_x, 2) + pow(z - this->current_z, 2));
        }
    }; // class StageControlNode

} // namespace stage_control

//RCLCPP_COMPONENTS_REGISTER_NODE(stage_control::StageControlNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<stage_control::StageControlNode>();

    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
