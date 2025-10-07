#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Include ROS2 action interfaces
#include <rclcpp_action/rclcpp_action.hpp>
// #include "/home/final-project/ur_yt_ws/install/ur_yt_sim/include/ur_yt_sim/ur_yt_sim/action/move_it_pose.hpp" // Replace with your custom action if needed
#include "action_tutorials_interfaces/action/moveit_pose.hpp"

using MoveItPose = action_tutorials_interfaces::action::MoveitPose; // Example action

class MoveItPoseActionServer : public rclcpp::Node
{
public:
    using GoalHandleMoveItPose = rclcpp_action::ServerGoalHandle<MoveItPose>;

    // MoveItPoseActionServer()
    // : Node("moveit_pose_action_server")
    // {
    //     RCLCPP_INFO(this->get_logger(), "Initializing MoveIt pose action server...");

    //     // Initialize MoveGroupInterface
    //     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
    //     move_group_->setPoseReferenceFrame("base_link");
    //     move_group_->setPlanningTime(10.0);

    //     RCLCPP_INFO(this->get_logger(), "Pose reference frame set to: %s", move_group_->getPoseReferenceFrame().c_str());
    //     RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    //     RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());

    //     // Action server
    //     action_server_ = rclcpp_action::create_server<MoveItPose>(
    //         this,
    //         "plan_execute_pose",
    //         std::bind(&MoveItPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    //         std::bind(&MoveItPoseActionServer::handle_cancel, this, std::placeholders::_1),
    //         std::bind(&MoveItPoseActionServer::handle_accepted, this, std::placeholders::_1)
    //     );
    // }
    MoveItPoseActionServer()
    : Node("moveit_pose_action_server")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt pose action server...");

        // Action server setup (can remain here)
        action_server_ = rclcpp_action::create_server<MoveItPose>(
            this,
            "plan_execute_pose",
            std::bind(&MoveItPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveItPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveItPoseActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

    void init_move_group()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setPlanningTime(10.0);

        RCLCPP_INFO(this->get_logger(), "Pose reference frame set to: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    }

private:
    rclcpp_action::Server<MoveItPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const MoveItPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new MoveIt pose goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing MoveIt trajectory...");
        auto goal = goal_handle->get_goal();
        auto pose = goal->pose;
        RCLCPP_INFO(this->get_logger(), "Received Pose: position(%.3f, %.3f, %.3f) orientation(%.3f, %.3f, %.3f, %.3f)",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        
        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion orientation;
        // orientation.setRPY(-3.136, 0.000, -1.570);
        // target_pose.orientation = tf2::toMsg(orientation);
        // target_pose.position.x = 0.495;
        // target_pose.position.y = 0.0;
        // target_pose.position.z = 0.164;
        orientation.setRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = pose.position.x;
        target_pose.position.y = pose.position.y;
        target_pose.position.z = pose.position.z;


        // auto goal = goal_handle->get_goal();
        // geometry_msgs::msg::Pose target_pose = goal->pose;

        move_group_->setPoseTarget(target_pose, "tool0");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        auto result = std::make_shared<MoveItPose::Result>();
        if (success)
        {
            move_group_->move();
            RCLCPP_INFO(this->get_logger(), "Motion execution completed successfully.");
            result->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed!");
            result->success = false;
        }

        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItPoseActionServer>();
    node->init_move_group(); // Initialize MoveGroupInterface after node creation
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
