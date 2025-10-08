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
#include "custom_interfaces/action/moveit_pose.hpp"

using MoveItPose = custom_interfaces::action::MoveitPose; // Custom action

const double BASE_LINK_X_OFFSET = 0.0;
const double BASE_LINK_Y_OFFSET = 0.0;
const double BASE_LINK_Z_OFFSET = 0.8;

class MoveItPoseActionServer : public rclcpp::Node
{
public:
    using GoalHandleMoveItPose = rclcpp_action::ServerGoalHandle<MoveItPose>;

    explicit MoveItPoseActionServer(const rclcpp::NodeOptions &options)
    : Node("moveit_pose_action_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt pose action server...");

        // Declare and log use_sim_time
        // this->declare_parameter("use_sim_time", true);
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

        // Declare kinematics parameters
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_timeout", 0.005);
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_attempts", 3);

        // Create the action server
        action_server_ = rclcpp_action::create_server<MoveItPose>(
            this,
            "plan_execute_pose",
            std::bind(&MoveItPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveItPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveItPoseActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }
    // MoveItPoseActionServer()
    // : Node("moveit_pose_action_server")
    // {
    //     RCLCPP_INFO(this->get_logger(), "Initializing MoveIt pose action server...");

    //     bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    //     RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

    //     this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    //     this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_search_resolution", 0.005);
    //     this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_timeout", 0.005);
    //     this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_attempts", 3);

    //     // Action server setup (can remain here)
    //     action_server_ = rclcpp_action::create_server<MoveItPose>(
    //         this,
    //         "plan_execute_pose",
    //         std::bind(&MoveItPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    //         std::bind(&MoveItPoseActionServer::handle_cancel, this, std::placeholders::_1),
    //         std::bind(&MoveItPoseActionServer::handle_accepted, this, std::placeholders::_1)
    //     );
    // }

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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
        target_pose.position.x = pose.position.x + BASE_LINK_X_OFFSET;
        target_pose.position.y = pose.position.y + BASE_LINK_Y_OFFSET;
        target_pose.position.z = pose.position.z + BASE_LINK_Z_OFFSET;

        // Path Constraints
        // moveit_msgs::msg::Constraints path_constraints;

        // moveit_msgs::msg::JointConstraint joint_constraint_shoulder_pan_joint;
        // joint_constraint_shoulder_pan_joint.joint_name = "shoulder_pan_joint";
        // joint_constraint_shoulder_pan_joint.position = 0.0;  // Set desired position
        // joint_constraint_shoulder_pan_joint.tolerance_above = M_PI;  // Set tolerance
        // joint_constraint_shoulder_pan_joint.tolerance_below = M_PI;  // Set tolerance
        // joint_constraint_shoulder_pan_joint.weight = 1.0;  // Set weight
        // path_constraints.joint_constraints.push_back(joint_constraint_shoulder_pan_joint);

        // moveit_msgs::msg::JointConstraint joint_constraint_shoulder_lift_joint;
        // joint_constraint_shoulder_lift_joint.joint_name = "shoulder_lift_joint";
        // joint_constraint_shoulder_lift_joint.position = 0.0;  // Set desired position
        // joint_constraint_shoulder_lift_joint.tolerance_above = M_PI;  // Set tolerance
        // joint_constraint_shoulder_lift_joint.tolerance_below = M_PI;  // Set tolerance
        // joint_constraint_shoulder_lift_joint.weight = 1.0;  // Set weight
        // path_constraints.joint_constraints.push_back(joint_constraint_shoulder_lift_joint);

        // moveit_msgs::msg::JointConstraint joint_constraint_elbow_joint;
        // joint_constraint_elbow_joint.joint_name = "elbow_joint";
        // joint_constraint_elbow_joint.position = 0.0;  // Set desired position
        // joint_constraint_elbow_joint.tolerance_above = M_PI / 2;  // Set tolerance
        // joint_constraint_elbow_joint.tolerance_below = M_PI / 2;  // Set tolerance
        // joint_constraint_elbow_joint.weight = 1.0;  // Set weight
        // path_constraints.joint_constraints.push_back(joint_constraint_elbow_joint);

        // moveit_msgs::msg::JointConstraint joint_constraint_wrist_3_joint;
        // joint_constraint_wrist_3_joint.joint_name = "wrist_3_joint";
        // joint_constraint_wrist_3_joint.position = 0.0;  // Set desired position
        // joint_constraint_wrist_3_joint.tolerance_above = M_PI / 2;  // Set tolerance
        // joint_constraint_wrist_3_joint.tolerance_below = M_PI / 2;  // Set tolerance
        // joint_constraint_wrist_3_joint.weight = 1.0;  // Set weight
        // path_constraints.joint_constraints.push_back(joint_constraint_wrist_3_joint);

        // move_group_->setPathConstraints(path_constraints);
    
        // auto goal = goal_handle->get_goal();
        // geometry_msgs::msg::Pose target_pose = goal->pose;

        // move_group_->setPoseTarget(target_pose, "tool0");
        move_group_->setJointValueTarget(target_pose, "tool0");

        // Collision object

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.resize(2);

        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "world";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[0].primitives[0].dimensions = {0.608, 2.0, 0.8};
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.576;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = 0.4;
        collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects[1].id = "base";
        collision_objects[1].header.frame_id = "world";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[1].primitives[0].dimensions = {1, 1, 0.79}; 
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = -0.3;
        collision_objects[1].primitive_poses[0].position.y = 0.0;
        collision_objects[1].primitive_poses[0].position.z = 0.4;
        collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;


        // Add objects to the scene
        planning_scene_interface.applyCollisionObjects(collision_objects);


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
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
    auto node = std::make_shared<MoveItPoseActionServer>(options);
    node->init_move_group(); // Initialize MoveGroupInterface after node creation
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
