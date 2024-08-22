#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <kdl_kinematics/kdl_kinematics.h>

int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_test_node");

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    auto robot_model = robot_model_loader.getModel();

    if (!robot_model)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model.");
        rclcpp::shutdown();
        return 1;
    }
        
    // Declare and retrieve kinematics parameters
    std::string kinematics_solver;
    double search_resolution, timeout;

    // Declare parameters
    node->declare_parameter<std::string>("panda_arm.kinematics_solver", "kdl_kinematics/KDLKinematics");
    node->declare_parameter<double>("panda_arm.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter<double>("panda_arm.kinematics_solver_timeout", 0.005);

    // Retrieve parameters
    if (node->get_parameter("panda_arm.kinematics_solver", kinematics_solver)) {
    RCLCPP_INFO(node->get_logger(), "Kinematics solver: %s", kinematics_solver.c_str());
    } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to get kinematics solver parameter");
           }

    if (node->get_parameter("panda_arm.kinematics_solver_search_resolution", search_resolution)) {
    RCLCPP_INFO(node->get_logger(), "Search resolution: %f", search_resolution); 
    } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get search resolution parameter");
    }

    if (node->get_parameter("panda_arm.kinematics_solver_timeout", timeout)) {
    RCLCPP_INFO(node->get_logger(), "Timeout: %f", timeout);
    } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get timeout parameter");
    }
    /*
    RCLCPP_INFO(node->get_logger(), "Kinematics solver: %s", kinematics_solver.c_str());
    RCLCPP_INFO(node->get_logger(), "Search resolution: %f", search_resolution);
    RCLCPP_INFO(node->get_logger(), "Timeout: %f", timeout);
    */
    // Initialize the KDL IK solver plugin with all required parameters
    kdl_kinematics::KDLKinematics ik_solver;

    std::string group_name = "panda_arm"; // Replace with your group name
    std::string base_frame = "base_link";   // Replace with your robot's base frame
    std::vector<std::string> tip_frames = {"tool0"};  // Replace with your robot's end-effector frame(s)
    double search_discretization = 0.01;

    if (!ik_solver.initialize(node, *robot_model, group_name, base_frame, tip_frames, search_discretization))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize the IK solver.");
        rclcpp::shutdown();
        return 1;
    }
       
    // Define the target end-effector pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = base_frame;
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;

    // Prepare the necessary inputs
    const auto joint_model_group = robot_model->getJointModelGroup(group_name);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> initial_guess(joint_names.size(), 0.0);
    std::vector<double> joint_angles;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    // Optional options (can be customized as needed)
    kinematics::KinematicsQueryOptions options;

    //double timeout = 0.1;  // Timeout in seconds

    // Call the IK solver with all required parameters
    bool found_ik = ik_solver.searchPositionIK(
        target_pose.pose,              // Target pose
        initial_guess,                 // Initial guess for joint values
        timeout,                       // Timeout
        joint_angles,                  // Output solution (joint angles)
        error_code,                    // Error code
        options                        // Kinematics query options
    );

    if (found_ik && error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "IK solution found:");
        for (std::size_t i = 0; i < joint_angles.size(); ++i)
        {
            RCLCPP_INFO(node->get_logger(), "Joint %d: %f", static_cast<int>(i), joint_angles[i]);
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "IK solution could not be found.");
    }

    // Shutdown the ROS 2 node
    rclcpp::shutdown();
    return 0;
}

