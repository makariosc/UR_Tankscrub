#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <math.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_sim_test_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialized with the name of the planning group we want to control and plan for
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Use raw pointers to refer to the planning group for improved performance (??!!)
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // This is for adding and removing collision objects in our virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Demonstration of how to print basic debug information:
    
    // Print the name of the reference frame of the robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Print the name of the end-effector link of the group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // List all groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available planning groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // List all joints in the move group:
    ROS_INFO_NAMED("tutorial", "Available joints:");
    std::copy(move_group.getJoints().begin(), move_group.getJoints().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));


    // Initialize plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Create pointer that references current robot's current state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] += M_PI;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");


    move_group.move();

}