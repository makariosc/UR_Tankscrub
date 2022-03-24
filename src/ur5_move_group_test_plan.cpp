#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_move_group_test");
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


    // Initializing visualization tools
    namespace rvt = rviz_visual_tools;

    // Initialized with the base link of the robot we want to visualize(?)
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    // Not strictly needed(?), but good practice to clear all markers after use 
    visual_tools.deleteAllMarkers();

    // This is a freaking rviz function.
    // Lets you press buttons in Rviz programatically
    visual_tools.loadRemoteControl();

    // Send all needed messages over the appropriate channels all at once
    visual_tools.trigger();

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

    // Create box wall just for kicks
    moveit_msgs::CollisionObject collision_object;
    
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "x+";

    // Define shape of the box wall
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 1.0;

    // Define pose of box
    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = 0.0;
    wall_pose.position.y = 0.6 + 0.1/2.0; // to account for wall thickness
    wall_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(wall_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.prompt("Press 'next' to continue.");

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;

    start_pose2.position.x = 0.0;
    start_pose2.position.y = 0.5;
    start_pose2.position.z = 0.5;

    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);
    
    geometry_msgs::Pose target_pose3 = start_pose2;
    
    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);

    // for (std::size_t i = 0; i < waypoints.size(); i++) {
    //     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // }    
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run on the plant");
    move_group.move();

    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = M_PI;
    // move_group.setJointValueTarget(joint_group_positions);

    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");




    // visual_tools.trigger();

}