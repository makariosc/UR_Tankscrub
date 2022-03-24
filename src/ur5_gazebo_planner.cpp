#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_gazebo_planner");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialized with the name of the planning group we want to control and plan for
    // This should be loaded into the ROS Parameter server at the start of the program.
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Get raw pointers that point to planning group for improved performance
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Visual tools
    namespace rvt = rviz_visual_tools;

    // Initialized with the base link of the robot we want to visualize
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    // Initialize plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Create pointer that references current robot's current state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Create box wall just for kicks
    moveit_msgs::CollisionObject collision_object;
    
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "x+";

    const double WALL_THICKNESS = 0.1;
    const double WALL_LENGTH = 1.2;

    // Define a wall primitive
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = WALL_LENGTH;
    primitive.dimensions[1] = WALL_THICKNESS;
    primitive.dimensions[2] = WALL_LENGTH;

    for (int i = 0; i < 5; i++) {
        collision_object.primitives.push_back(primitive);
    }

    
    // Define wall poses
    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = 0.0;
    wall_pose.position.y = 0.6 + WALL_THICKNESS/2.0; // to account for wall thickness
    wall_pose.position.z = 0.0;
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.position.y = -0.6 - WALL_THICKNESS/2.0;
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.orientation.w = 0.707;
    wall_pose.orientation.z = 0.707;
    wall_pose.position.y = 0;
    wall_pose.position.x = 0.6 + WALL_THICKNESS/2.0;
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.position.x = -0.6 - WALL_THICKNESS/2.0;
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.orientation.x = 0.707;
    wall_pose.orientation.y = 0.0;
    wall_pose.orientation.z = 0.0;
    wall_pose.orientation.w = 0.707;
    wall_pose.position.x = 0;
    wall_pose.position.y = 0;
    wall_pose.position.z = 0.6 + WALL_THICKNESS/2.0;
    collision_object.primitive_poses.push_back(wall_pose);

    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.prompt("Press 'next' to continue.");    

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;

    // Orientation
    start_pose2.orientation.w = 1;

    // Position
    start_pose2.position.x = 0.29;
    start_pose2.position.y = 0.525;
    start_pose2.position.z = 0.525;

    // Program the trajectory we want to follow
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;
    double height_traveled = 0;
    while (height_traveled < 0.1) 
    {
        target_pose3.position.y -= 0.525 * 2; 
        waypoints.push_back(target_pose3);
    
        target_pose3.position.z -= 0.15;
        waypoints.push_back(target_pose3);

        target_pose3.position.y += 0.525 * 2;
        waypoints.push_back(target_pose3);

        target_pose3.position.z -= 0.15;
        waypoints.push_back(target_pose3);

        height_traveled += 0.3;
    }

    move_group.setMaxVelocityScalingFactor(0.001);
    move_group.setPlanningTime(30.0);

    // Some planning parameters
    const int PLANNING_ATTEMPTS = 15;
    const int PLANNING_TIME = 10;
    double fraction_planned = -1;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    moveit_msgs::RobotTrajectory trajectory;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        start_state.setFromIK(joint_model_group, start_pose2);
        move_group.setStartState(start_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);

    // for (std::size_t i = 0; i < waypoints.size(); i++) {
    //     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // }    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to send state to Gazebo.");

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    gazebo_msgs::SetModelConfiguration srv;

    srv.request.model_name = "robot";
    srv.request.urdf_param_name = "robot_description";
    srv.request.joint_names = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", 
                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    start_state.copyJointGroupPositions(
        start_state.getRobotModel()->getJointModelGroup(move_group.getName()), 
        srv.request.joint_positions);

    // std::cout << srv.request.joint_positions << std::endl;

    if (client.call(srv)) {
        ROS_INFO("Called gazebo service set_model_configuration.");
        ROS_INFO("Status: %d", (bool)srv.response.success);
    }
    else {
        ROS_ERROR("Failed to call gazebo service set_model_configuration.");
        return 1;
    }

}