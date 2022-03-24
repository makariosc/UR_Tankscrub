#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_move_group_test");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialized with the name of the planning group we want to control and plan for
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Get raw pointers that point to planning group for improved performance
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Initialized with the base link of the robot we want to visualize
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    // Create pointer that references current robot's current state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();


    // ################### CREATE WALL OBSTACLES ###################


    const double WALL_THICKNESS = 0.1;
    const double WALL_LENGTH = 1.2;

    // Create wall box primitive
    moveit_msgs::CollisionObject collision_object;
    
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "boxes";
    collision_object.operation = collision_object.ADD;

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
    
    // Initialize walls
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);

    geometry_msgs::PoseStamped wall_pose_out;
    geometry_msgs::PoseStamped wall_pose_init;

    wall_pose_init.pose.orientation.w = 1.0;

    // Sleep to give time for the tf broadcasters to spin up
    ros::Duration(0.05).sleep();

    std::vector<std::string> wall_transform_ids = 
      {"wall_yminus", "wall_yplus", "wall_xminus", "wall_xplus", "wall_zplus"};
    
    for (std::string id : wall_transform_ids)
    {
        // For each of the wall transforms, add a wall. The transform identified by `id` is published by
        // the `static_transform_publisher`s that are launched in `hardcoded_planner.launch`.
        wall_pose_init.header.frame_id = id;
        buffer.transform(wall_pose_init, wall_pose_out, "base");
        collision_object.primitive_poses.push_back(wall_pose_out.pose);
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects {collision_object};
    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.prompt("Press 'next' to continue.");    

    // ################### INITIALIZE POSITION ################### 

    const double PATH_SIDE_LENGTH = 0.44 * 2;

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose waypoint_pose;

    // Orientation
    waypoint_pose.orientation.w = 0.707;
    waypoint_pose.orientation.x = 0.707;
    waypoint_pose.orientation.y = 0;
    waypoint_pose.orientation.z = 0;

    // Position -- currently hardcoded
    waypoint_pose.position.x = 0;
    waypoint_pose.position.y = -0.55;
    waypoint_pose.position.z = PATH_SIDE_LENGTH/2;

    move_group.setStartState(*current_state);
    move_group.setPoseTarget(waypoint_pose);

    // Initialize plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move to the starting position.");
    move_group.execute(plan);

    // update current_state with the current position
    current_state = move_group.getCurrentState();

    visual_tools.prompt("Press 'next' to plan cartesian path and visualize");


    // ################### Plan the wall cleaning trajectory ###################
    std::vector<geometry_msgs::Pose> waypoints;
    move_group.setStartState(*current_state);

    // TODO: Make this relative to the origin in the wall, not the base.
    geometry_msgs::Pose target_pose3 = waypoint_pose;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.x -= PATH_SIDE_LENGTH/2;
    waypoints.push_back(target_pose3);

    const double STEP_SIZE = 0.15;
    double height_traveled = 0;

    while (height_traveled < 0.3) 
    {
        geometry_msgs::Pose nextpose = waypoints.back();

        nextpose.position.z -= STEP_SIZE;
        waypoints.push_back(nextpose);

        if (waypoints.back().position.x < 0) {
            nextpose.position.x += PATH_SIDE_LENGTH/2;
        } else {
            nextpose.position.x -= PATH_SIDE_LENGTH/2;
        }

        waypoints.push_back(nextpose);


        height_traveled += STEP_SIZE;
    }

    move_group.setMaxVelocityScalingFactor(0.001);

    // Some planning parameters
    const int PLANNING_ATTEMPTS = 50;
    const int PLANNING_TIME = 10;
    double fraction_planned = -1;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    moveit_msgs::RobotTrajectory cart_trajectory;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        move_group.setStartState(*current_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            cart_trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire cartesian trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(cart_trajectory, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to start the trajectory.");
    
    move_group.execute(cart_trajectory);

    current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(1.0);


    visual_tools.prompt("Press 'next' to move to next wall.");

    // Orientation
    waypoint_pose.orientation.w = 0.707;
    waypoint_pose.orientation.x = 0;
    waypoint_pose.orientation.y = -0.707;
    waypoint_pose.orientation.z = 0;

    // Position 
    waypoint_pose.position.x = -0.55;
    waypoint_pose.position.y = -PATH_SIDE_LENGTH/2;
    waypoint_pose.position.z = PATH_SIDE_LENGTH/2;

    move_group.setStartState(*current_state);
    move_group.setPoseTarget(waypoint_pose);

    // Initialize plan

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move to the starting position.");
    move_group.execute(plan);

    // update current_state with the current position
    current_state = move_group.getCurrentState();

    visual_tools.prompt("Press 'next' to plan cartesian path and visualize");

    // ################### Plan the wall cleaning trajectory ###################
    waypoints.clear();
    move_group.setStartState(*current_state);

    // TODO: Make this relative to the origin in the wall, not the base.
    target_pose3 = waypoint_pose;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.y += PATH_SIDE_LENGTH;
    waypoints.push_back(target_pose3);

    height_traveled = 0;
    while (height_traveled < 0.3) 
    {
        geometry_msgs::Pose nextpose = waypoints.back();

        nextpose.position.z -= STEP_SIZE;
        waypoints.push_back(nextpose);

        if (waypoints.back().position.y < 0) {
            nextpose.position.y += PATH_SIDE_LENGTH;
        } else {
            nextpose.position.y -= PATH_SIDE_LENGTH;
        }

        waypoints.push_back(nextpose);


        height_traveled += STEP_SIZE;
    }

    move_group.setMaxVelocityScalingFactor(0.001);

    fraction_planned = -1;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        move_group.setStartState(*current_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            cart_trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire cartesian trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(cart_trajectory, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to start the trajectory.");
    
    move_group.execute(cart_trajectory);

    current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(1.0);


    visual_tools.prompt("Press 'next' to move to next wall.");

    // Orientation
    waypoint_pose.orientation.w = 0.707;
    waypoint_pose.orientation.x = -0.707;
    waypoint_pose.orientation.y = 0;
    waypoint_pose.orientation.z = 0;

    // Position 
    waypoint_pose.position.x = -PATH_SIDE_LENGTH/2;
    waypoint_pose.position.y = 0.55;
    waypoint_pose.position.z = PATH_SIDE_LENGTH/2;

    move_group.setStartState(*current_state);
    move_group.setPoseTarget(waypoint_pose);

    // Initialize plan

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move to the starting position.");
    move_group.execute(plan);

    // update current_state with the current position
    current_state = move_group.getCurrentState();

    visual_tools.prompt("Press 'next' to plan cartesian path and visualize");

    // ################### Plan the wall cleaning trajectory ###################
    waypoints.clear();
    move_group.setStartState(*current_state);

    // TODO: Make this relative to the origin in the wall, not the base.
    target_pose3 = waypoint_pose;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.x += PATH_SIDE_LENGTH;
    waypoints.push_back(target_pose3);

    height_traveled = 0;
    while (height_traveled < 0.3) 
    {
        geometry_msgs::Pose nextpose = waypoints.back();

        nextpose.position.z -= STEP_SIZE;
        waypoints.push_back(nextpose);

        if (waypoints.back().position.x < 0) {
            nextpose.position.x += PATH_SIDE_LENGTH;
        } else {
            nextpose.position.x -= PATH_SIDE_LENGTH;
        }

        waypoints.push_back(nextpose);


        height_traveled += STEP_SIZE;
    }

    move_group.setMaxVelocityScalingFactor(0.001);

    fraction_planned = -1;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        move_group.setStartState(*current_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            cart_trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire cartesian trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(cart_trajectory, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to start the trajectory.");
    
    move_group.execute(cart_trajectory);

    current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(1.0);


    visual_tools.prompt("Press 'next' to move to next wall.");

    // Orientation
    waypoint_pose.orientation.w = 0.707;
    waypoint_pose.orientation.x = 0;
    waypoint_pose.orientation.y = 0.707;
    waypoint_pose.orientation.z = 0;

    // Position 
    waypoint_pose.position.x = 0.55;
    waypoint_pose.position.y = PATH_SIDE_LENGTH/2;
    waypoint_pose.position.z = PATH_SIDE_LENGTH/2;

    move_group.setStartState(*current_state);
    move_group.setPoseTarget(waypoint_pose);

    // Initialize plan

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move to the starting position.");
    move_group.execute(plan);

    // update current_state with the current position
    current_state = move_group.getCurrentState();

    visual_tools.prompt("Press 'next' to plan cartesian path and visualize");

    // ################### Plan the wall cleaning trajectory ###################
    waypoints.clear();
    move_group.setStartState(*current_state);

    // TODO: Make this relative to the origin in the wall, not the base.
    target_pose3 = waypoint_pose;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.y -= PATH_SIDE_LENGTH;
    waypoints.push_back(target_pose3);

    height_traveled = 0;
    while (height_traveled < 0.3) 
    {
        geometry_msgs::Pose nextpose = waypoints.back();

        nextpose.position.z -= STEP_SIZE;
        waypoints.push_back(nextpose);

        if (waypoints.back().position.y < 0) {
            nextpose.position.y += PATH_SIDE_LENGTH;
        } else {
            nextpose.position.y -= PATH_SIDE_LENGTH;
        }

        waypoints.push_back(nextpose);


        height_traveled += STEP_SIZE;
    }

    move_group.setMaxVelocityScalingFactor(0.001);

    fraction_planned = -1;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        move_group.setStartState(*current_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            cart_trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire cartesian trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(cart_trajectory, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to start the trajectory.");
    
    move_group.execute(cart_trajectory);

    current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(1.0);


    visual_tools.prompt("Press 'next' to move to next wall.");

    // Orientation
    waypoint_pose.orientation.w = 0.707;
    waypoint_pose.orientation.x = 0.707;
    waypoint_pose.orientation.y = 0;
    waypoint_pose.orientation.z = 0;

    // Position 
    waypoint_pose.position.x = PATH_SIDE_LENGTH/2;
    waypoint_pose.position.y = -0.55;
    waypoint_pose.position.z = PATH_SIDE_LENGTH/2;

    move_group.setStartState(*current_state);
    move_group.setPoseTarget(waypoint_pose);

    // Initialize plan

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move to the starting position.");
    move_group.execute(plan);

    // update current_state with the current position
    current_state = move_group.getCurrentState();

    visual_tools.prompt("Press 'next' to plan cartesian path and visualize");

    // ################### Plan the wall cleaning trajectory ###################
    waypoints.clear();
    move_group.setStartState(*current_state);

    // TODO: Make this relative to the origin in the wall, not the base.
    target_pose3 = waypoint_pose;
    waypoints.push_back(target_pose3);
    
    target_pose3.position.x -= (PATH_SIDE_LENGTH/2 + 0.02);
    waypoints.push_back(target_pose3);

    height_traveled = 0;
    while (height_traveled < 0.3) 
    {
        geometry_msgs::Pose nextpose = waypoints.back();

        nextpose.position.z -= STEP_SIZE;
        waypoints.push_back(nextpose);

        if (waypoints.back().position.x < 0) {
            nextpose.position.x += (PATH_SIDE_LENGTH/2 + 0.02);
        } else {
            nextpose.position.x -= (PATH_SIDE_LENGTH/2 + 0.02);
        }

        waypoints.push_back(nextpose);


        height_traveled += STEP_SIZE;
    }

    move_group.setMaxVelocityScalingFactor(0.001);

    fraction_planned = -1;

    for (int i = 0; i < PLANNING_ATTEMPTS; i++)
    {
        move_group.setStartState(*current_state);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        double fraction_planned_tmp = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_tmp);

        // If we plan a better trajectory, take that instead.
        if (fraction_planned_tmp > fraction_planned)
        {
            fraction_planned = fraction_planned_tmp;
            cart_trajectory = trajectory_tmp;
        }

        // std::cout << fraction_planned << std::endl;

        // If we've planned the entire trajectory, no need to make further attempts.
        if (fraction_planned >= 0.999)
        {
            break;
        }

    }

    ROS_INFO("Planned %lf %% of the entire cartesian trajectory", fraction_planned * 100);

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(cart_trajectory, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to start the trajectory.");
    
    move_group.execute(cart_trajectory);

    current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(1.0);


    visual_tools.prompt("Press 'next' to move to next wall.");

}