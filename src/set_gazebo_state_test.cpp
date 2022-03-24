#include "ros/ros.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_srv_client_test");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    gazebo_msgs::SetModelConfiguration srv;

// string model_name
// string urdf_param_name
// string[] joint_names
// float64[] joint_positions

// bool success
// string status_message


//   662  rosservice call /gazebo/set_model_configuration "model_name: 'robot'
//   663  urdf_param_name: 'robot_description'
//   664  joint_names:
//   665  - 'shoulder_lift_joint'
//   666  - 'elbow_joint'
//   667  joint_positions:
//   668  - -1.57
//   669  - 1.57

// <joint name="elbow_joint" value="0" />
// <joint name="shoulder_lift_joint" value="0" />
// <joint name="shoulder_pan_joint" value="0" />
// <joint name="wrist_1_joint" value="0" />
// <joint name="wrist_2_joint" value="0" />
// <joint name="wrist_3_joint" value="0" />

    srv.request.model_name = "robot";
    srv.request.urdf_param_name = "robot_description";
    srv.request.joint_names = {"shoulder_lift_joint"};
    srv.request.joint_positions = {-1.57};

    if (client.call(srv)) {
        ROS_INFO("Called gazebo service set_model_configuration.");
        ROS_INFO("Status: %d", (bool)srv.response.success);
    }
    else {
        ROS_ERROR("Failed to call gazebo service set_model_configuration.");
        return 1;
    }

    return 0;
}