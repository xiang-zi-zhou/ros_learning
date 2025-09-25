#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
 
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_circle_node");
    ros::NodeHandle nh;
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
 
    ros::Rate rate(20.0);
 
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    double radius = 5.0;
    double flight_altitude = 3.0;
    double angular_velocity = 0.5;
    geometry_msgs::PoseStamped pose;
 
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();
    ros::Time start_time;
    bool mission_started = false;
    
    geometry_msgs::Point takeoff_point;
    geometry_msgs::Point circle_center;
    bool initial_pos_set = false;
 
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) &&
               offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else {
            if(!current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
 
        if (current_state.mode == "OFFBOARD" && current_state.armed) {
            
            if (!initial_pos_set) {
                takeoff_point = current_pose.pose.position;
                circle_center = takeoff_point;
                circle_center.x -= radius;
                initial_pos_set = true;
            }
 
            if (!mission_started) {
                pose.pose.position.x = takeoff_point.x;
                pose.pose.position.y = takeoff_point.y;
                pose.pose.position.z = flight_altitude;
 
                double dist_to_start = std::sqrt(std::pow(current_pose.pose.position.x - takeoff_point.x, 2) +
                                                 std::pow(current_pose.pose.position.y - takeoff_point.y, 2) +
                                                 std::pow(current_pose.pose.position.z - flight_altitude, 2));
 
                if (dist_to_start < 0.2) {
                    start_time = ros::Time::now();
                    mission_started = true;
                }
            } else {
                double elapsed_time = (ros::Time::now() - start_time).toSec();
                double angle = angular_velocity * elapsed_time;
 
                pose.pose.position.x = circle_center.x + radius * cos(angle);
                pose.pose.position.y = circle_center.y + radius * sin(angle);
                pose.pose.position.z = flight_altitude;
 
                double yaw_angle = atan2(circle_center.y - pose.pose.position.y, circle_center.x - pose.pose.position.x);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw_angle);
                pose.pose.orientation = tf2::toMsg(q);
 
                if (angle >= 2 * M_PI) {
                    pose.pose.position.x = takeoff_point.x;
                    pose.pose.position.y = takeoff_point.y;
                    pose.pose.position.z = flight_altitude;
                }
            }
        } else {
            pose = current_pose;
        }
 
        local_pos_pub.publish(pose);
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}    