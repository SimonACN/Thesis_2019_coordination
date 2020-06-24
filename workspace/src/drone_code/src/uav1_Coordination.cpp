/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include "coordination.h"

geometry_msgs::PoseStamped curr_pos;


void positionCallback(const geometry_msgs::PoseStamped& msg) {
curr_pos = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordination_node");
    ros::NodeHandle nh;

	ros::Subscriber local_pos = nh.subscribe("mavros/local_position/pose", 1,positionCallback);
	ros::Publisher coord_state_pub = nh.advertise<std_msgs::Int8>
            ("coordination_state", 1);
	
	ros::Rate rate(1);
    
    while(ros::ok()){
       
	    if((15 > curr_pos.pose.position.x - 1 && 15 < curr_pos.pose.position.x + 1) && (7 > curr_pos.pose.position.y - 1 && 7 < curr_pos.pose.position.y + 1)){
			coordination_state = MEET;
			std_msgs::Int8 coord_state_msg;
			coord_state_msg.data = coordination_state;
			coord_state_pub.publish(coord_state_msg);
			ROS_INFO("UAV1: CHANGING TO MEET");
		}
		
		if((0 > curr_pos.pose.position.x - 1 && 0 < curr_pos.pose.position.x + 1) && (0 > curr_pos.pose.position.y - 1 && 0 < curr_pos.pose.position.y + 1) && coordination_state != EXPLORE){
			ros::Time begin = ros::Time::now();
			ros::Time end = ros::Time::now();
			while(end - begin < ros::Duration(6.0))
			{end = ros::Time::now();
			 ros::spinOnce();
			 rate.sleep();
			}
			
			coordination_state = EXPLORE;
			std_msgs::Int8 coord_state_msg;
			coord_state_msg.data = coordination_state;
			coord_state_pub.publish(coord_state_msg);
			ROS_INFO("UAV1: CHANGING TO EXPLORE");
		}
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
