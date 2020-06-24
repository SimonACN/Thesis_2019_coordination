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

void explorationCallback(const std_msgs::Bool::ConstPtr& msg) {
if(!msg->data){
	coordination_state = HOME;
	ROS_INFO("CHANGING TO HOME");
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordination_node");
    ros::NodeHandle nh;

	ros::Subscriber local_pos = nh.subscribe("mavros/local_position/pose", 1,positionCallback);
	ros::Publisher coord_state_pub = nh.advertise<std_msgs::Int8>
            ("coordination_state", 1);
	ros::Subscriber exploration_comp = nh.subscribe<std_msgs::Bool>("exploration_complete",1,explorationCallback);

	ros::Rate rate(10);
    
	double offY, offX;
	nh.param<double>("explore_node/homeY", offY,0);
	nh.param<double>("explore_node/homeX", offX,0);
	home.pose.position.x = -offX;
	home.pose.position.y = -offY;
	
    while(ros::ok()){
    	ros::Time begin;
		ros::Time end;
	   switch(coordination_state){
			case EXPLORE:{
				while((0 > curr_pos.pose.position.x - 1 && 0 < curr_pos.pose.position.x + 1) && (0 > curr_pos.pose.position.y - 1 && 0 < curr_pos.pose.position.y + 1)){
					ros::spinOnce();
					rate.sleep();
				}
				
				begin = ros::Time::now();
				end = ros::Time::now();
				while(end - begin < ros::Duration(30.0) && coordination_state == EXPLORE){
					end = ros::Time::now();
					ros::spinOnce();
        			rate.sleep();
				}
				if(coordination_state == EXPLORE){
					coordination_state = MEET;
					std_msgs::Int8 coord_state_msg;
					coord_state_msg.data = coordination_state;
					int counter = 0;
					while(counter < 3){
						counter++;
						coord_state_pub.publish(coord_state_msg);
						ros::spinOnce();
        				rate.sleep();
					}
					coord_state_pub.publish(coord_state_msg);
					ROS_INFO("CHANGING TO MEET");
				}
				break;
			}
			case MEET:{
				if((home.pose.position.x > curr_pos.pose.position.x - 1 && home.pose.position.x < curr_pos.pose.position.x + 1) && (home.pose.position.y > curr_pos.pose.position.y - 1 && home.pose.position.y < curr_pos.pose.position.y + 1)){
					coordination_state = MEETING;
					std_msgs::Int8 coord_state_msg;
					coord_state_msg.data = coordination_state;
					coord_state_pub.publish(coord_state_msg);
				}
				break;
			}
			case MEETING: {
				ros::Time begin = ros::Time::now();
				ros::Time end = ros::Time::now();
				while(end - begin < ros::Duration(10.0))
				{
					end = ros::Time::now();
					ros::spinOnce();
					rate.sleep();
				}
			
				coordination_state = EXPLORE;
				std_msgs::Int8 coord_state_msg;
				coord_state_msg.data = coordination_state;
				coord_state_pub.publish(coord_state_msg);
				ROS_INFO("CHANGING TO EXPLORE");
				break;
			}
			
			case HOME:
				std_msgs::Int8 coord_state_msg;
				coord_state_msg.data = coordination_state;
				coord_state_pub.publish(coord_state_msg);
				
				break;
				
	   }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
