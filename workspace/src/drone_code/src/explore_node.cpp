/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#define MAX_GOALS 6

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void positionCallback(const geometry_msgs::PoseStamped& msg) {
curr_pos = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explore_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos = nh.subscribe("/mavros/local_position/pose", 1,positionCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	


	float x_goals[MAX_GOALS] = {0, 15, 15, -5, -5, 0};
	float y_goals[MAX_GOALS] = {-10, -10, 10, 10, -10, 0};
    geometry_msgs::PoseStamped pose[MAX_GOALS];
	for (int i = 0; i < MAX_GOALS; i++)
	{
		pose[i].pose.position.x = x_goals[i];
		pose[i].pose.position.y = y_goals[i];
	}
  
    //pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14);

    //send a few setpoints before starting
    /*for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
	*/
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	int counter = 0;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
	
		if((pose[counter].pose.position.x > curr_pos.pose.position.x - 1 && pose[counter].pose.position.x < curr_pos.pose.position.x + 1) && (pose[counter].pose.position.y > curr_pos.pose.position.y - 1 && pose[counter].pose.position.y < curr_pos.pose.position.y + 1) && counter < MAX_GOALS-1)
		{
			counter++;
			ROS_INFO("Next goal");
			
		}
        local_pos_pub.publish(pose[counter]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
