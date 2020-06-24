/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>



octomap_msgs::Octomap received_map;
nav_msgs::OccupancyGrid received_gridmap;

void ReceivedMapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
received_map = *msg;
}

void ReceivedProjectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
received_gridmap.info = msg->info;
received_gridmap.header = msg->header;
received_gridmap.data = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav1_communication");
    ros::NodeHandle nh;

	ros::Subscriber receivedMap_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav1/octomap_full", 1, ReceivedMapCallback);
	ros::Subscriber receivedMap2_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav2/octomap_full", 1, ReceivedMapCallback);
	ros::Subscriber receivedMap3_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav3/octomap_full", 1, ReceivedMapCallback);
	ros::Publisher receivedMap_pub = nh.advertise<octomap_msgs::Octomap>
	("received_map", 1);
	ros::Subscriber receivedProjectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav1/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Subscriber receivedProjectedMap2_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav2/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Subscriber receivedProjectedMap3_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav3/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Publisher receivedProjectedMap_pub = nh.advertise<nav_msgs::OccupancyGrid>
	("received_projected_map", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1);


    while(ros::ok()){
       
        receivedMap_pub.publish(received_map);
		ros::spinOnce();
		rate.sleep();
		receivedProjectedMap_pub.publish(received_gridmap);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
