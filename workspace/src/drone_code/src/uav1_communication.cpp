

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include "drone_code/coord.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>



octomap_msgs::Octomap received_map;
nav_msgs::OccupancyGrid received_gridmap;
drone_code::coord receivedFrontier;
geometry_msgs::PoseArray frontiers;
bool receivedFrontiersFlag = false;

void ReceivedMapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
received_map = *msg;
}

void ReceivedProjectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
received_gridmap.info = msg->info;
received_gridmap.header = msg->header;
received_gridmap.data = msg->data;
}

void ReceivedFrontierCallback(const drone_code::coord::ConstPtr& msg) {
	receivedFrontier.ID = msg->ID;
	//receivedFrontier.frontierLocation = msg->frontierLocation;
	frontiers.poses[msg->ID] = msg->frontierLocation.pose;
	receivedFrontiersFlag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav1_communication");
    ros::NodeHandle nh;

	ros::Subscriber receivedMap_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav0/octomap_full", 1, ReceivedMapCallback);
	ros::Subscriber receivedMap2_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav2/octomap_full", 1, ReceivedMapCallback);
	ros::Subscriber receivedMap3_sub = nh.subscribe<octomap_msgs::Octomap>
	("/uav3/octomap_full", 1, ReceivedMapCallback);
	ros::Publisher receivedMap_pub = nh.advertise<octomap_msgs::Octomap>
	("received_map", 1);
	ros::Subscriber receivedProjectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav0/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Subscriber receivedProjectedMap1_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav2/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Subscriber receivedProjectedMap2_sub = nh.subscribe<nav_msgs::OccupancyGrid>
    ("/uav3/projected_map", 1, ReceivedProjectedMapCallback);
	ros::Publisher receivedProjectedMap_pub = nh.advertise<nav_msgs::OccupancyGrid>
	("received_projected_map", 1);
	ros::Subscriber receivedFrontiers_sub = nh.subscribe<drone_code::coord>
    ("/uav0/frontierGoal", 1, ReceivedFrontierCallback);
	ros::Publisher receivedFrontiers_pub = nh.advertise<geometry_msgs::PoseArray>
	("receivedFrontierGoal", 1);


	for(int i = 0; i < 1; i++){
		geometry_msgs::PoseStamped init;
		frontiers.poses.push_back(init.pose);
	}


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1);


    while(ros::ok()){
       
        receivedMap_pub.publish(received_map);
		receivedProjectedMap_pub.publish(received_gridmap);
		if(receivedFrontiersFlag){
			receivedFrontiers_pub.publish(frontiers);
		}
		ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
