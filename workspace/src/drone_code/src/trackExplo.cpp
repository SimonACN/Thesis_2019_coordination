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
#include <fstream>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid uav0_gridmap;
nav_msgs::OccupancyGrid uav1_gridmap;
nav_msgs::OccupancyGrid uav2_gridmap;
nav_msgs::OccupancyGrid uav3_gridmap;
nav_msgs::OccupancyGrid merged_gridmap;
bool uav0gridmapInit = false;
bool uav1gridmapInit = false;
bool uav2gridmapInit = false;
bool uav3gridmapInit = false;
const float resolution = 0.5;

double calc_exploration_area(nav_msgs::OccupancyGrid&);
void MergeProjectedMap(nav_msgs::OccupancyGrid&, nav_msgs::OccupancyGrid&);


void MergeProjectedMap(nav_msgs::OccupancyGrid& gridmap1,nav_msgs::OccupancyGrid& gridmap2){
	float offsetWidth = 0, offsetHeight = 0, offsetWidthPM1 = 0, offsetHeightPM1 = 0;
	nav_msgs::OccupancyGrid tempGridmap;
	tempGridmap.info = gridmap1.info;
	tempGridmap.data.resize(tempGridmap.info.height * tempGridmap.info.width,-1);
	for(int i = 0; i<gridmap1.info.width*gridmap1.info.height-1; i++){
	tempGridmap.data[i] = gridmap1.data[i];

	}

	//Determine which origin is the lowest

	//Height = origin y-position
	if(gridmap1.info.origin.position.y < gridmap2.info.origin.position.y){
		offsetHeight = ceil((gridmap2.info.origin.position.y - gridmap1.info.origin.position.y)/resolution); //Offset of grid cells when merging the maps
		if((gridmap1.info.height + (gridmap1.info.origin.position.y/resolution)) <= (gridmap2.info.height + (gridmap2.info.origin.position.y/resolution))){
			int extend = gridmap1.info.height + (gridmap1.info.origin.position.y - gridmap2.info.origin.position.y)/resolution;
			if(extend < 0)
			{
				gridmap1.info.height = gridmap1.info.height + (std::abs(extend) + gridmap2.info.height);
			}
			else if(extend > 0)
			{
				gridmap1.info.height = gridmap1.info.height + gridmap2.info.height - extend;
			}
			
			else
			{
				gridmap1.info.height = gridmap1.info.height + gridmap2.info.height;
			}
			
		}
	}
	else if(gridmap1.info.origin.position.y > gridmap2.info.origin.position.y){
		offsetHeightPM1 = ceil((gridmap1.info.origin.position.y - gridmap2.info.origin.position.y)/resolution); //Offset of grid cells when merging the maps
		gridmap1.info.origin.position.y = gridmap2.info.origin.position.y;
		gridmap1.info.height = gridmap1.info.height + offsetHeightPM1; //Extend height by origin change
		
		
	}
	
	//Width = origin x-position
	if(gridmap1.info.origin.position.x < gridmap2.info.origin.position.x){
		offsetWidth = ceil((gridmap2.info.origin.position.x - gridmap1.info.origin.position.x)/resolution); //Offset of grid cells when merging the maps
		if((gridmap1.info.width + (gridmap1.info.origin.position.x/resolution)) <= (gridmap2.info.width + (gridmap2.info.origin.position.x/resolution))){
			int extend = gridmap1.info.width + (gridmap1.info.origin.position.x - gridmap2.info.origin.position.x)/resolution;
			if(extend < 0)
			{
				gridmap1.info.width = gridmap1.info.width + std::abs(extend) + gridmap2.info.width;
			}
			else if(extend > 0)
			{
				gridmap1.info.width = gridmap1.info.width + gridmap2.info.width - extend;
			}
			
			else
			{
				gridmap1.info.width = gridmap1.info.width + gridmap2.info.width;
			}
			
		}
	}
	else if(gridmap1.info.origin.position.x > gridmap2.info.origin.position.x){
		offsetWidthPM1 = ceil((gridmap1.info.origin.position.x - gridmap2.info.origin.position.x)/resolution); //Offset of grid cells when merging the maps
		gridmap1.info.origin.position.x = gridmap2.info.origin.position.x;
		gridmap1.info.width = gridmap1.info.width + offsetWidthPM1; //Extend width by origin change
		
	}

	
	gridmap1.data.clear();
	gridmap1.data.resize(gridmap1.info.height* gridmap1.info.width,-1);

	//Grid map is like a matrix. Column = height and row = width relative to y-position direction in Rviz.
	for(int c = 0; c<tempGridmap.info.height; c++){
		for(int r = 0; r <tempGridmap.info.width;r++){
			//Copy old map to the new one.
			if(tempGridmap.data[(c*tempGridmap.info.width)+r] != UNKNOWN)
				gridmap1.data[((c+offsetHeightPM1)*gridmap1.info.width)+(r+offsetWidthPM1)] = tempGridmap.data[(c*tempGridmap.info.width)+r];
			
		}		
	}


	
	
	for(int c = 0; c<gridmap2.info.height; c++){
		for(int r = 0; r <gridmap2.info.width;r++){
			//Add only new data to local map from received map
			if(gridmap1.data[((c+offsetHeight)*gridmap1.info.width)+(r+offsetWidth)] == UNKNOWN)
				gridmap1.data[((c+offsetHeight)*gridmap1.info.width)+(r+offsetWidth)] = gridmap2.data[(c*gridmap2.info.width)+r];
		}			
	}
	


}



double calc_exploration_area(nav_msgs::OccupancyGrid& Gridmap){
double exploredArea = 0;
for(int i = 0; i<Gridmap.info.height*Gridmap.info.width; i++){
	if (Gridmap.data[i] != -1){
		exploredArea++;
	}	
}

return ((exploredArea*(RESOLUTION*RESOLUTION))/total_area)*100;
}


void uav0projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	uav0_gridmap.info = msg->info;
	uav0_gridmap.header = msg->header;
	uav0_gridmap.data = msg->data;
	
	if(!uav0gridmapInit){
		merged_gridmap.info = msg->info;
		merged_gridmap.header = msg->header;
		merged_gridmap.data = msg->data;
		
		merged_gridmap.info.origin.position.x = -20;
		merged_gridmap.info.origin.position.y = -20;
		
		merged_gridmap.info.width = 80;		
		merged_gridmap.info.height = 80;

		merged_gridmap.data.clear();
		merged_gridmap.data.resize(merged_gridmap.info.height* merged_gridmap.info.width,-1);		
		
	}
	uav0gridmapInit = true;
}

void uav1projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	uav1_gridmap.info = msg->info;
	uav1_gridmap.header = msg->header;
	uav1_gridmap.data = msg->data;
	uav1gridmapInit = true;
}

void uav2projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	uav2_gridmap.info = msg->info;
	uav2_gridmap.header = msg->header;
	uav2_gridmap.data = msg->data;
	uav2gridmapInit = true;
}

void uav3projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	uav3_gridmap.info = msg->info;
	uav3_gridmap.header = msg->header;
	uav3_gridmap.data = msg->data;
	uav3gridmapInit = true;
}

void CoordStateCallback(const std_msgs::Int8::ConstPtr& msg){
coordination_state = msg->data;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_exploration");
    ros::NodeHandle nh;

	ros::Subscriber uav0projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("uav0/projected_map", 1, uav0projectedMapCallback);
	ros::Subscriber uav1projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("uav1/projected_map", 1, uav1projectedMapCallback);
	ros::Subscriber uav2projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
        ("uav2/projected_map", 1, uav2projectedMapCallback);
	ros::Subscriber uav3projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
        ("uav3/projected_map", 1, uav3projectedMapCallback);
			
	ros::Subscriber coord_state_sub = nh.subscribe<std_msgs::Int8>
        ("uav0/coordination_state", 1, CoordStateCallback);
	ros::Subscriber coord_state_sub2 = nh.subscribe<std_msgs::Int8>
        ("uav1/coordination_state", 1, CoordStateCallback);

	
	ros::Publisher coord_state_pub0 = nh.advertise<std_msgs::Int8>
            ("uav0/coordination_state", 1);
	ros::Publisher coord_state_pub1 = nh.advertise<std_msgs::Int8>
            ("uav1/coordination_state", 1);
	ros::Publisher coord_state_pub2 = nh.advertise<std_msgs::Int8>
            ("uav2/coordination_state", 1);
	ros::Publisher coord_state_pub3 = nh.advertise<std_msgs::Int8>
            ("uav3/coordination_state", 1);
			
	ros::Publisher logDist_pub0 = nh.advertise<std_msgs::Bool>
            ("uav0/logDistance", 1);
	ros::Publisher logDist_pub1 = nh.advertise<std_msgs::Bool>
            ("uav1/logDistance", 1);
	ros::Publisher logDist_pub2 = nh.advertise<std_msgs::Bool>
        ("uav2/logDistance", 1);
	ros::Publisher logDist_pub3 = nh.advertise<std_msgs::Bool>
        ("uav3/logDistance", 1);
		
		
	ros::Publisher mergedmap_pub = nh.advertise<nav_msgs::OccupancyGrid>
	("merged_map", 1);
	
	ros::Rate rate(1);
    std::ofstream explorationFile;
	std::ofstream timeFile;
	double exploration_coverage = 0;
	double start;
	bool time_started;
	bool distLogged = false;
	
	while(ros::Time::now().toSec() == 0){}
	start = ros::Time::now().toSec();
    while(ros::ok()){
		switch(coordination_state){
			case EXPLORE:
			case HOME:
			case MEET:
			case MEETING:
				if(uav0gridmapInit && uav1gridmapInit && uav2gridmapInit/* && uav3gridmapInit*/){
					if(!uav0_gridmap.data.empty() && !uav1_gridmap.data.empty() && !uav2_gridmap.data.empty()/* && !uav3_gridmap.data.empty()*/){
				
						//Calculate exploration individually
						exploration_coverage = calc_exploration_area(uav0_gridmap);
						explorationFile.open("/home/simon/results/uav0_exploration.txt",std::ios::app);
						explorationFile << exploration_coverage << ",\n";
						explorationFile.close();
						timeFile.open("/home/simon/results/uav0_time.txt",std::ios::app);
						timeFile << ros::Time::now().toSec() - start << ",\n";
						timeFile.close();
				
						exploration_coverage = calc_exploration_area(uav1_gridmap);
						explorationFile.open("/home/simon/results/uav1_exploration.txt",std::ios::app);
						explorationFile << exploration_coverage << ",\n";
						explorationFile.close();
						timeFile.open("/home/simon/results/uav1_time.txt",std::ios::app);
						timeFile << ros::Time::now().toSec() - start << ",\n";
						timeFile.close();
						
						
						exploration_coverage = calc_exploration_area(uav2_gridmap);
						explorationFile.open("/home/simon/results/uav2_exploration.txt",std::ios::app);
						explorationFile << exploration_coverage << ",\n";
						explorationFile.close();
						timeFile.open("/home/simon/results/uav2_time.txt",std::ios::app);
						timeFile << ros::Time::now().toSec() - start << ",\n";
						timeFile.close();
						/*
						exploration_coverage = calc_exploration_area(uav3_gridmap);
						explorationFile.open("/home/simon/results/uav3_exploration.txt",std::ios::app);
						explorationFile << exploration_coverage << ",\n";
						explorationFile.close();
						timeFile.open("/home/simon/results/uav3_time.txt",std::ios::app);
						timeFile << ros::Time::now().toSec() - start << ",\n";
						timeFile.close();
*/
				
						//Merge maps and calculate exploration coverage
						MergeProjectedMap(merged_gridmap, uav0_gridmap);
						MergeProjectedMap(merged_gridmap, uav1_gridmap);
						
						MergeProjectedMap(merged_gridmap, uav2_gridmap);/*
						MergeProjectedMap(merged_gridmap, uav3_gridmap);
				*/
						exploration_coverage = calc_exploration_area(merged_gridmap);
						explorationFile.open("/home/simon/results/total_exploration.txt",std::ios::app);
						explorationFile << exploration_coverage << ",\n";
						explorationFile.close();
						timeFile.open("/home/simon/results/merged_time.txt",std::ios::app);
						timeFile << ros::Time::now().toSec() - start << ",\n";
						timeFile.close();
				
						ROS_INFO("LOGGING TIME AND COVERAGE");
						ROS_INFO("Total exploration coverage: %.2f%%, Time: %f", exploration_coverage, ros::Time::now().toSec()-start);
						
						if(exploration_coverage > 75.0 && !distLogged){
							std_msgs::Bool log_msg;
							log_msg.data = true;
							logDist_pub0.publish(log_msg);
							logDist_pub1.publish(log_msg);
							logDist_pub2.publish(log_msg);
							logDist_pub3.publish(log_msg);
							ROS_INFO("LOGGING DISTANCE");
							distLogged = true;
						}
						
						if((ros::Time::now().toSec() - start) > ros::Duration(120.0).toSec()){
						std_msgs::Int8 coord_state_msg;
						coord_state_msg.data = HOME;
						coord_state_pub0.publish(coord_state_msg);
						coord_state_pub1.publish(coord_state_msg);
						coord_state_pub2.publish(coord_state_msg);
						coord_state_pub3.publish(coord_state_msg);
						ROS_INFO("GO HOME");
						}
						
							
					}

				}
				else{
					//Map starts at zero
					explorationFile.open("/home/simon/results/uav0_exploration.txt",std::ios::app);
					explorationFile << 0 << ",\n";
					explorationFile.close();
					timeFile.open("/home/simon/results/uav0_time.txt",std::ios::app);
					timeFile << ros::Time::now().toSec() - start << ",\n";
					timeFile.close();
				
					explorationFile.open("/home/simon/results/uav1_exploration.txt",std::ios::app);
					explorationFile << 0 << ",\n";
					explorationFile.close();
					timeFile.open("/home/simon/results/uav1_time.txt",std::ios::app);
					timeFile << ros::Time::now().toSec() - start << ",\n";
					timeFile.close();
					
					explorationFile.open("/home/simon/results/uav2_exploration.txt",std::ios::app);
					explorationFile << 0 << ",\n";
					explorationFile.close();
					timeFile.open("/home/simon/results/uav2_time.txt",std::ios::app);
					timeFile << ros::Time::now().toSec() - start << ",\n";
					timeFile.close();
					/*
					explorationFile.open("/home/simon/results/uav3_exploration.txt",std::ios::app);
					explorationFile << 0 << ",\n";
					explorationFile.close();
					timeFile.open("/home/simon/results/uav3_time.txt",std::ios::app);
					timeFile << ros::Time::now().toSec() - start << ",\n";
					timeFile.close();
				*/
					explorationFile.open("/home/simon/results/total_exploration.txt",std::ios::app);
					explorationFile << 0 << ",\n";
					explorationFile.close();
					timeFile.open("/home/simon/results/merged_time.txt",std::ios::app);
					timeFile << ros::Time::now().toSec() - start << ",\n";
					timeFile.close();
					}
					
			break;
			
		}
		mergedmap_pub.publish(merged_gridmap);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
