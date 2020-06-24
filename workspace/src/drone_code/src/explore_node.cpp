/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "coordination.h"
#include <nav_msgs/Path.h>
#include "drone_code/coord.h"
#define UNKNOWN -1
#define FREE 0
#define FRONTIER_REGION 100
#define FRONTIER_EDGE 10
#define MAX_GOALS 6
#define MINIMUM_FRONTIER_EDGE 4
#define RESOLUTION 0.5
#define MAX_HEIGHT 16
#define MIN_HEIGHT -16
#define MAX_WIDTH 16
#define MIN_WIDTH -16

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;
nav_msgs::OccupancyGrid gridmap;
nav_msgs::OccupancyGrid frontierMap;
geometry_msgs::PoseArray receivedFrontiers;
bool gridMapInit = false;
std::vector<geometry_msgs::PoseStamped> frontierRegions;
std::vector<geometry_msgs::PoseStamped> inaccFrontierRegions;
std::ofstream distFile;
double max_height = MAX_HEIGHT;
double min_height = MIN_HEIGHT;
double max_width = MAX_WIDTH;
double min_width = MIN_WIDTH;
int ID;
std::string frameId;
std::string fileName;
tf::TransformListener* pListener = NULL;
nav_msgs::Path actualPath;
bool receivedFrontiersFlag = false;

void zeroPad();
void frontierEdgeDetection();
void frontierRegionExtraction();
int frontierDecision();
double pathLength(const nav_msgs::Path&);

double pathLength(const nav_msgs::Path& path) {
  
  double total_dist = 0.0;
  
  for (int i = 1; i < path.poses.size(); ++i) {
    double dx,dy,dz;
	double distance;
	
	dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
	dy = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.y;
	dz = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.z;
	distance = std::sqrt((dx*dx) + (dy*dy) + (dz*dz));
	total_dist += distance;
  }
  
  return total_dist;
}

void frontierEdgeDetection(){
	int frontierDetection = 0;
	for(int c = 1; c<gridmap.info.height-1; c++){
		for(int r = 1; r <gridmap.info.width-1; r++){
			//Check every free space in the gridmap
			if(gridmap.data[(c*gridmap.info.width)+r] == FREE){
				//Frontier edge detection, check neighbors
				frontierDetection = gridmap.data[(c*gridmap.info.width)+(r+1)] + gridmap.data[((c+1)*gridmap.info.width)+(r)] + gridmap.data[((c-1)*gridmap.info.width)+(r)]
				+ gridmap.data[(c*gridmap.info.width)+(r-1)];
				if(frontierDetection % 100 != 0){
					//Map frontier edge and correct zero pad
					frontierMap.data[((c)*frontierMap.info.width)+(r)] = FRONTIER_EDGE;
				}
			}
		}							
	}
}




void frontierRegionExtraction(){
	int frontierEdges = 0;	
	frontierRegions.clear();
	//Check every cell in the frontier map except cells marked as frontier region
	for(int c = 1; c<frontierMap.info.height-1; c++){					
		for(int r = 1; r <frontierMap.info.width-1; r++){
			if(frontierMap.data[(c*frontierMap.info.width)+r] != FRONTIER_REGION){
				frontierEdges = 0;
				for(int x = -1; x < 2; x++){
					for(int y = -1; y < 2; y++){
						//Count the number of frontier edges around the cell, excluding the cell itself. 3x3 matrix
						if(frontierMap.data[((c+x)*frontierMap.info.width)+(r+y)] == FRONTIER_EDGE && (x != 0 || y != 0)){
							frontierEdges++;										
						}
					}
				}					
				if(frontierEdges >= 4){
					for(int x = -1; x < 2; x++){
						for(int y = -1; y < 2; y++){
							frontierMap.data[((c+x)*frontierMap.info.width)+(r+y)] = FRONTIER_REGION;
						}
						
					}
					if(((((c*RESOLUTION) + frontierMap.info.origin.position.y) <= max_height) && (((c*RESOLUTION) + frontierMap.info.origin.position.y) >= min_height)) && ((((r*RESOLUTION) + frontierMap.info.origin.position.x) <= max_width) && (((r*RESOLUTION) + frontierMap.info.origin.position.x) >= min_width))){
						geometry_msgs::PoseStamped frontier;
						frontier.pose.position.y = (c*RESOLUTION) + frontierMap.info.origin.position.y;
						frontier.pose.position.x = (r*RESOLUTION) + frontierMap.info.origin.position.x;
						frontier.header.frame_id = frontierMap.header.frame_id;
						frontierRegions.push_back(frontier);
					}
				}
			}
		}		
	}
	ROS_INFO("%s: Detected %lu frontier region(s)",std::to_string(ID).c_str(),frontierRegions.size());	
}

int frontierDecision(){
	double dx = 0;
	double dy = 0;
	std::vector<double> dist;
	std::vector<int> indexList;
	int chosenIndex;
	
	//Transform current position frame
	geometry_msgs::PoseStamped curr_pos_new;
	try{
       	pListener->transformPose(frontierMap.header.frame_id,curr_pos,curr_pos_new);

    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a pose : %s", ex.what());
    }	
	
	//Calculate distances
	for(int i = 0; i < frontierRegions.size(); i++){	
		double distance;
		dx = frontierRegions[i].pose.position.x - curr_pos_new.pose.position.x;
		dy = frontierRegions[i].pose.position.y - curr_pos_new.pose.position.y;
		distance = std::sqrt(dx*dx + dy*dy);
		dist.push_back(distance);
	
	}
	//Sort by lowest
	std::vector<double> tempDist;
	for (int i=0; i<dist.size(); i++){ 
        tempDist.push_back(dist[i]);		
	}
	std::sort(dist.begin(),dist.end());
	//Make index list of distances of each frontier region before the sorting
	for(int i = 0; i<dist.size(); i++){
		for(int x = 0; x<tempDist.size();x++){
			if(dist[i] == tempDist[x]){
				indexList.push_back(x);
			}
		}
	}
	
	//Find frontier region based on distance and return corresponding index of the region 
	for(int i = 0; i<indexList.size();i++){
		if(ID == 0){
			if(!inaccFrontierRegions.empty()){
				bool inList = false;
				for(int j = 0; j < inaccFrontierRegions.size(); j++){
					if(frontierRegions[indexList[i]].pose.position.x == inaccFrontierRegions[j].pose.position.x && frontierRegions[indexList[i]].pose.position.y == inaccFrontierRegions[j].pose.position.y){
						inList = true;
					}
					
				}
				if(!inList){
					chosenIndex = indexList[i];
					break;
				}
			}
			else if(inaccFrontierRegions.empty()){
				chosenIndex = indexList[i];
				break;
			}
		}
		else{
			if(!inaccFrontierRegions.empty()){
				bool inList = false;
				for(int j = 0; j < inaccFrontierRegions.size(); j++){
					if(frontierRegions[indexList[i]].pose.position.x == inaccFrontierRegions[j].pose.position.x && frontierRegions[indexList[i]].pose.position.y == inaccFrontierRegions[j].pose.position.y){
						inList = true;
					}
					
				}
				if(!inList){
					std::vector<double> distance;
					double dx; 
					double dy;
					int distanceCounter = 0;
					//Check if frontier is farther than lower ID UAVs
					for(int j = 0; j < ID; j++){
						dx = frontierRegions[indexList[i]].pose.position.x - receivedFrontiers.poses[j].position.x;
						dy = frontierRegions[indexList[i]].pose.position.y - receivedFrontiers.poses[j].position.y;
						distance.push_back(std::sqrt(dx*dx + dy*dy));
						if(distance[j] >= frontierDistance){
						distanceCounter++;
						}
					}
				
					if(distanceCounter == ID){
						chosenIndex = indexList[i];
						break;
					}
					else if(i == indexList.size()-1){
						chosenIndex = indexList[0];
						break;
					}
				}
			}
			else if(inaccFrontierRegions.empty()){
				std::vector<double> distance;
				double dx; 
				double dy;
				int distanceCounter = 0;
				//Check if frontier is farther than lower ID UAVs
				for(int j = 0; j < ID; j++){
					dx = frontierRegions[indexList[i]].pose.position.x - receivedFrontiers.poses[j].position.x;
					dy = frontierRegions[indexList[i]].pose.position.y - receivedFrontiers.poses[j].position.y;
					distance.push_back(std::sqrt(dx*dx + dy*dy));
					if(distance[j] >= frontierDistance){
						distanceCounter++;
					}
				}
				
				if(distanceCounter == ID){
					chosenIndex = indexList[i];
					break;
				}
				else if(i == indexList.size()-1){
					chosenIndex = indexList[0];
					break;
				}
			}
			
		}
	}
	

	ROS_INFO("INDEXLIST: %d",chosenIndex);
	return chosenIndex;
}

void zeroPad()
{
	nav_msgs::OccupancyGrid tempGridmap;
	tempGridmap.info = gridmap.info;
	tempGridmap.data.resize(tempGridmap.info.height * tempGridmap.info.width,-1);
	for(int i = 0; i<gridmap.info.width*gridmap.info.height-1; i++){
	tempGridmap.data[i] = gridmap.data[i];

	}
	
	gridmap.info.origin.position.x = gridmap.info.origin.position.x - 0.5;
	gridmap.info.origin.position.y = gridmap.info.origin.position.y - 0.5;
	
	gridmap.info.width = gridmap.info.width + 1;
	gridmap.info.height = gridmap.info.height + 1;
	
	
	gridmap.data.clear();
	gridmap.data.resize(gridmap.info.height* gridmap.info.width,-1);
	
		//Grid map is like a matrix. Column = height and row = width relative to y-position direction in Rviz.
	for(int c = 0; c<tempGridmap.info.height; c++){
		for(int r = 0; r <tempGridmap.info.width;r++){
			//Copy old map to the new one.
			if(tempGridmap.data[(c*tempGridmap.info.width)+r] != UNKNOWN)
				gridmap.data[((c+1)*gridmap.info.width)+(r+1)] = tempGridmap.data[(c*tempGridmap.info.width)+r];
			
		}		
	}
	
}



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void positionCallback(const geometry_msgs::PoseStamped& msg) {
curr_pos = msg;
}

void projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	gridmap.info = msg->info;
	gridmap.header = msg->header;
	gridmap.data = msg->data;
	gridMapInit = true;
}

void CoordStateCallback(const std_msgs::Int8::ConstPtr& msg){
coordination_state = msg->data;
}

void actualPathCallback(const nav_msgs::Path::ConstPtr& msg){
actualPath.poses = msg->poses;
actualPath.header = msg->header;
}

void logDistanceCallback(const std_msgs::Bool::ConstPtr& msg){
	distFile.open(fileName,std::ios::app);
	distFile << pathLength(actualPath) << ",\n";
	distFile.close();
}

void frontierCallback(const geometry_msgs::PoseArray::ConstPtr& msg){

	for(int i = 0; i < ID; i++){
		receivedFrontiers.poses.push_back(msg->poses[i]);
	}
	receivedFrontiersFlag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explore_node");
    ros::NodeHandle nh;
	
	//Read params
	double offsetStartX;
	double offsetStartY;
	double offY, offX;
	nh.param<double>("explore_node/start_pos_x", offsetStartX,0.0);
  	nh.param<double>("explore_node/start_pos_y", offsetStartY,0.0);
	nh.param<int>("explore_node/ID", ID,0);
	nh.param<double>("explore_node/homeY", offY,0);
	nh.param<double>("explore_node/homeX", offX,0);
	nh.param<std::string>("explore_node/frame_id", frameId,"local_origin");
	fileName = "/home/simon/results/dist" + std::to_string(ID) + ".txt";
	
	max_height = max_height - offsetStartY;
	min_height = min_height - offsetStartY;
	
	max_width = max_width - offsetStartX;
	min_width = min_width - offsetStartX;
	
	pListener = new(tf::TransformListener);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("move_base_simple/goal", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos = nh.subscribe("mavros/local_position/pose", 1,positionCallback);
	
	ros::Subscriber projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("projected_map", 1, projectedMapCallback);
			
	ros::Publisher frontierMap_pub = nh.advertise<nav_msgs::OccupancyGrid>
        ("Frontier_map", 1);
	ros::Subscriber coord_state_sub = nh.subscribe<std_msgs::Int8>
        ("coordination_state", 1, CoordStateCallback);
	
	ros::Publisher exploration_comp_pub = nh.advertise<std_msgs::Bool>("exploration_complete",1);
		
	ros::Subscriber actualPath_sub = nh.subscribe<nav_msgs::Path>
        ("actual_path", 1, actualPathCallback);
		
	ros::Subscriber logDist_sub = nh.subscribe<std_msgs::Bool>("logDistance",1,logDistanceCallback);		

	ros::Publisher frontierGoal_pub = nh.advertise<drone_code::coord>("frontierGoal",1);

	ros::Subscriber receivedFrontierGoal_sub = nh.subscribe<geometry_msgs::PoseArray>("receivedFrontierGoal",1,frontierCallback);	


    ros::Rate rate(10);
	ros::Rate rateGoal(5);
	tf::TransformListener listener;
	
	home.pose.position.x = -offX;
	home.pose.position.y = -offY;
	home.header.frame_id = frameId;
	
	bool navigating = false;
	bool takeOff = false;
	int frontierIndex;
	bool distanceLogged = false;


    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	
	
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	int counter = 0;
	ros::Time nav_start;
	ros::Time nav_end;
	int region_retry = 1;
	geometry_msgs::PoseStamped frontierGoal;
	double start;
	double end;
	while(ros::Time::now().toSec() == 0){}
	start = ros::Time::now().toSec();
	
	if(ID == 0){
		receivedFrontiersFlag = true;
	}
	
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
	
		switch(coordination_state){
			case EXPLORE:{
				if(gridMapInit && current_state.mode == "OFFBOARD" && current_state.armed && !navigating && receivedFrontiersFlag){
					
					if(gridmap.data.size() > 1)
					{
						//Find frontier region and select closest one.
						zeroPad();
						frontierMap.info = gridmap.info; 
						frontierMap.header = gridmap.header;
						frontierMap.data.clear();
						frontierMap.data.resize(frontierMap.info.height * frontierMap.info.width,UNKNOWN);
						frontierEdgeDetection();
						frontierRegionExtraction();
						if(!frontierRegions.empty())
							frontierIndex = frontierDecision();
					
						else{
							//No frontier region, go home
							frontierRegions.push_back(home);
							frontierIndex = 0;
							std_msgs::Bool home_msg;
							home_msg.data = false;
							exploration_comp_pub.publish(home_msg);

						}
					}		
					frontierMap_pub.publish(frontierMap);
					navigating = true;
					nav_start = ros::Time::now();
					ROS_INFO("%s: Navigating to (%f, %f)",std::to_string(ID).c_str(), frontierRegions[frontierIndex].pose.position.x, frontierRegions[frontierIndex].pose.position.y);
					frontierRegions[frontierIndex].pose.orientation.w = -1;
					try{
       					pListener->transformPose(frameId,frontierRegions[frontierIndex],frontierGoal);

    				}
    				catch(tf::TransformException& ex){
        				ROS_ERROR("Received an exception trying to transform a pose : %s", ex.what());
    				}
					local_pos_pub.publish(frontierGoal);
					
				}
				nav_start = ros::Time::now();
				while(navigating && coordination_state != MEET && coordination_state != HOME){

		
   
   					//Publish frontier waypoint
					local_pos_pub.publish(frontierGoal);
					
					//Publish frontier goal for coordination
					drone_code::coord frontierCoord;
					frontierCoord.ID = ID;
					frontierCoord.frontierLocation = frontierRegions[frontierIndex];
					frontierGoal_pub.publish(frontierCoord);
					if((frontierGoal.pose.position.x > curr_pos.pose.position.x - 1 && frontierGoal.pose.position.x < curr_pos.pose.position.x + 1) && (frontierGoal.pose.position.y > curr_pos.pose.position.y - 1 && frontierGoal.pose.position.y < curr_pos.pose.position.y + 1)){
						navigating = false;
						ROS_INFO("%s: Done navigating to frontier",std::to_string(ID).c_str());
					}
					nav_end = ros::Time::now();
					if(nav_end - nav_start > ros::Duration(20.0)){
						navigating = false;
						inaccFrontierRegions.push_back(frontierRegions[frontierIndex]);
						ROS_INFO("%s: Navigation timeout, frontier inaccessible",std::to_string(ID).c_str());
					}	
			
					ros::spinOnce();
					rateGoal.sleep();
				}
			break;
			}
			
			case MEET:
				local_pos_pub.publish(home);
				break;
			case MEETING:
				
				local_pos_pub.publish(home);
				break;
			
			case HOME:
				local_pos_pub.publish(home);
				//Publish frontier goal for coordination
				drone_code::coord frontierCoord;
				frontierCoord.ID = ID;
				frontierCoord.frontierLocation = frontierRegions[frontierIndex];
				frontierGoal_pub.publish(frontierCoord);
				if(!distanceLogged){
					distFile.open(fileName,std::ios::app);
					distFile << pathLength(actualPath) << ",\n";
					distFile.close();
					distanceLogged = true;
					ROS_INFO("Logging distance at home");
				}
				break;
				
		}
		ros::spinOnce();
		rate.sleep();
		end = ros::Time::now().toSec();
		if(end - start > ros::Duration(120.0).toSec()){
			std_msgs::Bool home_msg;
			home_msg.data = false;
			exploration_comp_pub.publish(home_msg);
			
		}
    }

    return 0;
}
