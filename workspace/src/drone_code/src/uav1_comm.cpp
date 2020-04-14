#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath> 
#define UNKNOWN -1



octomap::OcTree* octree_1;

octomap::OcTree* mergedMap;
visualization_msgs::MarkerArray occupied_cells;
ros::Subscriber projectedMap_sub;
ros::Subscriber occupiedcells_sub;
nav_msgs::OccupancyGrid mergedGridmap;

bool visualflag = false;
bool initFlag = false;
bool initPM = false;
const float resolution = 0.5;


void update3DMap();
void mergeMap(octomap::OcTree*);
void reset3DMap();
void initMergedMap();
void MergeProjectedMap(nav_msgs::OccupancyGrid);
void initPRM(nav_msgs::OccupancyGrid);


void initPRM(nav_msgs::OccupancyGrid initMap){
	mergedGridmap.info = initMap.info;
	mergedGridmap.header = initMap.header;
	mergedGridmap.data.resize(mergedGridmap.info.height* mergedGridmap.info.width,-1);
	for(int i = 0; i<initMap.info.width*initMap.info.height-1; i++){
	mergedGridmap.data[i] = initMap.data[i];
	}
	initPM = true;
}

//Merge projected maps
void MergeProjectedMap(nav_msgs::OccupancyGrid gridmap2){
	float offsetWidth = 0, offsetHeight = 0, offsetWidthPM1 = 0, offsetHeightPM1 = 0;
	nav_msgs::OccupancyGrid tempGridmap;
	tempGridmap.info = mergedGridmap.info;
	tempGridmap.data.resize(tempGridmap.info.height * tempGridmap.info.width,-1);
	for(int i = 0; i<mergedGridmap.info.width*mergedGridmap.info.height-1; i++){
	tempGridmap.data[i] = mergedGridmap.data[i];

	}

	//Determine which origin is the lowest

	//Height = origin y-position
	if(mergedGridmap.info.origin.position.y < gridmap2.info.origin.position.y){
		offsetHeight = ceil((gridmap2.info.origin.position.y - mergedGridmap.info.origin.position.y)/resolution); //Offset of grid cells when merging the maps
	}
	else if(mergedGridmap.info.origin.position.y > gridmap2.info.origin.position.y){
		offsetHeightPM1 = ceil((mergedGridmap.info.origin.position.y - gridmap2.info.origin.position.y)/resolution); //Offset of grid cells when merging the maps
		mergedGridmap.info.origin.position.y = gridmap2.info.origin.position.y;
		mergedGridmap.info.height = mergedGridmap.info.height + offsetHeightPM1; //Extend height by origin change
		
		
	}
	
	//Width = origin x-position
	if(mergedGridmap.info.origin.position.x < gridmap2.info.origin.position.x){
		offsetWidth = ceil((gridmap2.info.origin.position.x - mergedGridmap.info.origin.position.x)/resolution); //Offset of grid cells when merging the maps
	}
	else if(mergedGridmap.info.origin.position.x > gridmap2.info.origin.position.x){
		offsetWidthPM1 = ceil((mergedGridmap.info.origin.position.x - gridmap2.info.origin.position.x)/resolution); //Offset of grid cells when merging the maps
		mergedGridmap.info.origin.position.x = gridmap2.info.origin.position.x;
		mergedGridmap.info.width = mergedGridmap.info.width + offsetWidthPM1; //Extend width by origin change
		
	}

	
	mergedGridmap.data.clear();
	mergedGridmap.data.resize(mergedGridmap.info.height* mergedGridmap.info.width,-1);

	//Grid map is like a matrix. Column = height and row = width relative to y-position direction in Rviz.
	for(int c = 0; c<tempGridmap.info.height; c++){
		for(int r = 0; r <tempGridmap.info.width;r++){
			//Copy old map to the new one.
			if(tempGridmap.data[(c*tempGridmap.info.width)+r] != UNKNOWN)
				mergedGridmap.data[((c+offsetHeightPM1)*mergedGridmap.info.width)+(r+offsetWidthPM1)] = tempGridmap.data[(c*tempGridmap.info.width)+r];
			
		}		
	}


	
	
	for(int c = 0; c<gridmap2.info.height; c++){
		for(int r = 0; r <gridmap2.info.width;r++){
			//Add only new data to local map from received map
			if(mergedGridmap.data[((c+offsetHeight)*mergedGridmap.info.width)+(r+offsetWidth)] == UNKNOWN)
				mergedGridmap.data[((c+offsetHeight)*mergedGridmap.info.width)+(r+offsetWidth)] = gridmap2.data[(c*gridmap2.info.width)+r];
		}			
	}
	


}

//Initialize local merged map
void initMergedMap()
{

	
	mergedMap = new octomap::OcTree(octree_1->getResolution());
	mergedMap->setProbHit(octree_1->getProbHit());
	mergedMap->setProbMiss(octree_1->getProbMiss());
	mergedMap->setClampingThresMin(octree_1->getClampingThresMin());
	mergedMap->setClampingThresMax(octree_1->getClampingThresMax());
	initFlag = true;
		
}

void reset3DMap(){
occupied_cells.markers[mergedMap->getTreeDepth()].points.clear();
occupied_cells.markers[mergedMap->getTreeDepth()].action = visualization_msgs::Marker::DELETE;
}


void mergeMap(octomap::OcTree* map1)
{
	for (octomap::OcTree::leaf_iterator it = map1->begin_leafs(); it != map1->end_leafs(); ++it) {
	
	octomap::OcTreeKey nodeKey = it.getKey(); //Get coordinates
	octomap::OcTreeNode *node = mergedMap->search(nodeKey); //Search for node
	//If node exists
	if (node != NULL) {
		float logOddsProb = 0;
		//If both maps show node is occupied
		if(map1->isNodeOccupied(*it) && mergedMap->isNodeOccupied(node)){
			
		
		//If both maps show node is occupied, prioritize highest probability as that has more data on it. If both are equal, just keep same probability
		if(it->getLogOdds() > node->getLogOdds())
			logOddsProb = it->getLogOdds();
		else if (it->getLogOdds() < node->getLogOdds())
			logOddsProb = node->getLogOdds();
		else
			logOddsProb = node->getLogOdds();
		
		mergedMap->setNodeValue(nodeKey,logOddsProb);
		}
		//If both maps show node is free
		else if(!map1->isNodeOccupied(*it) && !mergedMap->isNodeOccupied(node)){
		
		//If both maps show node is occupied, prioritize lowest probability as that has more data on it. If both are equal, just keep same probability
		if(it->getLogOdds() < node->getLogOdds())
			logOddsProb = it->getLogOdds();
		else if (it->getLogOdds() >= node->getLogOdds())
			logOddsProb = node->getLogOdds();
		
		mergedMap->setNodeValue(nodeKey,logOddsProb);
		}
		//If both maps show have conflicting information, keep prioritize occupancy from received map.
		else
		{
			
			float logOddsProb;
			float logOddsDiffMax;
			float logOddsDiffMin;
			//If received map has occupied node and current map is free
			if(map1->isNodeOccupied(node) && !mergedMap->isNodeOccupied(node)){
				logOddsDiffMin = mergedMap->getClampingThresMinLog() - node->getLogOdds();
				logOddsDiffMax = map1->getClampingThresMaxLog() - it->getLogOdds();
				
				
				if(logOddsDiffMax < std::abs(logOddsDiffMin))
					logOddsProb = it->getLogOdds();
				else if (logOddsDiffMax >= std::abs(logOddsDiffMin))
					logOddsProb = node->getLogOdds();
					
			}
			//If received map has free node and current map is occupied
			else if(!map1->isNodeOccupied(node) && mergedMap->isNodeOccupied(node)){
				logOddsDiffMin = map1->getClampingThresMinLog() - it->getLogOdds();
				logOddsDiffMax = mergedMap->getClampingThresMaxLog() - node->getLogOdds();
				
				if(logOddsDiffMax > std::abs(logOddsDiffMin))
					logOddsProb = it->getLogOdds();
				else if (logOddsDiffMax <= std::abs(logOddsDiffMin))
					logOddsProb = node->getLogOdds();
			}
			
			/*
			//Prioritize occupancy
			if(map1->isNodeOccupied(*it) || mergedMap->isNodeOccupied(node)){
				logOddsProb = std::max(it->getLogOdds(),node->getLogOdds());
			}
			mergedMap->setNodeValue(nodeKey,logOddsProb);
			*/
		}
		
	}
		//If node doesn't exist, set probability and node key
	else {
		mergedMap->setNodeValue(nodeKey, it->getLogOdds());		
		}
		
	//Visualizer	
	if(mergedMap->isNodeOccupied(*it)){
	geometry_msgs::Point cubeCenter;
    cubeCenter.x = it.getX();
    cubeCenter.y = it.getY();
    cubeCenter.z = it.getZ();
	if(cubeCenter.z > 1)
		occupied_cells.markers[mergedMap->getTreeDepth()].points.push_back(cubeCenter);
	
	}
	}
}


void update3DMap(){
//Visualize map of merged map
//reset3DMap();


	if (occupied_cells.markers[mergedMap->getTreeDepth()].points.size() > 0)
        occupied_cells.markers[mergedMap->getTreeDepth()].action = visualization_msgs::Marker::ADD;
    else
        occupied_cells.markers[mergedMap->getTreeDepth()].action = visualization_msgs::Marker::DELETE;
	


	

}


//Local map callback
void OctomapFullCallback(const octomap_msgs::Octomap::ConstPtr& msg){
octomap::AbstractOcTree* tree1 = octomap_msgs::msgToMap(*msg);
octree_1 = dynamic_cast<octomap::OcTree*>(tree1);
if(!initFlag)
	initMergedMap();

if(visualflag){
mergeMap(octree_1);
update3DMap();
}


}


//Merging map callback
void MergeMapCallback(const octomap_msgs::Octomap::ConstPtr& msg){
octomap::AbstractOcTree* tree0;
octomap::OcTree* octree_0;
tree0 = octomap_msgs::msgToMap(*msg);
octree_0 = dynamic_cast<octomap::OcTree*>(tree0);

if(visualflag && initFlag){
	mergeMap(octree_0);
	update3DMap();
}
}




//Get visualizer configurations
void VisualCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
occupied_cells.markers = msg->markers;
visualflag = true;
occupiedcells_sub.shutdown();
}

//projected map configurations
void projectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	nav_msgs::OccupancyGrid gridmap;
	gridmap.info = msg->info;
	gridmap.header = msg->header;
	gridmap.data = msg->data;
	if(!initPM)
		initPRM(gridmap);
	MergeProjectedMap(gridmap);

}

//Get other projected map
void MergeProjectedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	nav_msgs::OccupancyGrid receivedGridmap;
	receivedGridmap.info = msg->info;
	receivedGridmap.header = msg->header;
	receivedGridmap.data = msg->data;
	if(initPM)
		MergeProjectedMap(receivedGridmap);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav1_comm");
    ros::NodeHandle nh;
	
	occupiedcells_sub = nh.subscribe<visualization_msgs::MarkerArray>
            ("uav1/occupied_cells_vis_array", 1, VisualCallback);
	projectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("uav1/projected_map", 1, projectedMapCallback);
	ros::Subscriber mergeProjectedMap_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("uav0/projected_map", 1, MergeProjectedMapCallback);
    ros::Subscriber map_sub = nh.subscribe<octomap_msgs::Octomap>
            ("uav1/octomap_full", 1, OctomapFullCallback);
	ros::Subscriber mergeMap_sub = nh.subscribe<octomap_msgs::Octomap>
            ("uav0/octomap_full", 1, MergeMapCallback);
	
    
    
	
	ros::Publisher occipiedCellsMerged_pub = nh.advertise<visualization_msgs::MarkerArray>
            ("merged_occupied_cells", 1);
	ros::Publisher merged2DMap_pub = nh.advertise<nav_msgs::OccupancyGrid>
            ("merged_projected_map", 1);


	ros::Rate rate(1);
	
	
	
	
	
	
	while(ros::ok()){
    occipiedCellsMerged_pub.publish(occupied_cells);
	merged2DMap_pub.publish(mergedGridmap);
	ros::spinOnce();
    rate.sleep();
	}
    

    return 0;
}