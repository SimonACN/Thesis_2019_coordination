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



octomap::OcTree* octree_1;

octomap::OcTree* mergedMap;
visualization_msgs::MarkerArray occupied_cells;
ros::Subscriber projectedMap_sub;
ros::Subscriber occupiedcells_sub;
nav_msgs::OccupancyGrid mergedGridmap;

bool visualflag = false;
bool initFlag = false;
bool initPM = false;



void update3DMap();
void mergeMap(octomap::OcTree*, octomap::OcTree::leaf_iterator);
void reset3DMap();
void initMergedMap();
void MergeProjectedMap(nav_msgs::OccupancyGrid);
void initPRM(nav_msgs::OccupancyGrid);


void initPRM(nav_msgs::OccupancyGrid initMap){
	mergedGridmap.info = initMap.info;
	mergedGridmap.header = initMap.header;
	mergedGridmap.data = initMap.data;
	initPM = true;
}

//Merge projected maps
//Work in progress
void MergeProjectedMap(nav_msgs::OccupancyGrid gridmap2){
	float offsetX, offsetY, offsetXPM1, offsetYPM1 = 0;
	
	nav_msgs::OccupancyGrid tempGridmap;
	tempGridmap.data = mergedGridmap.data;
	//Determine which origin is the lowest
	
	//Width
	if(mergedGridmap.info.origin.position.x < gridmap2.info.origin.position.x){
		offsetX = ceil(-mergedGridmap.info.origin.position.x); //Offset of grid cells when merging the maps
	}
	else if(mergedGridmap.info.origin.position.x > gridmap2.info.origin.position.x){
		mergedGridmap.info.origin.position.x = gridmap2.info.origin.position.x;
		mergedGridmap.info.height = mergedGridmap.info.height + ceil(-mergedGridmap.info.origin.position.x); //Extend width by origin change
		offsetXPM1 = ceil(-mergedGridmap.info.origin.position.x); //Offset of grid cells when merging the maps
	}
	
	//Height
	if(mergedGridmap.info.origin.position.y < gridmap2.info.origin.position.y){
		offsetY = ceil(-mergedGridmap.info.origin.position.y); //Offset of grid cells when merging the maps
	}
	else if(mergedGridmap.info.origin.position.y > gridmap2.info.origin.position.y){
		mergedGridmap.info.origin.position.y = gridmap2.info.origin.position.x;
		mergedGridmap.info.height = mergedGridmap.info.height + ceil(-mergedGridmap.info.origin.position.y); //Extend width by origin change
		offsetYPM1 = ceil(-mergedGridmap.info.origin.position.y); //Offset of grid cells when merging the maps
	}
	
	mergedGridmap.data.clear();
	mergedGridmap.data.resize(mergedGridmap.info.height* mergedGridmap.info.width,-1);
	
	for(int x = 0; x<tempGridmap.info.width; x++){
		for(int y = 0; y <tempGridmap.info.height;y++){
			//Copy old map to the new one.
			if(tempGridmap.data[x*y] != -1)
				mergedGridmap.data[(x+offsetXPM1)*(y*offsetYPM1)] = tempGridmap.data[x*y];
		}			
	}
	
	for(int x = 0; x<gridmap2.info.width; x++){
		for(int y = 0; y <gridmap2.info.height;y++){
			//Add only new data to local map from received map
			if(mergedGridmap.data[(x+offsetX)*(y*offsetY)] != -1)
				mergedGridmap.data[(x+offsetX)*(y*offsetY)] = gridmap2.data[x*y];
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


void mergeMap(octomap::OcTree* map1, octomap::OcTree::leaf_iterator it)
{
	octomap::point3d point = it.getCoordinate(); //Get coordinates
	octomap::OcTreeNode *nodeIn1 = map1->search(point); //Search for node
	if (nodeIn1 != NULL) { 
		//If local map has occupied/free node, Prioritize local map.
		if(octree_1->isNodeOccupied(*it)){
			octomap::OcTreeNode *newNode = mergedMap->updateNode(point, true);
			newNode->setLogOdds(it->getLogOdds());
		}
		
		else{
			octomap::OcTreeNode *newNode = mergedMap->updateNode(point, false);
			newNode->setLogOdds(it->getLogOdds());
			}
		}
		//If node doesn't exist, copy probability and node key
		else{
			octomap::OcTreeKey nodeKey = map1->coordToKey(point);
			mergedMap->updateNode(nodeKey, it->getLogOdds());

			}
			
			
}


void update3DMap(){
//Visualize map of merged map
reset3DMap();
for (octomap::OcTree::leaf_iterator it = mergedMap->begin_leafs(); it != mergedMap->end_leafs(); ++it){
	if(mergedMap->isNodeOccupied(*it)){
	geometry_msgs::Point cubeCenter;
    cubeCenter.x = it.getX();
    cubeCenter.y = it.getY();
    cubeCenter.z = it.getZ();
	if(cubeCenter.z > 1){
    occupied_cells.markers[mergedMap->getTreeDepth()].points.push_back(cubeCenter);
	if (occupied_cells.markers[mergedMap->getTreeDepth()].points.size() > 0)
        occupied_cells.markers[mergedMap->getTreeDepth()].action = visualization_msgs::Marker::ADD;
    else
        occupied_cells.markers[mergedMap->getTreeDepth()].action = visualization_msgs::Marker::DELETE;
	}
	}
}
}


//Local map callback
void OctomapFullCallback(const octomap_msgs::Octomap::ConstPtr& msg){
octomap::AbstractOcTree* tree1 = octomap_msgs::msgToMap(*msg);
octree_1 = dynamic_cast<octomap::OcTree*>(tree1);
if(!initFlag)
	initMergedMap();

if(visualflag){
for (octomap::OcTree::leaf_iterator it = octree_1->begin_leafs(); it != octree_1->end_leafs(); ++it) {
mergeMap(octree_1,it);

}
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
for (octomap::OcTree::leaf_iterator it = octree_0->begin_leafs(); it != octree_0->end_leafs(); ++it) {
 mergeMap(octree_0,it);
}
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
	//if(initPM)
		//MergeProjectedMap(receivedGridmap);
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