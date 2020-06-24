#ifndef COORDINATION_H
#define COORDINATION_H
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h> 
#define EXPLORE 0
#define MEET 1
#define MEETING 2
#define RESOLUTION 0.5
#define UNKNOWN -1
#define FREE 0
#define HOME 9
#define frontierDistance 10


int coordination_state = EXPLORE;
geometry_msgs::PoseStamped home;
const double total_area = 40*40;










#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H