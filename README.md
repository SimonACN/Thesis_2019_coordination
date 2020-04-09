# Thesis2019_code

# Source files

ROS packages are located in workspace/src.

# drone_code
This is the developed package for the coordinated exploration. Source codes are located under drone_code/src.

uav1_comm.cpp is the source code for the node that merges the 3D and 2D maps and publishes them for rviz.

uavX_node is the source code for basic navigation with global path planner.

Use multi_global_planner_depth-camera.launch to launch the relevant nodes and configurations for multi-vehicle exploration and global_planner_depth-camera.launch for solo exploration
