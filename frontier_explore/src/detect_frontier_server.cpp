#include <ros/ros.h>
#include <frontier_explore/occupancy_map.hpp>
#include <frontier_explore/DetectFrontiers.h>

//Detect_frontiers server node. 
//This service takes in nav_msg/OccupancyGrid, then return a vector of qualified frontier coordinates as response.

//Srv callback function
bool detect_frontiers(frontier_explore::DetectFrontiers::Request &req, frontier_explore::DetectFrontiers::Response &res){
    
    //Load the map to a OccupancyMap object
    OccupancyMap grid_map(req.raw_map);

    res.frontiers.clear();

    //Detect and return the frontiers
    grid_map.detect_frontier_centroids(res.frontiers);
    ros::Duration(1.5).sleep();
    
    ROS_INFO_STREAM("Finished. " << res.frontiers.size() << " qualified frontier classes detected. ");

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_frontier_servers");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("detect_frontiers", detect_frontiers);
  ROS_INFO("Ready to detect froniters.");
  ros::spin();

  return 0;
}