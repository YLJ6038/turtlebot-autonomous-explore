#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>


#include <vector>
#include <utility>
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <frontier_explore/occupancy_map.hpp>
#include <frontier_explore/DetectFrontiers.h>

class FrontierExplore{

private:

    ros::NodeHandle nh;//NodeHandle

    //Actionlib server for move_base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movePlanner;

    ros::Subscriber map_sub;//Subscriber of map message
    ros::Publisher vel_pub;//Publisher to advertise robot twist
    ros::ServiceClient frontier_cli;//Client of detect_frontiers service
    ros::Publisher rviz_pub;//Publisher for rviz marker

    //Listener for robot pose
    tf::TransformListener robotpose_listener;
    tf::StampedTransform robotpose;

    //wwist message to control robot
    geometry_msgs::Twist twist_msg;

    double frequency;//ros rate
    double velZ_ang;//angular velocity

    //srv object of detect_frontiers service
    frontier_explore::DetectFrontiers DF_srv;

public:
    //function: Constructor
    FrontierExplore( ros::NodeHandle& nh );

    //function: Deconstructor
    ~FrontierExplore();

    //function: init the robot and its motion
    void init_robot();

    //function: rotate the robot to scan the environment
    void rotate360();

    //function: callback function of map_sub, update map with the newly received msg
    void map_subCB(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);

    //function: make robot explore the given frontier points
    bool explore_centroids(std::vector<geometry_msgs::Point>& frontier_centroids);

    //function: calculate the distance between the frontier and current robot location
    double get_dist(const geometry_msgs::Point& frontier_centroid);

    //function: move robot to the given point
    bool move_to_centroid(const geometry_msgs::Point& frontier_centroid);

    //function: mark the frontier target in RVIZ
    void mark_centroid(const geometry_msgs::Point& frontier_centroid);

    //function: execute frontier-based exploration 
    bool start_explore();
};