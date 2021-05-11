#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>

#include <info_map.hpp>

using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class InfoExplore{

private:

    ros::NodeHandle nh;
    MoveBaseClient ac;

    //Name of generated octomap
    std::string octomap_name_3d;

    //Subscriber and publisher
    ros::Subscriber kinect_sub;
    ros::Publisher GoalMarker_pub;
    ros::Publisher Candidates_pub;
    ros::Publisher Frontier_points_pub;
    ros::Publisher pub_twist;

    //Velocity command
    geometry_msgs::Twist twist_cmd;

    //InfoMap object that analyzes the map and generate candidates   
    InfoMap env_map;

    //Pose and orientation of goal candidate
    point3d goal_vp;
    tf::Quaternion goal_heading;

    //tf_listener
    tf::TransformListener *tf_listener;
    tf::StampedTransform transform;

public:

    InfoExplore( ros::NodeHandle& nh );
    ~InfoExplore();
    
    //Function: start the explore
    void start_explore();

    //Function: rotate the robot to get the initial scan of the environment
    void init_scan();

    //Function: visualize the candidates with marker arrays in rviz
    void visualize_candidates();

    //Function: visualize the goal candidate with a marker in rviz
    void visualize_goal();

    //Function: request the move_base server to move the robot to goal
    bool move_to_dest(point3d goal_p, tf::Quaternion goal_q);

    //Function: callback function for kinect_sub
    void kinectCB( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg );

};

