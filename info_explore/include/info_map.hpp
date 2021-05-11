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

#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>
#include <vector>

#include <gpregressor.h>

using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



struct sensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
}; 




class InfoMap{

    friend class InfoExplore;

private: 

    int num_of_samples_eva; //Sample number of initial evaluation
    int num_of_bay; //number of bayopt iteration

    //Octotree of current map
    octomap::OcTree new_tree;
    octomap::OcTree* cur_tree; 

    point3d kinect_orig; //Kinect origin position
    sensorModel Kinect_360; //Kinect sensor model

    GPRegressor GP;//Gaussian process regressor

    //vectors for MI-related calculation
    vector<vector<point3d> > frontier_groups;
    vector<pair<point3d, point3d> > candidates;
    vector<double> MIs;
    vector<int> idx_MIs;

   

public:

    InfoMap();
    ~InfoMap();
    
    //Function: Count free volume of current octotree
    double countFreeVolume(const octomap::OcTree *octree);

    //Function: Ray cast function
    octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position, const point3d &sensor_orientation);

    //Function: Generate a frontier map from current octomap
    vector<vector<point3d> > extractFrontierPoints(const octomap::OcTree *octree);

    //Function: Generate sample candidates in the frontier map
    vector<pair<point3d, point3d> > extractCandidateViewPoints(vector<vector<point3d> > frontier_groups, point3d sensor_orig, int n );

    //Function: Mutual information calculator
    double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before);

    //Function: Sort the candidates based on Mutual information value. Return a sorted index vector.
    vector<int> sort_MIs(const vector<double> &v);

    //Function: Evaluate mutual information of a candidate queue. Return a queue of MI value
    vector<double> evaluate_MIs(double& prior_MI);

    //Function: Baysian optimization for MI estimation. Return a index vector sorted by optimized MI value
    vector<int> bayopt_MIs(double& prior_MI);

    //Function: Clear history data
    void clear_history();

    //Function: Update the candidate and index queue based on current octomap
    bool get_explore_candidates();
};
