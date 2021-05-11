#include <ros/ros.h>

#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <frontier_explore/grid_cell.hpp>
#include <boost/range/irange.hpp>
#include <cstdint>

class OccupancyMap{

private:

    //map meta data
    double resolution;
    uint32_t height;
    uint32_t width;
    geometry_msgs::Pose origin;

    //data structure to store the occupancy grid map
    std::vector< std::vector<GridCell> > grid_map;

public:
    //function: Constructor
    OccupancyMap(const nav_msgs::OccupancyGrid& raw_map );

    //function: Deconstructor
    ~OccupancyMap();
    
    //function: load and store the OccupancyGrid message in grip_map
    void load_map(const nav_msgs::OccupancyGrid& raw_map);

    //function: Check if a grid cell is on frontier
    bool is_cell_on_frontier(const int& x_, const int& y_);

    //function: Get all frontier cells from the map, return the total number of frontiers 
    int get_frontier_cells();

    //function: Classify the frontiers into different classes, return a vector that containing grid indices of each class
    std::vector< std::vector< std::pair<uint32_t, uint32_t> > > classify_frontier_cells();

    //function: Check if a frontier is neighbored by any other frontier
    bool is_frontier_isolated(const int& x, const int& y, int& class_num);

    //function: Convert grid indices to world frame coordinates
    std::vector<geometry_msgs::Point> grid2world(std::vector< std::pair<uint32_t, uint32_t> >& frontiers_grid);
    
    //function: Get the geometric centroids of the frontier classes
    geometry_msgs::Point get_class_centroid(const std::vector<geometry_msgs::Point>& class_world);

    //function: Get the frontier centroids of the map, store the result in frontier_centroids
    bool detect_frontier_centroids(std::vector<geometry_msgs::Point>& frontier_centroids, int& min_class_size);
};

