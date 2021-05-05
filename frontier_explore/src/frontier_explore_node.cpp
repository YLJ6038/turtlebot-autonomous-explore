#include <frontier_explore/frontier_explore.hpp>

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "frontier_exploration_node");
  
    ros::NodeHandle nh;
    FrontierExplore frontier_explorer(nh);
 
    frontier_explorer.start_explore();

    return 0;
}