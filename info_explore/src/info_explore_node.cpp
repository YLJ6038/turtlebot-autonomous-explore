#include <info_explore.hpp>




//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "information_exploration_node");
  
    ros::NodeHandle nh;

    
    InfoExplore info_explorer(nh);
 
    info_explorer.start_explore();

    return 0;
}