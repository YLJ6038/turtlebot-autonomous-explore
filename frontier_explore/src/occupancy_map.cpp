#define NoClass -1 
#define MinClassSize 140 

#include <frontier_explore/occupancy_map.hpp>

//Constructor
OccupancyMap::OccupancyMap( const nav_msgs::OccupancyGrid& raw_map ){
    
    //Load the map meta data
    resolution = raw_map.info.resolution;
    width = raw_map.info.width;
    height = raw_map.info.height;
    origin = raw_map.info.origin;

    //Store the raw map message in grip_map
    load_map(raw_map);
}

//Deconstructor
OccupancyMap::~OccupancyMap(){}


void OccupancyMap::load_map(const nav_msgs::OccupancyGrid& raw_map){
    ROS_INFO("Loading grid map.");

    double idx = 0;
    GridCell cell;
    std::vector<GridCell> cell_set;

    for(int h=0;h<height;h++){
        for(int w=0;w<width;w++){

            //Store the occupancy data of each cell
            cell.set_prob_val(raw_map.data[idx]);
            cell.set_class_ID(NoClass);

            cell_set.push_back(cell);
            idx++;
        }
        grid_map.push_back(cell_set);
        cell_set.clear();
    }
    std::cout << "total cell number: " << idx << std::endl;
}


int OccupancyMap::get_frontier_cells(){

    int counter = 0;//Counter to get total number of frontiers
    //Check if cells on frontier
    for(int h=0;h<height;h++){
        for(int w=0;w<width;w++){

            //A frontier cell must have 0 probablity value(means open area)
            if(grid_map[h][w].get_prob_val() == 0){
                if(is_cell_on_frontier(h, w)){
                    grid_map[h][w].set_is_frontier(true);
                    counter++;
                }else{
                    grid_map[h][w].set_is_frontier(false);
                }
            }
        }
    }
    return counter;
}


bool OccupancyMap::is_cell_on_frontier(const int& x, const int& y){

    //If at least one neighbor cell is unknown(-1), then this cell is a frontier
    for(int h =(x-1);h<(x+2);h++){
        for(int w =(y-1);w<(y+2);w++){
            if (h >= 0 && w >= 0 && h < height && w < width &&
            grid_map[h][w].get_prob_val() == -1) {
                return true;
            }
        }
    }
    return false;
}


//Return a vector whose element is a class of frontiers. Each Class is represented by a vector of grid indices(h,w) pair. 
std::vector< std::vector< std::pair<uint32_t, uint32_t> > > OccupancyMap::classify_frontier_cells(){

    int class_iter = 1;
    int class_num = 1;

    std::vector< std::pair<uint32_t, uint32_t> > init_class;
    std::vector< std::vector< std::pair<uint32_t, uint32_t> > > frontier_classes;

    bool isolated_flg = false;

    for(int h=0;h<height;h++){
        for(int w=0;w<width;w++){
            //Check if cell is frontier, only frontier can be classfied
            if(grid_map[h][w].get_is_frontier()){

                //Check if this frontier is isolated
                isolated_flg = is_frontier_isolated(h, w, class_num);

                if(isolated_flg){

                    //Init a new class, if the frontier is isolated
                    init_class.push_back(std::make_pair(h, w));
                    frontier_classes.push_back(init_class);
                    grid_map[h][w].set_class_ID(class_iter);

                    init_class.clear();
                    class_iter++;

                }else{
                    //Add this grid to the corresponding class
                    grid_map[h][w].set_class_ID(class_num);
                    frontier_classes[class_num-1].push_back(std::make_pair(h, w));
                }
            }
        }
    }
    return frontier_classes;
}


//If frontier is not isolated(has a classified neighbor), then its class number is the same as its neighbor
bool OccupancyMap::is_frontier_isolated(const int& x, const int& y, int& class_num){
    for(int h=(x-1);h<(x+1);h++){

        //Traverse the neighbor cells of frontier(x,y)
        for(int w=(y-1);w<(y+2);w++){
            if (h >= 0 && w >= 0 && h < height && w < width &&
            grid_map[h][w].get_class_ID() != NoClass){
                //Not isolated, so class_ID of frontier(x,y) is the same as neighbor's ID. 
                //Record the class_ID.
                class_num = grid_map[h][w].get_class_ID();
                return false;
            }
        }
    }
    return true;
}



std::vector<geometry_msgs::Point> OccupancyMap::grid2world(std::vector< std::pair<uint32_t, uint32_t> >& frontiers_grid){
    
    //Grid indices -> world coordinates
    std::vector<geometry_msgs::Point> class_world;
    geometry_msgs::Point point_world;

    //world_pose = origin_pose + resolution * indices
    for(auto iter: frontiers_grid){
        point_world.x = origin.position.x + resolution * (double)(iter.second);
        point_world.y = origin.position.y + resolution * (double)(iter.first);
        class_world.push_back(point_world);
    }
    return class_world;
}


//Return the world coordinates of the centroid of a frontier class
geometry_msgs::Point OccupancyMap::get_class_centroid(const std::vector<geometry_msgs::Point>& class_world) {
    
    geometry_msgs::Point centroid_point;
    double sum_x = 0;
    double sum_y = 0;
    for(auto iter: class_world){
        sum_x += iter.x;
        sum_y += iter.y;
    }
    centroid_point.x = sum_x / class_world.size();
    centroid_point.y = sum_y / class_world.size();
    return centroid_point;
}



bool OccupancyMap::detect_frontier_centroids(std::vector<geometry_msgs::Point>& frontier_centroids){

    int frontier_total_num = get_frontier_cells();
    std::cout << "Number of frontiers: " << frontier_total_num << std::endl;

    std::vector< std::vector< std::pair<uint32_t, uint32_t> > > frontier_classes;
    frontier_classes = classify_frontier_cells();
    std::cout << "Number of classes: " << frontier_classes.size() << std::endl;

    geometry_msgs::Point centroid_point;

    for(auto class_element: frontier_classes){
        //A frontier class is qualified only if its size is larger than MinClassSize
        if(class_element.size() > MinClassSize){

            //Transform the coordinates from grid to world, and calculate the centroid
            centroid_point = get_class_centroid( grid2world(class_element) );
            //Push back to the result frontiers
            frontier_centroids.push_back(centroid_point);
        }
    }
    return true;
}

