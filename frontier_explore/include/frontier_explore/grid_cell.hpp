#include <ros/ros.h>
#include <utility>

//Class representing a grid cell in grip_map
class GridCell {

private:
    
    bool is_frontier;//Indicator if the cell is on frontier
    int class_ID;//class number
    int8_t prob_val;//probability value of this cell given by occupancy map

public:
    GridCell();
    ~GridCell();

    bool get_is_frontier();
    void set_is_frontier(const bool& is_frontier_);

    int get_class_ID();
    void set_class_ID(const int& class_ID_);

    int8_t get_prob_val();
    void set_prob_val(const int8_t& prob_val_);
};