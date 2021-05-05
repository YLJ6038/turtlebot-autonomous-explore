#define NoClass -1
#define UnDetect -2
#include <frontier_explore/grid_cell.hpp>

//Implementation of GridCell class

GridCell::GridCell(){
    is_frontier = false;
    class_ID = NoClass;
    prob_val = UnDetect;
}

GridCell::~GridCell(){}


bool GridCell::get_is_frontier(){
    return is_frontier;
}

void GridCell::set_is_frontier(const bool& is_frontier_){
    is_frontier = is_frontier_;
}

int GridCell::get_class_ID(){
    return class_ID;
}

void GridCell::set_class_ID(const int& group_ID_){
    class_ID = group_ID_;
}

int8_t GridCell::get_prob_val(){
    return prob_val;
}

void GridCell::set_prob_val(const int8_t& prob_val_){
    prob_val = prob_val_;
}