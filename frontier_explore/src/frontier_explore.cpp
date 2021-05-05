#include <frontier_explore/frontier_explore.hpp>


FrontierExplore::FrontierExplore( ros::NodeHandle& nh ): 
nh( nh ),
movePlanner("move_base", true){

    //Init subscribers, publishers and clients
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
    map_sub = nh.subscribe("/map", 1000, &FrontierExplore::map_subCB, this);
    frontier_cli = nh.serviceClient<frontier_explore::DetectFrontiers>("detect_frontiers");
    rviz_pub = nh.advertise<visualization_msgs::Marker>("frontier_goal",10);

    //Set frequency and rotation velocity
    frequency = 15;
    velZ_ang = 0.628319;

    //Init motion of turtlebot
    init_robot();
}


void FrontierExplore::init_robot(){
    //Init the robot motion by setting zero velocity
    geometry_msgs::Twist init;
    vel_pub.publish(init);
    ros::Duration(1).sleep();
}


FrontierExplore::~FrontierExplore(){
    //Stop the robot motion by setting zero velocity
    geometry_msgs::Twist stop;
    vel_pub.publish(stop);
    ros::Duration(1).sleep();
}


void FrontierExplore::map_subCB(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg){

    //Update the DF_srv object with newly received grid_msg
    DF_srv.request.raw_map.data.clear();
    DF_srv.request.raw_map.data = grid_msg->data;
    DF_srv.request.raw_map.info.height = grid_msg->info.height;
    DF_srv.request.raw_map.info.width = grid_msg->info.width;
    DF_srv.request.raw_map.info.origin = grid_msg->info.origin;
    DF_srv.request.raw_map.info.resolution = grid_msg->info.resolution;

}


void FrontierExplore::rotate360(){

    ros::Rate rate(frequency);
    //Set rotate duration to 20seconds. The robot will rotate 720degree 
    double scan_duration = 20;

    for (int i=0; i<(int)(scan_duration * frequency); i++){
        geometry_msgs::Twist move;
        move.angular.z = velZ_ang;
        vel_pub.publish(move);

        ros::spinOnce(); 
        rate.sleep();
    }
    geometry_msgs::Twist stop;
    vel_pub.publish(stop);
    ros::Duration(0.5).sleep();
}



double FrontierExplore::get_dist(const geometry_msgs::Point& frontier_centroid){
    //Calculate the distance between the frontier and current robot location
    return std::hypot(frontier_centroid.x - robotpose.getOrigin().x(), frontier_centroid.y - robotpose.getOrigin().y());
}


bool FrontierExplore::explore_centroids(std::vector<geometry_msgs::Point>& frontier_centroids){

    //Retrieve current robot pose with tf listener
    try {robotpose_listener.lookupTransform("/map", "/base_link", ros::Time(0), robotpose);
    } catch (tf::LookupException &e) {
        ROS_ERROR("%s", e.what());
        ros::Duration(3.0).sleep();
    }
    
    //Sort the frontiers by distance in an ascending order
    sort(frontier_centroids.begin(),frontier_centroids.end(),
        [this](geometry_msgs::Point& p1, geometry_msgs::Point& p2){
            return get_dist(p1) < get_dist(p2);
        });

    ROS_INFO("Try navigation to the nearest frontier. Move_base server planning..");
    for(auto centroid: frontier_centroids){
        //Only consider frontier centroids that are more than 1.2m away
        if(get_dist(centroid) > 1.3){
            
            //Visualize frontier target in rviz
            mark_centroid(centroid);
            ros::spinOnce();

            //Move to the centroid if it is reachable. Otherwise get false
            if(move_to_centroid(centroid)){
                ROS_INFO("Current frontier is explored. Waiting for next scan...");
                ros::Duration(1).sleep();
                return true;

            }else{
                //Try next frontier centroid in queue if navigation failed
                ROS_INFO("Try next frontier candidate. ");
                continue;
            }
        }
    }
    //NO valid frontier is reachable, return false and complete the exploration.
    ROS_INFO("NO valid frontier is reachable. Exploration complete.");
    ros::Duration(1).sleep();
    return false;
}

void FrontierExplore::mark_centroid(const geometry_msgs::Point& frontier_centroid){

    //Init a marker in rviz to visualize current frontier target
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.ns = "cube_marker";
    marker.type = visualization_msgs::Marker::CUBE;

    marker.pose.position.x = frontier_centroid.x;
    marker.pose.position.y = frontier_centroid.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w =  1.0;

    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.2;

    marker.color.r = 1.00f;
    marker.color.g = 0.00f;
    marker.color.b = 0.00f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(7);

    rviz_pub.publish(marker);
}

bool FrontierExplore::move_to_centroid(const geometry_msgs::Point& frontier_centroid){
    
    while (!movePlanner.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the move_base action server to come up");
    
    //Set the goal point for move_base
    move_base_msgs::MoveBaseGoal goal_point;
    goal_point.target_pose.header.frame_id = "/map";
    goal_point.target_pose.header.stamp = ros::Time::now();
    goal_point.target_pose.pose.position.x = frontier_centroid.x;
    goal_point.target_pose.pose.position.y = frontier_centroid.y;
    goal_point.target_pose.pose.orientation.w = 1.0;

    movePlanner.sendGoal(goal_point);//Send the goal to server

    movePlanner.waitForResult();
    if (movePlanner.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successful navigation");
        return true;
    }else{
        ROS_INFO("Failed navigation");
        return false;
    }
}


bool FrontierExplore::start_explore(){

    std::vector<geometry_msgs::Point> frontier_centroids;
    bool valid_frontier_exists = true;

    ros::Rate rate(frequency);
    while(ros::ok){

        ROS_INFO("Start Scan.");
        rotate360();//Rotate robot to scan the world
        ROS_INFO("End Scan.");
        ros::spinOnce();

        //call detect_frontiers server
        if(frontier_cli.call(DF_srv)){

            frontier_centroids = DF_srv.response.frontiers;
            if(frontier_centroids.size() != 0){
                //Explore the detected frontiers
                valid_frontier_exists = explore_centroids(frontier_centroids);
                if(!valid_frontier_exists) return true;
            }else{
                //Eploration finished if no more frontier detected.
                ROS_INFO("Exploration completed.");
                return true;
            }
        }else{
            ROS_ERROR("Failed to call detect_frontiers service.");
        }

        frontier_centroids.clear();
        ros::spinOnce();
        rate.sleep();
    }  
}
