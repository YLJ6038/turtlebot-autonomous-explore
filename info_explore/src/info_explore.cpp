#define PI 3.1415926
#include <info_explore.hpp>



InfoExplore::InfoExplore( ros::NodeHandle& nh ):
nh(nh),
ac("move_base", true)
{

    //Init time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    //Init octomap_name
    strftime(buffer,80,"Octomap3D_%m%d_%R.ot",timeinfo);
    octomap_name_3d = buffer;

    //Init subscriber & publisher
    kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &InfoExplore::kinectCB, this);
    GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "/Goal_Marker", 1 );
    Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("/Candidate_MIs", 1);
    Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("/Frontier_points", 1);
    pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    tf_listener = new tf::TransformListener();
    
}

InfoExplore::~InfoExplore(){}


void InfoExplore::kinectCB( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;
    

    ros::Duration(0.07).sleep();

    tf_listener->waitForTransform("/map",cloud2_msg->header.frame_id, cloud2_msg->header.stamp, ros::Duration(5.0));

    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        std::cout << cloud_local->header.frame_id << std::endl;
        ros::Duration(0.01).sleep();
    }

    // Insert points into octomap one by one...

    
    for (int i = 1; i< cloud->width; i++)
    {
        for (int j = 1; j< cloud->height; j++)
        {
            if(isnan(cloud->at(i,j).x))     continue;
            if(cloud->at(i,j).z < -1.0)     continue;  
            hits.push_back(point3d(cloud->at(i,j).x, cloud->at(i,j).y, cloud->at(i,j).z));
        }
    }
    
    env_map.cur_tree->insertPointCloud(hits, env_map.kinect_orig, env_map.Kinect_360.max_range);
    
    env_map.cur_tree->write(octomap_name_3d);
    ROS_INFO("Entropy(3d map) : %f", env_map.countFreeVolume(env_map.cur_tree));
    
    delete cloud;
    delete cloud_local;
}




void InfoExplore::init_scan(){
    
    bool got_tf = false;
    for(int i =0; i < 8; i++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
            env_map.kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
            std::cout << "tf Got." << std::endl;
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what()); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();
        ros::Duration(0.7).sleep();

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_INFO("Rotating..");
        while (ros::Time::now() - start_turn < ros::Duration(3.0)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed

        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);

    }
}


void InfoExplore::visualize_candidates(){

    visualization_msgs::MarkerArray CandidatesMarker_array;
    CandidatesMarker_array.markers.resize(env_map.candidates.size());

    tf::Quaternion MI_heading;
    MI_heading.setRPY(0.0, -PI/2, 0.0);
    MI_heading.normalize();

    for (int i = 0; i < env_map.candidates.size(); i++)
    {
        CandidatesMarker_array.markers[i].header.frame_id = "map";
        CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
        CandidatesMarker_array.markers[i].ns = "candidates";
        CandidatesMarker_array.markers[i].id = i;
        CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
        CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
        CandidatesMarker_array.markers[i].pose.position.x = env_map.candidates[i].first.x();
        CandidatesMarker_array.markers[i].pose.position.y = env_map.candidates[i].first.y();
        CandidatesMarker_array.markers[i].pose.position.z = env_map.candidates[i].first.z();
        CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
        CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
        CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
        CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
        CandidatesMarker_array.markers[i].scale.x = (double)2.0*env_map.MIs[i]/env_map.MIs[env_map.idx_MIs[0]];
        CandidatesMarker_array.markers[i].scale.y = 0.2;
        CandidatesMarker_array.markers[i].scale.z = 0.2;
        CandidatesMarker_array.markers[i].color.a = (double)env_map.MIs[i]/env_map.MIs[env_map.idx_MIs[0]];
        CandidatesMarker_array.markers[i].color.r = 1.0;
        CandidatesMarker_array.markers[i].color.g = 0.55;
        CandidatesMarker_array.markers[i].color.b = 0.22;
    }
    Candidates_pub.publish(CandidatesMarker_array);
    CandidatesMarker_array.markers.clear();
}



void InfoExplore::visualize_goal(){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "goal_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_vp.x();
    marker.pose.position.y = goal_vp.y();
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = goal_heading.x();
    marker.pose.orientation.y = goal_heading.y();
    marker.pose.orientation.z = goal_heading.z();
    marker.pose.orientation.w = goal_heading.w();
    marker.scale.x = 1.0;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    GoalMarker_pub.publish( marker );
}



bool InfoExplore::move_to_dest(point3d goal_p, tf::Quaternion goal_q) {

  ac.cancelAllGoals();

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_p.x();
  goal.target_pose.pose.position.y = goal_p.y();
  goal.target_pose.pose.position.z = goal_p.z();
bool got_tf = false;
    for(int i =0; i < 10; i++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);// need to change tf of kinect###############
            env_map.kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
            std::cout << "Got" << std::endl;
        }
        catch (tf::TransformException &ex) {
            //ROS_WARN("Wait for tf: Kinect frame"); 
            ROS_ERROR("%s", ex.what());
        } 
        
        }
    }
    ros::spinOnce();

    ros::Duration(0.7).sleep();
  goal.target_pose.pose.orientation.y = goal_q.y();
  goal.target_pose.pose.orientation.z = goal_q.z();
  goal.target_pose.pose.orientation.w = goal_q.w();

  ROS_INFO("Sending robot to the viewpoint...");
  ac.sendGoal(goal);

  // while(ros::ok())
    ac.waitForResult(ros::Duration(120.0));

  // Returns true iff we reached the goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}



void InfoExplore::start_explore(){

    
    ROS_INFO("Start Explore. Init the scan. ");
    ros::Duration(1).sleep();

    init_scan();

    bool arrived = false;
    
    while(ros::ok()){

        bool candidate_not_empty = env_map.get_explore_candidates();
        
        if(candidate_not_empty){

            visualize_candidates();

            arrived = false;
            int idx_ptr = 0;

            while(!arrived){

                goal_vp = point3d(env_map.candidates[env_map.idx_MIs[idx_ptr]].first.x(),
                                  env_map.candidates[env_map.idx_MIs[idx_ptr]].first.y(),
                                  env_map.candidates[env_map.idx_MIs[idx_ptr]].first.z());
                
                goal_heading.setRPY(0.0, 0.0, env_map.candidates[env_map.idx_MIs[idx_ptr]].second.yaw());
                goal_heading.normalize();
                ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f  %3.2f", 
                          env_map.MIs[env_map.idx_MIs[idx_ptr]], goal_vp.x(), goal_vp.y(), goal_vp.z() );

                visualize_goal();

                arrived = move_to_dest(goal_vp, goal_heading);

                if(arrived){
                    // Update the initial location of the robot
                    bool got_tf = false;
                    while(!got_tf){
                    try{
                        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
                        env_map.kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                        got_tf = true;
                    }
                    catch (tf::TransformException ex) {
                        ROS_WARN("Wait for tf: Kinect frame"); 
                    } 
                    ros::Duration(0.05).sleep();
                    }
                    // Update Octomap
                    ros::spinOnce();
                    ROS_INFO("Succeed, new Map Free Volume: %f", env_map.countFreeVolume(env_map.cur_tree));

                }else{
                    ROS_WARN("Failed to drive to the %d th goal, switch to the sub-optimal..", idx_ptr);
                    idx_ptr++;
                    if(idx_ptr > env_map.MIs.size()) {
                        ROS_ERROR("None of the goal is valid for path planning, shuting down the node");
                        nh.shutdown();
                    }
                }
            }
        }else{
            nh.shutdown();
        }
    }
    nh.shutdown();
}