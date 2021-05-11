#define PI 3.1415926
#define octo_reso 0.05

#include <info_map.hpp>

InfoMap::InfoMap():
Kinect_360(128, 96, 2*PI*57/360, 2*PI*43/360, 6),
GP(100, 3, 0.01),
new_tree(octo_reso){

    num_of_samples_eva = 15;
    num_of_bay = 3;
    cur_tree = &new_tree;
}


InfoMap::~InfoMap(){}


vector<int> InfoMap::sort_MIs(const vector<double> &v){
    vector<int> idx(v.size());
    iota(idx.begin(), idx.end(),0);

    sort(idx.begin(), idx.end(), 
        [&v](int i1, int i2) {return v[i1] > v[i2];});

    return idx;
}



double InfoMap::countFreeVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}




octomap::Pointcloud InfoMap::castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    octomap::Pointcloud hits;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(Kinect_360.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, Kinect_360.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * Kinect_360.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}


vector<vector<point3d> > InfoMap::extractFrontierPoints(const octomap::OcTree *octree) {

    vector<vector<point3d> > frontier_groups;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true;         // whether or not a frontier point
    bool belong_old;            //whether or not belong to old group
    double distance;
    double R1 = 0.4;            //group size
    double x_cur, y_cur, z_cur;


    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!octree->isNodeOccupied(*n))
        {
         x_cur = n.getX();
         y_cur = n.getY();
         z_cur = n.getZ();

         if(z_cur < 0.4)    continue;
         if(z_cur > 0.4 + octo_reso)    continue;
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
            {
                n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                if(!n_cur_frontier)
                {
                    frontier_true = true;
                    continue;            
                }

            }
            if(frontier_true)// && num_free >5 )
            {
                // divede frontier points into groups
                if(frontier_groups.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                    frontier_groups.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                            distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)) ;
                            if(distance < R1){
                               frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                               belong_old = true;
                               break;
                            }
                    }
                    if(!belong_old){
                               frontier_points.resize(1);
                               frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                               frontier_groups.push_back(frontier_points);
                               frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    return frontier_groups;
}


vector<pair<point3d, point3d> > InfoMap::extractCandidateViewPoints(vector<vector<point3d> > frontier_groups, point3d sensor_orig, int n ) {
    double R2_min = 1.0;        // distance from candidate view point to frontier centers, in meters.
    double R2_max = 5.0;
    double R3 = 0.3;        // to other frontiers

    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d> > candidates;
    double z = sensor_orig.z();
    double x, y, yaw, distance_can;

        for(vector<vector<point3d> >::size_type u = 0; u < frontier_groups.size(); u++) {
            for(double yaw = 0; yaw < 2*PI; yaw += PI*2/n )
                for(double R2 = R2_min; R2<=R2_max; R2+=0.5) { 
                x = frontier_groups[u][0].x() - R2 * cos(yaw);
                y = frontier_groups[u][0].y() - R2 * sin(yaw);

                bool candidate_valid = true;
                n_cur_3d = cur_tree->search(point3d(x, y, z));


                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2)) < 0.25){
                  candidate_valid = false;// delete candidates close to sensor_orig
                  continue;
                }

                else{

                    // check candidate to other frontiers;
                    for(vector<vector<point3d> >::size_type n = 0; n < frontier_groups.size(); n++)
                        for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                            distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2));
                            if(distance_can < R3){
                                candidate_valid = false;        //delete candidates close to frontier
                                continue;
                            }
                    }
                
                    // volumn check
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                        for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                            for (double z_buf = sensor_orig.z()-0.1; z_buf <sensor_orig.z()+0.3; z_buf += octo_reso)
                            {
                                n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                if(!n_cur_3d)       continue;
                                else if (cur_tree->isNodeOccupied(n_cur_3d)){
                                candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                continue;
                                }  
                            }

                }

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
            }
        }
    return candidates;
}


double InfoMap::calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, Kinect_360.max_range, true, true);
    double after = countFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}

vector<double> InfoMap::evaluate_MIs(double& prior_MI){
    vector<double>  mu_info(candidates.size());
    
    #pragma omp parallel for
    for(int i = 0; i < candidates.size(); i++) {
        auto c = candidates[i];
        // Evaluate Mutual Information
        octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, c.second);
        
        // Normalize the MI with distance
        mu_info[i] = calc_MI(cur_tree, c.first, hits, prior_MI) / 
            sqrt(pow(c.first.x()-kinect_orig.x(),2) + pow(c.first.y()-kinect_orig.y(),2));
    }
    return mu_info;
}


vector<int> InfoMap::bayopt_MIs(double& prior_MI){

    double train_time, test_time;
    vector<pair<point3d, point3d>> gp_test_poses = candidates;

    for (int bay_itr = 0; bay_itr < num_of_bay; bay_itr++) {
        //Initialize gp regression
        
        MatrixXf gp_train_x(candidates.size(), 3), gp_train_label(candidates.size(), 1), gp_test_x(gp_test_poses.size(), 3);

        for (int i=0; i< candidates.size(); i++){
            gp_train_x(i,0) = candidates[i].first.x();
            gp_train_x(i,1) = candidates[i].first.y();
            gp_train_x(i,2) = candidates[i].second.z();
            gp_train_label(i) = MIs[i];
        }

        for (int i=0; i< gp_test_poses.size(); i++){
            gp_test_x(i,0) = gp_test_poses[i].first.x();
            gp_test_x(i,1) = gp_test_poses[i].first.y();
            gp_test_x(i,2) = gp_test_poses[i].second.z();
        }

        // Perform GP regression
        MatrixXf gp_mean_MI, gp_var_MI;
        train_time = ros::Time::now().toSec();
        GP.train(gp_train_x, gp_train_label);
        train_time = ros::Time::now().toSec() - train_time;

        test_time = ros::Time::now().toSec();
        GP.test(gp_test_x, gp_mean_MI, gp_var_MI);
        test_time = ros::Time::now().toSec() - test_time;

        // Get Acquisition function
        double beta = 2.4;
        vector<double>  bay_acq_fun(gp_test_poses.size());
        for (int i = 0; i < gp_test_poses.size(); i++) {
            bay_acq_fun[i] = gp_mean_MI(i) + beta*gp_var_MI(i);
        }
        vector<int> idx_acq = sort_MIs(bay_acq_fun);

        // evaluate MI, add to the candidate
        auto c = gp_test_poses[idx_acq[0]];
        octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, c.second);
        candidates.push_back(c);
        MIs.push_back(calc_MI(cur_tree, c.first, hits, prior_MI));
        gp_test_poses.erase(gp_test_poses.begin()+idx_acq[0]);
    }
    return sort_MIs(MIs);
}

void InfoMap::clear_history(){

    frontier_groups.clear();
    candidates.clear();
    MIs.clear();
    idx_MIs.clear();
}


bool InfoMap::get_explore_candidates(){

    clear_history();
    frontier_groups = extractFrontierPoints(cur_tree);

    candidates = extractCandidateViewPoints(frontier_groups, kinect_orig, 30); 
    std::random_shuffle(candidates.begin(),candidates.end()); // shuffle to select a subset
    ROS_INFO("Candidate View Points: %lu Genereated, %d evaluating...", candidates.size(), num_of_samples_eva);

    int temp_size = candidates.size()-3;
    if (temp_size < 1) {
        ROS_ERROR("Very few candidates generated, finishing with exploration...");
        return false;
    }
    candidates.resize(min(num_of_samples_eva,temp_size));
    frontier_groups.clear();

    double prior_MI = countFreeVolume(cur_tree);
    MIs = evaluate_MIs(prior_MI);
    idx_MIs = bayopt_MIs(prior_MI);
    return true;
}