## ROS API 
### info\_explore\_node
The info explore node takes in the point cloud data collected by Kinect and generates commands for the robot to explore the unknown environment.  It uses octomap to model the environment and acquire the mutual information value on the map. 

The exploration procedures are defined and implemented in the *InfoExplore* class. The computational part of this node is implemented in the *InfoMap* class.
#### 1. Actions Called 
* move\_base ([move\_base\_msgs/MoveBaseAction](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))
>move\_base actionlib API for posting navigation goals. The target exploration candidate pose is sent to move\_base server as a goal to pursue in the world. See [move_base API](http://wiki.ros.org/move_base#Action_API) for details.

#### 2. Subscribed Topics
* /camera/depth/points ([sensor\_msgs/PointCloud2](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
>Point cloud message generated by the Kinect sensor. This data is then transformed by the callback function to update the octomap of current environment.

#### 3. Published Topics
* mobile\_base/commands/velocity ([geometry\_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
>The motion control message for turtlebot. The node publish this message to rotate the robot to scan the environment.

* Frontier\_point ([visualization\_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))
>Markers that visualize the current frontier map in rviz.

* Candidate\_MIs ([visualization\_msgs/MarkerArray](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/MarkerArray.html))
>Marker(arrow) array that visualizes the exploration candidates sampled on the map. Each arrow represents a candidate. The height and color of the arrow markers indicate the MI value of the candidates.

* Goal\_Marker ([visualization\_msgs/MarkerArray](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))
>Marker that visualizes the pose of next optimal candidate in rviz. 

#### 4. Required tf Transfroms
`map -> base_link`  
> This transformation is typically provided by SLAM nodes, e.g. *slam_gmapping* node.











