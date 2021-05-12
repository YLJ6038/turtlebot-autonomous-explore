## ROS API 
### 1. frontier\_explore\_node
The frontier explore node takes in the occupancy grid map messages and generates commands for the robot to explore the unknown environment.   This exploration node is implemented in the *FrontierExplore* class.
#### 1.1 Actions Called 
* move\_base ([move\_base\_msgs/MoveBaseAction](http://docs.ros.org/en/api/move_base_msgs/html/action/MoveBase.html))
>move\_base actionlib API for posting navigation goals. The target frontier pose is sent to move\_base server as a goal to pursue in the world. See [move_base API](http://wiki.ros.org/move_base#Action_API) for details.

#### 1.2 Subscribed Topics
* map ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
>Map which is used for frontier detection and exploration planning. This topic is published by *slam\_gmapping* node, the message contains grid data that marks the space as unknown, occupied or open.

#### 1.3 Published Topics
* mobile\_base/commands/velocity ([geometry\_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
>The motion control message for turtlebot. The node publish this message to rotate the robot to scan the environment.

* frontier\_goal ([visualization\_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))
>Marker that visualizes the next frontier target in rviz.

#### 1.4 Services Called
* detect\_frontiers (frontier\_explore/DetectFrontiers)
>Self-defined computational service that detects the frontiers from map. See section 2 for details.

#### 1.5 Parameters
~`minclasssize`  (`int`, default: `135`)
> A frontier class is valid only if its size is larger than this `minclasssize` . The optimal value of this parameter is highly related with the resolution of current OccupancyGrid map. 135 is a recommended value for the map with resolution 0.01.

~`minexploredist`  (`double`, default: `1.3`)
> The minimum distance of each exploration action. A frontier target is valid only if the distance between the target and the robot is larger than this `minexploredist`.  1.3m is a recommended value for the simulation environment.

#### 1.6 Required tf Transfroms
`map -> base_link`  
> This transformation is typically provided by SLAM nodes, e.g. *slam_gmapping* node.

## 2. detect\_frontier\_server
The detect\_frontier\_server node provide a computational service called *DetectFrontiers*. It takes in the OccupancyGrid map and outputs a queue of qualified frontier centroids that are detected from the map. The computation part is implemented in the *OccupancyMap* class.

### 2.1 Services provided
* detect\_frontiers (frontier_explore/DetectFrontiers)

Definition of *DetectFrontiers.srv* 
<pre>nav_msgs/OccupancyGrid raw_map
int32 minclasssize
~~~
geometry_msgs/Point[] frontiers</pre>

**Request message**: `raw_map` is the OccupancyGrid map obtained from laser scan,  `minclasssize` is the parameter set in frontier\_explore\_node.

**Response message**: `frontiers` is a vector of geometry points, which represents the geometry centroids of the frontier classes detected from map.









