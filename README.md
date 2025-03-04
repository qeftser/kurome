
# Kurome

This is the core navigation system for the Bender ARS 2024 season robot. 
It includes a path planner, path follower, and graph slam system for use
with the robot. The system is designed to run with any set of point cloud
or lidar sensors, but expects odometry and/or local beacon or gps input to
be provided independently.

## Dependencics

### ROS2 Iron

This is the ros installation this system was built on. Certain parts may
work on other versions, but no promises :)
```
https://docs.ros.org/en/iron/index.html
```

### tf2 & tf2_ros

These are normally bundled with ros2, but could be external. Used for the
transforms between reference frames in the system.
```
https://wiki.ros.org/tf2
```

### Gazebo ros2

This is used for the simulation and can be skipped if you don't have an 
interest in doing the simulation work.
```
https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
https://gazebosim.org/docs/latest/ros_installation
```

### Robot Localization

This is an external ros2 node used for generating the odom -> base_link
transformation in the simulations. It is not needed outside of the
simulation, but it is reccomended for use with your robot. Good software!
```
https://docs.ros.org/en/noetic/api/robot_localization/html/index.html
```

### SFML

This is a multimedia library that is used for the gui systems bundled with
some of the nodes. It is needed for the software to compile.
```
https://www.sfml-dev.org
```

### Eigen

Eigen is used for some of the internal matrix math. 
Version 3.3 is known to work.
```
https://eigen.tuxfamily.org/index.php?title=Main_Page
```

### Ceres

Ceres is used for the builtin graph slam algorithm for optimization. 
Version 2.2 is known to work.
```
ceres-solver.org
```

### graph_slam_backend

graph_slam_backend is used for the qeftser graph slam algorithm. This
will probably be removed as things go forward, as it is not useable for
large pose graphs.  
```
https://github.com/qeftser/graph_slam_backend
```

## Usage

Clone the repository into your ros2 workspace 
source directory.
```
cd ./ros_workspace/src
git clone https://github.com/qeftser/kurome
```

In the main directory, build the project.
```
cd ..
colcon build
```

Source the files
```
source install/setup.sh
```

If everything build correctly and all dependencies are
installed, you should now have access to all of the nodes
associated with this project.

## Nodes

All nodes used by the system. Associated example config and launch files
for these nodes will be located in the config and launch directories in this
project. The deme config and launch file shows an example of using the entire
system on a robot.

### Pino

This is the node that houses the SLAM system. It's main 
purposes are to generate the map of the environment and
the map -> odom transformation.

#### Subscriptions

| Type                         | Description                                                       |
|------------------------------|-------------------------------------------------------------------|
| [nav_msgs/Odometry][1]       | The pose estimate from the associated beacon                      |
| [nav_msgs/Odometry][1]       | The pose estimate from the associated odometry                    |
| [sensor_msgs/LaserScan][2]   | Input from all laser sensors on the robot                         |
| [sensor_msgs/PointCloud2][3] | Input from all the point cloud sensors on the robot               |
| [geometry_msgs/Twist][4]     | Velocity input. Used instead of odometry if use_odom_vel is false |

#### Publishers

| Type                                | Description                                            |
|-------------------------------------|--------------------------------------------------------|
| [nav_msgs/OccupancyGrid][5]         | The map produced by the SLAM system.                   |
| [visualization_msgs/MarkerArray][6] | A visualization of the SLAM system for use with rviz2. |
| [geometry_msgs/TransformStamped][7] | The map -> odom transformation                         |

#### Parameters

| Name                             | Type   | Default       | Description                                                                                                                                        |
|----------------------------------|--------|---------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| map_out                          | string | "map"         | The topic to publish the constructed [map][5] on                                                                                                   |
| map_publish_interval             | double | 1.0           | The time interval in seconds to publish the map at                                                                                                 |
| publish_visualization            | bool   | false         | Whether to publish the visualization of the SLAM system                                                                                            |
| visualization_topic              | string | "pino/visual" | The topic to publish the [visualization][6] on                                                                                                     |
| beacon_in                        | string | "beacon"      | The topic to recieve the beacon/gps [odometry][1] on                                                                                               |
| odom_in                          | string | "odom"        | The topic to recieve the [odometry][1] on                                                                                                          |
| scan_in                          | string | "scan"        | The topic to reciece the [laser scan][2] messages on                                                                                               |
| cloud_in                         | string | "points"      | The topic to recieve the [point cloud][3] messages on                                                                                              |
| use_odom_vel                     | bool   | true          | Whether to rely on odometry to provide velocity input                                                                                              |
| vel_in                           | string | "cmd_vel"     | The [velocity][4] input topic to use if use_odom_vel is false                                                                                      |
| time_error                       | double | 0.01          | The time in seconds an odometry or beacon measurement needs to be within for it to be used in the SLAM system                                      |
| beacon_lost_time                 | double | 0.25          | The time in seconds to wait for a beacon message before declaring the beacon as lost                                                               |
| fix_beacon_nodes                 | bool   | true          | Whether to fix nodes where beacon odometry was used in the pose graph optimization                                                                 |
| estimate_movement_updates        | bool   | false         | Whether to use an internal motion model to estimate the movement past the last odometry                                                            |
| aggregate_sensor_data            | bool   | true          | Whether to bundle multiple sensor readings as one for processing                                                                                   |
| aggregation_interval             | double | 0.1           | The time period in seconds to aggregate observation data before sending it to the SLAM system                                                      |
| algorithm                        | string | "builtin"     | The SLAM algorithm to use. Options are: "builtin", "qeftser". The qeftser algorithm is bad so don't use it                                         |
| bin_size                         | double | 1.0           | The size of the spatial bins in meters for the nearest neighbor search. Try a smaller value if you have a lot of nodes in one spot                 |
| linear_update_distance           | double | 0.3           | The distance in meters the robot must move to trigger an update to the SLAM system                                                                 |
| angular_update_dist              | double | 0.3           | The movement in radians of the robot needed to trigger an update to the SLAM system                                                                |
| lidar_acceptance_threshold       | double | 0.5           | The certainty of a LiDAR scan match needed for it to update the SLAM system                                                                        |
| point_cloud_acceptance_threshold | double | 0.5           | The certainty of an ICP scan match needed to update the SLAM system                                                                                |
| node_association_dist            | double | 0.5           | The area in meters around the robot that is searched for nodes to perform loop closure with                                                        |
| recent_length                    | int    | 7             | The number of recent observations to keep in backlog                                                                                               |
| loop_closure_dist                | int    | 1             | The number of nodes - 1 that are needed between the current one and the one to loop close against. Setting to 1 will result in always loop closing |

### Yoriko

The pathfinding node. Will find a path through the provided map
and publish the individual poses on the path for further processing. It
also has a GUI for use with debugging. 

#### GUI Controls

| Key | Action                                     |
|-----|--------------------------------------------|
| q   | Close the gui                              |
| l   | Move the camera left                       |
| h   | Move the camera right                      |
| j   | Move the camera down                       |
| k   | Move the camera up                         |
| i   | Zoom in                                    |
| o   | Zoom out                                   |
| g   | Set a new goal position for this node only |

The arrow keys will also work for movement of the camera

#### Subscriptions

| Type                           | Description                                    |
|--------------------------------|------------------------------------------------|
| [geometry_msgs/PoseStamped][8] | The goal to try and reach                      |
| [nav_msgs/Odometry][1]         | The pose estimate from the associated odometry |
| [nav_msgs/OccupancyGrid][5]    | The map to pathfind over                       |

#### Publishers

| Type               | Description                     |
|--------------------|---------------------------------|
| [nav_msgs/Path][9] | The map produced by the planner |

#### Parameters

| Name                     | Type   | Default     | Description                                                                                                                    |
|--------------------------|--------|-------------|--------------------------------------------------------------------------------------------------------------------------------|
| path_topic               | string | "path"      | The topic to publish the [path][9] on                                                                                          |
| goal_topic               | string | "goal_pose" | The topic to recieve the [goal][8] on                                                                                          |
| position_topic           | string | "odom"      | The topic to recieve the [position estimate][1] on                                                                             |
| grid_topic               | string | "map"       | The topic to recieve the [map][5] on                                                                                           |
| algorithm                | string | "a_star"    | The algorithm for the planner to use. Options are "a_star", "mi_a_star", "rrt_x_fn", "mi_rrt_x_fn". Only "a_star" works though |
| collision_radius         | double | 0.5         | The distance to stay away from obstacles when pathing                                                                          |
| publish_rate             | double | 0.1         | The time interval in seconds to publish the path                                                                               |
| launch_gui               | bool   | false       | Whether to launch the GUI on startup                                                                                           |
| obstacle_threshold       | int    | 60          | The certainty on a [map][5] position needed to mark it as an obstacle. On the interval \[0-100\]                               |
| allow_out_of_bounds_goal | bool   | true        | Whether to allow goals that are outside of the map boundary                                                                    |

#### RRTX-FN and MI_RRTX-FN Parameters

Parameters specific to these algorithms

| Name                  | Type   | Default | Description                                                                        |
|-----------------------|--------|---------|------------------------------------------------------------------------------------|
| dominance_region      | double | 2.0     | The area around a tree node that the node dominates                                |
| cull_range            | double | 1.5     | The area to remove nodes in when an obstacle is added                              |
| expansion_length      | double | 1.25    | The length of an edge on the tree                                                  |
| node_limit            | int    | 30000   | The total number of nodes allowed in the tree                                      |
| generation_tick_speed | int    | 10      | Time in milliseconds to wait between adding nodes                                  |
| bin_size              | double | 2.0     | Size of the spatial bins used for indexing. Set higher if producing a lot of nodes |
| turning_radius        | double | 2.5     | Minimum turning radius allowed on the path. ** MI_RRTX_FN ** only                  |

#### A* and MI_A* Parameters

Parameters specific to these algorithms

| Name            | Type   | Default | Description |
|-----------------|--------|---------|---------------------------------------------------------------------------------------------------|
| queue_limit     | int    | 40000   | The total number of nodes allowed in the A* queue before the algorithm fails                      |
| backtrack_count | int    | -1      | The number of nodes to recompute on update. Set to -1 to recompute the entire path every timestep |
| turning_radius  | double | 2.5     | Minimum turning radius allowed on the path. ** MI_A* ** only                                      |
| angle_slice     | double | 15.0    | Degrees to produce new nodes on. ** MI_A* only **                                                 |
| position_slice  | double | 0.25    | Seperation in meters on between path node. ** MI_A* only **                                       |
| forward_step    | double | 1.0     | Movement taken at each timestep. ** MI_A* only **                                                 |

### Misao

The smoothing and path following node. Will accept a path and map
as input, smooth the path for following, and then publish velocity
commands to steer the robot along it. GUI is avaliable for debugging.

#### GUI Controls

| Key | Action                      |
|-----|-----------------------------|
| q   | Close the gui               |
| l   | Move the camera left        |
| h   | Move the camera right       |
| j   | Move the camera down        |
| k   | Move the camera up          |
| i   | Zoom in                     |
| o   | Zoom out                    |
| g   | Publish a new goal position |

The arrow keys will also work for movement of the camera

#### Subscriptions

| Type                           | Description                                     |
|--------------------------------|-------------------------------------------------|
| [nav_msgs/Path][9]             | The path to recieve for smoothing and following |
| [nav_msgs/OccupancyGrid][5]    | The map to smooth over                          |
| [nav_msgs/Odometry][1]         | Position estimate of current position           |
| [geometry_msgs/PoseStamped][8] | The goal the robot is trying to reach           |

#### Publishers

| Type                                | Description                                                   |
|-------------------------------------|---------------------------------------------------------------|
| [geometry_msgs/Twist][4]            | The velocity control output from the path follower            |
| [geometry_msgs/PoseStamped][8]      | Goal published by the GUI to the same goal topic              |
| [visualization_msgs/MarkerArray][6] | Visualization of the smoothed path for visualization in rviz2 |

#### Parameters 

| Name                | Type   | Default        | Description                                                                                      |
|---------------------|--------|----------------|--------------------------------------------------------------------------------------------------|
| path_in             | string | "path"         | The topic to recieve the [path][9] to smooth/follow on                                           |
| odom_in             | string | "odom"         | The topic to recieve [odometry][1] on                                                            |
| goal_in             | string | "goal_pose"    | The topic to recieve/publish the [goal][8] on                                                    |
| vel_out             | string | "cmd_vel"      | The topic to publish the [velocity commands][4] to                                               |
| map_in              | string | "map"          | The topic to recieve the [map][5] to smooth over on                                              |
| publish_rate        | double | 0.1            | The time interval in seconds to publish messages                                                 |
| obstacle_threshold  | int    | 60             | The certainty on a [map][5] position needed to mark it as an obstacle. On the interval \[0-100\] |
| launch_gui          | bool   | false          | Whether to launch the associated GUI on startup                                                  |
| run_visualization   | bool   | false          | Whether or not to publish the [visualization][6] of the smoothed path                            |
| visualization_topic | string | "misao/visual" | The topic to publish the [visualization][6] on                                                   |
| collision_radius    | double | 0.3            | The minimum distanct to keep from obstacles                                                      |
| algorithm           | string | "elastic_band" | The algorithm to use for the smoother and follower. Options are "elastic_band"                   |
| desired_speed       | double | 0.1            | Speed in meters/second to have the robot move at                                                 |

#### Elastic Band Parameters 

parameterrs specific to the elastic band algorithm

| Name             | Type   | Default | Description                                                                                                    |
|------------------|--------|---------|----------------------------------------------------------------------------------------------------------------|
| band_length      | int    | -1      | The number of nodes on the path to smooth at one time. Set to -1 to smooth the entire path                     |
| influence_range  | double | 0.75    | The range that obstacles have influence on the elastic band                                                    |
| max_bubble       | double | 1.0     | The maximum bubble allowed by the system. Must be <= than the influence_range                                  |
| contraction_gain | double | 1.0     | The influence the contraction force has on the band. Set higher for a straighter path sooner                   |
| repulsion_gain   | double | 1.0     | The influence the repulsion force has on the band. Set higher to favor avoiding obstacles                      |
| damping_gain     | double | 0.75    | The total reduction of movement in the band. Set higher to slow the band down. Must be 0 < & < 1               |
| cycle_count      | int    | 128     | Total number of cycles to subject the band to at each time step. Set higher to smooth faster                   |
| advance_distance | double | 0.3     | Distance from a node on the path to determine it as reached and advance nodes. Set lower for tighter following |

### Brain

Node written specifically for the Lunabotics 2024 season
competition. Responsible for controlling the robot to
complete the obsectives of the competition.

```
    TODO: Node is unfinished
```

### Wall Filter

Node written to adhere to the requirements laid out in
the rules of the Lunabotics 2024 competition. Competition
walls are not allowed to be used for any kind of map generation
or navigation in the competition, so a node was nessesary to
filter the walls out. This node accepts input from the sensors,
brain, and odometry/map to determine which points can be accepted
and which ones lie outside of or on the walls of the competition arena.

```
    TODO: Node is unfinished
```

## TODO

 * Write the brain node
 * Write the wall filter node
 * Fix issues with correlative scan matcher
 * Impliment ICP for scan matching
 * Fix issues with elastic band snapping on initialization
 * Fix and update pathfinding algorithms for use

## Names

### Kurome

The project is named after Kurome, a character from the manga and television show Akame Ga Kill!

![kurome_akame](https://github.com/user-attachments/assets/2ea04f49-0c40-4ccb-82ed-5fef77d3b855)

### Pino

The SLAM system is named after Pino, the robot girl from the television show Ergo Proxy

![pino_jumping](https://github.com/user-attachments/assets/b26e6b7c-072b-4316-a60b-0d9ffa39d0e8)

### Yoriko

The pathfinder is named after Yoriko Yunoki, a character from the television show Battle Programmer Shirase

![yoriko_yunoki](https://github.com/user-attachments/assets/35a1616d-8e7f-414d-b456-49b1370a61ec)

### Misao

The smoother and path follower is named after Misao Amano, a character from the television show Battle Programmer Shirase

![misao_amano](https://github.com/user-attachments/assets/01a4d374-1428-475e-ba5f-896102d1ce28)
![misao_amano](https://github.com/user-attachments/assets/6226be0a-c202-4ee5-82c7-9fb81b2b977c)

## References

 * Real-Time Correlative Scan Matching
 * A Tutorial on Graph-Based SLAM
 * Probabilistic Robotics
 * RRTX: Asymptotically optimal single-query sampling-based motion planning with quick replanning
 * Wikipedia: A* Search Algorithm
 * Real-Time Modification of Collision-Free Paths

## _

[1]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
[3]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg
[4]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg
[5]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
[6]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/MarkerArray.msg
[7]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
[8]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
[9]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg

