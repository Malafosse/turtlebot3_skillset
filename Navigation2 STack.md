---
tags:
  - Software_Architecture
  - TB3
  - Nav2
  - ROS2
---
>[!quote] Source
```
@article{macenski2023survey,
      title={From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2},
      author={S. Macenski, T. Moore, DV Lu, A. Merzlyakov, M. Ferguson},
      year={2023},
      journal = {Robotics and Autonomous Systems}
}
```
---
> [!example] [Nav2 Website](https://navigation.ros.org/index.html)
> Nav2 provides **perception, planning, control, localization, visualization**, and much more to build highly reliable autonomous systems.
> 
> It uses **behavior trees** to create customized and intelligent navigation behavior via orchestrating many i**ndependent modular servers**.
> These separate servers communicate with the behavior tree (BT) over a **ROS interface such as an action server or service**. A robot may utilize potentially many different behavior trees to allow a robot to perform many types of unique tasks.
> 
> The expected inputs to Nav2 are **TF transformations** conforming to REP-105, a **map** source if utilizing the Static Costmap Layer, a **BT XML file**, and any relevant **sensor data** sources.

![[Pasted image 20240123094659.png]]

# How Nav2 Works

## Action Servers
Action servers are used in this stack to communicate with the highest level Behavior Tree (BT) navigator through a `NavigateToPose` action message. They are also used for the BT navigator to communicate with the subsequent smaller action servers to compute plans, control efforts, and recoveries. Each will have their own unique `.action` type in `nav2_msgs` for interacting with the servers

## Life-cycle Nodes
Within Nav2, we use a wrapper of LifecycleNodes, `nav2_util LifecycleNode`. This wrapper wraps much of the complexities of LifecycleNodes for typical applications. It also includes a `bond` connection for the lifecycle manager to ensure that after a server transitions up, it also remains active. If a server crashes, it lets the lifecycle manager know and transition down the system to prevent a critical failure.
![](https://foxglove.dev/images/blog/how-to-use-ros2-lifecycle-nodes/hero.jpeg)

## Behavior Trees
For this project, we use [BehaviorTree CPP V3](https://www.behaviortree.dev/) as the behavior tree library. We create node plugins which can be constructed into a tree, inside the `BT Navigator`. The node plugins are loaded into the BT and when the XML file of the tree is parsed, the registered names are associated. At this point, we can march through the behavior tree to navigate.

One reason this library is used is its **ability to load subtrees**. This means that the Nav2 behavior tree can be loaded into another higher-level BT to use this project as node plugin. Additionally, we supply a `NavigateToPoseAction` plugin (among others) for BT so the Nav2 stack can be called from a client application through the usual action interface.

## Navigation Servers

### Planners
Planners can be written to:
- Compute shortest path
- Compute complete coverage path
- Compute paths along sparse or predefined routes
- 
### Controllers
Also known as **local planners** in ROS 1, controllers can be written to:
- Follow a path
- Dock with a charging station using detectors in the odometric frame
- Board an elevator
- Interface with a tool

### Smoothers
As criteria for optimality of the path searched by a planner are usually reduced compared to reality, additional path refinement is often beneficial. Smoothers have been introduced for this purpose, typically responsible for r**educing path raggedness and smoothing abrupt rotations, but also for increasing distance from obstacles** and high-cost areas as the smoothers have access to a global environmental representation.

### Recovery
Recovery behaviors are a mainstay of **fault-tolerant systems**. The goal of recoveries are to deal with unknown or failure conditions of the system and **autonomously handle** them. 

Examples may include faults in the perception system resulting in the environmental representation being full of fake obstacles. The clear costmap recovery would then be triggered to allow the robot to move.

Another example would be if the robot was stuck due to dynamic obstacles or poor control. Backing up or spinning in place, if permissible, allow the robot to move from a poor location into free space it may navigate successfully.

Finally, in the case of a total failure, a recovery may be implemented to call an operator’s attention for help. This can be done via email, SMS, Slack, Matrix, etc.

It is important to note that the behavior server can hold any behavior to share access to expensive resources like costmaps or TF buffers, not just recovery behaviors. Each may have its own API.

### Robot Footprint
It is worth remarking that in the cost maps, we set a robot’s footprint either as a circle of radius `robot_radius` or as a vector of points `footprint` representing an arbitrary polygon if the robot is non-circular. This can also be adjusted over time using the costmap’s `~/footprint` topic, which will **update the polygon over time as needed due to changes in the robot’s state**, such as movement of an attached manipulator, picking up a pallet, or other actions that adjust a robot’s shape. That polygon will then automatically be used by the planners and controllers.

![[Pasted image 20240123105112.png]]

### Waypoint Following
The `nav2_waypoint_follower` contains a waypoint following program with a plugin interface for specific task executors. 

There are 2 schools of thoughts for fleet managers / dispatchers:
- **Dumb robot; smart centralized dispatcher**
	 `nav2_waypoint_follower` is fully sufficient to create a production-grade on-robot solution. Since the autonomy system / dispatcher is taking into account things like the robot’s pose, battery level, current task, and more when assigning tasks, the application on the robot just needs to worry about the task at hand

- **Smart robot; dumb centralized dispatcher**
	In this case, you should use the `nav2_behavior_tree` package to create a custom application-level behavior tree using navigation to complete the task. This can include subtrees like checking for the charge status mid-task for returning to dock or handling more than 1 unit of work in a more complex task. Soon, there will be a `nav2_bt_waypoint_follower`  that will allow you to create this application more easily.

## State Estimation
Within the navigation project, there are 2 major transformations that need to be provided, according to community standards. The **`map` to `odom` transform** is provided by a positioning system (localization, mapping, SLAM) and **`odom` to `base_link`** by an odometry system.

### Standards
[REP 105](https://www.ros.org/reps/rep-0105.html) defines the frames and conventions required for navigation and the larger ROS ecosystem. These conventions should be followed at all times to make use of the rich positioning, odometry, and SLAM projects available in the community.

![[Pasted image 20240123104756.png]]

In a nutshell, REP-105 says that you must, at minimum, build a TF tree that contains a full `map` -> `odom` -> `base_link` -> `[sensor frames]` for your robot. TF2 is the time-variant transformation library in ROS 2 we use to represent and obtain time synchronized transformations. It is the job of the global positioning system (GPS, SLAM, Motion Capture) to, at minimum, provide the `map` -> `odom` transformation. It is then the role of the odometry system to provide the `odom` -> `base_link` transformation. The remainder of the transformations relative to `base_link` should be static and defined in your [URDF](http://wiki.ros.org/urdf).

### Global Positioning: Localization and SLAM
It is the job of the global positioning system (GPS, SLAM, Motion Capture) to, at minimum, **provide the `map` -> `odom` transformation**. We provide `amcl` which is an Adaptive Monte-Carlo Localization technique based on a particle filter for localization in a static map. We also provide SLAM Toolbox as the default SLAM algorithm for use to position and generate a static map.

### Odometry
It is the role of the odometry system to provide the **`odom` -> `base_link` transformation**. Odometry can come from many sources including **LIDAR, RADAR, wheel encoders, VIO, and IMUs**. The goal of the odometry is to provide a smooth and continuous local frame based on robot motion. The global positioning system will update the transformation relative to the global frame to account for the odometric drift.

[Robot Localization](https://github.com/cra-ros-pkg/robot_localization/) is typically used for this fusion. It will take in `N` sensors of various types and provide a continuous and smooth odometry to TF and to a topic. A typical mobile robotics setup may have odometry from wheel encoders, IMUs, and vision fused in this manner.

## Environmental Representation
The environmental representation is the way the robot perceives its environment. It also acts as the central localization for various algorithms and data sources to **combine their information into a single space**. This space is then used by the controllers, planners, and recoveries to **compute their tasks safely and efficiently.**

### Costmaps and Layers
 A costmap is a regular 2D grid of cells containing a cost from **unknown, free, occupied, or inflated cost.** Costmap layers can be created to **detect and track obstacles** in the scene for collision avoidance using camera or depth sensors. Additionally, layers can be created to algorithmically change the underlying costmap based on some rule or heuristic. Finally, they may be used to buffer live data into the 2D or 3D world for binary obstacle marking.

### Costmap Filters
This annotated map is called “filter mask”. Costmap filters are a costmap **layer-based **approach of **applying spatial-dependent behavioral changes**. For example:
- Keep-out/safety zones where robots will never enter.
- Speed restriction areas. Maximum speed of robots going inside those areas will be limited.
- Preferred lanes for robots moving in industrial environments and warehouses.

### Other Forms
Various other forms of environmental representations exist. These include:
- **gradient maps,** which are similar to costmaps but represent surface gradients to check traversibility over
- **3D costmaps**, which represent the space in 3D, but then also requires 3D planning and collision checking
- **Mesh maps**, which are similar to gradient maps but with surface meshes at many angles
- “**Vector space”**, taking in sensor information and using machine learning to detect individual items and locations to track rather than buffering discrete points.

# Setting Up Navigation Plugins

## Planner Server
The algorithm plugins for the planner server find the robot’s path using a representation of its environment captured by its different sensors. Some of these algorithms operate by searching through the environment’s grid space while others expand the robot’s possible states while accounting for path feasibility.
As mentioned, the planner server may utilize plugins that work on the grid space such as the `NavFn Planner`, `Smac Planner 2D`, and `Theta Star Planner`. The [NavFn planner](https://navigation.ros.org/configuration/packages/configuring-navfn.html) is a navigation function planner that uses either Dijkstra or A*. Next, the [Smac 2D planner](https://navigation.ros.org/configuration/packages/configuring-smac-planner.html) implements a 2D A* algorithm using 4 or 8 connected neighborhoods with a smoother and multi-resolution query. Lastly, the [Theta Star planner](https://navigation.ros.org/configuration/packages/configuring-thetastar.html#) is an implementation of Theta* using either line of sight to create non-discretely oriented path segments.

| Plugin Name            | Supported Robot Types                                               |
| ---------------------- | ------------------------------------------------------------------- |
 | NavFn Planner          | Circular Differential, Circular Omnidirectional                     | 
| Smac Planner 2D        | Circular Differential, Circular Omnidirectional                     |
| Theta Star Planner     | Circular Differential, Circular Omnidirectional                     |
| Smac Hybrid-A* Planner | Non-circular or Circular Ackermann, Non-circular or Circular Legged |
| Smac Lattice Planner   | Non-circular Differential, Non-circular Omnidirectional             |

## Controller Server 
The default controller plugin is the [DWB controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html). It implements a modified Dynamic Window Approach (DWA) algorithm with configurable plugins to compute the control commands for the robot. This controller makes use of a `Trajectory Generator plugin` that **generates the set of possible trajectories**. These are then evaluated by one or more `Critic plugins`, each of which may give a different score based on how they are configured. The sum of the scores from these `Critic plugins` determine the overall score of a trajectory. **The best scoring trajectory then determines the output command velocity**.

| Plugin Name    | Supported Robot Types                            | Task                       |
| -------------- | ------------------------------------------------ | -------------------------- |
| DWB controller | Differential, Omnidirectional                    | Dynamic obstacle avoidance |
| TEB Controller | Differential, Omnidirectional, Ackermann, Legged | Dynamic obstacle avoidance |
| RPP controller | Differential, Ackermann, Legged                  | Exact path following       |

# [Nav2 Launch Options](https://navigation.ros.org/tuning/index.html#nav2-launch-options)
Nav2’s launch files are made to be very configurable. Obviously for any serious application, a user should use `nav2_bringup` as the basis of their navigation launch system, but should be moved to a specific repository for a users’ work. A typical thing to do is to have a `<robot_name>_nav` configuration package containing the launch and parameter files.

Within `nav2_bringup`, there is a main entryfile `tb3_simulation_launch.py`. This is the main file used for simulating the robot and contains the following configurations:

- `slam` : Whether or not to use AMCL or SLAM Toolbox for localization and/or mapping. Default `false` to AMCL.
    
- `map` : The filepath to the map to use for navigation. Defaults to `map.yaml` in the package’s `maps/` directory.
    
- `world` : The filepath to the world file to use in simulation. Defaults to the `worlds/` directory in the package.
    
- `params_file` : The main navigation configuration file. Defaults to `nav2_params.yaml` in the package’s `params/` directory.
    
- `autostart` : Whether to autostart the navigation system’s lifecycle management system. Defaults to `true` to transition up the Nav2 stack on creation to the activated state, ready for use.
    
- `use_composition` : Whether to launch each Nav2 server into individual processes or in a single composed node, to leverage savings in CPU and memory. Default `true` to use single process Nav2.
    
- `use_respawn` : Whether to allow server that crash to automatically respawn. When also configured with the lifecycle manager, the manager will transition systems back up if already activated and went down due to a crash. Only works in non-composed bringup since all of the nodes are in the same process / container otherwise.
    
- `use_sim_time` : Whether to set all the nodes to use simulation time, needed in simulation. Default `true` for simulation.
    
- `rviz_config_file` : The filepath to the rviz configuration file to use. Defaults to the `rviz/` directory’s file.
    
- `use_simulator` : Whether or not to start the Gazebo simulator with the Nav2 stack. Defaults to `true` to launch Gazebo.
    
- `use_robot_state_pub` : Whether or not to start the robot state publisher to publish the robot’s URDF transformations to TF2. Defaults to `true` to publish the robot’s TF2 transformations.
    
- `use_rviz` : Whether or not to launch rviz for visualization. Defaults to `true` to show rviz.
    
- `headless` : Whether or not to launch the Gazebo front-end alongside the background Gazebo simulation. Defaults to `true` to display the Gazebo window.
    
- `namespace` : The namespace to launch robots into, if need be.
    
- `use_namespace` : Whether or not to launch robots into this namespace. Default `false` and uses global namespace for single robot.
    
- `robot_name` : The name of the robot to launch.
    
- `robot_sdf` : The filepath to the robot’s gazebo configuration file containing the Gazebo plugins and setup to simulate the robot system.
    
- `x_pose`, `y_pose`, `z_pose`, `roll`, `pitch`, `yaw` : Parameters to set the initial position of the robot in the simulation.
