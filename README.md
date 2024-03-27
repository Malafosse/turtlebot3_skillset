# Skill-based architecture for Turtlebot3-Burger 

## Dependencies

[GitLab ONERA Robot Skills](https://gitlab.com/groups/onera-robot-skills)
- *[Compulsory]* robot_language
- *[Recommended]* Skillset GUI generator
- *[Optional]* SkiNet Release

[Tuto ONERA Robot Skills](https://onera-robot-skills.gitlab.io/introduction.html)
![](https://onera-robot-skills.gitlab.io/_images/intro.png)

## Description

`tb3_skillset` is a ROS2 node written in C++ that manages the *skills* of a TurtleBot3 robot. The architetcure provides functionalities such as navigating to a specific pose and retrieving the initial pose of the robot. In a 3-layer architecture, the skillset model stands as the executive layer, as an intermediary between a decisional layer and the functionnal layer of the robot. 

The skillset model is expressed in Robot Language (RL), a Domain Specific Language (DSL). You can find its definition in `turtlebot.rl` and the declaration of some message types it uses in `turtlebot.json`. From these two files, the skill-based architecture can be generated using the `robot_language` package. Formal verification can also be performed on the model using `SkiNet`. Finally, the `Skillset GUI Generator` allows the user to trigger events and *skills* at run time from a dedicated interface. 

The `robot_language` package generates an abstract manager class that implements the rules specified in RL and the necessary ROS2 message interfaces. 
It is also possible to generate a python client library that can be used as a high level pilot for the skillset implementation. 
Alternatively, the user can use the GUI prvided by the `Skillset GUI Generator` library.

In addition to the generated code, "hooks" have been implemented in the `tb3_skillset` node. These hooks are responsible for establishing the platform-specific communication with the functionnal layer of the robot composed of the [Nav2 Stack](https://navigation.ros.org/index.html) and the turtlebot's interface nodes. As the different elements of the architecture communicate through ROS2, the hooks have the role of creating the adequate ROS2 publishers, subscribers and clients and handling any event or fault state.   

## Features / *skills*

- **Go To Pose**: The robot can navigate to a specific pose. The remaining distance to the goal is continuously updated during the navigation process. If the goal is reached successfully, a success message is logged. If an error occurs, a failure message is logged.

- **Get Home**: The robot can retrieve its initial pose. The initial pose is published to a ROS2 topic. If the pose is published successfully, a success message is logged. If an error occurs, a failure message is logged.

## Installation

Please ensure that you have ROS2 installed on your system.

1. Clone this repository into your ROS2 workspace's `src` directory.
2. Navigate back to your ROS2 workspace root, then build the workspace with `colcon build`.
3. Don't forget to source the workspace's setup script with `source install/setup.bash`.


## *[Optional]* Model checking
> [!note] 
> Local Verification of the model
>
> Using the SMT solver z3.
> ```
> python3 -m robot_language ~/path-to/turtlebot.rl -v 2
> ```
> Visualize the Petri net 
> ```
> python3 -m skinet ~/path-to/turtlebot.rl -nd
> ```
> Global Verification
> ```
> python3 -m skinet ~/path-to/turtlebot.rl -ktz -a
> ```

## Code generation

>[!note] 
>**In the  `/src` directory:**
>
>*[Compulsory]* Abstract manager class and the ROS messages
>```
>python3 -m robot_language turtlebot.rl -d . -g turtlebot.json
>```
>*[Optional]* Python client library
>```
>python3 -m robot_language turtlebot.rl -d . -g turtlebot.json -c
>```
>*[Recommended]* Skillset GUI
>```
>python3 -m skillset_gui_generator turtlebot.rl -d . -g turtlebot.json
>```
>*[Optionnal]* If you ever need to regenerate a clean tb3_skillset node: 
>```
> python3 -m robot_language turtlebot.rl -d . -g turtlebot.json -p tb3_skillset
> ```

## Usage
>[!note] 
>Run the `tb3 skillset node`:
>```
>ros2 run tb3_skillset tb3_skillset_node
>```
>*[Recommended]* Run the `Skillset Manager GUI`
>```
>ros2 run turtlebot_skillset_gui_widgets turtlebot_skillset_gui_widgets_node -m /skillset_manager
>```
>You may need to adjust `/skillset_manager` depending on the name of the skillset node 

![](tb3_skillset_prev.png)

## Control a real TurtleBot3 or simulate one in Gazebo: 
[Crash course video](https://www.youtube.com/watch?v=idQb2pB-h2Q) Turtlebot3 - Nav2

[TurtleBot3 Website](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#learn)

[Nav2 Website](https://navigation.ros.org/index.html) alternatively, you can check out [my notes](Navigation2Stack.md).

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```
