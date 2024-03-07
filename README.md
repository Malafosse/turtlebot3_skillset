# Skill-based architecture for Turtlebot3-Burger 

## Dependencies

[GitLab ONERA Robot Skills](https://gitlab.com/groups/onera-robot-skills)
- robot_language
- Skillset GUI generator
- SkiNet Release
[Tuto ONERA Robot Skills](https://onera-robot-skills.gitlab.io/introduction.html)
![](https://onera-robot-skills.gitlab.io/_images/intro.png)

## Description

`Tb3SkillsetManager` is a ROS2 node written in C++ that manages the skills of a TurtleBot3 robot. It provides functionalities such as navigating to a specific pose and retrieving the initial pose of the robot.

## Features

- **Go To Pose**: The robot can navigate to a specific pose. The remaining distance to the goal is continuously updated during the navigation process. If the goal is reached successfully, a success message is logged. If an error occurs, a failure message is logged.

- **Get Home**: The robot can retrieve its initial pose. The initial pose is published to a ROS2 topic. If the pose is published successfully, a success message is logged. If an error occurs, a failure message is logged.

## Installation

Please ensure that you have ROS2 installed on your system.

1. Clone this repository into your ROS2 workspace's `src` directory.
2. Navigate back to your ROS2 workspace root, then build the workspace with `colcon build`.
3. Source the workspace's setup script with `source install/setup.bash`.

## Usage

To run the `Tb3SkillsetManager` node, use the following command:

```bash
ros2 run your_package tb3_skillset_manager
```

## Model checking
> [!note] 
> Local Verification
> Using the SMT solver z3.
> ```bash
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
>**In the /src directory:**
>Abstract manager class and the ROS messages
>```
>python3 -m robot_language turtlebot.rl -d . -g turtlebot.json
>```
>Python client library
>```
>python3 -m robot_language turtlebot.rl -d . -g turtlebot.json -c
>```
>Skillset GUI
>```
>python3 -m skillset_gui_generator turtlebot.rl -d . -g turtlebot.json
>```
>Specific node for the turtlebot
>```
> python3 -m robot_language turtlebot.rl -d . -g turtlebot.json -p tb3_skillset
> ```

## Launching the Skillset GUI
>[!note] 
>```bash
>ros2 run turtlebot_skillset_gui_widgets turtlebot_skillset_gui_widgets_node -m /skillset_manager
>```
>Where `/skillset_manager` is the name of the skillset node 

## Simulate a Turtlebot in Gazebo: 
Crash course video
[Vidéo crash course](https://www.youtube.com/watch?v=idQb2pB-h2Q) Turtlebot3 - Nav2

[TurtleBot3 Website](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#learn)
[Nav2 Website](https://navigation.ros.org/index.html) see also [[Navigation2 STack.md]]
