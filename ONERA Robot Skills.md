---
tags:
  - Skills
  - Training
  - TB3
---

# Resources
>[!example] [Tuto ONERA Robot Skills](https://onera-robot-skills.gitlab.io/introduction.html)

>[!example] [GitLab ONERA Robot Skills](https://gitlab.com/groups/onera-robot-skills)

![](https://onera-robot-skills.gitlab.io/_images/intro.png)

# Implementation of the TB3 Skillset 

Created the `turtlebot_rl_ws` containing [turtlebot.rl](file:////home/pmalafosse/workspaces/turtlebot_rl_ws/src/turtlebot.rl) and [turtlebot.json](file:////home/pmalafosse/workspaces/turtlebot_rl_ws/src/turtlebot.json)

>[!warning] 
>Set `"folder": "./"` in `turtlebot.json` for the skillset generator to find `turtlebot.rl` in the src directory.

## Generate Skillset

Generate the base skillset & interfaces containing the generic behavior.
```bash
python3 -m robot_language turtlebot.rl -g turtlebot.json
```

Generate the specific implementation TB3 
```bash
python3 -m robot_language turtlebot.rl -g turtlebot.json -p tb3_skillset
```

Build the packages
```bash
cd ..
colcon build
```

>[!success] It builds !

## Creating the Hooks
![[TB3-NAV2|1000]]

### [Node.hpp](file:////home/pmalafosse/workspaces/turtlebot_rl_ws/src/tb3/src/Node.hpp) 

Contains the class Tb3Node that inherits from SKILLSET_NODE

`#include "geometry_msgs/msg/pose_stamped.hpp"`
>[!example] geometry_msgs/msg/pose_stamped.hpp
>[pose_stamped.hpp](file:///opt/ros/humble/include/geometry_msgs/geometry_msgs/msg/pose_stamped.hpp)

`rclcpp::Client<nav2_msgs::action::NavigateToPose>`
>[!example] nav2_msgs/action/navigate_to_pose 
>[NavigateToPose.action](file:///opt/ros/humble/share/nav2_msgs/action/NavigateToPose.action)
> [navigate_to_pose.hpp](file:///opt/ros/humble/include/nav2_msgs/nav2_msgs/action/navigate_to_pose.hpp)

[ROS2: Writing an action client](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client)

### Node.cpp
Defines Tb3Node's methods

### tb3_node.cpp
Includes Node.cpp and contains the `main()`.

>[!danger] Need an example of skill hooked to an action from Baptiste 
>Baptiste has never done this before. 
>>[!success] Resolved 
>>[[Nav2 Stack & TurtleBot3]] out of the Skill-based scope. 
>>Next step: adapt it here.

# From the Action client to the Skillset's hook

>[!error] colcon build --packages-select tb3_skillset --cmake-clean-first
>fatal error: rclcpp_action/rclcpp_action.hpp: No such file or directory 16 | # include "rclcpp_action/rclcpp_action.hpp"
>![|300](https://media.tenor.com/dnfJcln1SwoAAAAM/luffy-bruh.gif)
>>[!success] 
>> Removed the `rclcpp_components` sources and use in node.hpp, cmakelist and packages.xml.
>>Commented`RCLCPP_COMPONENTS_REGISTER_NODE(tb3_skillset_ns::Tb3SkillsetNode)`

>[!error] 
>`/usr/bin/ld: cannot find -lturtlebot_skillset: No such file or directory collect2: error: ld returned 1 exit status`
> Not a single package was found by the constructor
>>[!success] 
>> You should use `${rclcpp_LIBRARIES}`, `${std_msgs_LIBRARIES}`, etc., in your `target_link_libraries()`.

## Result of the first test
>[!success] ros2 run tb3_skillset tb3_skillset_node
>Enter goal coordinates (x, y, z): 4 0 0
>Enter goal orientation (w): 1
>[INFO] [1707386458.130133193] [tb3_node]: Sending goal
>[INFO] [1707386458.131041494] [tb3_node]: Goal accepted by server, waiting for result
>[INFO] [1707386477.192483502] [tb3_node]: Navigation to pose succeeded
>![|300](https://media.tenor.com/c_YtVvLh34UAAAAM/one-piece.gif)

# Improving the skillset 

## Skillset UI
Launch navigate to pose from the UI by setting parameters.
Cloned the Skillset GUI Generator repo 
```bash
git clone git@gitlab.com:onera-robot-skills/skillset_gui_generator.git
```
>[!error] pip3 install --user skillset_gui_generator
> ERROR: Version None, unable to install
>>[!success] 
>>If the package is not available on PyPI or there is an issue with the PyPI version, you can try installing the package directly from the source code. 
>>Navigate to the root directory of the cloned repository and run:
>>`pip3 install --user .`


In the turtlebot_skillset workspace: 
```
python3 -m skillset_gui_generator turtlebot.rl -g turtlebot.json
```
>[!error]  colcon build --packages-select turtlebot_skillset_gui_widgets 
>CMake Error at CMakeLists.txt:22 (find_package)
>At line 22: find_package(gui_tools REQUIRED)
>>[!error] sudo apt install gui_tools
>>E: Unable to locate package gui_tools
>>![|300](https://media.tenor.com/nNCkdmgUbLkAAAAM/luffy.gif)
>
>>[!success] [gui_tools](https://gitlab.com/robotis/gui_tools/gui_tools)
>>Package sur le gitlab Robotis ONERA privé (Merci Baptiste).
>>Dependency:
>>```
>>sudo apt install libglfw3-dev
>>```
>>New workspace: ros2_ws
>>```
>>git clone --recurse-submodules git@gitlab.com:robotis/gui_tools/gui_tools.git
>>cd ..
>>colcon build --packages-select gui_tools
>>```
>
>>[!error] colcon build --packages-select turtlebot_skillset_gui_widgets 
>>stderr: turtlebot_skillset_gui_widgets /home/pmalafosse/workspaces/turtlebot_rl_ws/src/turtlebot_skillset_gui_widgets/src/main.cpp:3:10: fatal error: **argagg/argagg.hpp**: No such file or directory 3 | **# include <argagg/argagg.hpp**>
>
>>[!success] [argagg](https://github.com/vietjtnguyen/argagg)
>>Manually created `/usr/local/include/argagg/argagg.hpp` with [argagg.hpp](https://github.com/vietjtnguyen/argagg/blob/master/include/argagg/argagg.hpp)
>
>==The package *turtlebot_skillset_gui_widgets* builds now !==

## Lunching the UI
```
ros2 run turtlebot_skillset_gui_widgets turtlebot_skillset_gui_widgets_node 
```
The UI works nicely and is able to display the two skills I have defined in my skillset. The parameter values are modifiable and the events can be triggered.
<mark class=orange>Feedbacks from the events don't seem to be consistent with the content of the hooks though.

>[!warning] ToDo
> - Rewrite the hooks so the skillset actually uses the parameters in .msg instead of prompting the values to the user.
> - Remove the GoTo from the skillset's constructor
>
>![ToBeContinued|300](https://media.tenor.com/5GvDbYxG-lQAAAAM/to-be-continued-one-piece.gif)
>>[!success] Modifications done.

>[!warning] Where are the GUI parameters stored ?
>Need to see Baptiste to ask him how he does it.
><mark class=green>Very simple: `this->skill_get_home_input()->x` does the trick.

>[!error] Other issue: The skill on start does nothing...
>>[!warning] May be because of this in turtlebot.rl:
>>resource
>>```
>>Authority {
>>state { Teleop Skill }
>>initial Teleop
>>transition all
>>}
>>```
>>event
>>```
>>authority_to_skill {
>>guard Authority == Teleop
>>effect Authority -> Skill
>>}
>>```
>>
>>Possible de regénérer les packages sans perdre le contenu ? 
>>-> Backup nécessaire...
>>Changement  fait mais insuffisant. 
>>Pas d'effet visible
>
>>[!warning] Écrire les hooks manquants 
>>`bool skill_go_to_validate_hook();`
>>`void skill_go_to_start_hook();`
>>
>><mark class=red> Besoin d'un example</mark>
>><mark class=green> Accès à Robotis/SPOT ! (merci Baptiste)
>>/home/pmalafosse/workspaces/spot_skillset_cohoma-master</mark>

## Notes sur le [skillset Spot Cohoma](file:///home/pmalafosse/workspaces/spot_skillset_cohoma-master)

### Node.hpp[](file:///home/pmalafosse/workspaces/spot_skillset_cohoma-master/include/Node.hpp)
Définition of `class SpotSkillsetCohomaNode : public SKILLSET_NODE`
Nothing in the constructor !
- Clients
- Skills
- Skill timers
- Lease
- navigation params

### SpotNode[](file:///home/pmalafosse/workspaces/spot_skillset_cohoma-master)
Exemple de skill_validate_hook: `skill_go_to_graphnav_validate_hook`
```cpp
bool SpotSkillsetCohomaNode::skill_go_to_graphnav_validate_hook()
{
if (graphnav_not_setup_){
std::cout << "Graphnav server was never setup. Cannot use go_to_graphnav skill." << std::endl;
return false;
}
else {return true;}
}
```
>[!question] 
>Where is the value of `graphnav_not_setup_` changed ? 
>![ToBeContinued|300](https://media.tenor.com/5GvDbYxG-lQAAAAM/to-be-continued-one-piece.gif)



### SpotNodeDebug[](file:///home/pmalafosse/workspaces/spot_skillset_cohoma-master)
<mark class=red> I dont have access to the debug node :)</mark>


## Initial Pose
Define the initial pose from the skillset

## Navigate to Goal
Improve the options and feedback readability
