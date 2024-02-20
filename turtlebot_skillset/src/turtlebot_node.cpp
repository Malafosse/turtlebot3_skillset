#include "turtlebot_skillset/Node.hpp"

#include <iostream>

using namespace turtlebot_skillset;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotNode>("turtlebot", "generated skillset"));
    rclcpp::shutdown();
    return 0;
}
