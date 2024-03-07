#include "TurtlebotSkillsetNode.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tb3SkillsetManager>());
    rclcpp::shutdown();
    return 0;
}
