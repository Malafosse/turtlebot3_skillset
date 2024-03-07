#include "Node.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tb3SkillsetNode>());
    rclcpp::shutdown();
    return 0;
}
