#include "Node.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //auto options = rclcpp::NodeOptions();  // Not sure about that 
    //auto node = std::make_shared<tb3_skillset_ns::Tb3SkillsetNode>();
    auto node = std::make_shared<Tb3SkillsetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
