#ifndef TURTLEBOT_NODE_DEBUG_HPP
#define TURTLEBOT_NODE_DEBUG_HPP

#include "Node.hpp"

namespace turtlebot_skillset
{
    class TurtlebotNodeDebug: public TurtlebotNode
    {
    public:
        TurtlebotNodeDebug(const std::string node_name, const std::string info);
        virtual ~TurtlebotNodeDebug() {}

    private:
        
        void resource_authority_hook_(const rclcpp::Parameter &parameter);
        std::shared_ptr<rclcpp::ParameterEventHandler> resource_authority_handler_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> resource_authority_handle_;
        
        void resource_move_hook_(const rclcpp::Parameter &parameter);
        std::shared_ptr<rclcpp::ParameterEventHandler> resource_move_handler_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> resource_move_handle_;
        
        void resource_home_hook_(const rclcpp::Parameter &parameter);
        std::shared_ptr<rclcpp::ParameterEventHandler> resource_home_handler_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> resource_home_handle_;
        
    };
}
#endif
