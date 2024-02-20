#include "turtlebot_skillset/NodeDebug.hpp"

namespace turtlebot_skillset
{

    TurtlebotNodeDebug::TurtlebotNodeDebug(const std::string node_name, const std::string info) : TurtlebotNode(node_name, info)
    {
        
        //---------- Resource Authority ----------
        this->declare_parameter("resource_authority", "Teleop");
        resource_authority_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        resource_authority_handle_ = resource_authority_handler_->add_parameter_callback("resource_authority",
            [this](const rclcpp::Parameter &parameter)
            {
                    this->resource_authority_hook_(parameter);
            });
        
        //---------- Resource Move ----------
        this->declare_parameter("resource_move", "NotMoving");
        resource_move_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        resource_move_handle_ = resource_move_handler_->add_parameter_callback("resource_move",
            [this](const rclcpp::Parameter &parameter)
            {
                    this->resource_move_hook_(parameter);
            });
        
        //---------- Resource Home ----------
        this->declare_parameter("resource_home", "Lost");
        resource_home_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        resource_home_handle_ = resource_home_handler_->add_parameter_callback("resource_home",
            [this](const rclcpp::Parameter &parameter)
            {
                    this->resource_home_hook_(parameter);
            });
        
    }

    
    void TurtlebotNodeDebug::resource_authority_hook_(const rclcpp::Parameter &parameter)
    {
        auto state = parameter.as_string();
        
        if (state == "Teleop")
        {
            this->resource_authority_->set_next(AuthorityState::Teleop);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else if (state == "Skill")
        {
            this->resource_authority_->set_next(AuthorityState::Skill);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "resource 'Authority' state '%s' undefined", state.c_str());
        }
    }
    
    void TurtlebotNodeDebug::resource_move_hook_(const rclcpp::Parameter &parameter)
    {
        auto state = parameter.as_string();
        
        if (state == "Moving")
        {
            this->resource_move_->set_next(MoveState::Moving);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else if (state == "NotMoving")
        {
            this->resource_move_->set_next(MoveState::NotMoving);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "resource 'Move' state '%s' undefined", state.c_str());
        }
    }
    
    void TurtlebotNodeDebug::resource_home_hook_(const rclcpp::Parameter &parameter)
    {
        auto state = parameter.as_string();
        
        if (state == "Lost")
        {
            this->resource_home_->set_next(HomeState::Lost);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else if (state == "Initializing")
        {
            this->resource_home_->set_next(HomeState::Initializing);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else if (state == "Initialized")
        {
            this->resource_home_->set_next(HomeState::Initialized);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "resource 'Home' state '%s' undefined", state.c_str());
        }
    }
    
}
