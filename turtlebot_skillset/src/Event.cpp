#include "turtlebot_skillset/Node.hpp"

namespace turtlebot_skillset
{

    void TurtlebotNode::event_callback_(const turtlebot_skillset_interfaces::msg::EventRequest::UniquePtr msg)
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' event '%s' request", msg->name.c_str());
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = msg->id;
        message.response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        
        if (msg->name == "authority_to_skill") {
            message.response = event_authority_to_skill_();
        }
        
        else if (msg->name == "authority_to_teleop") {
            message.response = event_authority_to_teleop_();
        }
        
        else if (msg->name == "auto_Home") {
            message.response = event_auto_home_();
        }
        
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
    //-------------------------------------------------- authority_to_skill --------------------------------------------------

    int TurtlebotNode::event_authority_to_skill_()
    {
        // guard
        
        if (!((resource_move_->current() == MoveState::NotMoving)))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE;
        }
        
        
        // check effects
        if (!(
             resource_authority_->check_next(AuthorityState::Skill)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_authority_to_skill_hook();
        // set effects
        resource_authority_->set_next(AuthorityState::Skill);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_authority_to_skill_hook()
    {
    }

    void TurtlebotNode::event_authority_to_skill()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'authority_to_skill'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_authority_to_skill_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
    //-------------------------------------------------- authority_to_teleop --------------------------------------------------

    int TurtlebotNode::event_authority_to_teleop_()
    {
        // guard
        
        
        // check effects
        if (!(
             resource_authority_->check_next(AuthorityState::Teleop)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_authority_to_teleop_hook();
        // set effects
        resource_authority_->set_next(AuthorityState::Teleop);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_authority_to_teleop_hook()
    {
    }

    void TurtlebotNode::event_authority_to_teleop()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'authority_to_teleop'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_authority_to_teleop_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
    //-------------------------------------------------- auto_Home --------------------------------------------------

    int TurtlebotNode::event_auto_home_()
    {
        // guard
        
        
        // check effects
        if (!(
             resource_home_->check_next(HomeState::Initialized)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_auto_home_hook();
        // set effects
        resource_home_->set_next(HomeState::Initialized);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_auto_home_hook()
    {
    }

    void TurtlebotNode::event_auto_home()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'auto_Home'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_auto_home_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
}
