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
        
        else if (msg->name == "charge_battery") {
            message.response = event_charge_battery_();
        }
        
        else if (msg->name == "low_battery") {
            message.response = event_low_battery_();
        }
        
        else if (msg->name == "reinitialize_home") {
            message.response = event_reinitialize_home_();
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
        
        if (!((resource_move_->current() == MoveState::Idle)))
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

    
    //-------------------------------------------------- charge_battery --------------------------------------------------

    int TurtlebotNode::event_charge_battery_()
    {
        // guard
        
        if (!((resource_battery_status_->current() == BatteryStatusState::Low)))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE;
        }
        
        
        // check effects
        if (!(
             resource_battery_status_->check_next(BatteryStatusState::Normal)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_charge_battery_hook();
        // set effects
        resource_battery_status_->set_next(BatteryStatusState::Normal);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_charge_battery_hook()
    {
    }

    void TurtlebotNode::event_charge_battery()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'charge_battery'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_charge_battery_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
    //-------------------------------------------------- low_battery --------------------------------------------------

    int TurtlebotNode::event_low_battery_()
    {
        // guard
        
        if (!((resource_battery_status_->current() == BatteryStatusState::Normal)))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE;
        }
        
        
        // check effects
        if (!(
             resource_battery_status_->check_next(BatteryStatusState::Low)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_low_battery_hook();
        // set effects
        resource_battery_status_->set_next(BatteryStatusState::Low);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_low_battery_hook()
    {
    }

    void TurtlebotNode::event_low_battery()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'low_battery'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_low_battery_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
    //-------------------------------------------------- reinitialize_home --------------------------------------------------

    int TurtlebotNode::event_reinitialize_home_()
    {
        // guard
        
        
        // check effects
        if (!(
             resource_home_->check_next(HomeState::Lost)
        ))
        {
            return turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE;
        }
        
        // hook
        event_reinitialize_home_hook();
        // set effects
        resource_home_->set_next(HomeState::Lost);
        
        skills_invariants_();
        return turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS;
    }

    void TurtlebotNode::event_reinitialize_home_hook()
    {
    }

    void TurtlebotNode::event_reinitialize_home()
    {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'Anafi' event 'reinitialize_home'");
        auto message = turtlebot_skillset_interfaces::msg::EventResponse();
        message.id = info_;
        message.response = event_reinitialize_home_();
        event_pub_->publish(message);
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
    }

    
}
