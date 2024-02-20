#include "turtlebot_skillset/Node.hpp"

namespace turtlebot_skillset
{

    TurtlebotNode::TurtlebotNode(const std::string node_name, const std::string info)
        : Node(node_name)
        , info_(info)
        , resource_authority_(std::make_shared<Authority>())
        , resource_move_(std::make_shared<Move>())
        , resource_home_(std::make_shared<Home>())
        , skill_go_to_state_(SkillState::Ready)
        , skill_go_to_id_("")
        , skill_go_to_input_(std::make_shared<turtlebot_skillset_interfaces::msg::SkillGoToInput>()) 
        , skill_get_home_state_(SkillState::Ready)
        , skill_get_home_id_("")
        , skill_get_home_input_(std::make_shared<turtlebot_skillset_interfaces::msg::SkillGetHomeInput>()) 
        {
        //-------------------- QoS --------------------
        auto qos_soft = rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile();
        auto qos_hard = rclcpp::QoS(1).reliable().keep_last(1).transient_local();
        auto qos_event = rclcpp::QoS(1).reliable().keep_all().durability_volatile();
        //-------------------- Skillset --------------------
        status_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "~/turtlebot_skillset/status_request", qos_event, 
            [this](std_msgs::msg::Empty::UniquePtr msg) { 
                this->TurtlebotNode::status_callback_(std::move(msg)); 
            });
        event_sub_ = this->create_subscription<turtlebot_skillset_interfaces::msg::EventRequest>(
            "~/turtlebot_skillset/event_request", qos_event, 
            [this](turtlebot_skillset_interfaces::msg::EventRequest::UniquePtr msg) {
                this->TurtlebotNode::event_callback_(std::move(msg));
            });
        status_pub_ = this->create_publisher<turtlebot_skillset_interfaces::msg::SkillsetStatus>(
            "~/turtlebot_skillset/status", qos_event);
        event_pub_ = this->create_publisher<turtlebot_skillset_interfaces::msg::EventResponse>(
            "~/turtlebot_skillset/event_response", qos_event);
        //-------------------- Data --------------------
        
        //-------------------- Skill --------------------
        
        //---------- Skill GoTo ----------
        skill_go_to_request_sub_ = this->create_subscription<turtlebot_skillset_interfaces::msg::SkillGoToRequest>(
            "~/turtlebot_skillset/skill/go_to/request", qos_event, 
            [this](turtlebot_skillset_interfaces::msg::SkillGoToRequest::UniquePtr msg) { 
                this->TurtlebotNode::skill_go_to_callback_(std::move(msg)); 
            });
        skill_go_to_response_pub_ = this->create_publisher<turtlebot_skillset_interfaces::msg::SkillGoToResponse>(
            "~/turtlebot_skillset/skill/go_to/response", qos_event);
        
        skill_go_to_interrupt_sub_ = this->create_subscription<turtlebot_skillset_interfaces::msg::SkillInterrupt>(
            "~/turtlebot_skillset/skill/go_to/interrupt", qos_event, 
            [this](turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg) { 
                this->TurtlebotNode::skill_go_to_interrupt_callback_(std::move(msg)); 
            });
        
        //---------- Skill getHome ----------
        skill_get_home_request_sub_ = this->create_subscription<turtlebot_skillset_interfaces::msg::SkillGetHomeRequest>(
            "~/turtlebot_skillset/skill/get_home/request", qos_event, 
            [this](turtlebot_skillset_interfaces::msg::SkillGetHomeRequest::UniquePtr msg) { 
                this->TurtlebotNode::skill_get_home_callback_(std::move(msg)); 
            });
        skill_get_home_response_pub_ = this->create_publisher<turtlebot_skillset_interfaces::msg::SkillGetHomeResponse>(
            "~/turtlebot_skillset/skill/get_home/response", qos_event);
        
        skill_get_home_interrupt_sub_ = this->create_subscription<turtlebot_skillset_interfaces::msg::SkillInterrupt>(
            "~/turtlebot_skillset/skill/get_home/interrupt", qos_event, 
            [this](turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg) { 
                this->TurtlebotNode::skill_get_home_interrupt_callback_(std::move(msg)); 
            });
        
    }

    turtlebot_skillset_interfaces::msg::SkillsetStatus TurtlebotNode::status_() const {
        auto message = turtlebot_skillset_interfaces::msg::SkillsetStatus();
        // Stamp
        message.stamp = rclcpp::Node::now();
        // resources
        auto rs = turtlebot_skillset_interfaces::msg::ResourceState();
        
        rs = turtlebot_skillset_interfaces::msg::ResourceState();
        rs.name = "Authority";
        rs.state = to_string(resource_authority_->current());
        message.resources.push_back(rs);
        
        rs = turtlebot_skillset_interfaces::msg::ResourceState();
        rs.name = "Move";
        rs.state = to_string(resource_move_->current());
        message.resources.push_back(rs);
        
        rs = turtlebot_skillset_interfaces::msg::ResourceState();
        rs.name = "Home";
        rs.state = to_string(resource_home_->current());
        message.resources.push_back(rs);
        
        // skills
        
        message.skill_go_to = turtlebot_skillset_interfaces::msg::SkillGoToStatus();
        message.skill_go_to.name = "GoTo";
        message.skill_go_to.id = skill_go_to_id_;
        switch (skill_go_to_state_)
        {
        case SkillState::Ready:
            message.skill_go_to.state = turtlebot_skillset_interfaces::msg::SkillGoToStatus::READY;
            break;        
        case SkillState::Running:
            message.skill_go_to.state = turtlebot_skillset_interfaces::msg::SkillGoToStatus::RUNNING;
            break;        
        case SkillState::Interrupting:
            message.skill_go_to.state = turtlebot_skillset_interfaces::msg::SkillGoToStatus::INTERRUPTING;
            break;        
        default:
            break;
        }
        message.skill_go_to.input = *skill_go_to_input_;
        
        message.skill_get_home = turtlebot_skillset_interfaces::msg::SkillGetHomeStatus();
        message.skill_get_home.name = "getHome";
        message.skill_get_home.id = skill_get_home_id_;
        switch (skill_get_home_state_)
        {
        case SkillState::Ready:
            message.skill_get_home.state = turtlebot_skillset_interfaces::msg::SkillGetHomeStatus::READY;
            break;        
        case SkillState::Running:
            message.skill_get_home.state = turtlebot_skillset_interfaces::msg::SkillGetHomeStatus::RUNNING;
            break;        
        case SkillState::Interrupting:
            message.skill_get_home.state = turtlebot_skillset_interfaces::msg::SkillGetHomeStatus::INTERRUPTING;
            break;        
        default:
            break;
        }
        message.skill_get_home.input = *skill_get_home_input_;
        
        // Info
        message.info = info_;
        return message;
    }

    void TurtlebotNode::status_callback_(const std_msgs::msg::Empty::UniquePtr msg)
    {
        mutex_.lock();
        (void)msg;
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' status request");
        auto message = status_();
        status_pub_->publish(message);
        mutex_.unlock();
    }

    //-------------------------------------------------- Resources --------------------------------------------------
    
    std::string TurtlebotNode::get_authority_state()
    {
        mutex_.lock();
        std::string state = to_string(resource_authority_->current());
        mutex_.unlock();
        return state;
    }
    std::string TurtlebotNode::get_authority_state_hook()
    {
        std::string state = to_string(resource_authority_->current());
        return state;
    }
    
    std::string TurtlebotNode::get_move_state()
    {
        mutex_.lock();
        std::string state = to_string(resource_move_->current());
        mutex_.unlock();
        return state;
    }
    std::string TurtlebotNode::get_move_state_hook()
    {
        std::string state = to_string(resource_move_->current());
        return state;
    }
    
    std::string TurtlebotNode::get_home_state()
    {
        mutex_.lock();
        std::string state = to_string(resource_home_->current());
        mutex_.unlock();
        return state;
    }
    std::string TurtlebotNode::get_home_state_hook()
    {
        std::string state = to_string(resource_home_->current());
        return state;
    }
    

    //-------------------------------------------------- Skillset Status --------------------------------------------------
    turtlebot_skillset_interfaces::msg::SkillsetStatus TurtlebotNode::get_skillset_status()
    {
        mutex_.lock();
        auto status = this->status_();
        mutex_.unlock();
        return status;
    }
}
