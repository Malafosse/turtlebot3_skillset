#pragma once


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlebot_skillset_interfaces/msg/skillset_status.hpp>
#include <turtlebot_skillset_interfaces/msg/event_request.hpp>
#include <turtlebot_skillset_interfaces/msg/event_response.hpp>
#include <turtlebot_skillset_interfaces/msg/data_request.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_interrupt.hpp>


#include <turtlebot_skillset_interfaces/msg/skill_go_to_status.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_go_to_input.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_go_to_progress.hpp>

#include <turtlebot_skillset_interfaces/msg/skill_go_to_request.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_go_to_response.hpp>

#include <turtlebot_skillset_interfaces/msg/skill_get_home_status.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_get_home_input.hpp>


#include <turtlebot_skillset_interfaces/msg/skill_get_home_request.hpp>
#include <turtlebot_skillset_interfaces/msg/skill_get_home_response.hpp>


class TurtlebotSkillsetClient
{
  public:
    TurtlebotSkillsetClient(const std::string &name, rclcpp::Node::SharedPtr node);
    inline turtlebot_skillset_interfaces::msg::SkillsetStatus get_status() { return status_; };

    //----- skills
    
    std::string start_go_to();
    void interrupt_go_to(std::string id);
    void interrupt_go_to();
    
    std::string start_get_home();
    void interrupt_get_home(std::string id);
    void interrupt_get_home();
    
    //----- data getters
    
    //----- resource getters
    
    inline std::string get_resource_authority() const { return resource_state_.at("authority"); };
    
    inline std::string get_resource_move() const { return resource_state_.at("move"); };
    
    inline std::string get_resource_home() const { return resource_state_.at("home"); };
    
    inline std::string get_resource_battery_status() const { return resource_state_.at("battery_status"); };
    

  protected:
    std::string name_;
    rclcpp::Node::SharedPtr node_;

    //----- status
    turtlebot_skillset_interfaces::msg::SkillsetStatus status_;
    void request_status();
    double time_since_status();
    //----- events
    std::map<std::string, turtlebot_skillset_interfaces::msg::EventResponse> events_;
    std::map<std::string, std::string> events_ids_;
    std::map<std::string, rclcpp::Time> events_stamps_;
    std::string send_event(std::string event);
    double time_since_event(std::string event) const;
    //----- data
    
    //----- resources
    std::map<std::string, std::string> resource_state_;
    //----- skills
    
    turtlebot_skillset_interfaces::msg::SkillGoToStatus go_to_status_;
    turtlebot_skillset_interfaces::msg::SkillGoToInput go_to_input_;
    turtlebot_skillset_interfaces::msg::SkillGoToProgress go_to_progress_;
    
    turtlebot_skillset_interfaces::msg::SkillGoToResponse go_to_result_;
    
    turtlebot_skillset_interfaces::msg::SkillGetHomeStatus get_home_status_;
    turtlebot_skillset_interfaces::msg::SkillGetHomeInput get_home_input_;
    
    
    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse get_home_result_;
    

  private:
    rclcpp::QoS qos_best_;
    rclcpp::QoS qos_reliable_;

    std::string generate_id() const;

    //----- status
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr status_pub_;
    rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillsetStatus>::SharedPtr status_sub_;
    void status_callback_(const turtlebot_skillset_interfaces::msg::SkillsetStatus::SharedPtr msg);
    //----- events
    rclcpp::Publisher<turtlebot_skillset_interfaces::msg::EventRequest>::SharedPtr event_pub_;
    rclcpp::Subscription<turtlebot_skillset_interfaces::msg::EventResponse>::SharedPtr event_sub_;
    void event_callback_(const turtlebot_skillset_interfaces::msg::EventResponse::SharedPtr msg);
    //----- data
    
    //----- skills
    
    rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillGoToRequest>::SharedPtr go_to_request_pub_;
    rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillInterrupt>::SharedPtr go_to_interrupt_pub_;
    rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillGoToResponse>::SharedPtr go_to_response_sub_;
    void go_to_response_callback(const turtlebot_skillset_interfaces::msg::SkillGoToResponse::SharedPtr msg);
    
    rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillGoToProgress>::SharedPtr go_to_progress_sub_;
    void go_to_progress_callback(const turtlebot_skillset_interfaces::msg::SkillGoToProgress::SharedPtr msg);
    
    
    rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillGetHomeRequest>::SharedPtr get_home_request_pub_;
    rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillInterrupt>::SharedPtr get_home_interrupt_pub_;
    rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillGetHomeResponse>::SharedPtr get_home_response_sub_;
    void get_home_response_callback(const turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SharedPtr msg);
    
    
};
