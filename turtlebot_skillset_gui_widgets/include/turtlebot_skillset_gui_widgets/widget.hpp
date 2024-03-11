#pragma once

#include "client.hpp"


class TurtlebotSkillsetWidget: public TurtlebotSkillsetClient
{
  public:
    TurtlebotSkillsetWidget(const std::string &name, rclcpp::Node::SharedPtr node,
      bool display_data = true, bool display_resources = true, bool display_events = true);

    void update();
    bool update_window();
    void process();
    
    void display_start_go_to();
    inline void set_go_to_inputs(turtlebot_skillset_interfaces::msg::SkillGoToInput input) { go_to_input_ = input; };
    inline void set_go_to_input_x(std_msgs::msg::Float64 input) { go_to_input_.x = input; };
    inline void set_go_to_input_y(std_msgs::msg::Float64 input) { go_to_input_.y = input; };
    inline void set_go_to_input_theta(std_msgs::msg::Float64 input) { go_to_input_.theta = input; };
    
    
    
    void display_start_get_home();
    inline void set_get_home_inputs(turtlebot_skillset_interfaces::msg::SkillGetHomeInput input) { get_home_input_ = input; };
    inline void set_get_home_input_x(std_msgs::msg::Float64 input) { get_home_input_.x = input; };
    inline void set_get_home_input_y(std_msgs::msg::Float64 input) { get_home_input_.y = input; };
    inline void set_get_home_input_theta(std_msgs::msg::Float64 input) { get_home_input_.theta = input; };
    
    
    
  private:
    bool display_data_;
    bool display_resources_;
    bool display_events_;

    
    void event_row_authority_to_skill();
    void event_button_authority_to_skill();
    
    void event_row_authority_to_teleop();
    void event_button_authority_to_teleop();
    
    void event_row_charge_battery();
    void event_button_charge_battery();
    
    void event_row_low_battery();
    void event_button_low_battery();
    
    void event_row_reinitialize_home();
    void event_button_reinitialize_home();
    
    double event_response_timeout_;

    void skill_response_text(int result_code); 
    void skill_state_button(std::string skill, unsigned int state);
    
    bool subscribe_currentpose_;
    
    
    bool active_go_to_;
    
    bool active_get_home_;
    
};
