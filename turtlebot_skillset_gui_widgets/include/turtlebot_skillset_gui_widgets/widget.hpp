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
    inline void set_go_to_input_x(double input) { go_to_input_.x = input; };
    inline void set_go_to_input_y(double input) { go_to_input_.y = input; };
    inline void set_go_to_input_w(double input) { go_to_input_.w = input; };
    
    
    
    void display_start_get_home();
    inline void set_get_home_inputs(turtlebot_skillset_interfaces::msg::SkillGetHomeInput input) { get_home_input_ = input; };
    inline void set_get_home_input_x(double input) { get_home_input_.x = input; };
    inline void set_get_home_input_y(double input) { get_home_input_.y = input; };
    inline void set_get_home_input_w(double input) { get_home_input_.w = input; };
    
    
    
  private:
    bool display_data_;
    bool display_resources_;
    bool display_events_;

    
    void event_row_authority_to_skill();
    void event_button_authority_to_skill();
    
    void event_row_authority_to_teleop();
    void event_button_authority_to_teleop();
    
    void event_row_auto_Home();
    void event_button_auto_Home();
    
    double event_response_timeout_;

    void skill_response_text(int result_code); 
    void skill_state_button(std::string skill, unsigned int state);
    
    
    bool active_go_to_;
    
    bool active_get_home_;
    
};
