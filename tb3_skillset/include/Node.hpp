#ifndef TB3_SKILLSET_NODE_HPP
#define TB3_SKILLSET_NODE_HPP


#if defined(SKILLSET_DEBUG_MODE)
#include "turtlebot_skillset/NodeDebug.hpp"
#define SKILLSET_NODE turtlebot_skillset::TurtlebotNodeDebug
#else
#include "turtlebot_skillset/Node.hpp"
#define SKILLSET_NODE turtlebot_skillset::TurtlebotNode
#endif
using namespace std::chrono_literals;

class Tb3SkillsetNode : public SKILLSET_NODE
{
public:
    Tb3SkillsetNode();
    //-------------------- Event Hook --------------------
    // void event_authority_to_skill_hook();
    // void event_authority_to_teleop_hook();
    // void event_charge_battery_hook();
    // void event_low_battery_hook();
    // void event_reinitialize_home_hook();
    // void event_enable_exploration_hook();
    // void event_enable_navigation_hook();
    
    //-------------------- Skill Hook --------------------
    // bool skill_go_to_validate_hook();
    // void skill_go_to_start_hook();
    // void skill_go_to_on_start();
    // void skill_go_to_invariant_authority_to_skill_hook();
    // void skill_go_to_invariant_battery_ok_hook();
    // void skill_go_to_invariant_nav_enabled_hook();
    // turtlebot_skillset_interfaces::msg::SkillGoToProgress skill_go_to_progress_hook();
    // void skill_go_to_interrupt_hook();
    
    // void skill_go_to_on_interrupting();
    
    
    // bool skill_get_home_validate_hook();
    // void skill_get_home_start_hook();
    // void skill_get_home_on_start();
    
    // void skill_get_home_interrupt_hook();
    
    
    // bool skill_take_picture_validate_hook();
    // void skill_take_picture_start_hook();
    // void skill_take_picture_on_start();
    
    // void skill_take_picture_interrupt_hook();
    
    
    // bool skill_explore_validate_hook();
    // void skill_explore_start_hook();
    // void skill_explore_on_start();
    // void skill_explore_invariant_authority_to_skill_hook();
    // void skill_explore_invariant_battery_ok_hook();
    // void skill_explore_invariant_exploration_enabled_hook();
    
    // void skill_explore_interrupt_hook();
    
    // void skill_explore_on_interrupting();
    
    
    // bool skill_save_map_validate_hook();
    // void skill_save_map_start_hook();
    // void skill_save_map_on_start();
    
    // void skill_save_map_interrupt_hook();
    
    
};
#endif
