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
    
    //-------------------- Skill Hook --------------------
    // bool skill_go_to_validate_hook();
    // void skill_go_to_start_hook();
    // void skill_go_to_on_start();
    // void skill_go_to_invariant_authority_to_skill_hook();
    // void skill_go_to_invariant_battery_ok_hook();
    // turtlebot_skillset_interfaces::msg::SkillGoToProgress skill_go_to_progress_hook();
    // void skill_go_to_interrupt_hook();
    
    // void skill_go_to_on_interrupting();
    
    
    // bool skill_get_home_validate_hook();
    // void skill_get_home_start_hook();
    // void skill_get_home_on_start();
    
    // void skill_get_home_interrupt_hook();
    
    
};
#endif
