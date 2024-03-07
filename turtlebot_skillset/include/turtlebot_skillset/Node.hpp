#ifndef TURTLEBOT_NODE_HPP
#define TURTLEBOT_NODE_HPP

#include "Resource.hpp"

#include <string>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtlebot_skillset_interfaces/msg/skillset_status.hpp"
#include "turtlebot_skillset_interfaces/msg/resource_state.hpp"
#include "turtlebot_skillset_interfaces/msg/data_request.hpp"
#include "turtlebot_skillset_interfaces/msg/event_request.hpp"
#include "turtlebot_skillset_interfaces/msg/event_response.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_interrupt.hpp"
#include "turtlebot_skillset_interfaces/msg/data_currentpose.hpp"
#include "turtlebot_skillset_interfaces/msg/data_currentpose_response.hpp"


#include "turtlebot_skillset_interfaces/msg/skill_go_to_input.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_go_to_request.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_go_to_response.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_go_to_progress.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_go_to_status.hpp"

#include "turtlebot_skillset_interfaces/msg/skill_get_home_input.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_get_home_request.hpp"
#include "turtlebot_skillset_interfaces/msg/skill_get_home_response.hpp"



using namespace std::chrono_literals;

namespace turtlebot_skillset
{

    enum class SkillState
    {
        Ready,
        Running,
        Interrupting
    };
    
    //-------------------- turtlebot --------------------

    class TurtlebotNodeDebug;

    class TurtlebotNode : public rclcpp::Node
    {
        friend class TurtlebotNodeDebug;

    public:
        TurtlebotNode(const std::string node_name, const std::string info);
        virtual ~TurtlebotNode() {}

        std::string info() const { return info_; }

    protected:
        //-------------------- Data --------------------
        //----- currentpose -----
        turtlebot_skillset_interfaces::msg::DataCurrentpose get_data_currentpose();
        void set_data_currentpose(geometry_msgs::msg::PoseStamped data);
        turtlebot_skillset_interfaces::msg::DataCurrentpose get_data_currentpose_hook();
        void set_data_currentpose_hook(geometry_msgs::msg::PoseStamped data);
        
        //-------------------- Resource --------------------
        std::string get_authority_state();
        std::string get_authority_state_hook();
        std::string get_move_state();
        std::string get_move_state_hook();
        std::string get_home_state();
        std::string get_home_state_hook();
        std::string get_battery_status_state();
        std::string get_battery_status_state_hook();
        
        //-------------------- Status --------------------
        turtlebot_skillset_interfaces::msg::SkillsetStatus get_skillset_status();
        //-------------------- Event Hook --------------------
        virtual void event_authority_to_skill_hook();
        void event_authority_to_skill();
        virtual void event_authority_to_teleop_hook();
        void event_authority_to_teleop();
        virtual void event_charge_battery_hook();
        void event_charge_battery();
        virtual void event_low_battery_hook();
        void event_low_battery();
        virtual void event_reinitialize_home_hook();
        void event_reinitialize_home();
        
        //-------------------- Skill GoTo --------------------
        const turtlebot_skillset_interfaces::msg::SkillGoToInput::SharedPtr skill_go_to_input() const; 
        inline SkillState skill_go_to_state() const { return skill_go_to_state_; }
        virtual bool skill_go_to_validate_hook();
        virtual void skill_go_to_start_hook();
        virtual void skill_go_to_on_start();
        virtual void skill_go_to_invariant_authority_to_skill_hook();
        virtual void skill_go_to_invariant_battery_ok_hook();
        virtual turtlebot_skillset_interfaces::msg::SkillGoToProgress skill_go_to_progress_hook();
        inline std::chrono::milliseconds skill_go_to_progress_period() const { return 1000ms; }
        virtual void skill_go_to_interrupt_hook();
        virtual void skill_go_to_on_interrupting();
        bool skill_go_to_interrupted();
        bool skill_go_to_success_ok();
        bool skill_go_to_failure_ko();
        
        //-------------------- Skill getHome --------------------
        const turtlebot_skillset_interfaces::msg::SkillGetHomeInput::SharedPtr skill_get_home_input() const; 
        inline SkillState skill_get_home_state() const { return skill_get_home_state_; }
        virtual bool skill_get_home_validate_hook();
        virtual void skill_get_home_start_hook();
        virtual void skill_get_home_on_start();
        virtual void skill_get_home_interrupt_hook();
        bool skill_get_home_success_ok();
        bool skill_get_home_failure_ko();
        
    private:
        //-------------------- Skillset --------------------
        turtlebot_skillset_interfaces::msg::SkillsetStatus status_() const;
        //-------------------- Event --------------------
        int event_authority_to_skill_();
        int event_authority_to_teleop_();
        int event_charge_battery_();
        int event_low_battery_();
        int event_reinitialize_home_();
        void skills_invariants_();
        //-------------------- Skill --------------------
        turtlebot_skillset_interfaces::msg::SkillGoToResponse skill_go_to_response_initialize_() const;
        turtlebot_skillset_interfaces::msg::SkillGoToResponse skill_go_to_preconditions_();
        turtlebot_skillset_interfaces::msg::SkillGoToResponse skill_go_to_start_();
        turtlebot_skillset_interfaces::msg::SkillGoToResponse skill_go_to_invariants_();
        turtlebot_skillset_interfaces::msg::SkillGoToResponse skill_go_to_all_invariants_();
        void skill_go_to_interrupted_();turtlebot_skillset_interfaces::msg::SkillGetHomeResponse skill_get_home_response_initialize_() const;
        turtlebot_skillset_interfaces::msg::SkillGetHomeResponse skill_get_home_preconditions_();
        turtlebot_skillset_interfaces::msg::SkillGetHomeResponse skill_get_home_start_();
        turtlebot_skillset_interfaces::msg::SkillGetHomeResponse skill_get_home_invariants_();
        turtlebot_skillset_interfaces::msg::SkillGetHomeResponse skill_get_home_all_invariants_();
        void skill_get_home_interrupted_();
        //---------- Callback ----------
        void status_callback_(const std_msgs::msg::Empty::UniquePtr msg);
        // void data_callback_(const std_msgs::msg::String::UniquePtr msg);
        void event_callback_(const turtlebot_skillset_interfaces::msg::EventRequest::UniquePtr msg);
        //---------- currentpose ----------
        
        void data_currentpose_request_callback_(const turtlebot_skillset_interfaces::msg::DataRequest::UniquePtr msg);
        
        //---------- GoTo ----------
        void skill_go_to_callback_(const turtlebot_skillset_interfaces::msg::SkillGoToRequest::UniquePtr msg);void skill_go_to_progress_callback_();
        void skill_go_to_interrupt_callback_(const turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg);
        //---------- getHome ----------
        void skill_get_home_callback_(const turtlebot_skillset_interfaces::msg::SkillGetHomeRequest::UniquePtr msg);
        void skill_get_home_interrupt_callback_(const turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg);

        std::mutex mutex_;
        std::string info_;

        //---------- Data ----------
        rclcpp::Time data_currentpose_stamp_;
        geometry_msgs::msg::PoseStamped data_currentpose_;
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::DataCurrentpose>::SharedPtr data_currentpose_pub_;
        
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::DataRequest>::SharedPtr data_currentpose_request_sub_;
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::DataCurrentposeResponse>::SharedPtr data_currentpose_response_pub_;
        
        //---------- Resource ----------
        std::shared_ptr<Authority> resource_authority_;
        std::shared_ptr<Move> resource_move_;
        std::shared_ptr<Home> resource_home_;
        std::shared_ptr<BatteryStatus> resource_battery_status_;
        
        //---------- Topics ----------
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr status_sub_;
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_sub_;
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::EventRequest>::SharedPtr event_sub_;
        //
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillsetStatus>::SharedPtr status_pub_;
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::EventResponse>::SharedPtr event_pub_;
        
        //---------- GoTo ----------
        SkillState skill_go_to_state_;
        std::string skill_go_to_id_;
        turtlebot_skillset_interfaces::msg::SkillGoToInput::SharedPtr skill_go_to_input_;
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillGoToRequest>::SharedPtr skill_go_to_request_sub_;
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillInterrupt>::SharedPtr skill_go_to_interrupt_sub_;
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillGoToResponse>::SharedPtr skill_go_to_response_pub_;rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillGoToProgress>::SharedPtr skill_go_to_progress_pub_;
        rclcpp::TimerBase::SharedPtr skill_go_to_progress_timer_;
        //---------- getHome ----------
        SkillState skill_get_home_state_;
        std::string skill_get_home_id_;
        turtlebot_skillset_interfaces::msg::SkillGetHomeInput::SharedPtr skill_get_home_input_;
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillGetHomeRequest>::SharedPtr skill_get_home_request_sub_;
        rclcpp::Subscription<turtlebot_skillset_interfaces::msg::SkillInterrupt>::SharedPtr skill_get_home_interrupt_sub_;
        rclcpp::Publisher<turtlebot_skillset_interfaces::msg::SkillGetHomeResponse>::SharedPtr skill_get_home_response_pub_;
    };
}
#endif
