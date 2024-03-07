#include "Node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

class Tb3SkillsetManager : public Tb3SkillsetNode
{
public:
    Tb3SkillsetManager();
    //-------------------- Event Hook --------------------
    // void event_authority_to_skill_hook();
    // void event_authority_to_teleop_hook();
    // void event_charge_battery_hook();
    // void event_low_battery_hook();
    
    //-------------------- Skill GoTo --------------------
    // bool skill_go_to_validate_hook();
    // void skill_go_to_start_hook();
    void skill_go_to_on_start();
    void skill_go_to_invariant_authority_to_skill_hook();
    void skill_go_to_invariant_battery_ok_hook();
    turtlebot_skillset_interfaces::msg::SkillGoToProgress skill_go_to_progress_hook();
    // void skill_go_to_interrupt_hook();
    void skill_go_to_on_interrupting();
    
    //-------------------- Skill GetHome --------------------
    // bool skill_get_home_validate_hook();
    // void skill_get_home_start_hook();
    void skill_get_home_on_start();
    void skill_get_home_invariant_not_moving_hook();
    // void skill_get_home_interrupt_hook();
    void skill_get_home_on_interrupting();
    
private:

    // Skill GetHome
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr GetHome_publisher_;

    // Skill GoTo
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr go_to_client_;
    using GotoGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    void go_to_response_callback_(const GotoGoalHandle::SharedPtr & goal_handle);
    void go_to_feedback_callback_(GotoGoalHandle::SharedPtr, const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback);
    void go_to_result_callback_(const GotoGoalHandle::WrappedResult & result);
    void go_to_cancel_();
    double go_to_distance_remaining_;
    GotoGoalHandle::SharedPtr go_to_goal_handle_;
};
