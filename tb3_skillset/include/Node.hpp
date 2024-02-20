#ifndef TB3_SKILLSET_NODE_HPP
#define TB3_SKILLSET_NODE_HPP


#if defined(SKILLSET_DEBUG_MODE)
#include "turtlebot_skillset/NodeDebug.hpp"
#define SKILLSET_NODE turtlebot_skillset::TurtlebotNodeDebug
#else
#include "turtlebot_skillset/Node.hpp"
#define SKILLSET_NODE turtlebot_skillset::TurtlebotNode
#endif

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals; // Why ? 

//namespace tb3_skillset_ns {
class Tb3SkillsetNode : public SKILLSET_NODE
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

    Tb3SkillsetNode() : SKILLSET_NODE("tb3_node","This is a test"){
        this->NtP_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose"
        );

        this->GetHome_publisher_ = create_publisher<PoseWithCovarianceStamped>(
            "initialpose",
            10
        );
/*
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Tb3SkillsetNode::skill_go_to_on_start, this));
*/
    }
    //-------------------- Event Hook --------------------
    void event_authority_to_skill_hook();
    void event_authority_to_teleop_hook();
    void event_auto_home_hook();
    
    //-------------------- Skill Hook --------------------
    //bool skill_go_to_validate_hook();
    //void skill_go_to_start_hook();
    void skill_go_to_on_start();
    void skill_go_to_invariant_authority_to_skill_hook();
    void skill_go_to_interrupt_hook();
    
    // bool skill_get_home_validate_hook();
    // void skill_get_home_start_hook();
    void skill_get_home_on_start();
    void skill_get_home_invariant_not_moving_hook();
    void skill_get_home_interrupt_hook();

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr NtP_action_client_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr GetHome_publisher_;
    //rclcpp::TimerBase::SharedPtr timer_;

    //void promptForGoal(geometry_msgs::msg::PoseStamped & pose);
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result) ;
    
}; // Class

//} // NS

//RCLCPP_COMPONENTS_REGISTER_NODE(tb3_skillset_ns::Tb3SkillsetNode)
#endif
