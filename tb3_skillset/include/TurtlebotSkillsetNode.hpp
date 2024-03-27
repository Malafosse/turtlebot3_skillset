#include "Node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class Tb3SkillsetManager : public Tb3SkillsetNode
{
public:
    Tb3SkillsetManager();
    //-------------------- Event Hook --------------------
    void event_authority_to_skill_hook();
    void event_authority_to_teleop_hook();
    void event_charge_battery_hook();
    void event_low_battery_hook();
    void event_enable_exploration_hook(); 
    void event_enable_navigation_hook(); 
    
    //-------------------- Skill GoTo --------------------
    bool skill_go_to_validate_hook();
    // void skill_go_to_start_hook();
    void skill_go_to_on_start();
    void skill_go_to_invariant_authority_to_skill_hook();
    void skill_go_to_invariant_battery_ok_hook();
    void skill_go_to_invariant_nav_enabled_hook();
    turtlebot_skillset_interfaces::msg::SkillGoToProgress skill_go_to_progress_hook();
    // void skill_go_to_interrupt_hook();
    void skill_go_to_on_interrupting();
    
    //-------------------- Skill GetHome --------------------
    // bool skill_get_home_validate_hook();
    // void skill_get_home_start_hook();
    void skill_get_home_on_start();

    //-------------------- Skill TakePicture --------------------
    // bool skill_take_picture_validate_hook();
    // void skill_take_picture_start_hook();
    void skill_take_picture_on_start();
    // void skill_take_picture_interrupt_hook();
    
    //-------------------- Skill Explore --------------------
    // bool skill_explore_validate_hook();
    // void skill_explore_start_hook();
    void skill_explore_on_start();
    void skill_explore_invariant_authority_to_skill_hook();
    void skill_explore_invariant_battery_ok_hook();
    void skill_explore_invariant_exploration_enabled_hook();
    // void skill_explore_interrupt_hook();
    void skill_explore_on_interrupting();
    
    //-------------------- Skill SaveMap --------------------
    // bool skill_save_map_validate_hook();
    // void skill_save_map_start_hook();
    void skill_save_map_on_start();
    // void skill_save_map_interrupt_hook();

    
private:
    rclcpp::Node::SharedPtr node_handle_;

    // Skill GetHome
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr GetHome_publisher_;

    // Skill GoTo
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr go_to_client_;
    using GotoGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    void go_to_response_callback_(const GotoGoalHandle::SharedPtr & goal_handle);
    void go_to_feedback_callback_(GotoGoalHandle::SharedPtr, const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback);
    void go_to_result_callback_(const GotoGoalHandle::WrappedResult & result);
    void diagnostic_callback_();
    void go_to_cancel_();
    double go_to_distance_remaining_;
    GotoGoalHandle::SharedPtr go_to_goal_handle_;

    // skill take picture
    // image_transport::ImageTransport image_transport_;
    // image_transport::Subscriber image_sub_;
    // cv::Mat _ipl_img;
    // bool save_image(std::string filename);

    // Skill Explore
    void explore_cancel_();

    // Diagnostic
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscriber_;
    void nav2_diagnostic_callback_(const diagnostic_msgs::msg::DiagnosticArray& diagnostic);
    bool nav2_active_;

};
