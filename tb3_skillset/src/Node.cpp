#include "TurtlebotSkillsetNode.hpp"

using namespace std::placeholders;

Tb3SkillsetNode::Tb3SkillsetNode() : SKILLSET_NODE("skillset_manager", "turtlebot_skillset"){}

Tb3SkillsetManager::Tb3SkillsetManager() : Tb3SkillsetNode(),  node_handle_(std::shared_ptr<Tb3SkillsetManager>(this, [](auto *) {})){
    RCLCPP_INFO(get_logger(), "Tb3SkillsetManager is created");
    // Skill GetHome
    this->GetHome_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10
    );
    // Move to 
    this->go_to_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose"); //need to check the name of the action server
}

// ======================================== Skill GoTo ============================================

void Tb3SkillsetManager::skill_go_to_on_start(){
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  auto input = this->skill_go_to_input();
  goal_msg.pose = input->goalpose;
  RCLCPP_INFO_STREAM(this->get_logger(), 
        "Sending goal " << nav2_msgs::action::to_yaml(goal_msg));

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Tb3SkillsetManager::go_to_response_callback_, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Tb3SkillsetManager::go_to_feedback_callback_, this, _1, _2);
  send_goal_options.result_callback = 
    std::bind(&Tb3SkillsetManager::go_to_result_callback_, this, _1);

  this->go_to_client_->async_send_goal(goal_msg, send_goal_options);
}

void Tb3SkillsetManager::skill_go_to_invariant_authority_to_skill_hook(){
  this->go_to_cancel_();
}
void Tb3SkillsetManager::skill_go_to_invariant_battery_ok_hook(){
  this->go_to_cancel_();
}
void Tb3SkillsetManager::skill_go_to_on_interrupting(){
  this->go_to_cancel_();
}

turtlebot_skillset_interfaces::msg::SkillGoToProgress Tb3SkillsetManager::skill_go_to_progress_hook(){
  turtlebot_skillset_interfaces::msg::SkillGoToProgress progress;
  progress.distance_remaining.data = go_to_distance_remaining_;
  return progress;
}

void Tb3SkillsetManager::go_to_response_callback_(const GotoGoalHandle::SharedPtr & goal_handle){
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    this->skill_go_to_failure_ko();
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    this->go_to_goal_handle_ = goal_handle;
  }
}

void Tb3SkillsetManager::go_to_feedback_callback_(GotoGoalHandle::SharedPtr, const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback){
  go_to_distance_remaining_ = feedback->distance_remaining;
}
void Tb3SkillsetManager::go_to_result_callback_(const GotoGoalHandle::WrappedResult & result){
  switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "MoveTo goal was reached: returning success");
            this->skill_go_to_success_ok();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "MoveTo goal was in error: returning failure");
            this->skill_go_to_failure_ko();
            return;
    }
}
void Tb3SkillsetManager::go_to_cancel_(){
  RCLCPP_INFO(this->get_logger(), "Cancelling MoveTo action");
  this->go_to_client_->async_cancel_goal(this->go_to_goal_handle_, 
        [=](action_msgs::srv::CancelGoal::Response::SharedPtr response) {
            (void)response;
            RCLCPP_INFO(this->get_logger(), "MoveTo action cancelled");
            this->skill_go_to_interrupted();
        });
}
    
// ======================================== Skill GetHome ============================================ 

void Tb3SkillsetManager::skill_get_home_on_start(){
  RCLCPP_INFO(this->get_logger(), "Retrieving initial pose");
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose; 
  auto input = this->skill_get_home_input();
  initial_pose = input->initialpose;
 
  try
  {
    RCLCPP_INFO(this->get_logger(), "Publishing initial pose");
    this->GetHome_publisher_->publish(initial_pose);
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send initial pose. Returning failure.");
    this->skill_get_home_failure_ko();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Initial pose sent.");
  this->skill_get_home_success_ok();
}