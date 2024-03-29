#include "TurtlebotSkillsetNode.hpp"

using namespace std::placeholders;
using std::placeholders::_1;

Tb3SkillsetNode::Tb3SkillsetNode() : SKILLSET_NODE("skillset_manager", "turtlebot_skillset"){}

Tb3SkillsetManager::Tb3SkillsetManager() : Tb3SkillsetNode(),  
  node_handle_(std::shared_ptr<Tb3SkillsetManager>(this, [](auto *) {}))
{
    RCLCPP_INFO(get_logger(), "Tb3SkillsetManager is created");
    // Skill GetHome
    this->GetHome_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10 //queue size
    );
    // Move to 
    this->go_to_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose");
    // Subscriber to the /diagnostic topic form nav2 
    this->diagnostic_subscriber_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10,
      std::bind(&Tb3SkillsetManager::nav2_diagnostic_callback_, this, _1)
    );

  // this->declare_parameter("image_transport", "compressed");
  // image_sub_ = image_transport_.subscribe("image", 1, 
  //       [=](const sensor_msgs::msg::Image::ConstSharedPtr img) {
  //           cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  //           _ipl_img = cv_ptr->image;
  //       });
}

// ======================================== Events ============================================
void Tb3SkillsetManager::event_authority_to_skill_hook(){
  RCLCPP_INFO(this->get_logger(), "Authority to skill was granted.");
}
void Tb3SkillsetManager::event_authority_to_teleop_hook(){
  RCLCPP_INFO(this->get_logger(), "Authority to teleop was granted. Cancelling any active skill.");
}
void Tb3SkillsetManager::event_charge_battery_hook(){
  RCLCPP_INFO(this->get_logger(), "Battery levels have been manually reset.");
}
void Tb3SkillsetManager::event_low_battery_hook(){
  RCLCPP_INFO(this->get_logger(), "Battery levels are low. Cancelling any active skill.");
}
void Tb3SkillsetManager::event_enable_exploration_hook(){
  RCLCPP_INFO(this->get_logger(), "Exploration has been enabled.");
}
void Tb3SkillsetManager::event_enable_navigation_hook(){
  RCLCPP_INFO(this->get_logger(), "Navigation has been enabled.");
}
// ======================================== Skill GoTo ============================================

bool Tb3SkillsetManager::skill_go_to_validate_hook(){
  if (!nav2_active_) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 is not active. Returning failure.");
    return false;
  }
  return true;
}

void Tb3SkillsetManager::skill_go_to_on_start(){
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  auto input = this->skill_go_to_input();
  goal_msg.pose.pose.position.x = input->x.data;
  goal_msg.pose.pose.position.y = input->y.data;
  goal_msg.pose.pose.orientation.w = input->theta.data; 
  goal_msg.pose.header.frame_id = "map";
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
  RCLCPP_ERROR(this->get_logger(), "Authority to skill was lost. Cancelling MoveTo action.");
  this->go_to_cancel_();
}
void Tb3SkillsetManager::skill_go_to_invariant_battery_ok_hook(){
  RCLCPP_ERROR(this->get_logger(), "Battery is not ok. Cancelling MoveTo action.");
  this->go_to_cancel_();
}
void Tb3SkillsetManager::skill_go_to_on_interrupting(){
  RCLCPP_ERROR(this->get_logger(), "Interrupting MoveTo action.");
  this->go_to_cancel_();
}

void Tb3SkillsetManager::skill_go_to_invariant_nav_enabled_hook(){
  RCLCPP_ERROR(this->get_logger(), "Navigation is not enabled. Cancelling MoveTo action.");
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
  // create a small subscriber to the /diagnostics topic to check if nav2 is active
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr small_diagnostic_subscriber_;
  
  RCLCPP_INFO(this->get_logger(), "Retrieving initial pose");
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose; 
  auto input = this->skill_get_home_input();
  initial_pose.pose.pose.position.x = input->x.data;
  initial_pose.pose.pose.position.y = input->y.data;
  initial_pose.pose.pose.orientation.w = input->theta.data;
  initial_pose.header.frame_id = "map";
  try
  {
    this->GetHome_publisher_->publish(initial_pose);
    RCLCPP_INFO(this->get_logger(), "Publishing initial pose");
   
    auto start_time = std::chrono::steady_clock::now();
    while (!nav2_active_) {
        // Check if timeout has occurred
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed_time > 2) {
            RCLCPP_ERROR(this->get_logger(), "Timeout occurred while waiting for Nav2 initialization feedback. Assuming success.");
            this->skill_get_home_success_ok();
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Initial pose sent. Nav2 initialized. Returning success.");
    this->skill_get_home_success_ok();
    return;
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to publish initial pose. Returning failure.");
    this->skill_get_home_failure_ko();
    return;
  }
}

// ======================================== Skill TakePicture ============================================
// bool Tb3SkillsetManager::save_image(std::string filename) {
//     RCLCPP_INFO_STREAM(this->get_logger(), "saving image to file " << filename);
//     try {
//         return cv::imwrite(filename, _ipl_img);
//     } catch (...) {
//         return false;
//     }
// }

void Tb3SkillsetManager::skill_take_picture_on_start(){
    RCLCPP_INFO(this->get_logger(), "Taking picture");
    // auto input = this->skill_take_picture_input();
    // std::string file_name = input->file_name.data;
    // if (save_image(file_name)) {
    //     RCLCPP_INFO(this->get_logger(), "Image saved as " << file_name);
    //     this->skill_take_picture_success_image_saved();
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Could not save " << file_name);
    //     this->skill_take_picture_failure_image_save_fail();
    // }
    this->skill_take_picture_success_image_saved();
}

// ======================================== Skill Explore ============================================
void Tb3SkillsetManager::skill_explore_on_start(){
  RCLCPP_INFO(this->get_logger(), "Starting Exploration.");
  this->skill_explore_success_ok();
}

void Tb3SkillsetManager::skill_explore_invariant_authority_to_skill_hook(){
  RCLCPP_INFO(this->get_logger(), "Authority to skill was lost, cancelling exploration.");
  this->explore_cancel_();
}
void Tb3SkillsetManager::skill_explore_invariant_battery_ok_hook(){
  RCLCPP_INFO(this->get_logger(), "Battery is not ok, cancelling exploration.");
  this->explore_cancel_();
}
void Tb3SkillsetManager::skill_explore_invariant_exploration_enabled_hook(){
  RCLCPP_INFO(this->get_logger(), "Exploration is not enabled, cancelling exploration.");
  this->explore_cancel_();
}
void Tb3SkillsetManager::skill_explore_on_interrupting(){
  RCLCPP_INFO(this->get_logger(), "Interrupting Exploration.");
  this->explore_cancel_();
}

void Tb3SkillsetManager::explore_cancel_(){
  RCLCPP_INFO(this->get_logger(), "Cancelling Exploration.");
  this->skill_explore_interrupted();
}
// ======================================== Skill SaveMap ============================================
void Tb3SkillsetManager::skill_save_map_on_start(){
  RCLCPP_INFO(this->get_logger(), "Saving map.");
  this->skill_save_map_success_map_saved();
}

// ======================================== Misc ============================================

void Tb3SkillsetManager::nav2_diagnostic_callback_(const diagnostic_msgs::msg::DiagnosticArray& diagnostic) {
    // Iterate through each DiagnosticStatus in the array
    for (const auto& status : diagnostic.status) {
        // Accessing name and level members
        if (status.name == "lifecycle_manager_navigation: Nav2 Health") {
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
                nav2_active_ = true;
                // RCLCPP_INFO(this->get_logger(), "Nav2 is active !");
            } else {
                nav2_active_ = false;
            }
        }
    }
}