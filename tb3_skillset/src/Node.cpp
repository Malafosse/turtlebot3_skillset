#include "Node.hpp"

//namespace tb3_skillset_ns{
void Tb3SkillsetNode::event_authority_to_skill_hook(){
  RCLCPP_INFO(this->get_logger(), "Authority was passed to the Skillset.");
}
void Tb3SkillsetNode::event_authority_to_teleop_hook(){
  RCLCPP_INFO(this->get_logger(), "Authority was passed to the operator.");
}
void Tb3SkillsetNode::event_auto_home_hook(){
  RCLCPP_INFO(this->get_logger(), "Home has been setup. Ready to navigate.");
}

void Tb3SkillsetNode::skill_go_to_on_start(){
  using namespace std::placeholders; 

    //this->timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Calling Nav2 Navigate to Pose action.");

    if (!this->NtP_action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.pose.position.x = this->skill_go_to_input()->x;
    goal_msg.pose.pose.position.y = this->skill_go_to_input()->y;
    goal_msg.pose.pose.orientation.w = this->skill_go_to_input()->w;
    goal_msg.pose.header.frame_id = "map";
    // Prompt user for goal coordinates and orientation
    //promptForGoal(goal_msg.pose);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&Tb3SkillsetNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Tb3SkillsetNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Tb3SkillsetNode::result_callback, this, _1);

    this->NtP_action_client_->async_send_goal(goal_msg, send_goal_options);
}

/*   
void Tb3SkillsetNode::promptForGoal(geometry_msgs::msg::PoseStamped & pose){
    std::cout << "Enter goal coordinates (x, y, z): ";
    std::cin >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z;

    std::cout << "Enter goal orientation (w): ";
    std::cin >> pose.pose.orientation.w;

    // Set frame_id
    pose.header.frame_id = "map"; 
}*/ 

void Tb3SkillsetNode::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle){
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void Tb3SkillsetNode::feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback){

    std::stringstream ss;
    ss << "Distance remaining: ";
    ss << feedback->distance_remaining << " meters.";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
} 

void Tb3SkillsetNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation to pose succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    rclcpp::shutdown();
}

void Tb3SkillsetNode::skill_go_to_invariant_authority_to_skill_hook(){
  // TODO
  RCLCPP_ERROR(this->get_logger(), "Go To Invariant error");
}

void Tb3SkillsetNode::skill_go_to_interrupt_hook(){
  // TODO
  RCLCPP_ERROR(this->get_logger(), "Interrupted skill GoTo");
}

void Tb3SkillsetNode::skill_get_home_on_start(){
  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose.pose.pose.position.x = this->skill_get_home_input()->x;
  initial_pose.pose.pose.position.y = this->skill_get_home_input()->y;
  initial_pose.pose.pose.orientation.w = this->skill_get_home_input()->w;
  initial_pose.header.frame_id = "map";
  RCLCPP_INFO(this->get_logger(), "Publishing initial pose");
  this->GetHome_publisher_->publish(initial_pose);
};

void Tb3SkillsetNode::skill_get_home_invariant_not_moving_hook(){
  RCLCPP_ERROR(this->get_logger(), "Get Home Invariant error");
};

void Tb3SkillsetNode::skill_get_home_interrupt_hook(){
  RCLCPP_ERROR(this->get_logger(), "Interrupted skill Get Home");
};

//}//ns

/*
uint8 SUCCESS = 0
uint8 ALREADY_RUNNING = 1
uint8 PRECONDITION_FAILURE = 2
uint8 VALIDATE_FAILURE = 3
uint8 START_FAILURE = 4
uint8 INVARIANT_FAILURE = 5
uint8 INTERRUPT = 6
uint8 FAILURE = 7
*/

/*
switch (this->skill_go_to_response().result){
    case SUCCESS:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo completed succesfully");
    case ALREADY_RUNNING:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo already running");
    case PRECONDITION_FAILURE:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo precondition failure");
    case VALIDATE_FAILURE:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo validate failure");
    case START_FAILURE:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo start failure");
    case INVARIANT_FAILURE:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo invariant failure");
    case INTERRUPT:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo interrupted");
    case FAILURE:
      RCLCPP_INFO(this->get_logger(), "Skill GoTo failure");
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
  }
*/