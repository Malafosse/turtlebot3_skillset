#include "turtlebot_skillset_gui_widgets/widget.hpp"

#include <gui_tools/gui_tools.h>

#include <sstream>

TurtlebotSkillsetWidget::TurtlebotSkillsetWidget(const std::string &name, rclcpp::Node::SharedPtr node,
      bool display_data, bool display_resources, bool display_events)
    : TurtlebotSkillsetClient(name, node)
    , display_data_(display_data)
    , display_resources_(display_resources)
    , display_events_(display_events)
    , event_response_timeout_(3.0)
    , subscribe_currentpose_(false)
    , active_go_to_(false)
    , active_get_home_(false)
{
    
        
        	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.header.stamp.sec", 0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.header.stamp.sec already declared");
        } 
        this->go_to_input_.goalpose.header.stamp.sec = node_->get_parameter("turtlebot.GoTo.goalpose.header.stamp.sec").as_int();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.header.stamp.nanosec", 0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.header.stamp.nanosec already declared");
        } 
        this->go_to_input_.goalpose.header.stamp.nanosec = node_->get_parameter("turtlebot.GoTo.goalpose.header.stamp.nanosec").as_int();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.header.frame_id", ""); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.header.frame_id already declared");
        } 
        this->go_to_input_.goalpose.header.frame_id = node_->get_parameter("turtlebot.GoTo.goalpose.header.frame_id").as_string();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.position.x", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.position.x already declared");
        } 
        this->go_to_input_.goalpose.pose.position.x = node_->get_parameter("turtlebot.GoTo.goalpose.pose.position.x").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.position.y", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.position.y already declared");
        } 
        this->go_to_input_.goalpose.pose.position.y = node_->get_parameter("turtlebot.GoTo.goalpose.pose.position.y").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.position.z", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.position.z already declared");
        } 
        this->go_to_input_.goalpose.pose.position.z = node_->get_parameter("turtlebot.GoTo.goalpose.pose.position.z").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.orientation.x", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.orientation.x already declared");
        } 
        this->go_to_input_.goalpose.pose.orientation.x = node_->get_parameter("turtlebot.GoTo.goalpose.pose.orientation.x").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.orientation.y", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.orientation.y already declared");
        } 
        this->go_to_input_.goalpose.pose.orientation.y = node_->get_parameter("turtlebot.GoTo.goalpose.pose.orientation.y").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.orientation.z", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.orientation.z already declared");
        } 
        this->go_to_input_.goalpose.pose.orientation.z = node_->get_parameter("turtlebot.GoTo.goalpose.pose.orientation.z").as_double();
        
	try {
            node_->declare_parameter("turtlebot.GoTo.goalpose.pose.orientation.w", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.GoTo.goalpose.pose.orientation.w already declared");
        } 
        this->go_to_input_.goalpose.pose.orientation.w = node_->get_parameter("turtlebot.GoTo.goalpose.pose.orientation.w").as_double();
        

        
    
        
        	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.header.stamp.sec", 0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.header.stamp.sec already declared");
        } 
        this->get_home_input_.initialpose.header.stamp.sec = node_->get_parameter("turtlebot.getHome.initialpose.header.stamp.sec").as_int();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.header.stamp.nanosec", 0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.header.stamp.nanosec already declared");
        } 
        this->get_home_input_.initialpose.header.stamp.nanosec = node_->get_parameter("turtlebot.getHome.initialpose.header.stamp.nanosec").as_int();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.header.frame_id", ""); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.header.frame_id already declared");
        } 
        this->get_home_input_.initialpose.header.frame_id = node_->get_parameter("turtlebot.getHome.initialpose.header.frame_id").as_string();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.position.x", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.position.x already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.position.x = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.position.x").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.position.y", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.position.y already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.position.y = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.position.y").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.position.z", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.position.z already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.position.z = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.position.z").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.x", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.orientation.x already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.orientation.x = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.x").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.y", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.orientation.y already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.orientation.y = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.y").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.z", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.orientation.z already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.orientation.z = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.z").as_double();
        
	try {
            node_->declare_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.w", 0.0); 
        } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "parameter turtlebot.getHome.initialpose.pose.pose.orientation.w already declared");
        } 
        this->get_home_input_.initialpose.pose.pose.orientation.w = node_->get_parameter("turtlebot.getHome.initialpose.pose.pose.orientation.w").as_double();
        
	// generation of double[36] not yet implemented

        
    
}

//-----------------------------------------------------------------------------

void TurtlebotSkillsetWidget::display_start_go_to() {
    
    active_go_to_ = true;
    
    active_get_home_ = false;
    
}

void TurtlebotSkillsetWidget::display_start_get_home() {
    
    active_go_to_ = false;
    
    active_get_home_ = true;
    
}



void TurtlebotSkillsetWidget::event_button_authority_to_skill() {
    if (ImGui::Button("authority_to_skill##turtlebot")) {
        events_["authority_to_skill"].id = this->send_event("authority_to_skill");
        events_["authority_to_skill"].response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        events_ids_[events_["authority_to_skill"].id] = "authority_to_skill";
    }
}

void TurtlebotSkillsetWidget::event_row_authority_to_skill() {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    event_button_authority_to_skill();
    auto evt = events_["authority_to_skill"];
    ImGui::TableNextColumn();
    ImVec4 color(1., 0., 0., 1.);
    std::string status = "UNKNOWN";
    switch (evt.response)
    {
    case turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS:
        status = "SUCCESS"; 
        color.x = 0.0; color.y = 1.0;
        break;
    case turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED:
        status = "UNDEFINED"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE:
        status = "GUARD_FAILURE"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE:
        status = "EFFECT_FAILURE"; break;
    }
    if (this->time_since_event("authority_to_skill") > event_response_timeout_)
        color.x = color.y = color.z = .6;
    if (!evt.id.empty())
        ImGui::TextColored(color, "%s", status.c_str());
}

void TurtlebotSkillsetWidget::event_button_authority_to_teleop() {
    if (ImGui::Button("authority_to_teleop##turtlebot")) {
        events_["authority_to_teleop"].id = this->send_event("authority_to_teleop");
        events_["authority_to_teleop"].response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        events_ids_[events_["authority_to_teleop"].id] = "authority_to_teleop";
    }
}

void TurtlebotSkillsetWidget::event_row_authority_to_teleop() {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    event_button_authority_to_teleop();
    auto evt = events_["authority_to_teleop"];
    ImGui::TableNextColumn();
    ImVec4 color(1., 0., 0., 1.);
    std::string status = "UNKNOWN";
    switch (evt.response)
    {
    case turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS:
        status = "SUCCESS"; 
        color.x = 0.0; color.y = 1.0;
        break;
    case turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED:
        status = "UNDEFINED"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE:
        status = "GUARD_FAILURE"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE:
        status = "EFFECT_FAILURE"; break;
    }
    if (this->time_since_event("authority_to_teleop") > event_response_timeout_)
        color.x = color.y = color.z = .6;
    if (!evt.id.empty())
        ImGui::TextColored(color, "%s", status.c_str());
}

void TurtlebotSkillsetWidget::event_button_charge_battery() {
    if (ImGui::Button("charge_battery##turtlebot")) {
        events_["charge_battery"].id = this->send_event("charge_battery");
        events_["charge_battery"].response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        events_ids_[events_["charge_battery"].id] = "charge_battery";
    }
}

void TurtlebotSkillsetWidget::event_row_charge_battery() {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    event_button_charge_battery();
    auto evt = events_["charge_battery"];
    ImGui::TableNextColumn();
    ImVec4 color(1., 0., 0., 1.);
    std::string status = "UNKNOWN";
    switch (evt.response)
    {
    case turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS:
        status = "SUCCESS"; 
        color.x = 0.0; color.y = 1.0;
        break;
    case turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED:
        status = "UNDEFINED"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE:
        status = "GUARD_FAILURE"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE:
        status = "EFFECT_FAILURE"; break;
    }
    if (this->time_since_event("charge_battery") > event_response_timeout_)
        color.x = color.y = color.z = .6;
    if (!evt.id.empty())
        ImGui::TextColored(color, "%s", status.c_str());
}

void TurtlebotSkillsetWidget::event_button_low_battery() {
    if (ImGui::Button("low_battery##turtlebot")) {
        events_["low_battery"].id = this->send_event("low_battery");
        events_["low_battery"].response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        events_ids_[events_["low_battery"].id] = "low_battery";
    }
}

void TurtlebotSkillsetWidget::event_row_low_battery() {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    event_button_low_battery();
    auto evt = events_["low_battery"];
    ImGui::TableNextColumn();
    ImVec4 color(1., 0., 0., 1.);
    std::string status = "UNKNOWN";
    switch (evt.response)
    {
    case turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS:
        status = "SUCCESS"; 
        color.x = 0.0; color.y = 1.0;
        break;
    case turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED:
        status = "UNDEFINED"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE:
        status = "GUARD_FAILURE"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE:
        status = "EFFECT_FAILURE"; break;
    }
    if (this->time_since_event("low_battery") > event_response_timeout_)
        color.x = color.y = color.z = .6;
    if (!evt.id.empty())
        ImGui::TextColored(color, "%s", status.c_str());
}

void TurtlebotSkillsetWidget::event_button_reinitialize_home() {
    if (ImGui::Button("reinitialize_home##turtlebot")) {
        events_["reinitialize_home"].id = this->send_event("reinitialize_home");
        events_["reinitialize_home"].response = turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED;
        events_ids_[events_["reinitialize_home"].id] = "reinitialize_home";
    }
}

void TurtlebotSkillsetWidget::event_row_reinitialize_home() {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    event_button_reinitialize_home();
    auto evt = events_["reinitialize_home"];
    ImGui::TableNextColumn();
    ImVec4 color(1., 0., 0., 1.);
    std::string status = "UNKNOWN";
    switch (evt.response)
    {
    case turtlebot_skillset_interfaces::msg::EventResponse::SUCCESS:
        status = "SUCCESS"; 
        color.x = 0.0; color.y = 1.0;
        break;
    case turtlebot_skillset_interfaces::msg::EventResponse::UNDEFINED:
        status = "UNDEFINED"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::GUARD_FAILURE:
        status = "GUARD_FAILURE"; break;
    case turtlebot_skillset_interfaces::msg::EventResponse::EFFECT_FAILURE:
        status = "EFFECT_FAILURE"; break;
    }
    if (this->time_since_event("reinitialize_home") > event_response_timeout_)
        color.x = color.y = color.z = .6;
    if (!evt.id.empty())
        ImGui::TextColored(color, "%s", status.c_str());
}


void TurtlebotSkillsetWidget::skill_response_text(int result_code) {
    std::string result;
    ImVec4 color(1., 0., 0., 1.);
    switch (result_code)
    {
    case 0: //turtlebot_skillset_interfaces::msg::SkillResult::SUCCESS:
        result = "SUCCESS"; 
        color.x = 0.; color.y = 1.;
        break;
    case 1: //turtlebot_skillset_interfaces::msg::SkillResult::ALREADY_RUNNING:
        result = "ALREADY RUNNING"; break;
    case 3: //turtlebot_skillset_interfaces::msg::SkillResult::VALIDATE_FAILURE:
        result = "VALIDATE FAILURE"; break;
    case 2: //turtlebot_skillset_interfaces::msg::SkillResult::PRECONDITION_FAILURE:
        result = "PRECONDITION FAILURE"; break;
    case 4: //turtlebot_skillset_interfaces::msg::SkillResult::START_FAILURE:
        result = "START FAILURE"; break;
    case 5: //turtlebot_skillset_interfaces::msg::SkillResult::INVARIANT_FAILURE:
        result = "INVARIANT FAILURE"; break;
    case 6: //turtlebot_skillset_interfaces::msg::SkillResult::INTERRUPT:
        result = "INTERRUPT";
        color.x = 1.; color.y = 1.;
        break;
    case 7: //turtlebot_skillset_interfaces::msg::SkillResult::FAILURE:
        result = "FAILURE"; break;            
    default:
        break;
    }
    ImGui::TextColored(color, "%s", result.c_str());
}

bool TurtlebotSkillsetWidget::update_window()
{
    bool widget_alive = true;
    ImGui::Begin("Turtlebot Skillset Manager", &widget_alive);
    update();
    ImGui::End();
    return widget_alive;
}

void TurtlebotSkillsetWidget::update()
{
    ImGui::Text("Status received: %6.1f", this->time_since_status());
    ImGui::SameLine();
    if (ImGui::Button("request status##turtlebot")) {
        this->request_status();
    }
    
	if (this->display_data_ && ImGui::CollapsingHeader("Data", ImGuiTreeNodeFlags_DefaultOpen)) {
        
        if (ImGui::TreeNode("currentpose")) {
            if (ImGui::Button("request data##turtlebot")) {
                this->data_currentpose_request();
            }
            ImGui::SameLine();
            ImGui::Checkbox("subscribe##turtlebot-currentpose", &subscribe_currentpose_);
            if (this->data_currentpose_.has_data) {
                if (ImGui::TreeNodeEx("currentpose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("header", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("stamp", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %d", "sec", this->data_currentpose_.value.header.stamp.sec);
	ImGui::Text("%s: %u", "nanosec", this->data_currentpose_.value.header.stamp.nanosec);
	ImGui::TreePop();
}	ImGui::Text("%s: %s", "frame_id", this->data_currentpose_.value.header.frame_id.c_str());
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("position", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->data_currentpose_.value.pose.position.x);
	ImGui::Text("%s: %.6f", "y", this->data_currentpose_.value.pose.position.y);
	ImGui::Text("%s: %.6f", "z", this->data_currentpose_.value.pose.position.z);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->data_currentpose_.value.pose.orientation.x);
	ImGui::Text("%s: %.6f", "y", this->data_currentpose_.value.pose.orientation.y);
	ImGui::Text("%s: %.6f", "z", this->data_currentpose_.value.pose.orientation.z);
	ImGui::Text("%s: %.6f", "w", this->data_currentpose_.value.pose.orientation.w);
	ImGui::TreePop();
}	ImGui::TreePop();
}	ImGui::TreePop();
}
            }
            else
                ImGui::Text("%s", "no Data");
            ImGui::TreePop();
        }
        
    }
    
    
	if (this->display_resources_ && ImGui::CollapsingHeader("Resources", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BeginTable("#turtlebot_resource_table", 2);
        ImGui::TableSetupColumn("name");
        ImGui::TableSetupColumn("state");
        ImGui::TableHeadersRow(); 
        for (auto r: status_.resources) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", r.name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", r.state.c_str());
        }
        ImGui::EndTable();
    }
    
    
	if (this->display_events_ && ImGui::CollapsingHeader("Events", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BeginTable("#turtlebot_events", 2);
        ImGui::TableSetupColumn("#turtlebot_event", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("#turtlebot_response", ImGuiTableColumnFlags_WidthStretch);
        
        event_row_authority_to_skill();
        
        event_row_authority_to_teleop();
        
        event_row_charge_battery();
        
        event_row_low_battery();
        
        event_row_reinitialize_home();
        
        ImGui::EndTable();
    }
    
    
	if (ImGui::CollapsingHeader("Turtlebot Skills", ImGuiTreeNodeFlags_DefaultOpen)) {
        
        {
            // Colored RadioButton
            static ImVec4 running_color({0.0, 1.0, 0.0, 1.0});
            static ImVec4 interrupting_color({1.0, 0.6, 0.0, 1.0});
            std::string label = "##color_turtlebot_GoTo";
            ImGui::BeginDisabled();
            if (go_to_status_.state == 1 /* RUNNING */)
                ImGui::PushStyleColor(ImGuiCol_CheckMark, running_color);
            if (go_to_status_.state == 2 /* INTERRUPTING */)
                ImGui::PushStyleColor(ImGuiCol_CheckMark, interrupting_color);
            ImGui::RadioButton(label.c_str(), (go_to_status_.state > 0));
            if (go_to_status_.state > 0 /* RUNNING or INTERRUPTING */)
                ImGui::PopStyleColor();
            ImGui::EndDisabled();    
        }
        // end Colored RadioButton
        ImGui::SameLine();
        if (ImGui::TreeNodeEx("GoTo")) {
            ImGui::Text("%s", go_to_status_.id.substr(0, 8).c_str());
            ImGui::SameLine();
            if (ImGui::Button("start##turtlebot-GoTo"))
                go_to_status_.id = this->start_go_to();
            ImGui::SameLine();
            if (ImGui::Button("interrupt##turtlebot-GoTo"))
                this->interrupt_go_to(go_to_status_.id);
            if (go_to_result_.id.compare(go_to_status_.id) == 0) {
                ImGui::SameLine();
                skill_response_text(go_to_result_.result);
            }
            
            if (ImGui::TreeNodeEx("input", ImGuiTreeNodeFlags_DefaultOpen)) {
                
                if (ImGui::TreeNodeEx("goalpose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("header", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("stamp", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputInt("sec", &this->go_to_input_.goalpose.header.stamp.sec);
	ImGui::InputInt("nanosec", (int*)&this->go_to_input_.goalpose.header.stamp.nanosec);
	ImGui::TreePop();
}	ImGui::InputText("frame_id", &this->go_to_input_.goalpose.header.frame_id, 80);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("position", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputDouble("x", &this->go_to_input_.goalpose.pose.position.x, 0.1, 1.0);
	ImGui::InputDouble("y", &this->go_to_input_.goalpose.pose.position.y, 0.1, 1.0);
	ImGui::InputDouble("z", &this->go_to_input_.goalpose.pose.position.z, 0.1, 1.0);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputDouble("x", &this->go_to_input_.goalpose.pose.orientation.x, 0.1, 1.0);
	ImGui::InputDouble("y", &this->go_to_input_.goalpose.pose.orientation.y, 0.1, 1.0);
	ImGui::InputDouble("z", &this->go_to_input_.goalpose.pose.orientation.z, 0.1, 1.0);
	ImGui::InputDouble("w", &this->go_to_input_.goalpose.pose.orientation.w, 0.1, 1.0);
	ImGui::TreePop();
}	ImGui::TreePop();
}	ImGui::TreePop();
}
                
                ImGui::TreePop();
            }
            
            
            if (ImGui::TreeNodeEx("progress", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (go_to_progress_.id.compare(go_to_status_.id) == 0) {
                    
                    if (ImGui::TreeNodeEx("distance_remaining", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "data", this->go_to_progress_.distance_remaining.data);
	ImGui::TreePop();
}
                    
                }
                ImGui::TreePop();
            }
            
            
            ImGui::Separator();
            ImGui::TreePop();
        }
        
        {
            // Colored RadioButton
            static ImVec4 running_color({0.0, 1.0, 0.0, 1.0});
            static ImVec4 interrupting_color({1.0, 0.6, 0.0, 1.0});
            std::string label = "##color_turtlebot_getHome";
            ImGui::BeginDisabled();
            if (get_home_status_.state == 1 /* RUNNING */)
                ImGui::PushStyleColor(ImGuiCol_CheckMark, running_color);
            if (get_home_status_.state == 2 /* INTERRUPTING */)
                ImGui::PushStyleColor(ImGuiCol_CheckMark, interrupting_color);
            ImGui::RadioButton(label.c_str(), (get_home_status_.state > 0));
            if (get_home_status_.state > 0 /* RUNNING or INTERRUPTING */)
                ImGui::PopStyleColor();
            ImGui::EndDisabled();    
        }
        // end Colored RadioButton
        ImGui::SameLine();
        if (ImGui::TreeNodeEx("getHome")) {
            ImGui::Text("%s", get_home_status_.id.substr(0, 8).c_str());
            ImGui::SameLine();
            if (ImGui::Button("start##turtlebot-getHome"))
                get_home_status_.id = this->start_get_home();
            ImGui::SameLine();
            if (ImGui::Button("interrupt##turtlebot-getHome"))
                this->interrupt_get_home(get_home_status_.id);
            if (get_home_result_.id.compare(get_home_status_.id) == 0) {
                ImGui::SameLine();
                skill_response_text(get_home_result_.result);
            }
            
            if (ImGui::TreeNodeEx("input", ImGuiTreeNodeFlags_DefaultOpen)) {
                
                if (ImGui::TreeNodeEx("initialpose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("header", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("stamp", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputInt("sec", &this->get_home_input_.initialpose.header.stamp.sec);
	ImGui::InputInt("nanosec", (int*)&this->get_home_input_.initialpose.header.stamp.nanosec);
	ImGui::TreePop();
}	ImGui::InputText("frame_id", &this->get_home_input_.initialpose.header.frame_id, 80);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("position", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputDouble("x", &this->get_home_input_.initialpose.pose.pose.position.x, 0.1, 1.0);
	ImGui::InputDouble("y", &this->get_home_input_.initialpose.pose.pose.position.y, 0.1, 1.0);
	ImGui::InputDouble("z", &this->get_home_input_.initialpose.pose.pose.position.z, 0.1, 1.0);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::InputDouble("x", &this->get_home_input_.initialpose.pose.pose.orientation.x, 0.1, 1.0);
	ImGui::InputDouble("y", &this->get_home_input_.initialpose.pose.pose.orientation.y, 0.1, 1.0);
	ImGui::InputDouble("z", &this->get_home_input_.initialpose.pose.pose.orientation.z, 0.1, 1.0);
	ImGui::InputDouble("w", &this->get_home_input_.initialpose.pose.pose.orientation.w, 0.1, 1.0);
	ImGui::TreePop();
}	ImGui::TreePop();
}	// generation of double[36] not yet implemented
	ImGui::TreePop();
}	ImGui::TreePop();
}
                
                ImGui::TreePop();
            }
            
            
            
            ImGui::Separator();
            ImGui::TreePop();
        }
        
    }
    
    ImGui::Separator();
    ImGui::Separator();
    
    if (this->active_go_to_) {
        ImGui::Text("%s", "GOTO");
        
        if (ImGui::TreeNodeEx(" input", ImGuiTreeNodeFlags_DefaultOpen)) {
            
            if (ImGui::TreeNodeEx("goalpose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("header", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("stamp", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %d", "sec", this->go_to_input_.goalpose.header.stamp.sec);
	ImGui::Text("%s: %u", "nanosec", this->go_to_input_.goalpose.header.stamp.nanosec);
	ImGui::TreePop();
}	ImGui::Text("%s: %s", "frame_id", this->go_to_input_.goalpose.header.frame_id.c_str());
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("position", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->go_to_input_.goalpose.pose.position.x);
	ImGui::Text("%s: %.6f", "y", this->go_to_input_.goalpose.pose.position.y);
	ImGui::Text("%s: %.6f", "z", this->go_to_input_.goalpose.pose.position.z);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->go_to_input_.goalpose.pose.orientation.x);
	ImGui::Text("%s: %.6f", "y", this->go_to_input_.goalpose.pose.orientation.y);
	ImGui::Text("%s: %.6f", "z", this->go_to_input_.goalpose.pose.orientation.z);
	ImGui::Text("%s: %.6f", "w", this->go_to_input_.goalpose.pose.orientation.w);
	ImGui::TreePop();
}	ImGui::TreePop();
}	ImGui::TreePop();
}
            
            ImGui::TreePop();
        }
        
        if (ImGui::Button("START##turtlebot-GoTo", ImVec2(10*ImGui::GetFontSize(), 2*ImGui::GetTextLineHeight())))
            go_to_status_.id = this->start_go_to();
        // ImGui::SameLine();
        if (ImGui::Button("INTERRUPT##turtlebot-GoTo", ImVec2(10*ImGui::GetFontSize(), 2*ImGui::GetTextLineHeight())))
            this->interrupt_go_to(go_to_status_.id);
        if (go_to_result_.id.compare(go_to_status_.id) == 0) {
            skill_response_text(go_to_result_.result);
        }
    }
    
    if (this->active_get_home_) {
        ImGui::Text("%s", "GETHOME");
        
        if (ImGui::TreeNodeEx(" input", ImGuiTreeNodeFlags_DefaultOpen)) {
            
            if (ImGui::TreeNodeEx("initialpose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("header", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("stamp", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %d", "sec", this->get_home_input_.initialpose.header.stamp.sec);
	ImGui::Text("%s: %u", "nanosec", this->get_home_input_.initialpose.header.stamp.nanosec);
	ImGui::TreePop();
}	ImGui::Text("%s: %s", "frame_id", this->get_home_input_.initialpose.header.frame_id.c_str());
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("pose", ImGuiTreeNodeFlags_DefaultOpen)) {
if (ImGui::TreeNodeEx("position", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->get_home_input_.initialpose.pose.pose.position.x);
	ImGui::Text("%s: %.6f", "y", this->get_home_input_.initialpose.pose.pose.position.y);
	ImGui::Text("%s: %.6f", "z", this->get_home_input_.initialpose.pose.pose.position.z);
	ImGui::TreePop();
}if (ImGui::TreeNodeEx("orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
	ImGui::Text("%s: %.6f", "x", this->get_home_input_.initialpose.pose.pose.orientation.x);
	ImGui::Text("%s: %.6f", "y", this->get_home_input_.initialpose.pose.pose.orientation.y);
	ImGui::Text("%s: %.6f", "z", this->get_home_input_.initialpose.pose.pose.orientation.z);
	ImGui::Text("%s: %.6f", "w", this->get_home_input_.initialpose.pose.pose.orientation.w);
	ImGui::TreePop();
}	ImGui::TreePop();
}	// generation of double[36] not yet implemented
	ImGui::TreePop();
}	ImGui::TreePop();
}
            
            ImGui::TreePop();
        }
        
        if (ImGui::Button("START##turtlebot-getHome", ImVec2(10*ImGui::GetFontSize(), 2*ImGui::GetTextLineHeight())))
            get_home_status_.id = this->start_get_home();
        // ImGui::SameLine();
        if (ImGui::Button("INTERRUPT##turtlebot-getHome", ImVec2(10*ImGui::GetFontSize(), 2*ImGui::GetTextLineHeight())))
            this->interrupt_get_home(get_home_status_.id);
        if (get_home_result_.id.compare(get_home_status_.id) == 0) {
            skill_response_text(get_home_result_.result);
        }
    }
    
}

void TurtlebotSkillsetWidget::process() {
    
    if (subscribe_currentpose_)
        this->create_data_currentpose_subscription();
    else
        this->destroy_data_currentpose_subscription();
    
}
