
#include <random>
#include "turtlebot_skillset_gui_widgets/client.hpp"

TurtlebotSkillsetClient::TurtlebotSkillsetClient(const std::string &name, rclcpp::Node::SharedPtr node)
    : name_(name)
    , node_(node)
    , qos_best_(1), qos_reliable_(1)
{
    //----- resource init
    
    resource_state_["authority"] = "";
    
    resource_state_["move"] = "";
    
    resource_state_["home"] = "";
    
    resource_state_["battery_status"] = "";
    
    qos_best_.best_effort();
    qos_best_.durability_volatile();
    qos_reliable_.reliable();
    qos_reliable_.durability_volatile();

    status_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
        name+"/turtlebot_skillset/status_request", qos_reliable_);
    status_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::SkillsetStatus>(
        name+"/turtlebot_skillset/status", qos_reliable_, std::bind(&TurtlebotSkillsetClient::status_callback_, this, std::placeholders::_1));

    event_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::EventRequest>(
        name+"/turtlebot_skillset/event_request", qos_reliable_);
    event_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::EventResponse>(
        name+"/turtlebot_skillset/event_response", qos_reliable_, std::bind(&TurtlebotSkillsetClient::event_callback_, this, std::placeholders::_1));
    
    data_currentpose_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::DataRequest>(
        name+"/turtlebot_skillset/data/currentpose/request", qos_reliable_);
    data_currentpose_response_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::DataCurrentposeResponse>(
        name+"/turtlebot_skillset/data/currentpose/response", qos_reliable_, std::bind(&TurtlebotSkillsetClient::data_currentpose_response_callback_, this, std::placeholders::_1));
    
    
    go_to_result_.id = "00000000";
    go_to_status_.id = "";
    go_to_status_.state = turtlebot_skillset_interfaces::msg::SkillGoToStatus::READY;
    go_to_request_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::SkillGoToRequest>(
        name+"/turtlebot_skillset/skill/go_to/request", qos_reliable_);
    go_to_interrupt_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::SkillInterrupt>(
        name+"/turtlebot_skillset/skill/go_to/interrupt", qos_reliable_);
    
    go_to_progress_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::SkillGoToProgress>(
        name+"/turtlebot_skillset/skill/go_to/progress", qos_best_, std::bind(&TurtlebotSkillsetClient::go_to_progress_callback, this, std::placeholders::_1));
    
    go_to_response_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::SkillGoToResponse>(
        name+"/turtlebot_skillset/skill/go_to/response", qos_reliable_, std::bind(&TurtlebotSkillsetClient::go_to_response_callback, this, std::placeholders::_1));
    
    get_home_result_.id = "00000000";
    get_home_status_.id = "";
    get_home_status_.state = turtlebot_skillset_interfaces::msg::SkillGetHomeStatus::READY;
    get_home_request_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::SkillGetHomeRequest>(
        name+"/turtlebot_skillset/skill/get_home/request", qos_reliable_);
    get_home_interrupt_pub_ = node_->create_publisher<turtlebot_skillset_interfaces::msg::SkillInterrupt>(
        name+"/turtlebot_skillset/skill/get_home/interrupt", qos_reliable_);
    
    get_home_response_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::SkillGetHomeResponse>(
        name+"/turtlebot_skillset/skill/get_home/response", qos_reliable_, std::bind(&TurtlebotSkillsetClient::get_home_response_callback, this, std::placeholders::_1));
    
    auto now = node_->get_clock()->now();
    
    events_stamps_["authority_to_skill"] = now;
    
    events_stamps_["authority_to_teleop"] = now;
    
    events_stamps_["charge_battery"] = now;
    
    events_stamps_["low_battery"] = now;
    
    events_stamps_["reinitialize_home"] = now;
    
    
    turtlebot_skillset_interfaces::msg::ResourceState r_authority;
    r_authority.name = "authority"; 
    r_authority.state = "";
    status_.resources.push_back(r_authority);
    
    turtlebot_skillset_interfaces::msg::ResourceState r_move;
    r_move.name = "move"; 
    r_move.state = "";
    status_.resources.push_back(r_move);
    
    turtlebot_skillset_interfaces::msg::ResourceState r_home;
    r_home.name = "home"; 
    r_home.state = "";
    status_.resources.push_back(r_home);
    
    turtlebot_skillset_interfaces::msg::ResourceState r_battery_status;
    r_battery_status.name = "battery_status"; 
    r_battery_status.state = "";
    status_.resources.push_back(r_battery_status);
    
    status_pub_->publish(std_msgs::msg::Empty());
}

std::string TurtlebotSkillsetClient::generate_id() const {
    std::random_device rd;
    auto seed_data = std::array<int, std::mt19937::state_size> {};
    std::generate(std::begin(seed_data), std::end(seed_data), std::ref(rd));
    std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
    std::mt19937 generator(seq);

    std::array<uint8_t, 16> data{ { 0 } };
    std::uniform_int_distribution<uint32_t>  distribution;
    uint8_t bytes[16];
    for (int i = 0; i < 16; i += 4)
        *reinterpret_cast<uint32_t*>(bytes + i) = distribution(generator);
    // variant must be 10xxxxxx
    bytes[8] &= 0xBF;
    bytes[8] |= 0x80;
    // version must be 0100xxxx
    bytes[6] &= 0x4F;
    bytes[6] |= 0x40;
    std::copy(std::begin(bytes), std::end(bytes), std::begin(data));
    std::stringstream s;
    // manipulate stream as needed
    s << std::hex << std::setfill(static_cast<char>('0'))
        << std::setw(2) << (int)data[0]
        << std::setw(2) << (int)data[1]
        << std::setw(2) << (int)data[2]
        << std::setw(2) << (int)data[3]
        << '-'
        << std::setw(2) << (int)data[4]
        << std::setw(2) << (int)data[5]
        << '-'
        << std::setw(2) << (int)data[6]
        << std::setw(2) << (int)data[7]
        << '-'
        << std::setw(2) << (int)data[8]
        << std::setw(2) << (int)data[9]
        << '-'
        << std::setw(2) << (int)data[10]
        << std::setw(2) << (int)data[11]
        << std::setw(2) << (int)data[12]
        << std::setw(2) << (int)data[13]
        << std::setw(2) << (int)data[14]
        << std::setw(2) << (int)data[15];
    return s.str();
}

void TurtlebotSkillsetClient::status_callback_(const turtlebot_skillset_interfaces::msg::SkillsetStatus::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] received skillset status", name_.c_str());
    this->status_ = *msg;
    for (auto r: msg->resources)
        resource_state_[r.name] = r.state;
    go_to_status_ = msg->skill_go_to;
    get_home_status_ = msg->skill_get_home;
    
}

double TurtlebotSkillsetClient::time_since_status() {
    rclcpp::Time now = node_->get_clock()->now();
    rclcpp::Time stamp = status_.stamp;
    return (now-stamp).seconds();
}

void TurtlebotSkillsetClient::request_status() {
    RCLCPP_INFO(node_->get_logger(), "[%s] Request status", name_.c_str());
    status_pub_->publish(std_msgs::msg::Empty());
}

//-----------------------------------------------------------------------------

std::string TurtlebotSkillsetClient::send_event(std::string event) {
    turtlebot_skillset_interfaces::msg::EventRequest request;
    request.id = generate_id();
    request.name = event;
    RCLCPP_INFO(node_->get_logger(), "[%s] send event %s (%s)", 
        name_.c_str(), request.name.c_str(), request.id.c_str());
    this->event_pub_->publish(request);
    return request.id;
}

void TurtlebotSkillsetClient::event_callback_(const turtlebot_skillset_interfaces::msg::EventResponse::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "[%s] received event %s response %d", 
        name_.c_str(), msg->id.c_str(), msg->response);
    events_[events_ids_[msg->id]] = *msg;
    events_stamps_[events_ids_[msg->id]] = node_->get_clock()->now();
}

double TurtlebotSkillsetClient::time_since_event(std::string event) const {
    auto now = node_->get_clock()->now();
    return (now - events_stamps_.at(event)).seconds();
}


//-----------------------------------------------------------------------------
void TurtlebotSkillsetClient::data_currentpose_response_callback_(const turtlebot_skillset_interfaces::msg::DataCurrentposeResponse::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "[%s] received data 'currentpose' response %s", 
        name_.c_str(), msg->id.c_str());
    this->data_currentpose_.has_data = msg->has_data;
    this->data_currentpose_.value = msg->value;
}

void TurtlebotSkillsetClient::data_currentpose_callback_(const turtlebot_skillset_interfaces::msg::DataCurrentpose::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] received data 'currentpose'", name_.c_str());
    this->data_currentpose_.has_data = true;
    this->data_currentpose_.value = msg->value;
}

std::string TurtlebotSkillsetClient::data_currentpose_request() {
    turtlebot_skillset_interfaces::msg::DataRequest request;
    request.id = generate_id();
    this->data_currentpose_pub_->publish(request);
    return request.id;
}

void TurtlebotSkillsetClient::create_data_currentpose_subscription() {
    if (! data_currentpose_sub_) {
        RCLCPP_INFO(node_->get_logger(), "[%s] create subsription to data 'currentpose'", name_.c_str());
        data_currentpose_sub_ = node_->create_subscription<turtlebot_skillset_interfaces::msg::DataCurrentpose>(
            name_+"/turtlebot_skillset/data/currentpose", qos_reliable_, 
            std::bind(&TurtlebotSkillsetClient::data_currentpose_callback_, this, std::placeholders::_1));
    }
}

void TurtlebotSkillsetClient::destroy_data_currentpose_subscription() {
    if (data_currentpose_sub_) {
        RCLCPP_INFO(node_->get_logger(), "[%s] reset subsription to data 'currentpose'", name_.c_str());
        data_currentpose_sub_.reset();
    }
}



//-----------------------------------------------------------------------------

void TurtlebotSkillsetClient::go_to_progress_callback(const turtlebot_skillset_interfaces::msg::SkillGoToProgress::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] received skill 'go_to' progress %s", 
        name_.c_str(), msg->id.c_str());
    this->go_to_progress_ = *msg;
}

void TurtlebotSkillsetClient::go_to_response_callback(const turtlebot_skillset_interfaces::msg::SkillGoToResponse::SharedPtr msg) {
    RCLCPP_INFO(this->node_->get_logger(), "[%s] received GoTo %s result %d %s", 
        name_.c_str(), msg->id.c_str(), msg->result, msg->name.c_str());
    this->go_to_result_ = *msg;
    
}

std::string TurtlebotSkillsetClient::start_go_to() {
    turtlebot_skillset_interfaces::msg::SkillGoToRequest msg;
    msg.input = go_to_input_;
    msg.id = generate_id();
    RCLCPP_INFO(this->node_->get_logger(), "[%s] start skill GoTo %s", 
        name_.c_str(), msg.id.c_str());
    go_to_request_pub_->publish(msg);
    return msg.id;
}

void TurtlebotSkillsetClient::interrupt_go_to(std::string id) {
    turtlebot_skillset_interfaces::msg::SkillInterrupt msg;
    msg.id = id;
    RCLCPP_INFO(this->node_->get_logger(), "[%s] interrupt skill GoTo %s", 
        name_.c_str(), msg.id.c_str());
    go_to_interrupt_pub_->publish(msg);
}

void TurtlebotSkillsetClient::interrupt_go_to() {
    this->interrupt_go_to(go_to_status_.id);
}

//-----------------------------------------------------------------------------

void TurtlebotSkillsetClient::get_home_response_callback(const turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SharedPtr msg) {
    RCLCPP_INFO(this->node_->get_logger(), "[%s] received getHome %s result %d %s", 
        name_.c_str(), msg->id.c_str(), msg->result, msg->name.c_str());
    this->get_home_result_ = *msg;
    
}

std::string TurtlebotSkillsetClient::start_get_home() {
    turtlebot_skillset_interfaces::msg::SkillGetHomeRequest msg;
    msg.input = get_home_input_;
    msg.id = generate_id();
    RCLCPP_INFO(this->node_->get_logger(), "[%s] start skill getHome %s", 
        name_.c_str(), msg.id.c_str());
    get_home_request_pub_->publish(msg);
    return msg.id;
}

void TurtlebotSkillsetClient::interrupt_get_home(std::string id) {
    turtlebot_skillset_interfaces::msg::SkillInterrupt msg;
    msg.id = id;
    RCLCPP_INFO(this->node_->get_logger(), "[%s] interrupt skill getHome %s", 
        name_.c_str(), msg.id.c_str());
    get_home_interrupt_pub_->publish(msg);
}

void TurtlebotSkillsetClient::interrupt_get_home() {
    this->interrupt_get_home(get_home_status_.id);
}

