#include "turtlebot_skillset/Node.hpp"

namespace turtlebot_skillset
{
    void TurtlebotNode::skills_invariants_() {
        bool effect = true;
        while (effect) {
            effect = false;
            if (skill_go_to_state_ == SkillState::Running || skill_go_to_state_ == SkillState::Interrupting) {
                auto message = skill_go_to_invariants_();
                if (message.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
                    // Stop Skill
                    skill_go_to_state_ = SkillState::Ready;
                    // Effects
                    effect = message.effect;
                    skill_go_to_response_pub_->publish(message);
                }
            }
            
            if (skill_get_home_state_ == SkillState::Running || skill_get_home_state_ == SkillState::Interrupting) {
                auto message = skill_get_home_invariants_();
                if (message.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
                    // Stop Skill
                    skill_get_home_state_ = SkillState::Ready;
                    // Effects
                    effect = message.effect;
                    skill_get_home_response_pub_->publish(message);
                }
            }
            
        }
    }
    //-------------------------------------------------- GoTo --------------------------------------------------
    const turtlebot_skillset_interfaces::msg::SkillGoToInput::SharedPtr TurtlebotNode::skill_go_to_input() const
    {
        return skill_go_to_input_;
    }
    //---------- Validate ----------
    bool TurtlebotNode::skill_go_to_validate_hook() {
        return true;
    }
    //---------- Start ----------
    void TurtlebotNode::skill_go_to_start_hook() {}
    void TurtlebotNode::skill_go_to_on_start() {}
    //---------- Invariant ----------
    void TurtlebotNode::skill_go_to_invariant_authority_to_skill_hook() {}
    //---------- Interrupt ----------
    void TurtlebotNode::skill_go_to_interrupted_() {
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' interrupt");
        auto message = skill_go_to_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::INTERRUPT;
        // Stop Skill
        skill_go_to_state_ = SkillState::Ready;
        // Check effects
        if ((
             resource_move_->check_next(MoveState::NotMoving)
            )) {
            // hook
            skill_go_to_interrupt_hook();
            // Set effects
            resource_move_->set_next(MoveState::NotMoving);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_go_to_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
    }
    void TurtlebotNode::skill_go_to_interrupt_hook() {}
    //---------- Success ----------
    bool TurtlebotNode::skill_go_to_success_ok() {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' success 'ok'");
        // Not Running -> false
        if (skill_go_to_state_ != SkillState::Running) {
            mutex_.unlock();
            return false;
        }
        auto message = skill_go_to_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS;
        message.name = "ok";
        // Output
        // Stop Skill
        skill_go_to_state_ = SkillState::Ready;
        // Check if effects fail
        if ((
             resource_move_->check_next(MoveState::NotMoving)
            )) {
            // Set effects
            resource_move_->set_next(MoveState::NotMoving);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_go_to_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        return true;
    }
    //---------- Failure ----------
    bool TurtlebotNode::skill_go_to_failure_ko() {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' failure 'ko'");
        // Not Running -> false
        if (skill_go_to_state_ != SkillState::Running) {
            mutex_.unlock();
            return false;
        }
        auto message = skill_go_to_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::FAILURE;
        message.name = "ko";
        // Stop Skill
        skill_go_to_state_ = SkillState::Ready;
        // Check effects
        if ((
             resource_move_->check_next(MoveState::NotMoving)
            )) {
            // Set effects
            resource_move_->set_next(MoveState::NotMoving);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_go_to_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        return true;
    }//------------------------- Inner Function -------------------------
    turtlebot_skillset_interfaces::msg::SkillGoToResponse TurtlebotNode::skill_go_to_response_initialize_() const {
        turtlebot_skillset_interfaces::msg::SkillGoToResponse result;
        result.id = skill_go_to_id_;
        result.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS;
        result.has_authority = true;
        
        result.not_moving = true;
        
        result.is_initialized = true;
        result.name = "";
        result.effect = false;
        result.postcondition = true;
        return result;
    }

    turtlebot_skillset_interfaces::msg::SkillGoToResponse TurtlebotNode::skill_go_to_preconditions_() {
        auto result = skill_go_to_response_initialize_();
        bool all_success = true;
        
        // ----- precondition has_authority -----
        result.has_authority = (resource_authority_->current() == AuthorityState::Skill);
        all_success = all_success && result.has_authority;
        // ----- precondition not_moving -----
        result.not_moving = (resource_move_->current() == MoveState::NotMoving);
        all_success = all_success && result.not_moving;
        // ----- precondition is_initialized -----
        result.is_initialized = (resource_home_->current() == HomeState::Initialized);
        all_success = all_success && result.is_initialized;
        if (!all_success) {
            result.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::PRECONDITION_FAILURE;
        }
        return result;
    }

    turtlebot_skillset_interfaces::msg::SkillGoToResponse TurtlebotNode::skill_go_to_start_() {
        auto message = skill_go_to_response_initialize_();
        if (!(
             resource_move_->check_next(MoveState::Moving)
            )) {
            message.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::START_FAILURE;
            return message;
        }
        else {
            message.effect = true;
        }
        
        // hook
        skill_go_to_start_hook();
        // set effects
        resource_move_->set_next(MoveState::Moving);
        return message;
    }

    turtlebot_skillset_interfaces::msg::SkillGoToResponse TurtlebotNode::skill_go_to_invariants_() {
        auto message = skill_go_to_response_initialize_();
        // ----- invariant authority_to_skill -----
        // guard
        if (!(((resource_authority_->current() == AuthorityState::Skill) && (resource_home_->current() == HomeState::Initialized)))) {
            message.name = "authority_to_skill";
            message.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::INVARIANT_FAILURE;
            // check effects
            if ((
                 resource_move_->check_next(MoveState::NotMoving)
                )) {
                skill_go_to_invariant_authority_to_skill_hook();
                resource_move_->set_next(MoveState::NotMoving);
                message.effect = true;
            }
        }
        
        return message;
    }

    turtlebot_skillset_interfaces::msg::SkillGoToResponse TurtlebotNode::skill_go_to_all_invariants_() {
        auto message = skill_go_to_response_initialize_();

        bool effect = true;
        while (effect) {
            effect = false;
            
            if (skill_go_to_state_ == SkillState::Running || skill_go_to_state_ == SkillState::Interrupting) {
                auto response = skill_go_to_invariants_();
                if (response.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
                    // Stop Skill
                    skill_go_to_state_ = SkillState::Ready;
                    // Effects
                    effect = response.effect;
                    // Response
                    skill_go_to_response_pub_->publish(response);
                    
                    message = response;
                    
                }
            }
            
            if (skill_get_home_state_ == SkillState::Running || skill_go_to_state_ == SkillState::Interrupting) {
                auto response = skill_get_home_invariants_();
                if (response.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
                    // Stop Skill
                    skill_get_home_state_ = SkillState::Ready;
                    // Effects
                    effect = response.effect;
                    // Response
                    skill_get_home_response_pub_->publish(response);
                    
                }
            }
            
        }
        return message;
    }

    //------------------------- Callback -------------------------

    void TurtlebotNode::skill_go_to_callback_(const turtlebot_skillset_interfaces::msg::SkillGoToRequest::UniquePtr request) {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' request");
        auto response = skill_go_to_response_initialize_();
        // Already Running
        if (skill_go_to_state_ == SkillState::Running || skill_go_to_state_ == SkillState::Interrupting) {
            // response
            response.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::ALREADY_RUNNING;
            skill_go_to_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Skill id
        skill_go_to_id_ = request->id;
        // Input
        skill_go_to_input_ = std::make_shared<turtlebot_skillset_interfaces::msg::SkillGoToInput>(request->input);
        // Precondition
        response = skill_go_to_preconditions_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
            // other invariants
            if (response.effect) {
                skill_go_to_all_invariants_();
            }
            // response
            skill_go_to_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Validate
        if (!skill_go_to_validate_hook()) {
            // response
            response.result = turtlebot_skillset_interfaces::msg::SkillGoToResponse::VALIDATE_FAILURE;
            skill_go_to_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Start
        response = skill_go_to_start_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
            // response
            skill_go_to_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Run
        skill_go_to_state_ = SkillState::Running;
        // Check Invariant (Loop)
        // TODO: check if effect ?
        response = skill_go_to_all_invariants_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGoToResponse::SUCCESS) {
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        // callback
        this->skill_go_to_on_start();
    }

    

    void TurtlebotNode::skill_go_to_interrupt_callback_(const turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg) {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' interrupt");
        if (skill_go_to_id_ != msg->id) {
            RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'GoTo' worng id");
            mutex_.unlock();
            return;    
        }
        // Not Running -> finish
        if (skill_go_to_state_ != SkillState::Running) {            
            mutex_.unlock();
            // TODO return something ?
            return;    
        }
        // if Interrupting
        skill_go_to_interrupted_();
        mutex_.unlock();
        
    }
    //-------------------------------------------------- getHome --------------------------------------------------
    const turtlebot_skillset_interfaces::msg::SkillGetHomeInput::SharedPtr TurtlebotNode::skill_get_home_input() const
    {
        return skill_get_home_input_;
    }
    //---------- Validate ----------
    bool TurtlebotNode::skill_get_home_validate_hook() {
        return true;
    }
    //---------- Start ----------
    void TurtlebotNode::skill_get_home_start_hook() {}
    void TurtlebotNode::skill_get_home_on_start() {}
    //---------- Invariant ----------
    void TurtlebotNode::skill_get_home_invariant_not_moving_hook() {}
    //---------- Interrupt ----------
    void TurtlebotNode::skill_get_home_interrupted_() {
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' interrupt");
        auto message = skill_get_home_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::INTERRUPT;
        // Stop Skill
        skill_get_home_state_ = SkillState::Ready;
        // Check effects
        if ((
             resource_home_->check_next(HomeState::Lost)
            )) {
            // hook
            skill_get_home_interrupt_hook();
            // Set effects
            resource_home_->set_next(HomeState::Lost);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_get_home_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
    }
    void TurtlebotNode::skill_get_home_interrupt_hook() {}
    //---------- Success ----------
    bool TurtlebotNode::skill_get_home_success_ok() {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' success 'ok'");
        // Not Running -> false
        if (skill_get_home_state_ != SkillState::Running) {
            mutex_.unlock();
            return false;
        }
        auto message = skill_get_home_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS;
        message.name = "ok";
        // Output
        // Stop Skill
        skill_get_home_state_ = SkillState::Ready;
        // Check if effects fail
        if ((
             resource_home_->check_next(HomeState::Initialized)
            )) {
            // Set effects
            resource_home_->set_next(HomeState::Initialized);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_get_home_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        return true;
    }
    //---------- Failure ----------
    bool TurtlebotNode::skill_get_home_failure_ko() {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' failure 'ko'");
        // Not Running -> false
        if (skill_get_home_state_ != SkillState::Running) {
            mutex_.unlock();
            return false;
        }
        auto message = skill_get_home_response_initialize_();
        message.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::FAILURE;
        message.name = "ko";
        // Stop Skill
        skill_get_home_state_ = SkillState::Ready;
        // Check effects
        if ((
             resource_home_->check_next(HomeState::Lost)
            )) {
            // Set effects
            resource_home_->set_next(HomeState::Lost);
            message.effect = true;
            // Invariants
            skills_invariants_();
        }
        // Post
        // Response
        skill_get_home_response_pub_->publish(message);
        // Status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        return true;
    }//------------------------- Inner Function -------------------------
    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse TurtlebotNode::skill_get_home_response_initialize_() const {
        turtlebot_skillset_interfaces::msg::SkillGetHomeResponse result;
        result.id = skill_get_home_id_;
        result.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS;
        result.not_moving = true;
        
        result.is_initialized = true;
        result.name = "";
        result.effect = false;
        result.postcondition = true;
        return result;
    }

    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse TurtlebotNode::skill_get_home_preconditions_() {
        auto result = skill_get_home_response_initialize_();
        bool all_success = true;
        
        // ----- precondition not_moving -----
        result.not_moving = (resource_move_->current() == MoveState::NotMoving);
        all_success = all_success && result.not_moving;
        // ----- precondition is_initialized -----
        result.is_initialized = (resource_home_->current() == HomeState::Lost);
        all_success = all_success && result.is_initialized;
        if (!all_success) {
            result.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::PRECONDITION_FAILURE;
        }
        return result;
    }

    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse TurtlebotNode::skill_get_home_start_() {
        auto message = skill_get_home_response_initialize_();
        if (!(
             resource_home_->check_next(HomeState::Initializing)
            )) {
            message.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::START_FAILURE;
            return message;
        }
        else {
            message.effect = true;
        }
        
        // hook
        skill_get_home_start_hook();
        // set effects
        resource_home_->set_next(HomeState::Initializing);
        return message;
    }

    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse TurtlebotNode::skill_get_home_invariants_() {
        auto message = skill_get_home_response_initialize_();
        // ----- invariant not_moving -----
        // guard
        if (!((resource_move_->current() == MoveState::NotMoving))) {
            message.name = "not_moving";
            message.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::INVARIANT_FAILURE;
            // check effects
            if ((
                 resource_home_->check_next(HomeState::Lost)
                )) {
                skill_get_home_invariant_not_moving_hook();
                resource_home_->set_next(HomeState::Lost);
                message.effect = true;
            }
        }
        
        return message;
    }

    turtlebot_skillset_interfaces::msg::SkillGetHomeResponse TurtlebotNode::skill_get_home_all_invariants_() {
        auto message = skill_get_home_response_initialize_();

        bool effect = true;
        while (effect) {
            effect = false;
            
            if (skill_go_to_state_ == SkillState::Running || skill_get_home_state_ == SkillState::Interrupting) {
                auto response = skill_go_to_invariants_();
                if (response.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
                    // Stop Skill
                    skill_go_to_state_ = SkillState::Ready;
                    // Effects
                    effect = response.effect;
                    // Response
                    skill_go_to_response_pub_->publish(response);
                    
                }
            }
            
            if (skill_get_home_state_ == SkillState::Running || skill_get_home_state_ == SkillState::Interrupting) {
                auto response = skill_get_home_invariants_();
                if (response.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
                    // Stop Skill
                    skill_get_home_state_ = SkillState::Ready;
                    // Effects
                    effect = response.effect;
                    // Response
                    skill_get_home_response_pub_->publish(response);
                    
                    message = response;
                    
                }
            }
            
        }
        return message;
    }

    //------------------------- Callback -------------------------

    void TurtlebotNode::skill_get_home_callback_(const turtlebot_skillset_interfaces::msg::SkillGetHomeRequest::UniquePtr request) {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' request");
        auto response = skill_get_home_response_initialize_();
        // Already Running
        if (skill_get_home_state_ == SkillState::Running || skill_get_home_state_ == SkillState::Interrupting) {
            // response
            response.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::ALREADY_RUNNING;
            skill_get_home_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Skill id
        skill_get_home_id_ = request->id;
        // Input
        skill_get_home_input_ = std::make_shared<turtlebot_skillset_interfaces::msg::SkillGetHomeInput>(request->input);
        // Precondition
        response = skill_get_home_preconditions_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
            // other invariants
            if (response.effect) {
                skill_get_home_all_invariants_();
            }
            // response
            skill_get_home_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Validate
        if (!skill_get_home_validate_hook()) {
            // response
            response.result = turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::VALIDATE_FAILURE;
            skill_get_home_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Start
        response = skill_get_home_start_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
            // response
            skill_get_home_response_pub_->publish(response);
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // Run
        skill_get_home_state_ = SkillState::Running;
        // Check Invariant (Loop)
        // TODO: check if effect ?
        response = skill_get_home_all_invariants_();
        if (response.result != turtlebot_skillset_interfaces::msg::SkillGetHomeResponse::SUCCESS) {
            // status
            auto status_message = status_();
            status_pub_->publish(status_message);
            mutex_.unlock();
            return;
        }
        // status
        auto status_message = status_();
        status_pub_->publish(status_message);
        mutex_.unlock();
        // callback
        this->skill_get_home_on_start();
    }

    

    void TurtlebotNode::skill_get_home_interrupt_callback_(const turtlebot_skillset_interfaces::msg::SkillInterrupt::UniquePtr msg) {
        mutex_.lock();
        RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' interrupt");
        if (skill_get_home_id_ != msg->id) {
            RCLCPP_DEBUG(this->get_logger(), "skillset 'turtlebot' skill 'getHome' worng id");
            mutex_.unlock();
            return;    
        }
        // Not Running -> finish
        if (skill_get_home_state_ != SkillState::Running) {            
            mutex_.unlock();
            // TODO return something ?
            return;    
        }
        // if Interrupting
        skill_get_home_interrupted_();
        mutex_.unlock();
        
    }
}
