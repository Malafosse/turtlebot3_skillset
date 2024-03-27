type{
    Float
    //Pose
    Name
}

skillset turtlebot {

    //data currentpose: Pose

    resource {
        authority {
            state { Teleop Skill }
            initial Skill
            transition all
        }
        move {
            state { Moving Idle }
            initial Idle
            transition all
        }
        home {
            state { Lost Initializing Initialized }
            initial Lost
            transition all
        }
        battery_status {
            state { Low Normal }
            initial Normal
            transition all
        }
        context {
            state { SLAM Navigation Idle }
            initial Idle
            transition all
        }
    }

    event {
        authority_to_skill {
            guard move == Idle
            effect authority -> Skill
        }
        authority_to_teleop {
            effect authority -> Teleop
        }
        charge_battery {
            guard battery_status == Low
            effect battery_status -> Normal
        }
        low_battery {
            guard battery_status == Normal
            effect battery_status -> Low
        }
        reinitialize_home {
            effect home -> Lost
        }
        enable_exploration {
            effect context -> SLAM
        }
        enable_navigation {
            effect context -> Navigation
        }
    }

    skill GoTo { // Go to a goal pose
        input {
            x: Float
            y: Float
            theta: Float 
        }
        precondition {
            nav_enabled: context == Navigation
            has_authority: authority == Skill
            not_moving: move == Idle
            is_initialized: home == Initialized
            battery_ok: battery_status == Normal
        }
        start move -> Moving
        invariant {
            authority_to_skill {
                guard authority == Skill and home == Initialized 
                effect move -> Idle
            }
            battery_ok {
                guard battery_status == Normal
                effect move -> Idle
            }
            nav_enabled {
                guard context == Navigation
                effect move -> Idle
            }
        }
        progress {
            period 1.0
            output distance_remaining: Float
            }
        interrupt {
            interrupting true
            effect move -> Idle
        }
        success ok {
            effect move -> Idle
        }
        failure ko {
            effect move -> Idle
        }
    }

    skill getHome { // Initialize the robot's position on the map
        input {
            x: Float
            y: Float
            theta: Float 
        }
        precondition{
            nav_enabled: context == Navigation
            not_moving: move == Idle
            is_not_initialized: home == Lost
        }
        start home -> Initializing
        success ok {
            effect home -> Initialized
        }
        failure ko { // May fail if nav2 is not running
            effect home -> Lost
        }
    }

    skill take_picture { // Take a picture and save it to a file
    	input {
    		file_name: Name
    	}
    	precondition {
    		is_idle: move == Idle
    	}
    	success image_saved {}
        failure image_save_fail {} 
    }

    skill explore { // Explore the environment for a given duration
        input {
            duration: Float
        }
        precondition {
            exploration_enabled: context == SLAM
            has_authority: authority == Skill
            not_moving: move == Idle
            battery_ok: battery_status == Normal
        }
        start move -> Moving
        invariant {
            authority_to_skill {
                guard authority == Skill
                effect move -> Idle
            }
            battery_ok {
                guard battery_status == Normal
                effect move -> Idle
            }
            exploration_enabled {
                guard context == SLAM
                effect move -> Idle
            }
        }
        interrupt {
            interrupting true
            effect move -> Idle
        }
        success ok {
            effect move -> Idle
        }
        failure ko {
            effect move -> Idle
        }
    }

    skill save_map { // Save the map generated by the SLAM algorithm
        input {
            file_name: Name
        }
        precondition {
            context_enabled: context == SLAM
            not_moving: move == Idle
        }
        success map_saved {}
        failure map_not_saved {}
    }
}