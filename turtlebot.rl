type{
    Float
    //Pose
}

skillset turtlebot {

    //data currentpose: Pose

    resource {
        authority {
            state { Teleop Skill }
            initial Teleop
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
    }

    skill GoTo { // Go to a goal pose
        input {
            x: Float
            y: Float
            theta: Float 
        }
        precondition {
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
            not_moving: move == Idle
            is_not_initialized: home == Lost
        }
        start home -> Initializing
        interrupt {
            interrupting false  // This skill cannot be interrupted, it is just an instant publish
        }
        success ok {
            effect home -> Initialized
        }
        failure ko { // May fail if nav2 is not running
            effect home -> Lost
        }
    }
}