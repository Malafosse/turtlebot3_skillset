type Float

skillset turtlebot {

    data {
        //TBD
    }

    resource {
        Authority {
            state { Teleop Skill }
            initial Teleop
            transition all
        }
        Move {
            state { Moving NotMoving }
            initial NotMoving
            transition all
        }
        Home {
            state { Lost Initializing Initialized }
            initial Lost
            transition all
        }
    }

    event {
        authority_to_skill {
            guard Move == NotMoving
            effect Authority -> Skill
        }
        authority_to_teleop {
            effect Authority -> Teleop
        }
        auto_Home {
            effect Home -> Initialized
        }
    }

    skill GoTo {
        input {
            // 2D waypoint for now
            x: Float
            y: Float
            w: Float
        }
        precondition {
            has_authority: Authority == Skill
            not_moving: Move == NotMoving
            is_initialized: Home == Initialized
        }
        start Move -> Moving
        invariant authority_to_skill {
            guard Authority == Skill and Home == Initialized 
            effect Move -> NotMoving
        }
        interrupt {
            interrupting false
            effect Move -> NotMoving
        }
        success ok {
            effect Move -> NotMoving
        }
        failure ko {
            effect Move -> NotMoving
        }
    }

    skill getHome {
        input {
            x: Float
            y: Float
            w: Float
        }
        precondition{
            not_moving: Move == NotMoving
            is_initialized: Home == Lost
        }
        start Home -> Initializing
        invariant not_moving {
            guard Move == NotMoving
            effect Home -> Lost
        }
        interrupt {
            interrupting false
            effect Home -> Lost
        }
        success ok {
            effect Home -> Initialized
        }
        failure ko {
            effect Home -> Lost
        }

    }
}