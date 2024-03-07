#include "turtlebot_skillset/Resource.hpp"

namespace turtlebot_skillset
{
    //-------------------------------------------------- authority --------------------------------------------------

    std::vector<AuthorityState> Authority::states() const
    {
        return std::vector<AuthorityState>{
            AuthorityState::Teleop, AuthorityState::Skill, 
        };
    }

    std::vector<Arc<AuthorityState>> Authority::transitions() const
    {
        return std::vector<Arc<AuthorityState>>{
            Arc<AuthorityState>(AuthorityState::Teleop, AuthorityState::Skill), 
            Arc<AuthorityState>(AuthorityState::Skill, AuthorityState::Teleop), 
            };
    }

    bool Authority::check_next(AuthorityState dst) const
    {
        if (current() == dst) {
            return true;
        }

        switch (current()) {
        case AuthorityState::Teleop:
            return (dst == AuthorityState::Skill);
        case AuthorityState::Skill:
            return (dst == AuthorityState::Teleop);
        default: return false;
        }
    
    }
    //-------------------------------------------------- move --------------------------------------------------

    std::vector<MoveState> Move::states() const
    {
        return std::vector<MoveState>{
            MoveState::Moving, MoveState::Idle, 
        };
    }

    std::vector<Arc<MoveState>> Move::transitions() const
    {
        return std::vector<Arc<MoveState>>{
            Arc<MoveState>(MoveState::Moving, MoveState::Idle), 
            Arc<MoveState>(MoveState::Idle, MoveState::Moving), 
            };
    }

    bool Move::check_next(MoveState dst) const
    {
        if (current() == dst) {
            return true;
        }

        switch (current()) {
        case MoveState::Moving:
            return (dst == MoveState::Idle);
        case MoveState::Idle:
            return (dst == MoveState::Moving);
        default: return false;
        }
    
    }
    //-------------------------------------------------- home --------------------------------------------------

    std::vector<HomeState> Home::states() const
    {
        return std::vector<HomeState>{
            HomeState::Lost, HomeState::Initializing, HomeState::Initialized, 
        };
    }

    std::vector<Arc<HomeState>> Home::transitions() const
    {
        return std::vector<Arc<HomeState>>{
            Arc<HomeState>(HomeState::Lost, HomeState::Initializing), 
            Arc<HomeState>(HomeState::Lost, HomeState::Initialized), 
            Arc<HomeState>(HomeState::Initializing, HomeState::Lost), 
            Arc<HomeState>(HomeState::Initializing, HomeState::Initialized), 
            Arc<HomeState>(HomeState::Initialized, HomeState::Lost), 
            Arc<HomeState>(HomeState::Initialized, HomeState::Initializing), 
            };
    }

    bool Home::check_next(HomeState dst) const
    {
        if (current() == dst) {
            return true;
        }

        switch (current()) {
        case HomeState::Lost:
            return (dst == HomeState::Initializing) || (dst == HomeState::Initialized);
        case HomeState::Initializing:
            return (dst == HomeState::Lost) || (dst == HomeState::Initialized);
        case HomeState::Initialized:
            return (dst == HomeState::Lost) || (dst == HomeState::Initializing);
        default: return false;
        }
    
    }
    //-------------------------------------------------- battery_status --------------------------------------------------

    std::vector<BatteryStatusState> BatteryStatus::states() const
    {
        return std::vector<BatteryStatusState>{
            BatteryStatusState::Low, BatteryStatusState::Normal, 
        };
    }

    std::vector<Arc<BatteryStatusState>> BatteryStatus::transitions() const
    {
        return std::vector<Arc<BatteryStatusState>>{
            Arc<BatteryStatusState>(BatteryStatusState::Low, BatteryStatusState::Normal), 
            Arc<BatteryStatusState>(BatteryStatusState::Normal, BatteryStatusState::Low), 
            };
    }

    bool BatteryStatus::check_next(BatteryStatusState dst) const
    {
        if (current() == dst) {
            return true;
        }

        switch (current()) {
        case BatteryStatusState::Low:
            return (dst == BatteryStatusState::Normal);
        case BatteryStatusState::Normal:
            return (dst == BatteryStatusState::Low);
        default: return false;
        }
    
    }
    
}


std::string to_string(const turtlebot_skillset::AuthorityState &x)
{
    switch (x)
    {
    case turtlebot_skillset::AuthorityState::Teleop:
        return "Teleop";
    case turtlebot_skillset::AuthorityState::Skill:
        return "Skill";
    }
    return "";
}

std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::AuthorityState &x)
{
    switch (x)
    {
    case turtlebot_skillset::AuthorityState::Teleop:
        out << "Teleop";
        break;
    case turtlebot_skillset::AuthorityState::Skill:
        out << "Skill";
        break;
    }
    return out;
}

std::string to_string(const turtlebot_skillset::MoveState &x)
{
    switch (x)
    {
    case turtlebot_skillset::MoveState::Moving:
        return "Moving";
    case turtlebot_skillset::MoveState::Idle:
        return "Idle";
    }
    return "";
}

std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::MoveState &x)
{
    switch (x)
    {
    case turtlebot_skillset::MoveState::Moving:
        out << "Moving";
        break;
    case turtlebot_skillset::MoveState::Idle:
        out << "Idle";
        break;
    }
    return out;
}

std::string to_string(const turtlebot_skillset::HomeState &x)
{
    switch (x)
    {
    case turtlebot_skillset::HomeState::Lost:
        return "Lost";
    case turtlebot_skillset::HomeState::Initializing:
        return "Initializing";
    case turtlebot_skillset::HomeState::Initialized:
        return "Initialized";
    }
    return "";
}

std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::HomeState &x)
{
    switch (x)
    {
    case turtlebot_skillset::HomeState::Lost:
        out << "Lost";
        break;
    case turtlebot_skillset::HomeState::Initializing:
        out << "Initializing";
        break;
    case turtlebot_skillset::HomeState::Initialized:
        out << "Initialized";
        break;
    }
    return out;
}

std::string to_string(const turtlebot_skillset::BatteryStatusState &x)
{
    switch (x)
    {
    case turtlebot_skillset::BatteryStatusState::Low:
        return "Low";
    case turtlebot_skillset::BatteryStatusState::Normal:
        return "Normal";
    }
    return "";
}

std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::BatteryStatusState &x)
{
    switch (x)
    {
    case turtlebot_skillset::BatteryStatusState::Low:
        out << "Low";
        break;
    case turtlebot_skillset::BatteryStatusState::Normal:
        out << "Normal";
        break;
    }
    return out;
}

