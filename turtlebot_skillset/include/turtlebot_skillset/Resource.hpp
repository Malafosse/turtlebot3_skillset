#ifndef TURTLEBOT_RESOURCE_HPP
#define TURTLEBOT_RESOURCE_HPP

#include <string>
#include <vector>
#include <ostream>

namespace turtlebot_skillset
{
    template <typename T>
    class Arc
    {
    public:
        Arc(T src, T dst) : src_(src), dst_(dst) {}
        inline T src() const { return src_; }
        inline T dst() const { return dst_; }
    private:
        T src_;
        T dst_;
    };

    template <typename T>
    class Resource
    {
    public:
        Resource(T initial) : current_(initial) {}
        virtual ~Resource() {};
        //
        virtual std::string name() const = 0;
        inline T current() const { return current_; };
        virtual std::vector<T> states() const = 0;
        virtual std::vector<Arc<T>> transitions() const = 0;
        virtual bool check_next(T dst) const = 0;
        //
        void set_next(T dst) {
            current_ = dst;
        }
    private:
        T current_;
    };

    //------------------------- authority -------------------------
    enum class AuthorityState
    {
        Teleop, Skill, 
    };

    class Authority : public Resource<AuthorityState>
    {
    public:
        Authority() : Resource(AuthorityState::Teleop) {}
        ~Authority() {}

        inline std::string name() const { return std::string("authority"); };
        std::vector<AuthorityState> states() const;
        std::vector<Arc<AuthorityState>> transitions() const;
        bool check_next(AuthorityState dst) const;
    };

    //------------------------- move -------------------------
    enum class MoveState
    {
        Moving, Idle, 
    };

    class Move : public Resource<MoveState>
    {
    public:
        Move() : Resource(MoveState::Idle) {}
        ~Move() {}

        inline std::string name() const { return std::string("move"); };
        std::vector<MoveState> states() const;
        std::vector<Arc<MoveState>> transitions() const;
        bool check_next(MoveState dst) const;
    };

    //------------------------- home -------------------------
    enum class HomeState
    {
        Lost, Initializing, Initialized, 
    };

    class Home : public Resource<HomeState>
    {
    public:
        Home() : Resource(HomeState::Lost) {}
        ~Home() {}

        inline std::string name() const { return std::string("home"); };
        std::vector<HomeState> states() const;
        std::vector<Arc<HomeState>> transitions() const;
        bool check_next(HomeState dst) const;
    };

    //------------------------- battery_status -------------------------
    enum class BatteryStatusState
    {
        Low, Normal, 
    };

    class BatteryStatus : public Resource<BatteryStatusState>
    {
    public:
        BatteryStatus() : Resource(BatteryStatusState::Normal) {}
        ~BatteryStatus() {}

        inline std::string name() const { return std::string("battery_status"); };
        std::vector<BatteryStatusState> states() const;
        std::vector<Arc<BatteryStatusState>> transitions() const;
        bool check_next(BatteryStatusState dst) const;
    };

    
}


std::string to_string(const turtlebot_skillset::AuthorityState &x);
std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::AuthorityState &x);

std::string to_string(const turtlebot_skillset::MoveState &x);
std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::MoveState &x);

std::string to_string(const turtlebot_skillset::HomeState &x);
std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::HomeState &x);

std::string to_string(const turtlebot_skillset::BatteryStatusState &x);
std::ostream &operator<<(std::ostream &out, const turtlebot_skillset::BatteryStatusState &x);


#endif
