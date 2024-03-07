#include "turtlebot_skillset/Node.hpp"

namespace turtlebot_skillset
{
    
    //-------------------------------------------------- currentpose --------------------------------------------------

    turtlebot_skillset_interfaces::msg::DataCurrentpose TurtlebotNode::get_data_currentpose()
    {
        turtlebot_skillset_interfaces::msg::DataCurrentpose message;
        mutex_.lock();
        message.stamp = data_currentpose_stamp_;
        message.value = data_currentpose_;
        mutex_.unlock();
        return message;
    }

    void TurtlebotNode::set_data_currentpose(geometry_msgs::msg::PoseStamped data)
    {
        mutex_.lock();
        // Data
        data_currentpose_ = data;
        data_currentpose_stamp_ = rclcpp::Node::now();
        // Message
        turtlebot_skillset_interfaces::msg::DataCurrentpose message;
        message.stamp = data_currentpose_stamp_;
        message.value = data_currentpose_;
        data_currentpose_pub_->publish(message);
        mutex_.unlock();
    }

    turtlebot_skillset_interfaces::msg::DataCurrentpose TurtlebotNode::get_data_currentpose_hook()
    {
        turtlebot_skillset_interfaces::msg::DataCurrentpose message;
        message.stamp = data_currentpose_stamp_;
        message.value = data_currentpose_;
        return message;
    }

    void TurtlebotNode::set_data_currentpose_hook(geometry_msgs::msg::PoseStamped data)
    {
        // Data
        data_currentpose_ = data;
        data_currentpose_stamp_ = rclcpp::Node::now();
        // Message
        turtlebot_skillset_interfaces::msg::DataCurrentpose message;
        message.stamp = data_currentpose_stamp_;
        message.value = data_currentpose_;
        data_currentpose_pub_->publish(message);
    }

    

    void TurtlebotNode::data_currentpose_request_callback_(const turtlebot_skillset_interfaces::msg::DataRequest::UniquePtr msg)
    {
        turtlebot_skillset_interfaces::msg::DataCurrentposeResponse message;
        message.id = msg->id;
        mutex_.lock();
        if (data_currentpose_stamp_.nanoseconds() > 0) {
            message.has_data = true;
            message.stamp = data_currentpose_stamp_;
            message.value = data_currentpose_;
        }
        else {
            message.has_data = false;
            message.stamp = data_currentpose_stamp_;
        }
        mutex_.unlock();
        data_currentpose_response_pub_->publish(message);
    }

    
}
