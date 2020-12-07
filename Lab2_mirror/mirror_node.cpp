// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>

#include <functional>
#include <iostream>

// access time units such as 100ms
using namespace std::chrono_literals;

// some shortcuts for message classes
using sensor_msgs::msg::JointState;
using baxter_core_msgs::msg::JointCommand;

// a useful function to get the index of a string in a vector of strings
// returns the size of the vector if not found
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
  const auto elem = std::find(names.begin(), names.end(), name);
  return std::distance(names.begin(), elem);
}

namespace lab2_mirror
{

class MirrorNode : public rclcpp::Node
{
public:
  MirrorNode(rclcpp::NodeOptions options) : Node("mirror", options)
  {
    // init subscriber
    subscription_ = this->create_subscription<JointState>("robot/joint_states", 10, std::bind(&MirrorNode::subscription_callback, this, std::placeholders::_1));

    // init publisher
    publisher_ = this->create_publisher<JointCommand>("/robot/limb/left/joint_command", 10);

    // init timer - the passed function will be called with the given rate
    timer_ = this->create_wall_timer(100ms, [this](){this->timer_callback();});
  }
  
private:
    // these suffixes may be useful
    const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
    std::vector<double> left_position = {0., 0., 0., 0., 0., 0., 0.};
    //JointState last_msg ;


    void subscription_callback(const JointState::UniquePtr msg)
    {
        std::vector<std::string> right(7);
        std::transform(std::begin(suffixes), std::end(suffixes), std::begin(right), [](std::string suffixe){return "right"+suffixe;});

        int nb_joints = size(msg->name);
        for(auto i(0); i < nb_joints ; i ++)
        {
            for(auto & suffixe : MirrorNode::suffixes)
            {
                if(msg->name[i] == "right"+suffixe)
                {
                    auto index = std::find(std::begin(suffixes), std::end(suffixes), suffixe);
                    left_position[index-std::begin(suffixes)] = msg->position[i] ;
                }
            }
        }
    }

    void timer_callback()
    {
        auto message = JointCommand();
        message.mode = 1 ;
        message.names = std::vector<std::string>(7);
        message.command = std::vector<double>(7);
        for(auto i(0); i<7; i++)
        {
            auto suffixe = suffixes[i];
            message.names[i] = "left"+suffixe;
            message.command[i] = (suffixe == "_s1" || suffixe == "_e1" || suffixe == "_w1") ? left_position[i] : -left_position[i];
        }
        print_JointCommand(message);
        publisher_->publish(message);
    }

    void print_JointCommand(JointCommand message)
    {
        std::cout << "---------------------------------------------------------------------------------" << std::endl ;
        std::cout << "Sending..." << std::endl ;
        std::cout << "Control mode : " << message.mode << std::endl;
        int joint_number = message.names.size();
        for(int i(0); i<joint_number;i++)
        {
            std::cout << message.names[i] << " joint is set to : " << message.command[i] << std::endl ;
        }
    }

    // declare any subscriber / publisher / timer
    rclcpp::Subscription<JointState>::SharedPtr subscription_;
    rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}



// boilerplate main function

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<lab2_mirror::MirrorNode>(options));
  rclcpp::shutdown();
  return 0;
}

