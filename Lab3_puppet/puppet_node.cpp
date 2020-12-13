// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include "ik_client.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using baxter_core_msgs::srv::SolvePositionIK;

namespace lab3_puppet
{

class PuppetNode : public rclcpp::Node
{
public:
  PuppetNode(rclcpp::NodeOptions options)
    : Node("puppet", options), ik_node("ik_node"), tf_buffer(get_clock()), tf_listener(tf_buffer), tf_static_br(*this)
  {
    // init whatever is needed for your node

    // init command message for left arm

    // init publisher to left arm command
    publisher_ = this->create_publisher<JointCommand>("/robot/limb/left/joint_command", 10);

    // init timer - the function publishCommand() should called with the given rate
    timer_ = this->create_wall_timer(100ms, [this](){this->publishCommand();});

    // IK service wrapper into IKNode
    ik_node.init("/ExternalTools/left/PositionKinematicsNode/IKService");

    // static broadcast from c++ (should / could be done from independent node
    /*
    geometry_msgs::msg::TransformStamped offset;
    offset.header.set__stamp(now());
    offset.header.frame_id = "right_gripper";
    offset.child_frame_id = "left_gripper_desired";
    offset.transform.translation.z = 0.1;
    offset.transform.rotation.y = 1;
    offset.transform.rotation.w = 0;
    tf_static_br.sendTransform(offset);*/
  }
  
private:

    // declare member variables for command publisher and timer
    rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    ServiceNodeSync<SolvePositionIK> ik_node;

    // TF 2 stuff
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::StaticTransformBroadcaster tf_static_br;

    void publishCommand()
    {
    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("base", "left_gripper_desired", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
        // get this transform with tf_buffer.lookupTransform("base", "left_gripper_target")
        auto transform = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero, tf2::durationFromSec(1.0));

        // build service request SolvePositionIK::Request from obtained transform
        SolvePositionIK::Request req;
        req.seed_mode = req.SEED_AUTO;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.set__stamp(now());
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation.x = transform.transform.rotation.x;
        pose.pose.orientation.y = transform.transform.rotation.y;
        pose.pose.orientation.z = transform.transform.rotation.z;
        pose.pose.orientation.w = transform.transform.rotation.w;
        req.pose_stamp.push_back(pose);

        // call service and get response
        auto response = ik_node.sendRequest(req);

        // copy response data to joint command and publish
        auto message = JointCommand();
        message.mode = 1 ;
        message.names = response.joints[0].name;
        message.command = response.joints[0].position;
        publisher_->publish(message);
    }
    }
};
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)


