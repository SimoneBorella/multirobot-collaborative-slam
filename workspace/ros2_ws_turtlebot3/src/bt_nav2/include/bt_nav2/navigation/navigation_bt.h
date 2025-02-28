#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class GoToPose : public BT::StatefulActionNode
{
public:
    GoToPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    // Method overrides
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {};

    static BT::PortsList providedPorts();

    // Action client callback
    void nav_to_pose_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_ptr_;
    bool done_flag_;
};