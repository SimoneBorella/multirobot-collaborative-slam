#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_bt.h"

#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
public:
	Navigation() : Node("navigation")
	{
		RCLCPP_INFO(get_logger(), "BT setup started");

		BT::BehaviorTreeFactory factory;

		BT::NodeBuilder builder =
			[=](const std::string &name, const BT::NodeConfiguration &config)
		{
			return std::make_unique<GoToPose>(name, config, shared_from_this());
		};

		factory.registerBuilder<GoToPose>("GoToPose", builder);

		const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("bt_nav2") + "/bt_xml";

		tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");

		RCLCPP_INFO(get_logger(), "BT setup finished");

		const auto timer_period = 500ms;

		timer_ = this->create_wall_timer(
			timer_period,
			std::bind(&Navigation::update_behavior_tree, this)
		);
	}

	void update_behavior_tree()
	{
		BT::NodeStatus tree_status = tree_.tickRoot();

		if (tree_status == BT::NodeStatus::RUNNING)
		{
			return;
		}
		else if (tree_status == BT::NodeStatus::SUCCESS)
		{
			RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
		}
		else if (tree_status == BT::NodeStatus::FAILURE)
		{
			RCLCPP_INFO(this->get_logger(), "Navigation Failed");
			timer_->cancel();
		}
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	BT::Tree tree_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Navigation>());
	rclcpp::shutdown();
	return 0;
}