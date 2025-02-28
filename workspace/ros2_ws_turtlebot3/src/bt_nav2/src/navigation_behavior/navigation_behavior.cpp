#include <functional>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavigationBehavior : public rclcpp::Node
{
public:
NavigationBehavior()
		: Node("navigation_behavior")
	{
		
	}

private:

	
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NavigationBehavior>());
	rclcpp::shutdown();
	return 0;
}