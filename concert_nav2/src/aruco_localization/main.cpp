#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "concert_nav2/aruco_localization.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<concert_nav2::ArucoLocalization>());
	rclcpp::shutdown();
	return 0;
}
