#ifndef CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_
#define CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace concert_nav2
{

	class ArucoLocalization : public rclcpp::Node
	{
	public:
		explicit ArucoLocalization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	private:
		void loadLandmarks(const std::string &path);
		void onDetection(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);
		void publishPose(
			const Eigen::Isometry3d &tMapBase, double xyVar, double yawVar,
			const builtin_interfaces::msg::Time &stamp);
		rcl_interfaces::msg::SetParametersResult onSetParameters(
			const std::vector<rclcpp::Parameter> &params);

		struct Landmark
		{
			Eigen::Isometry3d pose;
			double facingYaw;
		};
		std::map<int, Landmark> landmarks;

		std::string baseFrame;
		std::string mapFrame;

		
		double xyVarBase{};
		double xyVarPerM2{};
		double yawVarBase{};
		double yawVarPerM2{};

		// acceptance limits
		double maxRange{};		  // discard markers farther than this [m]
		double maxViewAngleRad{}; // discard grazing-incidence views [rad]
		double maxTiltRad{};	  // max roll+pitch of the best planar candidate [rad]

		// innovation gate vs EKF estimate
		bool gateEnabled{};
		double gateXyBase{};
		double gateYawBase{};
		double gateGrowthPerSec{};
		double gateMax{};
		double relockAfterSec{};

		bool hasPublished_ = false;
		bool rejecting_ = false;
		rclcpp::Time lastAcceptTime_;
		rclcpp::Time firstRejectTime_;

		std::shared_ptr<tf2_ros::Buffer> tfBuffer;
		std::shared_ptr<tf2_ros::TransformListener> tfListener;
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub;
		std::vector<rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr> subs;
		OnSetParametersCallbackHandle::SharedPtr paramCbHandle;
	};

} // namespace concert_nav2

#endif // CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_