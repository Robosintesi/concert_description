#ifndef CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_
#define CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_

#include <functional>
#include <map>
#include <memory>
#include <optional>
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
		struct Landmark
		{
			Eigen::Isometry3d pose;
			double facingYaw;
		};

		// Closest acceptable marker observation kept for a single EKF update.
		struct Candidate
		{
			int markerId;
			double distance;
			Eigen::Isometry3d mapTbase;
		};

		void declareParameters();
		void registerDoubleParameter(
			const std::string &name, double defaultValue, double &target, double scale = 1.0);
		void registerBoolParameter(
			const std::string &name, bool defaultValue, bool &target);
		rcl_interfaces::msg::SetParametersResult onSetParameters(
			const std::vector<rclcpp::Parameter> &params);

		void loadLandmarks(const std::string &path);
		void onDetection(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);
		std::optional<Candidate> evaluateMarker(
			const aruco_opencv_msgs::msg::MarkerPose &marker, const Eigen::Isometry3d &camTbase);
		bool hasAcceptableIncidence(
			const Landmark &landmark, const Eigen::Isometry3d &mapTcam, int markerId);
		bool passesInnovationGate(const Candidate &candidate);
		void publishPose(
			const Candidate &candidate, const builtin_interfaces::msg::Time &stamp);

		std::map<int, Landmark> landmarks_;

		std::string baseFrame_;
		std::string mapFrame_;

		double xyVarBase_{};
		double xyVarPerM2_{};
		double yawVarBase_{};
		double yawVarPerM2_{};

		// acceptance limits
		double maxRange_{};		   // discard markers farther than this [m]
		double maxViewAngleRad_{}; // discard grazing-incidence views [rad]
		double maxTiltRad_{};	   // max roll+pitch of the best planar candidate [rad]

		// innovation gate vs EKF estimate
		bool gateEnabled_{};
		double gateXyBase_{};
		double gateYawBase_{};
		double gateGrowthPerSec_{};
		double gateMax_{};
		double relockAfterSec_{};

		std::optional<rclcpp::Time> lastAcceptTime_;
		std::optional<rclcpp::Time> rejectingSince_;

		std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
		std::shared_ptr<tf2_ros::TransformListener> tfListener_;
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
		std::vector<rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr> detectionSubs_;
		OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;
		std::map<std::string, std::function<void(const rclcpp::Parameter &)>> parameterUpdaters_;
	};

} // namespace concert_nav2

#endif // CONCERT_NAV2__ARUCO_LOCALIZATION_HPP_
