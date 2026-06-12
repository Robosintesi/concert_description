#include "concert_nav2/aruco_localization.hpp"

#include <cmath>
#include <limits>

#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace concert_nav2
{
	namespace
	{

		constexpr double kDegToRad = M_PI / 180.0;
		constexpr double kUnobservedVariance = 1e6;
		constexpr double kMinIncidenceRayLength = 0.10;

		double yawOf(const Eigen::Isometry3d &pose)
		{
			const Eigen::Matrix3d &rot = pose.linear();
			return std::atan2(rot(1, 0), rot(0, 0));
		}

		double tiltOf(const Eigen::Isometry3d &pose)
		{
			const Eigen::Matrix3d &rot = pose.linear();
			const double pitch = std::atan2(-rot(2, 0), std::hypot(rot(2, 1), rot(2, 2)));
			const double roll = std::atan2(rot(2, 1), rot(2, 2));
			return std::abs(pitch) + std::abs(roll);
		}

		Eigen::Vector3d facingNormal(double yaw)
		{
			return {-std::sin(yaw), std::cos(yaw), 0.0};
		}

		Eigen::Matrix3d landmarkRotation(double facingYaw)
		{
			const Eigen::Vector3d zAxis = facingNormal(facingYaw);
			const Eigen::Vector3d yAxis = -Eigen::Vector3d::UnitZ();
			const Eigen::Vector3d xAxis = yAxis.cross(zAxis);
			Eigen::Matrix3d rot;
			rot.col(0) = xAxis;
			rot.col(1) = yAxis;
			rot.col(2) = zAxis;
			return rot;
		}

		struct PlanarHypothesis
		{
			Eigen::Isometry3d mapTbase;
			double tilt;
		};

		PlanarHypothesis findBestPlanarPose(
			const Eigen::Isometry3d &mapTmarker, const Eigen::Isometry3d &markerTbase)
		{
			PlanarHypothesis best{Eigen::Isometry3d::Identity(), std::numeric_limits<double>::max()};
			for (int quarterTurn = 0; quarterTurn < 4; ++quarterTurn)
			{
				Eigen::Isometry3d mounted = mapTmarker;
				mounted.rotate(Eigen::AngleAxisd(quarterTurn * M_PI_2, Eigen::Vector3d::UnitZ()));
				const Eigen::Isometry3d candidate = mounted * markerTbase;
				const double tilt = tiltOf(candidate);
				if (tilt < best.tilt)
				{
					best = {candidate, tilt};
				}
			}
			return best;
		}

	} // namespace

	ArucoLocalization::ArucoLocalization(const rclcpp::NodeOptions &options)
		: rclcpp::Node("aruco_localization", options)
	{
		declareParameters();
		loadLandmarks(declare_parameter<std::string>("landmarks_file", ""));

		tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

		posePub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/aruco/pose", 10);

		const auto detectionTopics = declare_parameter<std::vector<std::string>>(
			"detection_topics",
			{"/D435i_camera_front/aruco_detections", "/D435i_camera_back/aruco_detections"});
		for (const auto &topic : detectionTopics)
		{
			detectionSubs_.push_back(
				create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
					topic, 10, std::bind(&ArucoLocalization::onDetection, this, std::placeholders::_1)));
			RCLCPP_INFO(get_logger(), "subscribed to %s", topic.c_str());
		}
	}

	void ArucoLocalization::declareParameters()
	{
		baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");
		mapFrame_ = declare_parameter<std::string>("map_frame", "map");

		registerDoubleParameter("xy_var_base", 1.0e-3, xyVarBase_);
		registerDoubleParameter("xy_var_per_m2", 5.0e-3, xyVarPerM2_);
		registerDoubleParameter("yaw_var_base", 1.0e-3, yawVarBase_);
		registerDoubleParameter("yaw_var_per_m2", 5.0e-3, yawVarPerM2_);

		registerDoubleParameter("max_range", 8.0, maxRange_);
		registerDoubleParameter("max_view_angle_deg", 65.0, maxViewAngleRad_, kDegToRad);
		registerDoubleParameter("max_tilt_rad", 0.20, maxTiltRad_);

		registerBoolParameter("gate_enabled", true, gateEnabled_);
		registerDoubleParameter("gate_xy", 0.50, gateXyBase_);
		registerDoubleParameter("gate_yaw", 0.35, gateYawBase_);
		registerDoubleParameter("gate_growth_per_sec", 0.20, gateGrowthPerSec_);
		registerDoubleParameter("gate_max", 1.5, gateMax_);
		registerDoubleParameter("relock_after_sec", 10.0, relockAfterSec_);

		paramCbHandle_ = add_on_set_parameters_callback(
			std::bind(&ArucoLocalization::onSetParameters, this, std::placeholders::_1));
	}

	void ArucoLocalization::registerDoubleParameter(
		const std::string &name, double defaultValue, double &target, double scale)
	{
		target = declare_parameter<double>(name, defaultValue) * scale;
		parameterUpdaters_[name] = [&target, scale](const rclcpp::Parameter &param)
		{
			target = param.as_double() * scale;
		};
	}

	void ArucoLocalization::registerBoolParameter(
		const std::string &name, bool defaultValue, bool &target)
	{
		target = declare_parameter<bool>(name, defaultValue);
		parameterUpdaters_[name] = [&target](const rclcpp::Parameter &param)
		{
			target = param.as_bool();
		};
	}

	rcl_interfaces::msg::SetParametersResult ArucoLocalization::onSetParameters(
		const std::vector<rclcpp::Parameter> &params)
	{
		for (const auto &param : params)
		{
			const auto updater = parameterUpdaters_.find(param.get_name());
			if (updater != parameterUpdaters_.end())
			{
				updater->second(param);
			}
		}
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;
		return result;
	}

	void ArucoLocalization::loadLandmarks(const std::string &path)
	{
		if (path.empty())
		{
			RCLCPP_ERROR(get_logger(), "landmarks_file parameter is empty");
			return;
		}
		try
		{
			const YAML::Node root = YAML::LoadFile(path);
			const YAML::Node entries = root["landmarks"];
			if (!entries)
			{
				RCLCPP_ERROR(get_logger(), "no 'landmarks' key in %s", path.c_str());
				return;
			}

			for (const auto &entry : entries)
			{
				const int id = entry.first.as<int>();
				const YAML::Node &values = entry.second;

				Landmark landmark;
				landmark.facingYaw = values["yaw"].as<double>();
				landmark.pose = Eigen::Isometry3d::Identity();
				landmark.pose.linear() = landmarkRotation(landmark.facingYaw);
				landmark.pose.translation() = Eigen::Vector3d(
					values["x"].as<double>(), values["y"].as<double>(), values["z"].as<double>());
				landmarks_[id] = landmark;
			}
		}
		catch (const YAML::Exception &e)
		{
			RCLCPP_FATAL(get_logger(), "cannot parse %s: %s", path.c_str(), e.what());
			landmarks_.clear();
			return;
		}

		std::string ids;
		for (const auto &kv : landmarks_)
		{
			ids += std::to_string(kv.first) + " ";
		}
		RCLCPP_INFO(get_logger(), "loaded %zu landmarks: %s", landmarks_.size(), ids.c_str());
	}

	void ArucoLocalization::onDetection(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
	{
		if (msg->markers.empty())
		{
			return;
		}

		const std::string &camFrame = msg->header.frame_id;
		Eigen::Isometry3d camTbase;
		try
		{
			const auto tf = tfBuffer_->lookupTransform(camFrame, baseFrame_, tf2::TimePointZero);
			camTbase = tf2::transformToEigen(tf.transform);
		}
		catch (const tf2::TransformException &e)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"no TF %s <- %s: %s", camFrame.c_str(), baseFrame_.c_str(), e.what());
			return;
		}

		// Single-update policy: keep only the closest acceptable marker, so the EKF
		// never flickers between inconsistent simultaneous PnP solutions.
		std::optional<Candidate> best;
		for (const auto &marker : msg->markers)
		{
			const auto candidate = evaluateMarker(marker, camTbase);
			if (candidate && (!best || candidate->distance < best->distance))
			{
				best = candidate;
			}
		}

		if (!best || !passesInnovationGate(*best))
		{
			return;
		}

		publishPose(*best, msg->header.stamp);
		lastAcceptTime_ = now();
	}

	std::optional<ArucoLocalization::Candidate> ArucoLocalization::evaluateMarker(
		const aruco_opencv_msgs::msg::MarkerPose &marker, const Eigen::Isometry3d &camTbase)
	{
		const auto found = landmarks_.find(marker.marker_id);
		if (found == landmarks_.end())
		{
			return std::nullopt;
		}
		const Landmark &landmark = found->second;

		Eigen::Isometry3d camTmarker;
		tf2::fromMsg(marker.pose, camTmarker);
		const double distance = camTmarker.translation().norm();
		if (distance > maxRange_)
		{
			return std::nullopt; // PnP too noisy at long range
		}

		const Eigen::Isometry3d markerTbase = camTmarker.inverse() * camTbase;
		const PlanarHypothesis planar = findBestPlanarPose(landmark.pose, markerTbase);
		if (planar.tilt > maxTiltRad_)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 1000,
				"marker %d: no planar solution (tilt %.2f rad) — discarded",
				marker.marker_id, planar.tilt);
			return std::nullopt;
		}

		const Eigen::Isometry3d mapTcam = planar.mapTbase * camTbase.inverse();
		if (!hasAcceptableIncidence(landmark, mapTcam, marker.marker_id))
		{
			return std::nullopt;
		}

		return Candidate{marker.marker_id, distance, planar.mapTbase};
	}

	bool ArucoLocalization::hasAcceptableIncidence(
		const Landmark &landmark, const Eigen::Isometry3d &mapTcam, int markerId)
	{
		Eigen::Vector3d ray = mapTcam.translation() - landmark.pose.translation();
		ray.z() = 0.0;
		if (ray.norm() < kMinIncidenceRayLength)
		{
			return false;
		}

		const double cosIncidence = facingNormal(landmark.facingYaw).dot(ray.normalized());
		if (cosIncidence < std::cos(maxViewAngleRad_))
		{
			RCLCPP_DEBUG(
				get_logger(), "marker %d: incidence too high (cos=%.2f), skipped",
				markerId, cosIncidence);
			return false;
		}
		return true;
	}

	bool ArucoLocalization::passesInnovationGate(const Candidate &candidate)
	{
		if (!gateEnabled_ || !lastAcceptTime_)
		{
			return true;
		}

		Eigen::Isometry3d ekfPose;
		try
		{
			const auto tf = tfBuffer_->lookupTransform(mapFrame_, baseFrame_, tf2::TimePointZero);
			ekfPose = tf2::transformToEigen(tf.transform);
		}
		catch (const tf2::TransformException &)
		{
			return true; // no EKF estimate available -> accept (bootstrap-like)
		}

		const double age = std::max(0.0, (now() - *lastAcceptTime_).seconds());
		const double gateXy = std::min(gateXyBase_ + gateGrowthPerSec_ * age, gateMax_);
		const double gateYaw = std::min(gateYawBase_ + gateGrowthPerSec_ * age, gateMax_);

		const double xyError =
			(candidate.mapTbase.translation().head<2>() - ekfPose.translation().head<2>()).norm();
		const double yawError = std::abs(
			angles::shortest_angular_distance(yawOf(ekfPose), yawOf(candidate.mapTbase)));

		if (xyError <= gateXy && yawError <= gateYaw)
		{
			rejectingSince_.reset();
			return true;
		}

		if (!rejectingSince_)
		{
			rejectingSince_ = now();
		}
		if ((now() - *rejectingSince_).seconds() > relockAfterSec_)
		{
			// EKF and ArUco persistently disagree: assume the filter is lost, accept
			// this measurement as a re-anchor.
			RCLCPP_ERROR(
				get_logger(),
				"relock: EKF/ArUco disagree for >%.0fs (dxy=%.2f dyaw=%.2f), re-anchoring on marker %d",
				relockAfterSec_, xyError, yawError, candidate.markerId);
			rejectingSince_.reset();
			return true;
		}

		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"marker %d rejected by gate: dxy=%.2fm dyaw=%.2frad (gate %.2f/%.2f)",
			candidate.markerId, xyError, yawError, gateXy, gateYaw);
		return false;
	}

	void ArucoLocalization::publishPose(
		const Candidate &candidate, const builtin_interfaces::msg::Time &stamp)
	{
		const double yaw = yawOf(candidate.mapTbase);
		double qz = std::sin(yaw / 2.0);
		double qw = std::cos(yaw / 2.0);

		if (qw < 0.0)
		{
			qz = -qz;
			qw = -qw;
		}

		const double rangeSq = candidate.distance * candidate.distance;
		const double xyVar = xyVarBase_ + xyVarPerM2_ * rangeSq;
		const double yawVar = yawVarBase_ + yawVarPerM2_ * rangeSq;

		geometry_msgs::msg::PoseWithCovarianceStamped out;
		out.header.stamp = stamp;
		out.header.frame_id = mapFrame_;
		out.pose.pose.position.x = candidate.mapTbase.translation().x();
		out.pose.pose.position.y = candidate.mapTbase.translation().y();
		out.pose.pose.position.z = 0.0;
		out.pose.pose.orientation.z = qz;
		out.pose.pose.orientation.w = qw;

		auto &cov = out.pose.covariance;
		cov.fill(0.0);
		cov[0] = xyVar;
		cov[7] = xyVar;
		cov[14] = kUnobservedVariance;
		cov[21] = kUnobservedVariance;
		cov[28] = kUnobservedVariance;
		cov[35] = yawVar;

		posePub_->publish(out);
	}

} // namespace concert_nav2
