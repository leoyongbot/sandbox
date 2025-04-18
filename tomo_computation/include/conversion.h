#ifndef __TOMO_CONVERSION_H__
#define __TOMO_CONVERSION_H__
// Tomo Conversion library
#include "data_object.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace tomo
{
	//math conversion
	double degToRad(const double& degree);
	double radToDeg(const double& rad);
	std::vector<double> degToRad(const std::vector<double>& degrees);
	std::vector<double> radToDeg(const std::vector<double>& rads);
	std::vector<double> quatToEuler(const tf2::Quaternion& quat);
	std::vector<double> quatToEuler(const double x, const double y, const double z, const double w);
	tf2::Matrix3x3 eulerToTfMatrix(const std::vector<double>& euler_angles);
	std::vector<double> tfMatrixToEuler(const tf2::Matrix3x3& rotation_matrix);
	tf2::Matrix3x3 quaternionToTfMatrix(const tf2::Quaternion& quat);
	tf2::Quaternion tfMatrixToQuaternion(const tf2::Matrix3x3& rotation_matrix);
	//Eigen conversion
	Eigen::VectorXd vecToEigenVec(std::vector<double> vec);
	std::vector<double> eigenVecToVec(Eigen::VectorXd eigenVec);
	//string conversion
	std::string vecToString(const std::vector<double>& vec);
	std::string tfvec3ToString(const tf2::Vector3& vec);
	std::string tfquatToString(const tf2::Quaternion& quat);
	//Pose conversion
	std::vector<double> poseQuatToVec(const tf2::Vector3& pos, const tf2::Quaternion& quat);
	geometry_msgs::msg::Pose vecToPoseMsgs(const std::vector<double>& pose);
	geometry_msgs::msg::Pose poseQuatToPoseMsgs(const tf2::Vector3& pos, const tf2::Quaternion& quat);
	void vecToPoseQuat(const std::vector<double>& input, tf2::Vector3& pos, tf2::Quaternion& quat);
	TomoPose poseMsgsToTomoPose(const geometry_msgs::msg::Pose& pose);
	TomoPose poseStampedMsgsToTomoPose(const geometry_msgs::msg::PoseStamped& pose_stamp);
	geometry_msgs::msg::Pose tomoPoseToPoseMsgs(const TomoPose& tomo_pose);
	TomoPose tfTransformToTomoPose(const tf2::Transform& transform);
	tf2::Transform tomoPoseToTfTransform(const TomoPose& pose);
	tf2::Transform poseMsgsToTfTransform(const geometry_msgs::msg::Pose pose);
	geometry_msgs::msg::Pose tfTransStampedToPoseMsgs(const geometry_msgs::msg::TransformStamped& transformStamped);
	tf2::Transform posRotToTfTransform(const tf2::Matrix3x3& rotation, const tf2::Vector3& translation);
	tf2::Transform eigenToTfTransform(const Eigen::Matrix4d& eigen_mat);
	Eigen::Matrix4d transformStampedToMatrix(const geometry_msgs::msg::TransformStamped& transformStamped);
}  // namespace tomo

#endif