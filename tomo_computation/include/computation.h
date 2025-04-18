#ifndef __TOMO_COMPUTATION_H__
#define __TOMO_COMPUTATION_H__
// Tomo Computation Library
#include "data_object.h"
#include "definition.h"

namespace tomo
{

	// poses computation
	TomoPose multiplyTransformToTomoPose(const tf2::Transform tf1, const tf2::Transform tf2);
	TomoPose convertPoseInCamCoordinateToBase(const TomoPose& pose_in_cam, const TomoPose& head_pose);
	TomoPose convertPoseInBaseCoordinateToCam(const TomoPose& pose_in_base, const TomoPose& head_pose);
	TomoPose getAbsolutePose(const TomoPose& offset_pose, const TomoPose& base_pose);
	TomoPose getAbsolutePose(const TomoPose& offset_pose, const geometry_msgs::msg::Pose& base_pose);
	TomoPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, const TomoPose& base_pose);
	TomoPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, geometry_msgs::msg::Pose& base_pose);
	tf2::Vector3 getAbsolutePose(const tf2::Vector3& vector, const double distance, const TomoPose& base_pose);
	TomoPose getOffsetPose(const TomoPose& poselink1tolink2, const TomoPose& poselink2tolink3);
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& vector, const double distance, const tf2::Vector3& base_pose);
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& pose_begin, const tf2::Vector3& pose_end, const double distance,
									 const tf2::Vector3& base_pose);
	TomoPose movePoseAlongVector(const TomoPose& vector, const double distance, const TomoPose& base_pose);
	tf2::Transform getTransformBetweenPoses(const TomoPose& start_pose, const TomoPose& end_pose);
	TomoPose getControlPoseFromEff(const TomoPose& current_eef_pose, const TomoPose& current_tip_pose, const TomoPose& control_pose,
								   const TomoPose& placing_pose, bool orientation_base_on_tip);
	TomoPose getOffsettedManipulatingObjectInv(const TomoPose& offset_matrix_pose, const TomoPose& control_pose,
											   const TomoPose& placing_pose, bool orientation_base_on_tip);
	TomoPose trackingObject(const TomoPose& current_eef_pose, const TomoPose& current_tip_pose, const TomoPose& tomo_object_pose, int count,
							int side);
	std::vector<double> calculateCorrectedHeadJoints(const std::vector<double>& joints, const std::vector<std::vector<double>>& coef,
													 const std::vector<std::vector<double>>& power, const std::vector<double>& intercept);
	bool isPointInEllipse(const std::vector<double> ellipse, const double x, const double y);
	bool isPointInEllipse(const double centreX, const double centreY, const double halfWidth, const double halfHeight, const double x,
						  const double y);
	PlaneParameter getPosePlaneParameter(const TomoPose& pose, const Plane plane);
}  // namespace tomo

#endif