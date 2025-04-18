#include "computation.h"
#include "conversion.h"
#include "definition.h"
namespace tomo
{
	TomoPose multiplyTransformToTomoPose(const tf2::Transform tf1, const tf2::Transform tf2)
	{
		tf2::Transform out_tf = tf1 * tf2;
		return (tfTransformToTomoPose(out_tf));
	}
	TomoPose convertPoseInCamCoordinateToBase(const TomoPose& pose_in_cam, const TomoPose& head_pose)
	{
		return multiplyTransformToTomoPose(tomoPoseToTfTransform(head_pose), tomoPoseToTfTransform(pose_in_cam));
	}
	TomoPose convertPoseInBaseCoordinateToCam(const TomoPose& pose_in_base, const TomoPose& head_pose)
	{
		return multiplyTransformToTomoPose(tomoPoseToTfTransform(head_pose).inverse(), tomoPoseToTfTransform(pose_in_base));
	}
	TomoPose getAbsolutePose(const TomoPose& offset_pose, const TomoPose& base_pose)
	{
		return multiplyTransformToTomoPose(tomoPoseToTfTransform(base_pose), tomoPoseToTfTransform(offset_pose));
	}
	TomoPose getAbsolutePose(const TomoPose& offset_pose, const geometry_msgs::msg::Pose& base_pose)
	{
		TomoPose base_tomo_pose = poseMsgsToTomoPose(base_pose);
		return multiplyTransformToTomoPose(tomoPoseToTfTransform(base_tomo_pose), tomoPoseToTfTransform(offset_pose));
	}
	TomoPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, const TomoPose& base_pose)
	{
		TomoPose output = base_pose;
		output.pos[0] += offset_x;
		output.pos[1] += offset_y;
		output.pos[2] += offset_z;
		return output;
	}
	TomoPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, const geometry_msgs::msg::Pose& base_pose)
	{
		TomoPose output = poseMsgsToTomoPose(base_pose);
		return getAbsolutePose(offset_x, offset_y, offset_z, output);
	}
	tf2::Vector3 getAbsolutePose(const tf2::Vector3& vector, const double distance, const TomoPose& base_pose)
	{
		tf2::Transform origin_tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
		return origin_tf.inverse() * tomoPoseToTfTransform(base_pose) * (distance * vector.normalized());
	}
	TomoPose getOffsetPose(const TomoPose& poselink1tolink2, const TomoPose& poselink2tolink3)
	{
		return multiplyTransformToTomoPose(tomoPoseToTfTransform(poselink1tolink2), tomoPoseToTfTransform(poselink2tolink3));
	}
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& vector, const double distance, const tf2::Vector3& base_pose)
	{
		tf2::Vector3 result;
		result = base_pose * distance * vector.normalized();
		return result;
	}
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& pose_begin, const tf2::Vector3& pose_end, const double distance,
									 const tf2::Vector3& base_pose)
	{
		tf2::Vector3 dir_vec = pose_end - pose_begin;
		return (dir_vec, distance, base_pose);
	}
	TomoPose movePoseAlongVector(const TomoPose& vector, const double distance, const TomoPose& base_pose)
	{
		TomoPose output = base_pose;
		output.pos		= movePoseAlongVector(vector.pos, distance, base_pose.pos);
		return output;
	}
	tf2::Transform getTransformBetweenPoses(const TomoPose& start_pose, const TomoPose& end_pose)
	{
		return tomoPoseToTfTransform(start_pose).inverse() * tomoPoseToTfTransform(end_pose);
	}
	// not in use
	TomoPose getControlPoseFromEff(const TomoPose& current_eef_pose, const TomoPose& current_tip_pose, const TomoPose& control_pose,
								   const TomoPose& placing_pose, bool orientation_base_on_tip)
	{
		TomoPose eef_pose;
		TomoPose control_pose_		 = getAbsolutePose(placing_pose, control_pose);
		tf2::Transform offset_matrix = getTransformBetweenPoses(current_tip_pose, current_eef_pose);
		TomoPose offset_pose_inv	 = tfTransformToTomoPose(offset_matrix.inverse());
		TomoPose offset_pose		 = tfTransformToTomoPose(offset_matrix);
		if(orientation_base_on_tip == false) {
			// orientation is relative of object and eef link
			eef_pose		   = getAbsolutePose(offset_pose_inv, control_pose_);
			control_pose_.quat = eef_pose.quat;
		}
		eef_pose = getAbsolutePose(offset_pose, control_pose_);
		return eef_pose;
	}
	TomoPose getOffsettedManipulatingObjectInv(const TomoPose& offset_matrix_pose, const TomoPose& control_pose,
											   const TomoPose& placing_pose, bool orientation_base_on_tip)
	{
		TomoPose eef_pose;
		TomoPose control_pose_ = getAbsolutePose(placing_pose, control_pose);
		tf2::Transform transform(offset_matrix_pose.quat, offset_matrix_pose.pos);
		TomoPose offset_pose_inv = tfTransformToTomoPose(transform);
		TomoPose offset_pose	 = tfTransformToTomoPose(transform.inverse());
		if(orientation_base_on_tip == false) {
			// orientation is relative of object and eef link
			eef_pose		   = getAbsolutePose(offset_pose_inv, control_pose_);
			control_pose_.quat = eef_pose.quat;
		}
		eef_pose = getAbsolutePose(offset_pose, control_pose_);
		return eef_pose;
	}
	TomoPose trackingObject(const TomoPose& current_eef_pose, const TomoPose& current_tip_pose, const TomoPose& tomo_object_pose, int count,
							int side)
	{
		TomoPose offset_pose_1 = current_tip_pose;
		offset_pose_1.offsetPos(tomo_object_pose.pos);

		auto m0 = current_eef_pose.getRotationMat();
		double current_roll, current_pitch, current_yaw;
		m0.getRPY(current_roll, current_pitch, current_yaw);
		double x_local = current_tip_pose.pos[0] - current_eef_pose.pos[0];
		double y_local = current_tip_pose.pos[1] - current_eef_pose.pos[1];
		double x_offset_rotate, y_offset_rotate;

		if(side == 0 && count < 11) {
			current_yaw		= current_yaw + count * (0.15708);
			x_offset_rotate = x_local * cos(count * (0.15708)) - y_local * sin(count * (0.15708)) - x_local;
			y_offset_rotate = x_local * sin(count * (0.15708)) + y_local * cos(count * (0.15708)) - y_local;
		}
		else if(side == 0 && count >= 11) {
			current_yaw		= current_yaw - (count - 11) * (0.15708);
			x_offset_rotate = x_local * cos(-(count - 11) * (0.15708)) - y_local * sin(-(count - 11) * (0.15708)) - x_local;
			y_offset_rotate = x_local * sin(-(count - 11) * (0.15708)) + y_local * cos(-(count - 11) * (0.15708)) - y_local;
		}
		else if(side == 1 && count < 11) {
			current_yaw		= current_yaw - count * (0.15708);
			x_offset_rotate = x_local * cos(-count * (0.15708)) - y_local * sin(-count * (0.15708)) - x_local;
			y_offset_rotate = x_local * sin(-count * (0.15708)) + y_local * cos(-count * (0.15708)) - y_local;
		}
		else if(side == 1 && count >= 11) {
			current_yaw		= current_yaw + (count - 11) * (0.15708);
			x_offset_rotate = x_local * cos((count - 11) * (0.15708)) - y_local * sin((count - 11) * (0.15708)) - x_local;
			y_offset_rotate = x_local * sin((count - 11) * (0.15708)) + y_local * cos((count - 11) * (0.15708)) - y_local;
		}
		tf2::Quaternion q1;
		q1.setRPY(current_roll, current_pitch, current_yaw);
		// end_effector pose
		TomoPose eef_pose;
		eef_pose.pos[0] = current_eef_pose.pos[0] + offset_pose_1.pos[0] - x_offset_rotate;
		eef_pose.pos[1] = current_eef_pose.pos[1] + offset_pose_1.pos[1] - y_offset_rotate;
		eef_pose.pos[2] = current_eef_pose.pos[2] + offset_pose_1.pos[2];
		eef_pose.quat	= q1;
		return eef_pose;
	}
	std::vector<double> calculateCorrectedHeadJoints(const std::vector<double>& joints, const std::vector<std::vector<double>>& coef,
													 const std::vector<std::vector<double>>& power, const std::vector<double>& intercept)
	{
		if(coef.size() == 0 || power.size() == 0 || intercept.size() == 0) {
			std::cerr << " Invalid head_data.yaml data. Can not get corrected head joints!" << std::endl;
			return joints;
		}

		std::vector<double> result_joints = {0, 0};
		for(int i = 0; i < power.size(); i++) {
			result_joints[0] += coef[0][i] * pow(joints[0], power[i][0]) * pow(joints[1], power[i][1]);
			result_joints[1] += coef[1][i] * pow(joints[0], power[i][0]) * pow(joints[1], power[i][1]);
		}

		result_joints[0] = result_joints[0] + intercept[0] + joints[0];
		result_joints[1] = result_joints[1] + intercept[1] + joints[1];
		printf("    Head joints [%f, %f] corrected as [%f, %f]\n", joints[0], joints[1], result_joints[0], result_joints[1]);
		fflush(stdout);
		return result_joints;
	}
	bool isPointInEllipse(const double centreX, const double centreY, const double halfWidth, const double halfHeight, const double x,
						  const double y)
	{
		return (pow(x - centreX, 2) / pow(halfWidth, 2) + pow(y - centreY, 2) / pow(halfHeight, 2)) <= 1;
	}
	bool isPointInEllipse(const std::vector<double> ellipse, const double x, const double y)
	{
		return isPointInEllipse(ellipse[0], ellipse[1], ellipse[2], ellipse[3], x, y);
	}
	PlaneParameter getPosePlaneParameter(const TomoPose& pose, const Plane plane)
	{
		tf2::Vector3 pos;
		TomoPose plane_pose;
		PlaneParameter plane_param;
		// Set plane pose
		if(plane == PLANE_YZ) {
			pos = tf2::Vector3(1, 0, 0);
		}
		else if(plane == PLANE_ZX) {
			pos = tf2::Vector3(0, 1, 0);
		}
		else if(plane == PLANE_XY) {
			pos = tf2::Vector3(0, 0, 1);
		}
		plane_pose.pos	= pos;
		plane_pose.quat = tf2::Quaternion(0, 0, 0, 1);
		// trasform the plane relative to pose
		TomoPose Transformed_plane = getAbsolutePose(plane_pose, pose);
		plane_param.origin		   = pose.pos;
		plane_param.normal_vector  = Transformed_plane.pos - pose.pos;
		return plane_param;
	}
}  // namespace tomo
