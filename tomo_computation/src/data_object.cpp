#include "data_object.h"
#include "conversion.h"
namespace tomo
{
	void JointBase::setData(std::vector<double> joint_values, const bool isRadian, const std::string& planningGroup,
							const std::string& name)
	{
		joint_name		= name;
		joint_plangroup = planningGroup;
		joints			= isRadian ? joint_values : degToRad(joint_values);
	}
	std::string JointBase::toString() const
	{
		std::string output = "joint_name: " + joint_name + "planning_group: " + joint_plangroup + "joint_values: ";
		output += vecToString(joints);
		return output;
	}
	void JointBase::reset()
	{
		joint_name		= "";
		joint_plangroup = "";
		memset(&joints[0], 0, joints.size() * sizeof joints[0]);
	}
	TomoJoint::TomoJoint()
	{
		// empty joint
		reset();
	}
	TomoJoint::TomoJoint(const std::vector<double>& joint_values, const bool isRadian, const std::string& planningGroup,
						 const std::string& name)
	{
		setData(joint_values, isRadian, planningGroup, name);
	}
	void TomoJoint::offset(const std::vector<double>& offset)
	{
		if(offset.size() != joints.size()) {
			std::cerr << "TomoJoint::offset: offset vector size mismatch" << std::endl;
			return;
		}
		joints = eigenVecToVec(vecToEigenVec(joints) + vecToEigenVec(offset));
	}
#ifdef USE_NEW_OBJECT

#else
	TomoHandJoints::TomoHandJoints()
	{
		// setData(std::vector<double>(15, 0), true, "", "");
	}
	TomoHandJoints::TomoHandJoints(std::vector<double> joint_values, const bool isRadian, const std::string& planningGroup,
								   const std::string& name)
	{
		// setData(joint_values, isRadian, planningGroup, name);
	}

	TomoHeadJoints::TomoHeadJoints()
	{
		setData(std::vector<double>(2, 0), true, "", "");
	}
	TomoHeadJoints::TomoHeadJoints(std::vector<double> joint_values, const bool isRadian, const std::string& planningGroup,
								   const std::string& name)
	{
		setData(joint_values, isRadian, planningGroup, name);
	}
#endif
	std::string PoseBase::toString() const
	{
		std::string output = "pose_name: " + pose_name + "planning_group: " + pose_plangroup + "Pose: ";
		output			   = "Position: " + tfvec3ToString(pos) + "\nQuaternion:" + tfquatToString(quat) + "\n";
		return output;
	}
	std::vector<double> PoseBase::toVector() const
	{
		return poseQuatToVec(pos, quat);
	}
	geometry_msgs::msg::Pose PoseBase::toPoseMsgs() const
	{
		return poseQuatToPoseMsgs(pos, quat);
	}
	void PoseBase::setData(std::vector<double> pose_values, const std::string& planningGroup, const std::string& name)
	{
		pose_name	   = name;
		pose_plangroup = planningGroup;
		vecToPoseQuat(pose_values, pos, quat);
	}
	void PoseBase::reset()
	{
		std::vector<double> p = {0, 0, 0, 0, 0, 0, 1};
		setData(p);
	}
	TomoPose::TomoPose()
	{
		// empty pose
		reset();
	}
	TomoPose::TomoPose(const tf2::Vector3& p, const tf2::Quaternion& q)
	{
		reset();
		pos	 = p;
		quat = q;
	}
	TomoPose::TomoPose(double x, double y, double z, double qx, double qy, double qz, double qw)
	{
		setData({x, y, z, qx, qy, qz, qw});
	}
	tf2::Matrix3x3 TomoPose::getRotationMat() const
	{
		return tf2::Matrix3x3(quat);
	}
	tf2::Transform TomoPose::getTransform() const
	{
		return tf2::Transform(quat, pos);
	}
	std::vector<double> TomoPose::getRPY() const
	{
		double roll, pitch, yaw;
		getRotationMat().getRPY(roll, pitch, yaw);
		return {roll, pitch, yaw};
	}
	void TomoPose::offsetPos(const tf2::Vector3& offset)
	{
		pos += offset;
	}
	void TomoPose::offsetQuat(const tf2::Quaternion& offset)
	{
		quat += offset;
	}
	void TomoPose::offsetXYZ(const double x_offset, const double y_offset, const double z_offset)
	{
		tf2::Vector3 offset(x_offset, y_offset, z_offset);
		offsetPos(offset);
	}
	void TomoPose::offsetRPY(const double r_offset, const double p_offset, const double y_offset)
	{
		tf2::Quaternion q_offset;
		q_offset.setRPY(r_offset, p_offset, y_offset);
		offsetQuat(q_offset);
	}
}  // namespace tomo
