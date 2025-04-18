#ifndef __TOMO_DATA_OBJECT_H__
#define __TOMO_DATA_OBJECT_H__
// Tomo Data Object library

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstring>
#include <memory>
#include <vector>

#define USE_NEW_OBJECT	// comment out for legacy computation

// do not use all the base struct as object, use TomoJoint and TomoPose instead.
// to create cuztomized object, inherit from the base class and add in more feature accordingly.
namespace tomo
{
	// JOINT //
	struct JointBase
	{
		virtual ~JointBase() {};
		std::vector<double> joints;
		std::string joint_name;
		std::string joint_plangroup;
		std::string toString() const;
		void reset();
		void setData(std::vector<double> joint_values, const bool isRadian = true, const std::string& planningGroup = "",
					 const std::string& name = "");
	};
#ifdef USE_NEW_OBJECT
	struct TomoJoint final : public JointBase
	{
		TomoJoint();
		TomoJoint(const std::vector<double>& joint_values, const bool isRadian = true, const std::string& planningGroup = "", const std::string& name ="");
		void offset(const std::vector<double>& offset);
	};
#else
	struct TomoHandJoints final : JointBase
	{
		TomoHandJoints();
		TomoHandJoints(std::vector<double> joint_values, const bool isRadian = true, const std::string& planningGroup = "",
					   const std::string& name = "");
		TomoHandJoints(const std::string& planningGroup, double j11, double j12, double j21, double j31, double j41, double j51 = 0,
					   double j53 = 0, double j61 = 0, bool isRadian = true);
		TomoHandJoints(const std::string& planningGroup, const std::string& name, std::vector<double> angles, bool isRadian = true);
		void setData(const std::string& planningGroup, double j11, double j12, double j21, double j31, double j41, double j51, double j53,
					 double j61, bool isRadian = true);
	};
	struct TomoOSPlus final : public JointBase
	{
		TomoOSPlus()
		{
			joints = {0};
		};
	};
	struct TomoHeadJoints final : public JointBase
	{
		TomoHeadJoints();
		TomoHeadJoints(double j0, double j1, bool isRadian = true);
		TomoHeadJoints(const std::string& name, const std::vector<double>& angles, bool isRadian = true);
		void setData(double j0, double j1, bool isRadian = true);
	};
#endif
	// POSE //
	struct PoseBase
	{
		virtual ~PoseBase() {};
		tf2::Vector3 pos;
		tf2::Quaternion quat;
		std::string pose_name;
		std::string pose_plangroup;
		std::string toString() const;
		std::vector<double> toVector() const;
		geometry_msgs::msg::Pose toPoseMsgs() const;
		void reset();
		void setData(std::vector<double> pose_values, const std::string& planningGroup = "", const std::string& name = "");
	};
#ifdef USE_NEW_OBJECT
	struct TomoPose final : public PoseBase
	{
		TomoPose();
		TomoPose(const tf2::Vector3& p, const tf2::Quaternion& q);
		TomoPose(double x, double y, double z, double qx = 0, double qy = 0, double qz = 0, double qw = 1);
		//getter setter
		tf2::Matrix3x3 getRotationMat() const;
		tf2::Transform getTransform() const;
		std::vector<double> getRPY() const;
		//offsets
		void offsetPos(const tf2::Vector3& offset);
		void offsetQuat(const tf2::Quaternion& offset);
		void offsetXYZ(const double x_offset, const double y_offset, const double z_offset);
		void offsetRPY(const double r_offset, const double p_offset, const double y_offset);
	};
#endif
}  // namespace tomo
#endif