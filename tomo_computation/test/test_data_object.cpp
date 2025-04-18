#include "tomo_computation.h"
// very simple test of data object, should change this to gtest later

using namespace tomo;

int main()
{
	// tomo joint
	std::vector<double> joint_values = {0, 0, 0};
	std::vector<double> offset		 = {0.1, 0.2, 0.3};
	bool isRadian					 = true;
	// constructor
	std::cout << "construct tomo joint ..." << std::endl;
	TomoJoint tomo_joint1;
	TomoJoint tomo_joint2(joint_values);  // here asume is radian
	TomoJoint tomo_joint3(joint_values, isRadian);
	TomoJoint tomo_joint4(joint_values, isRadian, "RARM", "raTestJoint");
	// accessing elements
	std::cout << "getting joint values: " << tomo_joint4.joints[0] << ", " << tomo_joint4.joints[1] << ", " << tomo_joint4.joints[2]
			  << std::endl;
	std::cout << "getting joint name: " << tomo_joint4.joint_name << std::endl;
	std::cout << "getting joint plan group: " << tomo_joint4.joint_plangroup << std::endl;
	std::cout << "Offset tomo joint ..." << std::endl;
	tomo_joint4.offset(offset);
	std::cout << "after offset joint values: " << tomo_joint4.joints[0] << ", " << tomo_joint4.joints[1] << ", " << tomo_joint4.joints[2]
			  << std::endl;
	// tomo pose
	tf2::Vector3 p(1, 2, 3);
	tf2::Quaternion q(0, 0, 0, 1);
	tf2::Vector3 p_off(0.1, 0.2, 0.3);
	tf2::Quaternion q_off(0, 1, 0, 1);
	// constructor
	std::cout << "construct tomo pose ..." << std::endl;
	TomoPose tomo_pose1;
	TomoPose tomo_pose2(p, q);
	TomoPose tomo_pose3(1, 2, 3, 0, 0, 0, 1);
	// accessing elements
	std::cout << "getting pose string " << tomo_pose3.toString() << std::endl;
	tomo_pose3.offsetPos(p_off);
	std::cout << "offsetPos pose " << tomo_pose3.toString() << std::endl;
	tomo_pose3.offsetQuat(q_off);
	std::cout << "offsetQuat pose " << tomo_pose3.toString() << std::endl;
	tomo_pose3.offsetXYZ(p_off[0], p_off[1], p_off[2]);
	std::cout << "offsetXYZ pose " << tomo_pose3.toString() << std::endl;
	tomo_pose3.offsetRPY(M_PI_2, M_PI_2, M_PI_2);  // rotate along each axis by 90 deg
	std::cout << "offsetRPY pose " << tomo_pose3.toString() << std::endl;
	return 0;
}
