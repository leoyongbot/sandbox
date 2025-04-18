#include "tomo_computation.h"
#include <cstdio>
// check the CMakeList.txt, package.xml and  how to include the tomo_computation library

using namespace tomo;
int main()
{
	// object
	std::vector<double> joint = {1.0, 1.0, 1.0};
	std::vector<double> offset(3, 0.0);
	std::string goal_name = "goal_name";
	JointGoal joint_goal;
	joint_goal.goal_joint = joint;
	joint_goal.goal_joint.offset(offset);
	joint_goal.copyFrom(joint_goal, goal_name, offset, PLAN_LINEAR);
	joint_goal.reset();
	std::cout << "tomo test compute done" << std::endl;
	return 0;
}
