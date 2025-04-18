#ifndef __TOMO_DEFINITION_H__
#define __TOMO_DEFINITION_H__
// Defines ID of groups of joints and end effectors and their names
// Udupa; Dec'2020

// Made independent of different TomO builds
// These definitions are common to all  projects
// Additional project spcific definitions go to the project's package, and include this header
// Udupa; Feb'2022
// Tomo Definition

#include <map>
#include <string>

#include <variant>
#include <vector>

namespace tomo
{
	typedef enum
	{
		PLANE_XY,
		PLANE_YZ,
		PLANE_ZX
	} Plane;

	typedef enum
	{
		AXIS_X,
		AXIS_Y,
		AXIS_Z
	} Axis;

	typedef struct
	{
		// origin of the plane in global frame
		tf2::Vector3 origin;
		tf2::Vector3 normal_vector;
	} PlaneParameter;

	typedef enum
	{
		PLAN_PTP,
		PLAN_LINEAR
	} PlannerId;

}  // namespace tomo

#endif