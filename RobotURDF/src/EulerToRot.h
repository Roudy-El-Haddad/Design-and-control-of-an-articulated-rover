#pragma once

#include <Eigen/Dense>
#include<stdio.h>
#include <math.h>

using namespace Eigen;

//Converts Euler Angles into the equivalent Rotation Matrix 
// yaw, pitch, and roll are the XYZ fixed Euler Angles
//All angles are specified in radians 
Matrix3d EulerAngle_To_RotMatrix(double yaw, double pitch, double roll) {
	Matrix3d R; 

	// This algorithm is used to calculate j_R_i in which j is the parent link or joint.
	R(0, 0) = cos(pitch)*cos(roll);
	R(0, 1) = sin(yaw)*sin(pitch)*cos(roll) - sin(roll)*cos(yaw);
	R(0, 2) = sin(yaw)*sin(roll) + sin(pitch)*cos(yaw)*cos(roll);
	R(1, 0) = sin(roll)*cos(pitch);
	R(1, 1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
	R(1, 2) = -sin(yaw)*cos(roll) + sin(pitch)*sin(roll)*cos(yaw);
	R(2, 0) = -sin(pitch);
	R(2, 1) = sin(yaw)*cos(pitch);
	R(2, 2) = cos(yaw)*cos(pitch);

	return R; 
}