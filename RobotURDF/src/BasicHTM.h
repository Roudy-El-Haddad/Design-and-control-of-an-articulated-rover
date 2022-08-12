#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>
#include <stdlib.h>
#include <math.h>
//#include "tf_conversions/tf_eigen.h" // uncomment to use RotMatrix_To_EulerAngles

using namespace std;
using namespace Eigen;

// Compute a Rotation matrix for a rotation around the z axis by the specified angle 
Matrix3d rotz(const double & ga)
{
	return (Matrix3d() << cos(ga), -sin(ga), 0,
						sin(ga), cos(ga), 0,
						0, 0, 1).finished();
}
// Compute a Rotation matrix for a rotation around the y axis by the specified angle 
Matrix3d roty(const double & be)
{
	return (Matrix3d() << cos(be), 0, sin(be),
								0, 1, 0,
						-sin(be), 0, cos(be)).finished();
}
// Compute a Rotation matrix for a rotation around the x axis by the specified angle 
Matrix3d rotx(const double & al)
{
	return (Matrix3d() << 1, 0, 0,
						0, cos(al), -sin(al),
						0, sin(al), cos(al)).finished();
}

/*
Vector3d RotMatrix_To_EulerAngles(const Matrix3d & R)
{
	tf::Matrix3x3 temp;
	tf::matrixEigenToTF(R, temp);
	double roll, pitch, yaw;
	tf::Matrix3x3(temp).getRPY(roll, pitch, yaw);
	return(Vector3d() << roll, pitch, yaw).finished();
}
*/
