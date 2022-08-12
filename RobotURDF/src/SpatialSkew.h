#pragma once

#include <Eigen/Dense>
#include "skew.h"

using namespace Eigen;

//Computes the skew symetric matrix corresponding to a 6D (spatial) motion vector v
// Let m1 and m2 be a motion vectors, SS(m1)*m2 = m1 x m2 (spatial cross product) 
Matrix<double, 6, 6> SS(Matrix<double, 6, 1> v) {
	Matrix<double, 6, 6> Skew;

	Vector3d upper=v.block(0,0,3,1); // upper 3 elements of v
	Vector3d lower=v.block(3, 0, 3, 1); // lower 3 elements of v 

	Skew.block<3, 3>(0, 0).array() = S(upper);
	Skew.block<3, 3>(0, 3).setZero();
	Skew.block<3, 3>(3, 0).array() = S(lower);
	Skew.block<3, 3>(3, 3).array() = S(upper);

	return Skew;
}

//Computes the skew symetric matrix corresponding to a 6D (spatial) motion vector v
// Let m be a motion vector and f be a force vector, SSF(m)*f = m x f (spatial cross product) 
// SSF(v) = - SS(v)^T 
Matrix<double, 6, 6> SSF(Matrix<double, 6, 1> v) {
	Matrix<double, 6, 6> Skew;
	Skew = -SS(v).transpose(); 
	return Skew;
}
