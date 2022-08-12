#pragma once
#include <Eigen/Dense>
#include "skew.h"

using namespace Eigen; 

// Computes the spatial inertia from the mass, inertia, and centroidal position
MatrixXd I_spatial(double m, Matrix3d I, Vector3d c) {
	MatrixXd I_s(6,6); 
	Matrix3d skewc = S(c);
	I_s << I + m * skewc * skewc.transpose(), m * skewc,
		m * skewc.transpose(), m * Matrix3d::Identity(3, 3);
	return I_s; 
}