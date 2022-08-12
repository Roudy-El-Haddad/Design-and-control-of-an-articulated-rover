#pragma once

#include <Eigen/Dense>

using namespace Eigen;

//Computes the skew symetric matrix corresponding to vector v
//v can be of type Vector3d or Matrix<double, 3, 1> (both types are defined to be the same)
Matrix3d S(Vector3d v) {
Matrix3d Skew; 

Skew << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;

return Skew; 
}
