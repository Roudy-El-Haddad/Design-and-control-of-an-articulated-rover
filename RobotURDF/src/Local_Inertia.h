#pragma once
#include <Eigen/Dense>
using namespace Eigen;

// inertia structure: compact way to represent inertia 
struct inertia {
	double ixx, ixy, ixz, iyy, iyz, izz;
};


// Generate the inertia matrix of a body from its 6 independent components
MatrixXd I_local(double i_xx, double i_xy, double i_xz, double i_yy, double i_yz, double i_zz) {
	MatrixXd I(3,3); 
	I << i_xx, i_xy, i_xz,
		i_xy, i_yy, i_yz,
		i_xz, i_yz, i_zz; 
	return I; 
}

// Generate the inertia matrix of a body from the inertia structure 
MatrixXd I_local(inertia i) {
	MatrixXd I(3,3);
	I << i.ixx, i.ixy, i.ixz,
		i.ixy, i.iyy, i.iyz,
		i.ixz, i.iyz, i.izz;
	return I;
}
