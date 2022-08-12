#pragma once

#include <Eigen/Dense>
#include <vector>
#include "LINK.h"

using namespace Eigen; 

class Contact {
public: 
	MatrixXd p_c;
	MatrixXd c_X_w;

	MatrixXd w_p_c; 
	MatrixXd c_p_w;
	MatrixXd c_R_w; 

	MatrixXd b0_p_c; 
	MatrixXd c_X_b0; 

	MatrixXd c_X_i; 

	
	MatrixXd J_c; // Contact Jacobian v_c_linear = J_c q_dot
	MatrixXd Jc_spatial; // Spatial Contact Jacobian v_c_spatial = J_c_spatial q_dot 
	MatrixXd Jd_qd; // J_c_dot * q_dot 
	MatrixXd Jd_qd_spatial; // J_c_spatial_dot * q_dot 
};

class Wheel {
public:

	MatrixXd b0_p_w;
	MatrixXd b_p_w;
};