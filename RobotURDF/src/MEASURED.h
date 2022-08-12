#pragma once

#include <Eigen/Dense>
#include <vector>
#include "LINK.h"
#include "Model.h"

using namespace Eigen;

class Measured {
public: 
	Vector3d q_b_angular; 
	Vector3d q_b_linear; 
	VectorXd q_actuated = MatrixXd::Zero(9,1);

	Vector3d w_b;
	Vector3d v_b_linear;
	VectorXd qd_actuated = MatrixXd::Zero(9,1);

	Vector3d vd_b_linear; 
	Vector3d wd_b; 

	VectorXd Q_mes;
	VectorXd Qd_mes; 
	VectorXd Qdd_mes; 

	VectorXd tau_mes = MatrixXd::Zero(9,1);
	Measured(){
		q_b_angular<<0,0,0;
		q_b_linear<<0,0,0.4425; 
		q_actuated<<0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0; 
	}
};