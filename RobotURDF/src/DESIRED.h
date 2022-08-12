#pragma once

#include <Eigen/Dense>
#include <vector>
#include "LINK.h"
#include "Model.h"

using namespace Eigen;

class Desired {
public: 
	VectorXd CoM_linear_ref;
	VectorXd Base_angular_ref;
	VectorXd q_ref;
	VectorXd qd_ref; 
	VectorXd qdd_ref; 

	double CoM_initial_x; 

	Vector3d p_G_des; 
	Vector3d theta_G_des; 
	Vector3d v_G_linear_des; 
	Vector3d w_G_des; 
	VectorXd v_G_des; 
	VectorXd v_G_dot_des; 

	Vector3d q_b_linear_des;
	Vector3d theta_b_des; 
	VectorXd v_b_des; 
	Vector3d v_b_linear_des; 
	Vector3d w_b_des; 
	Vector3d vd_b_linear_des;

	VectorXd qd_w_des;
	VectorXd qdd_w_des; 

	VectorXd qd_r_des; 
	VectorXd qdd_r_des; 

	/* This function generates the linear COM reference along the x direction, i.e., x_G, (vx)_G, (vx_dot)_G */
	// The values along the y direction are set to be 0
	// The z direction is obtained somewhere else
	void CoM_x_ref_generator(const double & tw, const double & ta, const double & tu, const double & tf, const double & Vu, const double & t) {
		// tw is the waiting time;
		// ta-tw is the acceleration time;
		// tu-ta is the uniform CoM_linear_v_ref_xy[0] time;
		// tf-tu is the deacceleration time;
		CoM_initial_x = 0;
		Eigen::Vector2d CoM_linear_p_ref_xy, CoM_linear_v_ref_xy, CoM_linear_vd_ref_xy;
		if (t < tw) // Stand still phase 
		{
			CoM_linear_vd_ref_xy[0] = 0;
			CoM_linear_v_ref_xy[0] = 0;
			CoM_linear_p_ref_xy[0] = 0 + CoM_initial_x;   
		}
		else if (t >= tw && t < ta) // Acceleration phase 
		{
			CoM_linear_vd_ref_xy[0] = Vu / (ta - tw);
			CoM_linear_v_ref_xy[0] = Vu / (ta - tw) * (t - tw);
			CoM_linear_p_ref_xy[0] = Vu / (ta - tw) * (t - tw) * (t - tw) / 2 + CoM_initial_x;   
		}
		else if (t >= ta && t < tu) // Constant velocity phase 
		{
			CoM_linear_vd_ref_xy[0] = 0.;
			CoM_linear_v_ref_xy[0] = Vu;
			CoM_linear_p_ref_xy[0] = Vu * (t - ta) + Vu * (ta - tw) / 2 + CoM_initial_x; 
		}
		else if (t >= tu && t <= tf) // Deceleration phase 
		{
			CoM_linear_vd_ref_xy[0] = -Vu / (ta - tw);
			CoM_linear_v_ref_xy[0] = Vu - Vu / (ta - tw)*(t - tu);
			CoM_linear_p_ref_xy[0] = Vu * (t - tu) - Vu / (ta - tw) * (t - tu) * (t - tu) / 2 + Vu * (tu - ta) + Vu * (ta - tw) / 2 + CoM_initial_x; 
		}
		else // Steady-state phase  
		{
			CoM_linear_vd_ref_xy[0] = 0;
			CoM_linear_v_ref_xy[0] = 0;
		}
		CoM_linear_ref.resize(6);
		CoM_linear_ref << CoM_linear_p_ref_xy, CoM_linear_v_ref_xy, CoM_linear_vd_ref_xy; 



		q_b_linear_des[0] = CoM_linear_ref[0];
		q_b_linear_des[1] = CoM_linear_ref[1];

		v_b_linear_des[0] = CoM_linear_ref[2];
		v_b_linear_des[1] = CoM_linear_ref[3];

		vd_b_linear_des[0] = CoM_linear_ref[4];
		vd_b_linear_des[1] = CoM_linear_ref[5];

		/*
		q_b_linear_des[0] = 0;
		q_b_linear_des[1] = 0;

		v_b_linear_des[0] = 0;
		v_b_linear_des[1] = 0;

		vd_b_linear_des[0] = 0;
		vd_b_linear_des[1] = 0;
		*/
	}
};