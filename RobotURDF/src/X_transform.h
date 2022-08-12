#pragma once

#include <Eigen/Dense>
#include <vector>
#include "skew.h"
//#include "Composite_Inertia.h"
#include "LINK.h"

using namespace Eigen;

//Computes the spatial transform j_X_i from frame i to frame j
//R is the rotation matrix j_R_i from frame i to frame j 
//p is the position vector j_p_i from frame i to frame j
//p can be of type Vector3d or Matrix<double, 3, 1> (both types are defined to be the same)
MatrixXd X(Matrix3d R, Vector3d p) {
	MatrixXd X_trans(6, 6);
	// j_X_i = [	j_R_i			0_3x3
	//		     S(j_p_i)*j_R_i		j_R_i ]
	X_trans.block<3, 3>(0, 0).array() = R;
	X_trans.block<3, 3>(0, 3).setZero();
	X_trans.block<3, 3>(3, 0).array() = S(p)*R;
	X_trans.block<3, 3>(3, 3).array() = R;

	return X_trans; 
}

//Computes the spatial transform i_X_j from frame j to frame i
//R is the rotation matrix j_R_i from frame i to frame j 
//p is the position vector j_p_i from frame i to frame j
//p can be of type Vector3d or Matrix<double, 3, 1> (both types are defined to be the same)
MatrixXd Xinv(Matrix3d R, Vector3d p) {
	MatrixXd X_trans(6, 6);
	// (j_X_i)^-1 = [ 		(j_R_i)^T			0_3x3
	//				 - (j_R_i)^T * S(j_p_i)		(j_R_i)^T ]
	X_trans.block<3, 3>(0, 0).array() = R.transpose();
	X_trans.block<3, 3>(0, 3).setZero();
	X_trans.block<3, 3>(3, 0).array() = R.transpose()*S(R*p);
	X_trans.block<3, 3>(3, 3).array() = R.transpose();

	return X_trans;
}

// Computes the spatial transform from the base frame to the local frame 
// Performs the calculation for all links and stores the values in the link parameter
void n_X_1_Cal(std::vector<Link>& link){
	link[1].n_X_1 = Eigen::Matrix<double, 6, 6>::Identity(); // 1_X_1 = Identity(6,6)
	for (int i = 2; i <= Link::N_B; ++i)
	{
		if (link[i].parent == 1) 
			{link[i].n_X_1 = link[i].X; } // Because link[1].n_X_1 = Identity(6,6) 
		else 
			{link[i].n_X_1 = link[i].X * link[link[i].parent].n_X_1; } // Composition Law for X 
			  														  // i_X_1 = i_X_pi * pi_X_1
	}
}


// Computes the spatial transform from frame i to frame j 
MatrixXd j_X_i(std::vector<Link>& link, int j, int i) {
	MatrixXd jXi = Eigen::Matrix<double, 6, 6>::Identity(); // Initialization: i_X_i = Identity(6,6)
	if (j > i) {
		// j_X_i = link[j].X * ... * link[i+1].X   if i < j 
		for (int k = i + 1; k <= j; k++)
		{
			jXi = link[k].X * jXi;
		}
	}
	else if (i > j){
		// j_X_i = link[i+1].X_inv * ... * link[j].X_inv   if i > j 
		for (int k = i + 1; k <= j; k++)
		{
			jXi = jXi * link[k].X_inv;
		}
	}
	return jXi; 
}

// Computes the spatial transform from frame b to frame G 
void b_X_G(std::vector<Link>& link) {
	MatrixXd C_I_b = link[1].I_C; 
	// Note that we have to substract 1 from the index of the row and column
	// because matrices in C++ are indexed starting at 0 instead of 1
	// b_p_G = (1/m_G)*[C_I_b(3, 5) C_I_b(1, 6) C_I_b(2, 4)]; 
	Link::b_p_G << C_I_b(2, 4), C_I_b(0, 5), C_I_b(1, 3);
	Link::b_p_G = (1.0 / Link::mass)*Link::b_p_G;

	Link::b_R_G = link[1].R; // b_R_G = b_R_0

	Link::b_X_G = X(Link::b_R_G, Link::b_p_G);
}

/* Spatial Transforms relating to the contacts */
// The spatial transforms c_X_w and c_X_b0 
// are computed by the function inside of the Contact_link_update.h header 







/*MatrixXd c_X_w(std::vector<Link>& link, int i) {
MatrixXd jXi = Eigen::Matrix<double, 6, 6>::Identity(); // Initialization: i_X_i = Identity(6,6)

return jXi;
}*/
/*
MatrixXd pi_X_i(Matrix3d R, Vector3d p) {
	MatrixXd X_trans(6, 6);

	X_trans.block<3, 3>(0, 0).array() = R;
	X_trans.block<3, 3>(0, 3).setZero();
	X_trans.block<3, 3>(3, 0).array() = S(p)*R;
	X_trans.block<3, 3>(3, 3).array() = R;

	return X_trans;
}
*/
