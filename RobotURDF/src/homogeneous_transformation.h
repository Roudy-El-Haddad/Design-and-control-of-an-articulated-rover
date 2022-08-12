#pragma once

#include <Eigen/Dense>
#include <vector>
#include "LINK.h"

using namespace std;
using namespace Eigen;

//Computes the Homogeneous transformation matrix j_T_i from frame i to frame j
//R is the rotation matrix j_R_i from frame i to frame j 
//p is the position vector j_p_i from frame i to frame j
//p can be of type Vector3d or Matrix<double, 3, 1> (both types are defined to be the same)
MatrixXd HTM(Matrix3d R, Vector3d p) {
	MatrixXd T_trans(4, 4);
	T_trans <<		R,					p,
			   RowVector3d::Zero(),		1;
	return T_trans;
}

//Computes the Homogenous transformation matrix i_T_j from frame j to frame i
//R is the rotation matrix j_R_i from frame i to frame j 
//p is the position vector j_p_i from frame i to frame j
//p can be of type Vector3d or Matrix<double, 3, 1> (both types are defined to be the same)
MatrixXd HTM_inv(Matrix3d R, Vector3d p) {
	MatrixXd T_inv(4, 4);
	T_inv << R.transpose(),         -R.transpose() * p,
			 RowVector3d::Zero(),         1;
	return T_inv;
}

// Computes the HTMs for all the links using their R's and p's 
// Homogeneous Tranformation Matrix (HTM) from local frame to parent frame: p(i)_T_i
inline void HTM_update(std::vector<Link> & link)
{
	for (int i = 1; i <= Link::N_B; ++i)
	{
		link[i].T = HTM(link[i].R.transpose(), -link[i].R.transpose()*link[i].p);
		// or simply, link[i].T = HTM_inv(link[i].R, link[i].p);
	}

}
// i_R_p(i) = link[i].R --> p(i)_R_i = link[i].R.transpose()
// i_p_p(i) = link[i].p --> p(i)_p_i = -link[i].R.transpose()*link[i].p