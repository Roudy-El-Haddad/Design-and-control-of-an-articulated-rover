#pragma once
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <unordered_map>

#include "LINK.h"
#include "JOINT.h"
#include "CONTACT.h"

//#include "JointCallback.h"

#include "homogeneous_transformation.h"
#include "skew.h"
#include "SpatialSkew.h"
#include "X_transform.h"

using namespace Eigen; 

// Updates the following contact parameters: 
// c_X_b0 
// w_p_c
// c_R_w
// c_p_w 
// c_X_w
// b_p_w 
// b0_p_w 
// b0_p_c 
inline void contact_link_update(std::vector<Link> & link)
{
	// Get the wheel center positions.
	
	Eigen::Matrix4d b_T_w1, b_T_w2, b_T_w3; // Compute the floatingBase_T_wheel
	b_T_w1 = link[2].T  * link[3].T  * link[4].T;
	b_T_w2 = link[5].T  * link[6].T  * link[7].T;
	b_T_w3 = link[8].T * link[9].T * link[10].T;

	Eigen::Matrix4d b0_T_w1, b0_T_w2, b0_T_w3; // Compute the inertial_T_wheel
	b0_T_w1 = link[1].T * b_T_w1;
	b0_T_w2 = link[1].T * b_T_w2;
	b0_T_w3 = link[1].T * b_T_w3;

	Eigen::Vector3d z_w1, z_w2, z_w3; // Extract the z_w of each wheel from b0_T_w
	// b0_T_w = [ x_w  y_w  z_w  p_w
	//             0    0    0    1  ]
	// b0_T_w.block<3,1>(0,2) is z_w : The orientation of the z_w axis expressed in the inertial frame 
	z_w1 = b0_T_w1.block<3, 1>(0, 2);
	z_w2 = b0_T_w2.block<3, 1>(0, 2);
	z_w3 = b0_T_w3.block<3, 1>(0, 2);
	// Note: z_w is an abbreviation for b0_z_w or 0_z_w
	// Note: y_c = z_w 

	Eigen::Matrix<double, 4, 1> n_c1, n_c2, n_c3; // Compute the normal vector of each wheel median plane
	// Let z_w = [a b c]^T 
	// n_c =[a*c/(a^2+b^2), b*c/(a^2+b^2), -1, 0]^T 
	n_c1 << z_w1[0] * z_w1[2] / (pow(z_w1[0], 2) + pow(z_w1[1], 2)), z_w1[1] * z_w1[2] / (pow(z_w1[0], 2) + pow(z_w1[1], 2)), -1, 0;
	n_c2 << z_w2[0] * z_w2[2] / (pow(z_w2[0], 2) + pow(z_w2[1], 2)), z_w2[1] * z_w2[2] / (pow(z_w2[0], 2) + pow(z_w2[1], 2)), -1, 0;
	n_c3 << z_w3[0] * z_w3[2] / (pow(z_w3[0], 2) + pow(z_w3[1], 2)), z_w3[1] * z_w3[2] / (pow(z_w3[0], 2) + pow(z_w3[1], 2)), -1, 0; 


	Eigen::Matrix3d b0_R_c1, b0_R_c2, b0_R_c3; // Compute 0_R_c or b0_R_c
	// 0_R_c = [x_c y_c z_c]
	// x_c = [b  -a  0]^T / norm([b  -a  0]^T)
	// y_c = z_w, note: z_w is already a unit vector, i.e. norm(z_w) = 1 
	// z_c = - [a*c/(a^2+b^2), b*c/(a^2+b^2), -1]^T / norm ([a*c/(a^2+b^2), b*c/(a^2+b^2), -1]^T)
	// z_c = - n_c.head<3>() / norm(n_c.head<3>()) where n_c.head<3>() is the vector of the first 3 elements of n_c 
	b0_R_c1.col(0) << z_w1[1], -z_w1[0], 0;   b0_R_c1.col(0) = b0_R_c1.col(0) / b0_R_c1.col(0).norm();
	b0_R_c1.col(1) << z_w1;                   b0_R_c1.col(1) = b0_R_c1.col(1) / b0_R_c1.col(1).norm();
	b0_R_c1.col(2) << -n_c1.head<3>();        b0_R_c1.col(2) = b0_R_c1.col(2) / b0_R_c1.col(2).norm();

	b0_R_c2.col(0) << z_w2[1], -z_w2[0], 0;    b0_R_c2.col(0) = b0_R_c2.col(0) / b0_R_c2.col(0).norm();
	b0_R_c2.col(1) << z_w2;                    b0_R_c2.col(1) = b0_R_c2.col(1) / b0_R_c2.col(1).norm();
	b0_R_c2.col(2) << -n_c2.head<3>();         b0_R_c2.col(2) = b0_R_c2.col(2) / b0_R_c2.col(2).norm();

	b0_R_c3.col(0) << z_w3[1], -z_w3[0], 0;    b0_R_c3.col(0) = b0_R_c3.col(0) / b0_R_c3.col(0).norm();
	b0_R_c3.col(1) << z_w3;                    b0_R_c3.col(1) = b0_R_c3.col(1) / b0_R_c3.col(1).norm();
	b0_R_c3.col(2) << -n_c3.head<3>();         b0_R_c3.col(2) = b0_R_c3.col(2) / b0_R_c3.col(2).norm();

	Eigen::Vector4d b0_p_c1, b0_p_c2, b0_p_c3; // Compute 0_p_c or b0_p_c 
	// b0_T_w = [ x_w  y_w  z_w  p_w
	//             0    0    0    1  ]
	// b0_T_w.col(3) is [b0_p_w; 1] : The wheel center (homogeneous) position expressed in the inertial frame 
	// The term homogeneous is used to refer to the fact that we add a 4th component to the position which is a 1 
	// If 0_p_c and 0_p_w are taken as homogeneous position (4D vectors), we have 
	// 0_p_c = 0_p_w - r * [z_c(0) z_c(1) z_c(2) 1]^T 
	// 0_p_c = 0_p_w + r * n_c / norm(n_c)
	// Note: The positions presented in this code until now were 3D vectors 
	b0_p_c1 = b0_T_w1.col(3) + (Link::r) * n_c1 / n_c1.norm();
	b0_p_c2 = b0_T_w2.col(3) + (Link::r) * n_c2 / n_c3.norm();
	b0_p_c3 = b0_T_w3.col(3) + (Link::r) * n_c3 / n_c3.norm();

	/* Compute the contact parameter for each of the contacts */

	// Compute c_X_b0 = X (c_R_b0, c_p_b0)
	// c_R_b0 =(b0_R_c)^T and 
	// c_p_b0 = - (c_R_b0) b0_p_c = - (b0_R_c)^T b0_p_c
	Eigen::Vector3d c1_p_b0, c2_p_b0, c3_p_b0; 
	c1_p_b0 = -(b0_R_c1.transpose()) * b0_p_c1.head<3>();
	c2_p_b0 = -(b0_R_c2.transpose()) * b0_p_c2.head<3>();
	c3_p_b0 = -(b0_R_c3.transpose()) * b0_p_c3.head<3>();

	link[4].contact.c_X_b0 = X(b0_R_c1.transpose(), c1_p_b0);
	link[7].contact.c_X_b0 = X(b0_R_c2.transpose(), c2_p_b0);
	link[10].contact.c_X_b0 = X(b0_R_c3.transpose(), c3_p_b0);

	// Compute w_T_b0 = (b0_T_w)^-1  
	Eigen::Matrix<double, 4, 4> w1_T_b0, w2_T_b0, w3_T_b0;
	w1_T_b0 = HTM_inv(b0_T_w1.topLeftCorner<3, 3>(), b0_T_w1.topRightCorner<3, 1>());
	w2_T_b0 = HTM_inv(b0_T_w2.topLeftCorner<3, 3>(), b0_T_w2.topRightCorner<3, 1>());
	w3_T_b0 = HTM_inv(b0_T_w3.topLeftCorner<3, 3>(), b0_T_w3.topRightCorner<3, 1>());

	// Compute w_p_c = (w_T_b0 * b0_p_c).head<3>()
	// .head<3>() is used to extract the 3d position as b0_p_c was computed as a 4d position ([x, y, z, 1]^T) 
	link[4].contact.w_p_c = (w1_T_b0 * b0_p_c1).head<3>();
	link[7].contact.w_p_c = (w2_T_b0 * b0_p_c2).head<3>();
	link[10].contact.w_p_c = (w3_T_b0 * b0_p_c3).head<3>();

	// Compute c_R_w  = (c_R_b0)*(b0_R_w) 
	link[4].contact.c_R_w = b0_R_c1.transpose() * b0_T_w1.topLeftCorner<3, 3>();
	link[7].contact.c_R_w = b0_R_c2.transpose() * b0_T_w2.topLeftCorner<3, 3>();
	link[10].contact.c_R_w = b0_R_c3.transpose() * b0_T_w3.topLeftCorner<3, 3>(); 

	// Compute c_p_w = - c_R_w * w_p_c 
	link[4].contact.c_p_w = -(link[4].contact.c_R_w) * (link[4].contact.w_p_c);
	link[7].contact.c_p_w = -(link[7].contact.c_R_w) * (link[7].contact.w_p_c);
	link[10].contact.c_p_w = -(link[10].contact.c_R_w) * (link[10].contact.w_p_c);

	// Compute c_X_w = X (c_R_w, c_p_w)
	link[4].contact.c_X_w = X(link[4].contact.c_R_w, link[4].contact.c_p_w);
	link[7].contact.c_X_w = X(link[7].contact.c_R_w, link[7].contact.c_p_w);
	link[10].contact.c_X_w = X(link[10].contact.c_R_w, link[10].contact.c_p_w); 

	// Extract b_p_w from b_T_w  
	link[4].wheel.b_p_w = b_T_w1.topRightCorner<3, 1>();
	link[7].wheel.b_p_w = b_T_w2.topRightCorner<3, 1>();
	link[10].wheel.b_p_w = b_T_w3.topRightCorner<3, 1>();

	/*joint.legtip.pIN1_3D_all << link[4].legtip.pIN1_3D, link[8].legtip.pIN1_3D,
								link[12].legtip.pIN1_3D, link[16].legtip.pIN1_3D; */

	// Compute b0_p_w = (b0_T_b * [b_x_w, b_y_w, b_z_w, 1]^T).head<3> 
	link[4].wheel.b0_p_w = (link[1].T * b_T_w1.col(3)).head<3>();
	link[7].wheel.b0_p_w = (link[1].T * b_T_w2.col(3)).head<3>();
	link[10].wheel.b0_p_w = (link[1].T * b_T_w3.col(3)).head<3>();

	// Get b0_p_c 
	link[4].contact.b0_p_c = b0_p_c1.head<3>();
	link[7].contact.b0_p_c = b0_p_c2.head<3>();
	link[10].contact.b0_p_c = b0_p_c3.head<3>();
}