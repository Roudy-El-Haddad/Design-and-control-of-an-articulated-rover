#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "X_transform.h"
#include "skew.h"

#include "LINK.h"
//#include "Model.h"
#include "model_initialization.h"

//#include "JointCallback.h"

using namespace Eigen; 

/*Update the Centrifugal & Coriolis force for Joint Space Model*/
// Let us consider the dynamic model: M*qdd + C_G = tau where C_G = C + G
// If we set gravity to be zero, G=0, hence, C_G = C -> Model: M*qdd + C = tau 
// Computes C_G = C 
// This is the RNEA algorithm is gravity set to 0 
inline void floating_base_RNEA_C(std::vector<Link> & link) {
	Link::C.setZero(); // Initialize C 
	float g_value = 0; // No gravity (Without gravity) 

	// v_b = Phi_b * qd_b but Phi_b is the 6x6 identity matrix
	link[1].v = link[1].joint.qd; 

	link[1].X = X(link[1].R, link[1].p);
	link[1].X_inv = Xinv(link[1].R, link[1].p);

	link[1].vi_x_vJ << 0, 0, 0, 0, 0, 0;

	// a_b = -g*[0; 0; 0; 1_z_0] where 1_z_0 is the orientation of the z0 axis expressed in frame 1
	// The g vector is along - z0 
	link[1].a = -g_value * link[1].X.col(5); 

	int p; // variable used to store the link parent number (used for concisness)
	// loop forward
	for (int i = 2; i <= Link::N_B; i++) //sort the keys in increased order
	{
		p = link[i].parent;
		link[i].X = X(link[i].R, link[i].p); // Compute i_X_p(i)
		link[i].X_inv = Xinv(link[i].R, link[i].p); // Compute p(i)_X_i

		// Calculate the v_i in local frame
		link[i].vJ = link[i].Phi * link[i].joint.qd; // Compute Velocity accross Joint i: vJ = Phi * qd
		link[i].v = link[i].X * link[p].v + link[i].vJ; // v_i = X_i * v_p(i) + Phi_i * qd_i 

		// Calculate the a_i in local frame
		link[i].vi_x_vJ = SS(link[i].v) * link[i].vJ; // v_i x v_J
		link[i].a = link[i].X * link[p].a + link[i].vi_x_vJ; // a_i = i_X_p(i) * a_p(i) + v_i x v_J + c_J (but c_J = 0 for revolute Joints) 

		// Calculate the f_i in local frame
		link[i].v_Iv = SSF(link[i].v) * link[i].I_spatial * link[i].v; // Compute v x Iv, SSF is used because Iv is analoguous to a force vector
		link[i].f = link[i].I_spatial * link[i].a + link[i].v_Iv; // f_i = I_i * a_i + v x Iv

	}
	// Calculate the f1 in local frame
	link[1].v_Iv = SSF(link[1].v) * link[1].I_spatial * link[1].v; // v1 x I1 v1 (v_Iv for link 1) 
	link[1].f = link[1].I_spatial * link[1].a + link[1].v_Iv; // f_1 = I1 * a1 + v1 x I1 v1

	MatrixXd X_F; // Spatial transform for force vectors from local to parent: (p(i)_X_i)^F 
	// loop backward
	for (int i = Link::N_B; i > 1; i--)
	{
		p = link[i].parent;

		// C[i+4] = (Phi_i)^T * f_i = 3rd component of f_i because Phi_i=[0,0,1,0,0,0] (revolute) 
		// C[i+4] = (link[i].Phi.transpose() * link[i].f).value();
		Link::C[i + 4] = link[i].f[2]; // Compute element of C corresponding to link i 

		X_F = link[i].X.transpose(); // X_F = (p(i)_X_i)^-T = (i_X_p(i))^T
		link[p].f = link[p].f + X_F * link[i].f; // f_p(i) = f_p(i) + X_F * f_i 

	}

	// C.head(6) = link[1].Phi.transpose() * link[1].f; // = link[1].f because link[1].Phi = Identity
	Link::C.head(6) = link[1].f; // Compute the element of C_G corresponding to link 1
}

/*Update the Centrifugal & Coriolis force and gravity force for Joint Space Model*/ 
// Let us consider the dynamic model: M*qdd + C_G = tau where C_G = C + G
// Computes C_G = C + G 
// This is the RNEA algorithm 
inline void floating_base_RNEA(std::vector<Link> & link)
{
	Link::C_G.setZero(); // Initialize C_G 
	float g_value = -9.8; // Normal gravity (With gravity) 

	// v_b = Phi_b * qd_b but Phi_b is the 6x6 identity matrix
	link[1].v = link[1].joint.qd;

	//link[1].v_phi = SS(link[1].v);
	link[1].vi_x_vJ << 0, 0, 0, 0, 0, 0;

	// a_b = -g*[0; 0; 0; 1_z_0] where 1_z_0 is the orientation of the z0 axis expressed in frame 1
	// The g vector is along - z0 
	link[1].a = -g_value * link[1].X.col(5);

	int p; // variable used to store the link parent number (used for concisness)
	// loop forward
	for (int i = 2; i <= Link::N_B; i++) //sort the keys in increased order
	{
		p = link[i].parent;

		// Calculate the v_i in local frame
		link[i].vJ = link[i].Phi * link[i].joint.qd; // Compute Velocity accross Joint i: vJ = Phi * qd
		link[i].v = link[i].X * link[p].v + link[i].vJ; // v_i = X_i * v_p(i) + Phi_i * qd_i 

		// Calculate the ai in local frame
		//link[i].v_phi = SS(link[i].v) * link[i].Phi;
		link[i].vi_x_vJ = SS(link[i].v) * link[i].vJ; // v_i x v_J
		link[i].a = link[i].X * link[p].a + link[i].vi_x_vJ; // a_i = i_X_p(i) * a_p(i) + v_i x v_J + c_J (but c_J = 0 for revolute Joints) 

		// Calculate the fi in local frame
		link[i].v_Iv = SSF(link[i].v) * link[i].I_spatial * link[i].v; // Compute v x Iv, SSF is used because Iv is analoguous to a force vector
		link[i].f = link[i].I_spatial * link[i].a + link[i].v_Iv; // f_i = I_i * a_i + v x Iv
	}
	// Calculate the f1 in local frame
	link[1].v_Iv = SSF(link[1].v) * link[1].I_spatial * link[1].v; // v1 x I1 v1 (v_Iv for link 1) 
	link[1].f = link[1].I_spatial * link[1].a + link[1].v_Iv; // f_1 = I1 * a1 + v1 x I1 v1

	MatrixXd X_F; // Spatial transform for force vectors from local to parent: (p(i)_X_i)^F 
	// loop backward
	for (int i = Link::N_B; i > 1; i--)
	{
		p = link[i].parent;

		// C_G[i+4] = (Phi_i)^T * f_i = 3rd component of f_i because Phi_i=[0,0,1,0,0,0] (revolute) 
		// C_G[i+4] = (link[i].Phi.transpose() * link[i].f).value();
		Link::C_G[i + 4] = link[i].f[2]; // Compute element of C_G corresponding to link i 

		X_F = link[i].X.transpose(); // X_F = (p(i)_X_i)^-T = (i_X_p(i))^T
		link[p].f = link[p].f + X_F * link[i].f; // f_p(i) = f_p(i) + X_F * f_i 
	}
	// C_G.head(6) = link[1].Phi.transpose() * link[1].f; // = link[1].f because link[1].Phi = Identity
	Link::C_G.head(6) = link[1].f; // Compute the element of C_G corresponding to link 1
}
