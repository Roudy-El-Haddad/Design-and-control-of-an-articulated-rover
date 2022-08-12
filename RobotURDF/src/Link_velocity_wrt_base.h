#pragma once

#include <Eigen/Dense>

#include "LINK.h"
#include "JOINT.h"
//#include "Model.h"

//#include "JointCallback.h"
#include "SpatialSkew.h"

using namespace Eigen;

inline void v_wrt_base(std::vector<Link>& link)
{
	for (int i = 2; i <= Link::N_B; i++)
	{
		// v_wrt_b = v_i - i_X_1 * v_1 Where i_X_1 * v_1 is the velocity of the base expressed in frame i 
		link[i].v_wrt_b = link[i].v - link[i].n_X_1 * link[1].v; 
		// v_wrt_b_x_vJ = v_wrt_b x vJ Where vJ = Phi * qd 
		link[i].v_wrt_b_x_vJ = SS(link[i].v_wrt_b) * link[i].Phi * link[i].joint.qd;
	}
}