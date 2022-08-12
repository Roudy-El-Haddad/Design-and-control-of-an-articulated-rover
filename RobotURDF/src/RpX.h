#pragma once

#include <Eigen/Dense>
#include <vector>
#include "skew.h"
#include "LINK.h"
#include "BasicHTM.h"
#include "EulerToRot.h"

using namespace Eigen; 

// Initializes the rotation R, position p, and Spatial transformation X for all links
// 
void RpX_initialize(std::vector<Link>& link) {

}

// inline void R_p_update(std::vector<Link> & link, const Joint & joint) { 

// Updates the values of R, p for each link according to the current joint configuration 
inline void R_p_update(std::vector<Link> & link) {
	for (int i = 1; i <= Link::N_B; ++i)
	{
		if (i == 1) 
		{
			// For a floating joint, p and R are not constant, they are defined by the joint configuration 
			link[1].p << link[1].joint.q[3], link[1].joint.q[4], link[1].joint.q[5];
			link[1].R = EulerAngle_To_RotMatrix(link[1].joint.q[0], link[1].joint.q[1], link[1].joint.q[2]).transpose();
		}
		else
		{
			// For a revolute joint 
			// link[i].p is constant as it is independent of the joint variable
			// link[i].R is not constant, it varies with the joint variable 
			link[i].R = rotz(-link[i].joint.q[0]) * link[i].R_start;
		}
	}
}