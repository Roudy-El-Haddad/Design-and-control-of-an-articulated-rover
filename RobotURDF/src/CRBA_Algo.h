#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/LU>

#include "LINK.h"
#include "JOINT.h"
//#include "Model.h"

using namespace Eigen;

/*Update the Mass Matrix for Joint Space Model*/
// Let us consider the dynamic model: M*qdd + C_G = tau where C_G = C + G
// Computes M 
// This is the CRBA algorithm 
inline void floating_base_CRBA(std::vector<Link> & link)
{
	MatrixXd H_actuated(Link::N_actuated, Link::N_actuated);
	MatrixXd Flocal(6, Link::N_actuated);
	H_actuated.setZero();
	Flocal.setZero();

	// Initialization of the Composite rigid-body Inertia.
	for (int i = 1; i <= Link::N_B; i++)
	{
		link[i].I_C = link[i].I_spatial;
	}

	int p;
	for (int i = Link::N_B; i > 1; i--)
	{
		p = link[i].parent;
		// update the Composite rigid-body Inertia for each link.
		link[p].I_C = link[p].I_C + link[i].X.transpose() * link[i].I_C * link[i].X;
		// define a local viarable, Flocal
		// Flocal.col(i - 2) = link[i].I_C * link[i].Phi; 
		// But Phi = [0 0 1 0 0 0]^T (revolute) -> all columns of Flocal are zeros except for one
		Flocal.col(i - 2) = link[i].I_C.col(2);
		// generate the diagonal items, Mii
		// H(i-2,i-2) = (link[i].Phi.transpose() * Flocal.col(i - 2)).value();
		H_actuated(i - 2, i - 2) = Flocal(2, i - 2);

		int j = i;
		while (link[j].parent != 1)
		{
			Flocal.col(i - 2) = link[j].X.transpose() * Flocal.col(i - 2);
			j = link[j].parent;
			// H(i-2,j-2) = (Flocal.col(i-2).transpose() * link[j].phi).value();
			// H(i-2,j-2) = Flocal.col(i-2)[2];
			H_actuated(i - 2, j - 2) = Flocal(2, i - 2);
			H_actuated(j - 2, i - 2) = H_actuated(i - 2, j - 2);
		}
		Flocal.col(i - 2) = link[j].X.transpose() * Flocal.col(i - 2);
	}
	Link::M << link[1].I_C, Flocal, Flocal.transpose(), H_actuated;

}
