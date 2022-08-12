#pragma once

#include <Eigen/Dense>
#include <vector>
#include "skew.h"
#include "LINK.h"
#include "X_transform.h"
#include "SpatialSkew.h"

using namespace Eigen;

/* In this header file, we present two functions to compute the Composite Rigid-Body Inertia:
   A Recursive method and an explicit method 
   Only the Recursive method function will be used in our project as it produces a much smaller error */


   // Computes I_C of all links using the recursive formula 
   // I_C[i]=I_spatial[i]+Sum_{j in mu(i)} (I_C[j]) where mu(i) is the set of children of link i 
void I_Composite_recursive(std::vector<Link>& link) {

	// Initialization Step: 
	// Initialize I_C of link[p] to I_spatial of the same link
	for (int i = 1; i <= Link::N_B; i++) {
		link[i].I_C = link[i].I_spatial; // Initialize I_C of link[i] to I_spatial of the same link
	}

	// Recursive Step: 
	// Add to this link's I_C the values of the I_C of its children 
	int p;
	for (int i = Link::N_B; i >= 2; i--) {
		p = link[i].parent; // Initialize j to parent of link[i]
		link[p].I_C += link[i].X.transpose() * link[i].I_C * link[i].X;
	}
}


// Computes I_C of all links using the explicit formula 
// I_C[i]=I_spatial[i]+Sum_{j in nu(i)} (I_spatial[j]) where nu(i) is the set of all descendants of link i 
void I_Composite(std::vector<Link>& link) {
	int j; 
	MatrixXd jXi; 
	//Go through all links 
	for (int i = 1; i <= Link::N_B; i++) {
		link[i].I_C = link[i].I_spatial; // Initialize I_C of link[i] to I_spatial of the same link
		j = link[i].parent; // Initialize j to parent of link[i]

		// Add I_spatial of link[i] to all links on its way to the fixed base, 
		// i.e., add I_spatial of link[i] to all the ancestor links of link[i]
		// i.e., to its parent, parent of parent, etc.
		while (j != 0) { // While the parent is not the fixed base (0)
			jXi = j_X_i(link, j, i);
			link[j].I_C += jXi.transpose() * link[i].I_spatial * jXi;
			j = link[j].parent; // update j to the index of the parent of the current parent
		}
	}
}

//WRONG 
/*
// Computes I_C of all links using the recursive formula 
// I_C[i]=I_spatial[i]+Sum_{j in mu(i)} (I_C[j]) where mu(i) is the set of children of link i 
void I_Composite_recursive2(std::vector<Link>& link) {
	int index;
	// Initialization Step: 
	// Initialize I_C of link[p] to I_spatial of the same link
	for (int p = 1; p <= Link::N_B; p++) { link[p].I_C = link[p].I_spatial; }

	for (int k = Link::max_level; k >= 2; k--) { //Go through all levels from top to bottom 
		for (int i = 1; i <= Link::N_wheels; i++) { //Go through all branches/legs 
													//Operate on the link of level k from each leg i 

			index = k + i * (Link::max_level - 1); // As i varies, it goes through all the indices of joints with level k

			// Recursive Step: 
			// Add to this link's I_C the values of the I_C of its children 
			for (int j = 1; j <= Link::N_B; j++) {
				if (link[j].parent == index) {
					link[index].I_C += link[j].I_C;
				}
			}

			// The links of level 2 are all the children of link[1]/Floating base 
			if (k == 2) { link[1].I_C += link[index].I_C; }
		}
	}
} */