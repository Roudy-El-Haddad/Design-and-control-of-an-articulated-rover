#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "skew.h"
#include "FreeModeMatrix.h"
#include "X_transform.h"
#include "LINK.h"

using namespace Eigen; 

// Computes the jacobian of the link whose number is specified 
MatrixXd J(std::vector<Link>& link, int link_nb) {
	/* Alternative way of computing the offset 
	int Leg_nb = ceil((link_nb-1)/3); // =0 for base, =1 for right leg, =2 for front leg, =3 for left leg
	int offset = Leg_nb * 3; // Because each leg has 3 links 
	*/
	int offset = link_nb - link[link_nb].level; // better way of computing the offset
	// Note:  Leg_nb =  (link_nb - link[link_nb].level)/3.0; 

	MatrixXd J(6, Link::N_DOF);
	J.setZero();
	for (int i = 1; i <= link[link_nb].level; i++) {
		if (i == 1) {
			J.block<6, 6>(0, 0).array() = j_X_i(link, link_nb, i)*Phi("floating");
		}
		else {
			J.block<6, 1>(0, i+4).array() = j_X_i(link, link_nb, i + offset)*Phi("revolute");
		}
	}
}



/* Compact version of the function */
/*
MatrixXd J(std::vector<Link>& link, int link_nb) {
int offset = link_nb - link[link_nb].level;

MatrixXd J(6, Link::N_DOF);
J.setZero();
for (int i = 1; i <= link[link_nb].level; i++) {
if (i==1)
{J.block<6, link[i].joint.DOF>(0, i-1).array() = j_X_i(link, link_nb, i)*(link[i].joint.Phi);}
else 
{J.block<6, link[i].joint.DOF>(0, i+4).array() = j_X_i(link, link_nb, i + offset)*(link[i + offset].joint.Phi);} 
}
}
*/