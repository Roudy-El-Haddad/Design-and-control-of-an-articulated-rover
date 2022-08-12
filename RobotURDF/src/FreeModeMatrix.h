#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <string>
#include "JOINT.h"

using namespace std;
using namespace Eigen; 

//Returns the Free Mode Matrix for a Joint depending on its type 
MatrixXd Phi(Joint joint) {
	MatrixXd phi;
	phi.resize(6, joint.DOF);
	if (joint.type == revolute) phi << 0, 0, 1, 0, 0, 0;
	if (joint.type == prismatic) phi << 0, 0, 0, 0, 0, 1;
	if (joint.type == prismatic) phi << 0, 1, 0, 0, 0, 0;
	if (joint.type == floating) phi.setIdentity(6, 6);
	return phi;
}


MatrixXd Phi(string JointType) {
	MatrixXd phi; 

	if (JointType == "floating") phi.resize(6, 6);
	else phi.resize(6, 1); 

	if (JointType == "revolute") phi << 0, 0, 1, 0, 0, 0;
	if (JointType == "prismatic") phi << 0, 0, 0, 0, 0, 1;
	if (JointType == "contact") phi << 0, 1, 0, 0, 0, 0;
	if (JointType == "floating") phi.setIdentity(6,6);
	return phi; 
}
