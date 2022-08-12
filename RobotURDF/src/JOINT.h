#pragma once

#include <Eigen/Dense>
#include"LINK.h"

using namespace Eigen;

enum JointType{revolute, floating, prismatic, contact}; 

class Joint {
public: 
	int parent; // predecessor of joint (link preceeding the joint) 
	int child; // successor of the joint (link following the joint) 

	JointType type; // joint type 
	int DOF; // number of DOF of the joint 
	MatrixXd Phi; //Free mode matrix of the joint preceding this link 

	VectorXd q; // joint position 
	VectorXd qd; // joint velocity 
	VectorXd qdd; // joint acceleration 

}; 
