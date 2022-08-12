#pragma once

#include<iostream>
#include <Eigen/Dense>
#include <string.h>
#include "JOINT.h"
#include "CONTACT.h"

using namespace std;
using namespace Eigen; 

class Link {
public: 
	int link_nb; // Link number
	string name; // Link name 
	int parent; // Number of link parent
	int level; // How nested is the link in the Kinematic Tree

	// Rotation matrix relative to the parent link 
	// Rotation matrix Transfers a vector from parent frame to local frame: i_R_p(i), where p(i) is the parent
	Matrix3d R; 
	Matrix3d R_start; // Value of R at the start when q=0 
	// Position vector relative to the parent link 
	// Position of local frame origin in the parent frame: i_p_p(i)
	Vector3d p; 
	// Spatial transform from parent frame to local frame: i_X_p(i)
	MatrixXd X; 
	MatrixXd X_inv; // X^-1: Spatial transform from local to parent: p(i)_X_i
	// Spatial transform from base frame to local frame 
	MatrixXd n_X_1;

	// Homogeneous Tranformation Matrix (HTM) from local frame to parent frame: p(i)_T_i
	MatrixXd T; 

	//Link mass
	double m;
	//Inertia at the original of the local frame 
	Matrix3d I;
	//position of link centroid in local frame 
	Vector3d c; 
	//spatial inertia
	MatrixXd I_spatial; 
	//Composite Rigid-Body Inertia of the link 
	MatrixXd I_C; 

	//Free mode matrix of the joint preceding this link 
	MatrixXd Phi; 

	// Velocity, acceleration, and force 
	VectorXd v;
	VectorXd a;
	VectorXd f;

	// Link Jacobian (Spatial Jacobian) 
	MatrixXd J; 
	MatrixXd Jdot; // Time derivative of J 

	// Variables used in the RNEA Algorithm calculations 
	VectorXd vJ; // Velocity accross Joint i: vJ = Phi_i * qd_i
	VectorXd vi_x_vJ; // v_i x v_J where x: spatial cross product for motion 
	VectorXd v_Iv; // v_i x I_i v_i where I_i = link[i].I_spatial and x: spatial cross product for force

	VectorXd v_wrt_b; // Velocity of link i relative to the base frame 
	VectorXd v_wrt_b_x_vJ; // v_wrt_b x vJ

	// Joint preceding the Link 
	Joint joint; 
	// Contact object to access contact parameters for contact links
	Contact contact; 
	// Wheel object to access wheel parameters for wheel links
	Wheel wheel;
	
	// Parameters common to all links 
	static const int N_B; // Number of links/bodies 
	static const int N_DOF; // Number of DOFs 
	static const int N_actuated; // Number of actuated joints = N_DOF - 6
	static const int N_wheels; // N_wheels = N_legs 
	static const int Contact_DOF; // N_contacts * DOF_of_each_contact 
	static const int max_level; // Maximum depends of the nesting in the Kinematic Tree 
								// max_level = nb of Links per Legs + 1 
	static double mass; // mass of the system 
	static double L1; // Length of upper leg 
	static double L2; // Length of lower leg 
	static double r; // wheel radius 

	//static MatrixXd b_R_0; // Rotation matrix from the fixed frame to the floating base frame 
	static MatrixXd b_X_G; // spatial transform from centroidal frame to base frame 
	static Matrix3d b_R_G; 
	static Vector3d b_p_G; 

	static Vector3d p_G;
	static VectorXd v_G; 

	// Model Parameters 
	static MatrixXd M; 
	static VectorXd C; 
	static VectorXd C_G;
	static VectorXd G;

	// Joint Configuration for all joints 
	static VectorXd Q;
	static VectorXd Qd;
	static VectorXd Qdd;

	// Constructor
	Link() {
		X.resize(6, 6);
		X_inv.resize(6, 6);
		n_X_1.resize(6, 6);
		T.resize(4, 4);

		I_spatial.resize(6, 6);
		I_C.resize(6, 6);

		v.resize(6);
		a.resize(6);
		f.resize(6);

		vJ.resize(6);
		vi_x_vJ.resize(6);
		v_Iv.resize(6);

		v_wrt_b.resize(6);
		v_wrt_b_x_vJ.resize(6);
		
		Link::Q.resize(15);
		Link::Q<<0,0,0, 0,0,0.4425, 0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0; 

	}

};

// Parameters common to all links 
const int Link::N_B = 10;
const int Link::N_DOF = 15; // N_DOF = 15 = 6 + 9*1
const int Link::N_actuated = 9;
const int Link::N_wheels = 3; 
const int Link::Contact_DOF = 9; // = 3 DOF per contact * 3 contacts (wheels) 
const int Link::max_level = 4; // level 1: base, level 4: wheel 
double Link::mass = 0.0;
/* The link length values are measured from the SolidWorks file */
double Link::L1 = 0.210; // L1 = 210 mm 
double Link::L2 = 0.227; // L2 = 227 mm 
double Link::r = 0.0925; // r = 92.5 mm 

//MatrixXd Link::b_R_0 = Eigen::Matrix<double, 3, 3>::Identity(); 
MatrixXd Link::b_X_G = Eigen::Matrix<double, 6, 6>::Identity();
Matrix3d Link::b_R_G = Eigen::Matrix<double, 3, 3>::Identity();
Vector3d Link::b_p_G = Eigen::Vector3d::Zero();

Vector3d Link::p_G = Eigen::Vector3d::Zero();
VectorXd Link::v_G = Eigen::Matrix<double, 6, 1>::Zero(); 

MatrixXd Link::M = Eigen::Matrix<double, 15, 15>::Identity();
VectorXd Link::C = Eigen::Matrix<double, 15, 1>::Zero(); 
VectorXd Link::C_G = Eigen::Matrix<double, 15, 1>::Zero(); 
VectorXd Link::G = Eigen::Matrix<double, 15, 1>::Zero(); 

VectorXd Link::Q;
VectorXd Link::Qd = Eigen::Matrix<double, 15, 1>::Zero();
VectorXd Link::Qdd = Eigen::Matrix<double, 15, 1>::Zero(); 
