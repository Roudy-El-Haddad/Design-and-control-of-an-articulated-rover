#pragma once

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include "LINK.h"
#include "Model.h"
#include "X_transform.h"
#include "Local_Inertia.h"
#include "Spatial_Inertia.h"
#include "Composite_Inertia.h"
#include "FreeModeMatrix.h" 
#include "BasicHTM.h"
#include "EulerToRot.h"
#include "std_msgs/Float64MultiArray.h" // Uncomment when using ROS

using namespace std; 
using namespace Eigen;

inline void dynamics_model_initialization(std::vector<Link> & link) {
	// Pi value 
	double PI = 3.14159265358979;

	//////////// Link Data extracted from the URDF file ////////////////

	// Since all the legs are identical, they have the same mass and inertia 
	// Mass values 
	/* Mass values are read from the URDF file. Inside the <link> tag, there is an <inertial> tag, 
	   inside which there is a <mass> tag that has an attribute called value. "value" corresponds to the link's mass. */
	double m_FB = 13.248; // Floating base mass 

	double m_u = 2.2246; // upper leg mass 
	double m_l = 0.26283; // lower leg mass
	double m_w = 1.5614; // wheel mass

	/*
	double m_Lu = 2.2246; // Left upper leg mass 
	double m_Ll = 0.26283; // Left lower leg mass
	double m_Lw = 1.5614; // Left wheel mass

	double m_Fu = 2.2246; // Front upper leg mass 
	double m_Fl = 0.26283; // Front lower leg mass
	double m_Fw = 1.5614; // Front wheel mass

	double m_Ru = 2.2246; // Right upper leg mass 
	double m_Rl = 0.26283; // Right lower leg mass
	double m_Rw = 1.5614; // Right wheel mass
*/

	// Inertia values 
	/* Inertia values are read from the URDF file. Inside the <link> tag, there is an <inertial> tag,
	inside which there is a <inertia> tag that has the following attributes: ixx, ixy, ixz, iyy, iyz, izz.
	These correspond to the values needed to construct the link's inertia matrix. */
	inertia I_FB = { 0.11898,3.3312E-06,2.2815E-05,0.08179,7.645E-07,0.1793 }; // Floating base inertia 

	inertia I_u = { 0.0016906,-0.00028648,2.4229E-06,0.0050451,2.0718E-06,0.0061125 }; // upper leg inertia 
	inertia I_l = { 0.00019876,1.5906E-07,-3.583E-11,0.00060211,-2.1512E-11,0.0004548 }; // lower leg inertia
	inertia I_w = { 0.0018287,-4.8963E-06,-5.5018E-07,0.0018475,2.2527E-06,0.0033571 }; // wheel inertia

	/*
	inertia I_Lu = { 0.0016906,-0.00028648,0,0,0,0 }; // Left upper leg mass 
	inertia I_Ll = { 0,0,0,0,0,0 }; // Left lower leg mass
	inertia I_Lw = { 0,0,0,0,0,0 }; // Left wheel mass

	inertia I_Fu = { 0.0016906,-0.00028648,2.4229E-06,0.0050451,2.0718E-06,0.0061125 }; // Front upper leg mass 
	inertia I_Fl = { 0.00019876,1.5906E-07,-3.583E-11,0.00060211,-2.1512E-11,0.0004548 }; // Front lower leg mass
	inertia I_Fw = { 0.0018287,-4.8963E-06,-5.5018E-07,0.0018475,2.2527E-06,0.0033571 }; // Front wheel mass

	inertia I_Ru = { 0,0,0,0,0,0 }; // Right upper leg mass 
	inertia I_Rl = { 0,0,0,0,0,0 }; // Right lower leg mass
	inertia I_Rw = { 0,0,0,0,0,0 }; // Right wheel mass
*/

	// Centroidal position values 
	/* The cendroidal position values are read from the URDF file. Inside the <link> tag, there is an <inertial> tag,
	inside which there is an <origin> tag that has an attribute called xyz. "xyz" corresponds to the link's centroidal position. */
	Vector3d c_FB; c_FB << 0, 0, 0; // Floating base Centroidal position  

	Vector3d c_Lu; c_Lu << 0.02006, 0.0018989, -0.0014179; // Left upper leg Centroidal position 
	Vector3d c_Ll; c_Ll << 0.11637, -0.00010065, 2.6945E-09; // Left lower leg Centroidal position
	Vector3d c_Lw; c_Lw << 9.2704E-05, -0.00037961, 0.00038182; // Left wheel Centroidal position

	Vector3d c_Fu; c_Fu << 0.02006, 0.0018989, -0.0014179; // Front upper leg Centroidal position 
	Vector3d c_Fl; c_Fl << 0.11637, -0.00010065, 2.6945E-09; // Front lower leg Centroidal position
	Vector3d c_Fw; c_Fw << 9.2705E-05, -0.00037961, 0.00038182; // Front wheel Centroidal position

	Vector3d c_Ru; c_Ru << 0.02006, 0.0018989, -0.0014179; // Right upper leg Centroidal position 
	Vector3d c_Rl; c_Rl << 0.11637, -0.00010065, 2.6945E-09; // Right lower leg Centroidal position
	Vector3d c_Rw; c_Rw << 9.2705E-05, -0.00037961, 0.00038182; // Right wheel Centroidal position
	
	//////////// Joint Data extracted from the URDF file ////////////////
	
	/* Define the initial joint configuration */
	// This is needed to compute the initial rotation matrix (at t=0)  
	/* The initial joint values are measured from the SolidWorks file */
	// distances are in m and angles are in rad 

	// Define the 6 coordinates of the floating base 
	double qAL = 0, qBE = 0, qGA = 0, qX = 0, qY = 0, qZ = 0.4425;
	//Define each of the coordinates of the joints of the left leg 
	double q_LH = 0.896564661, q_LK = 1.70424217, q_LW = 0;
	//Define each of the coordinates of the joints of the front leg 
	double q_FH = 0.896564661, q_FK = 1.70424217, q_FW = 0;
	//Define each of the coordinates of the joints of the right leg 
	double q_RH = 0.896564661, q_RK = 1.70424217, q_RW = 0;

	// Position of local frame origin in the parent frame: i_p_p(i) (at the zero configuration and at any configuration) 
	// Note that since the joints are revolute joints, this value remains constant even when the configuration changes
	/* Inside the <joint> tag, there is an <origin> tag that has an attribute called xyz. 
	   "xyz" corresponds to i_p_p(i).*/
	Vector3d p_FB; p_FB << qX, qY, qZ; // Floating base 

	Vector3d p_Lh; p_Lh << -0.17101, 0.151, -0.062; // Left hip frame 
	Vector3d p_Lk; p_Lk << 0.21, 0, 0; // Left knee frame
	Vector3d p_Lw; p_Lw << 0.227, 0, 8.0621E-05; // Left wheel frame

	Vector3d p_Fh; p_Fh << 0.16399, 0, - 0.062; // Front hip frame 
	Vector3d p_Fk; p_Fk << 0.21, 0, 0; // Front knee frame
	Vector3d p_Fw; p_Fw << 0.227, 0, 8.0621E-05; // Front wheel frame

	Vector3d p_Rh; p_Rh << -0.17101, -0.151, -0.062; // Right hip frame
	Vector3d p_Rk; p_Rk << 0.21, 0, 0; // Right knee frame 
	Vector3d p_Rw; p_Rw << 0.227, 0, 8.0621E-05; // Right wheel frame

	// Rotation from parent frame to local frame for the zero joint configuration (all q's are zero) : i_R_p(i) (q=0)
	// Note that since the joints are revolute joints, this value changes with the joint configuration 
	/* Inside the <joint> tag, there is an <origin> tag that has an attribute called rpy.
	"rpy" corresponds to roll, pitch, and yaw angle (Euler angles) representing the rotation. 
	Using the EulerAngle_To_RotMatrix function, we compute p(i)_R_i from the Euler angles.
	In fact, the Euler angles give us the rotation from local to parent frame p(i)_R_i. 
	However, we want i_R_p(i). Hence, we transpose the result: i_R_p(i) = p(i)_R_i.transpose()*/
	Matrix3d R0_FB; R0_FB = EulerAngle_To_RotMatrix(qAL, qBE, qGA).transpose(); // Floating base 

	Matrix3d R0_Lh; R0_Lh = EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416).transpose(); // Left hip frame 
	Matrix3d R0_Lk; R0_Lk = EulerAngle_To_RotMatrix(0, 0, -1.7042).transpose(); // Left knee frame
	Matrix3d R0_Lw; R0_Lw = EulerAngle_To_RotMatrix(0, 0, -2.3339).transpose(); // Left wheel frame

	Matrix3d R0_Fh; R0_Fh = EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416).transpose(); // Front hip frame 
	Matrix3d R0_Fk; R0_Fk = EulerAngle_To_RotMatrix(0, 0, -1.7042).transpose(); // Front knee frame
	Matrix3d R0_Fw; R0_Fw = EulerAngle_To_RotMatrix(0, 0, - 2.3339).transpose(); // Front wheel frame

	Matrix3d R0_Rh; R0_Rh = EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416).transpose(); // Right hip frame
	Matrix3d R0_Rk; R0_Rk = EulerAngle_To_RotMatrix(0, 0, -1.7042).transpose(); // Right knee frame 
	Matrix3d R0_Rw; R0_Rw = EulerAngle_To_RotMatrix(0, 0, -2.3339).transpose(); // Right wheel frame

	// Rotation from parent frame to local frame for a configuration q_i : i_R_p(i)
	// Note that since the joints are revolute joints, this value changes with the joint configuration 
	/* (i_R_p(i) for q_i) = {rotz(q_i) * (p(i)_R_i for q_i=0)}.transpose() 
	   where (p(i)_R_i for q_i=0) is obtained from the Euler angles extracted from the URDF file and 
	   rotz(q_i) gives the rotation from the local frame to the parent frame caused by a rotation of q_i of the joint. */
	Matrix3d R_FB; R_FB = EulerAngle_To_RotMatrix(qAL, qBE, qGA).transpose(); // Floating base 

	Matrix3d R_Lh; R_Lh = (EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416)*rotz(q_LH)).transpose(); // Left hip frame 
	Matrix3d R_Lk; R_Lk = (EulerAngle_To_RotMatrix(0, 0, -1.7042)*rotz(q_LK)).transpose(); // Left knee frame
	Matrix3d R_Lw; R_Lw = (EulerAngle_To_RotMatrix(0, 0, -2.3339)*rotz(q_LW)).transpose(); // Left wheel frame

	Matrix3d R_Fh; R_Fh = (EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416)*rotz(q_FH)).transpose(); // Front hip frame 
	Matrix3d R_Fk; R_Fk = (EulerAngle_To_RotMatrix(0, 0, -1.7042)*rotz(q_FK)).transpose(); // Front knee frame
	Matrix3d R_Fw; R_Fw = (EulerAngle_To_RotMatrix(0, 0, -2.3339)*rotz(q_FW)).transpose(); // Front wheel frame

	Matrix3d R_Rh; R_Rh = (EulerAngle_To_RotMatrix(1.5708, 0.67423, 3.1416)*rotz(q_RH)).transpose(); // Right hip frame
	Matrix3d R_Rk; R_Rk = (EulerAngle_To_RotMatrix(0, 0, -1.7042)*rotz(q_RK)).transpose(); // Right knee frame 
	Matrix3d R_Rw; R_Rw = (EulerAngle_To_RotMatrix(0, 0, -2.3339)*rotz(q_RW)).transpose(); // Right wheel frame
	
	/*
	// Initialize ground contact coordinates 
	Vector3d p_Lc; p_Lc << 0, 0, 0; 
	Vector3d p_Fc; p_Fc << 0, 0, 0;
	Vector3d p_Rc; p_Rc << 0, 0, 0;
	*/


	/* Initialize fixed parameters of the links/bodies */
	// Initial Floating Base Parameters 
	int i = 1;
	link[i].link_nb = i;
	link[i].name = "FloatingBase";
	link[i].parent = 0;
	link[i].level = 1;

	link[i].m = m_FB;
	link[i].I = I_local(I_FB);
	link[i].c = c_FB;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_FB;
	link[i].R_start = R0_FB;
	link[i].R = R_FB; 
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv=Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("floating");

	link[i].joint.type = JointType::floating;
	link[i].joint.DOF = 6;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Right Upper Leg Parameters 
	i = 2;
	link[i].link_nb = i;
	link[i].name = "RightUpperLeg";
	link[i].parent = 1;
	link[i].level = 2;

	link[i].m = m_u;
	link[i].I = I_local(I_u);
	link[i].c = c_Ru;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Rh;
	link[i].R_start = R0_Rh;
	link[i].R = R_Rh;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = JointType::revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Right Lower Leg Parameters 
	i = 3;
	link[i].link_nb = i;
	link[i].name = "RightLowerLeg";
	link[i].parent = 2;
	link[i].level = 3;

	link[i].m = m_l;
	link[i].I = I_local(I_l);
	link[i].c = c_Rl;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Rk;
	link[i].R_start = R0_Rk;
	link[i].R = R_Rk;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Right Wheel Parameters 
	i = 4;
	link[i].link_nb = i;
	link[i].name = "RightWheel";
	link[i].parent = 3;
	link[i].level = 4;

	link[i].m = m_w;
	link[i].I = I_local(I_w);
	link[i].c = c_Rw;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Rw;
	link[i].R_start = R0_Rw;
	link[i].R = R_Rw;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	//link[i].contact.c_p_w = p_Rc;
	//link[i].contact.c_R_w = Matrix3d::Identity(3, 3);

	// Initial Front Upper Leg Parameters 
	i = 5;
	link[i].link_nb = i;
	link[i].name = "FrontUpperLeg";
	link[i].parent = 1;
	link[i].level = 2;

	link[i].m = m_u;
	link[i].I = I_local(I_u);
	link[i].c = c_Fu;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Fh;
	link[i].R_start = R0_Fh;
	link[i].R = R_Fh;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Front Lower Leg Parameters 
	i = 6;
	link[i].link_nb = i;
	link[i].name = "FrontLowerLeg";
	link[i].parent = 5;
	link[i].level = 3;

	link[i].m = m_l;
	link[i].I = I_local(I_l);
	link[i].c = c_Fl;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Fk;
	link[i].R_start = R0_Fk;
	link[i].R = R_Fk;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Front Wheel Parameters 
	i = 7;
	link[i].link_nb = i;
	link[i].name = "FrontWheel";
	link[i].parent = 6;
	link[i].level = 4;

	link[i].m = m_w;
	link[i].I = I_local(I_w);
	link[i].c = c_Fw;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Fw;
	link[i].R_start = R0_Fw;
	link[i].R = R_Fw;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// link[i].contact.c_p_w = p_Fc;
	// link[i].contact.c_R_w = Matrix3d::Identity(3, 3);

	// Initial Left Upper Leg Parameters 
	i = 8;
	link[i].link_nb = i;
	link[i].name = "LeftUpperLeg";
	link[i].parent = 1;
	link[i].level = 2;

	link[i].m = m_u;
	link[i].I = I_local(I_u);
	link[i].c = c_Lu;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Lh;
	link[i].R_start = R0_Lh;
	link[i].R = R_Lh;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Left Lower Leg Parameters 
	i = 9;
	link[i].link_nb = i;
	link[i].name = "LeftLowerLeg";
	link[i].parent = 8;
	link[i].level = 3;

	link[i].m = m_l;
	link[i].I = I_local(I_l);
	link[i].c = c_Ll;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Lk;
	link[i].R_start = R0_Lk;
	link[i].R = R_Lk;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint);

	// Initial Left Wheel Parameters 
	i = 10;
	link[i].link_nb = i;
	link[i].name = "LeftWheel";
	link[i].parent = 9;
	link[i].level = 4;

	link[i].m = m_w;
	link[i].I = I_local(I_w);
	link[i].c = c_Lw;
	link[i].I_spatial = I_spatial(link[i].m, link[i].I, link[i].c);

	link[i].p = p_Lw;
	link[i].R_start = R0_Lw;
	link[i].R = R_Lw;
	link[i].X = X(link[i].R, link[i].p);
	link[i].X_inv = Xinv(link[i].R, link[i].p);

	link[i].Phi = Phi("revolute");

	link[i].joint.type = revolute;
	link[i].joint.DOF = 1;
	link[i].joint.Phi = Phi(link[i].joint); 

	// link[i].contact.c_p_w = p_Lc;
	// link[i].contact.c_R_w = Matrix3d::Identity(3, 3);

	// Initial Joint positions, velocities, accelerations
	Link::Q.resize(15); Link::Q << qAL, qBE, qGA, qX, qY, qZ, q_RH, q_RK, q_RW, q_FH, q_FK, q_FW, q_LH, q_LK, q_LW; 
	Link::Qd.resize(15); Link::Qd.setZero(); 
	Link::Qdd.resize(15); Link::Qdd.setZero();
//cout<<Link::Q<<endl<<endl;
	for (i = 1; i <= Link::N_B; i++) {
		if (i == 1) {
			link[i].joint.q.resize(6); link[i].joint.qd.resize(6); link[i].joint.qdd.resize(6); 
			link[i].joint.q = Link::Q.head<6>(); link[i].joint.qd = Link::Qd.head<6>(); link[i].joint.qdd = Link::Qdd.head<6>();
		}
		else {
			link[i].joint.q.resize(1); link[i].joint.qd.resize(1); link[i].joint.qdd.resize(1);
			link[i].joint.q(0) = Link::Q(i+4); link[i].joint.qd(0) = Link::Qd(i+4); link[i].joint.qdd(0) = Link::Qdd(i+4);
		}
	}

	// System mass 
	Link::mass = 0.0; 
	for (i = 1; i <= Link::N_B; i++) Link::mass += link[i].m; 
//cout<<Link::mass<<endl<<endl;

	// Computes the Composite Rigid-Body inertia of all links 
	I_Composite_recursive(link); 

	// Compute the spatial transforms from the base from to local frame 
	n_X_1_Cal(link);
}
