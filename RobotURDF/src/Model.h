#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "LINK.h"
#include "JOINT.h"
#include "MEASURED.h"
#include "DESIRED.h"
#include "SpatialSkew.h"
#include "pseudoInverse.h"
#include "X_transform.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Vector3.h"

using namespace std; 
using namespace Eigen;

class model {
public:

	MatrixXd Jc;
	MatrixXd JG;

	MatrixXd Jc_bar;
	MatrixXd JG_bar;

	MatrixXd N_c;
	//MatrixXd N_G;

	MatrixXd J_G_null_c;
	MatrixXd J_G_null_c_bar;
	MatrixXd N_G_null_c;

	VectorXd tau_G; 

	MatrixXd Omega;
	
	Eigen::HouseholderQR<MatrixXd> QR;
	MatrixXd P_QR;
	MatrixXd R_full; 
	MatrixXd R; 
	MatrixXd Q; 
	MatrixXd S_u; 
	MatrixXd S_c; 
	MatrixXd S; 
	MatrixXd QR_Projector;

	VectorXd tau_feedback;

	// Feedforward 
	MatrixXd Lambda_c;
	MatrixXd mu_c;
	MatrixXd rho_c;

	MatrixXd Lambda_G;
	MatrixXd mu_G;
	MatrixXd rho_G;

	MatrixXd Lambda_G_c;
	MatrixXd mu_G_c;
	MatrixXd rho_G_c;

	MatrixXd F_c;
	MatrixXd F_G_c;

	VectorXd tau_feedforward;

	// Actuation Torque 
	VectorXd tau_act;

	// Bias Forces 
	MatrixXd Jc_dot_qd; // Bias contact Force 
	MatrixXd JG_dot_qd; // Bias COM Force 

	// Measured values object 
	Measured measured; 
	// Desired values object 
	Desired desired; 




	void get_Jc_and_BiasForce(std::vector<Link> & link) {
		VectorXd Phi_c; Phi_c.resize(6);
		Phi_c << 0, 1, 0, 0, 0, 0; // Contact Free Mode matrix 
		/* Compute the Jacobian and Bias force for leg 1 */
		// Compute the spatial transformations c_X_i for leg 1 
		link[4].contact.c_X_i = link[4].contact.c_X_w; // c1_X_w1
		link[3].contact.c_X_i = link[4].contact.c_X_i * link[4].X; // c1_X_k1 = c1_X_w1 * w1_X_k1
		link[2].contact.c_X_i = link[3].contact.c_X_i * link[3].X; // c1_X_h1 = c1_X_k1 * k1_X_h1 
		link[1].contact.c_X_i = link[2].contact.c_X_i * link[2].X; // c1_X_b = c1_X_h1 * h1_X_b 
		// Compute the Contact Bias Force for leg 1  
		link[4].contact.Jd_qd_spatial = link[1].contact.c_X_i * link[1].vi_x_vJ
									  + link[2].contact.c_X_i * link[2].vi_x_vJ
									  + link[3].contact.c_X_i * link[3].vi_x_vJ
									  + link[4].contact.c_X_i * link[4].vi_x_vJ
									  - SS(link[4].contact.c_X_i * link[4].v) * Phi_c *link[4].v[2];
		link[4].contact.Jd_qd = link[4].contact.Jd_qd_spatial.bottomRows(3); 
		// Compute the Contact Jacobian for leg 1 
		link[4].contact.Jc_spatial.resize(6,15);
		link[4].contact.Jc_spatial.setZero();
		link[4].contact.Jc_spatial.leftCols<6>() = link[1].contact.c_X_i * link[1].Phi;
		link[4].contact.Jc_spatial.col(2+4) = link[2].contact.c_X_i * link[2].Phi;
		link[4].contact.Jc_spatial.col(3+4) = link[3].contact.c_X_i * link[3].Phi;
		link[4].contact.Jc_spatial.col(4+4) = link[4].contact.c_X_i * link[4].Phi;
		link[4].contact.J_c = link[4].contact.Jc_spatial.bottomRows(3);

		/* Compute the Jacobian and Bias force for leg 2 */
		// Compute the spatial transformations c_X_i for leg 2 
		link[7].contact.c_X_i = link[7].contact.c_X_w; // c2_X_w2
		link[6].contact.c_X_i = link[7].contact.c_X_i * link[7].X; // c2_X_k2 = c2_X_w2 * w2_X_k2
		link[5].contact.c_X_i = link[6].contact.c_X_i * link[6].X; // c2_X_h2 = c2_X_k2 * k2_X_h2 
		link[1].contact.c_X_i = link[5].contact.c_X_i * link[5].X; // c2_X_b = c2_X_h2 * h2_X_b 
		// Compute the Contact Bias Force for leg 2  
		link[7].contact.Jd_qd_spatial = link[1].contact.c_X_i * link[1].vi_x_vJ
									  + link[5].contact.c_X_i * link[5].vi_x_vJ
									  + link[6].contact.c_X_i * link[6].vi_x_vJ
									  + link[7].contact.c_X_i * link[7].vi_x_vJ
									  - SS(link[7].contact.c_X_i * link[7].v) * Phi_c *link[7].v[2];
		link[7].contact.Jd_qd = link[7].contact.Jd_qd_spatial.bottomRows(3);
		// Compute the Contact Jacobian for leg 2 
		link[7].contact.Jc_spatial.resize(6,15);
		link[7].contact.Jc_spatial.setZero();
		link[7].contact.Jc_spatial.leftCols<6>() = link[1].contact.c_X_i * link[1].Phi;
		link[7].contact.Jc_spatial.col(5+4) = link[5].contact.c_X_i * link[5].Phi;
		link[7].contact.Jc_spatial.col(6+4) = link[6].contact.c_X_i * link[6].Phi;
		link[7].contact.Jc_spatial.col(7+4) = link[7].contact.c_X_i * link[7].Phi;
		link[7].contact.J_c = link[7].contact.Jc_spatial.bottomRows(3);

		/* Compute the Jacobian and Bias force for leg 3 */
		// Compute the spatial transformations c_X_i for leg 3 
		link[10].contact.c_X_i = link[10].contact.c_X_w; // c3_X_w3
		link[9].contact.c_X_i = link[10].contact.c_X_i * link[10].X; // c3_X_k3 = c3_X_w3 * w3_X_k3
		link[8].contact.c_X_i = link[9].contact.c_X_i * link[9].X; // c3_X_h3 = c3_X_k3 * k3_X_h3 
		link[1].contact.c_X_i = link[8].contact.c_X_i * link[8].X; // c3_X_b = c3_X_h3 * h3_X_b 
		// Compute the Contact Bias Force for leg 3  
		link[10].contact.Jd_qd_spatial = link[1].contact.c_X_i * link[1].vi_x_vJ
									   + link[8].contact.c_X_i * link[8].vi_x_vJ
									   + link[9].contact.c_X_i * link[9].vi_x_vJ
									   + link[10].contact.c_X_i * link[10].vi_x_vJ
									   - SS(link[10].contact.c_X_i * link[10].v) * Phi_c *link[10].v[2];
		link[10].contact.Jd_qd = link[10].contact.Jd_qd_spatial.bottomRows(3);
		// Compute the Contact Jacobian for leg 3 
		link[10].contact.Jc_spatial.resize(6,15);
		link[10].contact.Jc_spatial.setZero();
		link[10].contact.Jc_spatial.leftCols<6>() = link[1].contact.c_X_i * link[1].Phi;
		link[10].contact.Jc_spatial.col(8+4) = link[8].contact.c_X_i * link[8].Phi;
		link[10].contact.Jc_spatial.col(9+4) = link[9].contact.c_X_i * link[9].Phi;
		link[10].contact.Jc_spatial.col(10+4) = link[10].contact.c_X_i * link[10].Phi;
		link[10].contact.J_c = link[10].contact.Jc_spatial.bottomRows(3); 

		// Compute the system Contact Jacobian 
		Jc.resize(9, 15); 
		//Jc.resize(Link::Contact_DOF, Link::N_DOF); 
		Jc.block<3,15>(0,0) = link[4].contact.J_c; 
		Jc.block<3, 15>(3, 0) = link[7].contact.J_c;
		Jc.block<3, 15>(6, 0) = link[10].contact.J_c; 

		/*
		Jc = [Jc1
			  Jc2
			  Jc3] -> Jc is a (3*3) x N_DOF matrix, (3*3) = N_contact * DOF_of_each_contact = Contact_DOF  
		*/

		// Compute the system Bias Contact Force 
		Jc_dot_qd.resize(9,1); 
		//Jc_dot_qd.resize(Link::Contact_DOF,1); 
		Jc_dot_qd << link[4].contact.Jd_qd, link[7].contact.Jd_qd, link[10].contact.Jd_qd; // Bias force
	}
	void Compute_tau_G(std::vector<Link> & link, const double & t) {
		Matrix3d D_k_G, K_G, D_G; // Impedance controllers' gains : Stiffness K and Damping D 
		K_G.setZero();   K_G(0, 0) = 150; K_G(1, 1) = 100; K_G(2, 2) = 20; // Position 
		D_G.setZero();   D_G(0, 0) = 15; D_G(1, 1) = 10; D_G(2, 2) = 5; // Velocity 
		D_k_G.setZero(); D_k_G(0, 0) = 0.5; D_k_G(1, 1) = 0.3; D_k_G(2, 2) = 0; // Angular Momentum 

		MatrixXd c_I_G; 
		b_X_G(link); // Computes b_p_G, b_R_G, b_X_G
		c_I_G = (Link::b_X_G.transpose()*link[1].I_C*Link::b_X_G);
 
		Link::v_G = JG * Link::Qd; // Compute COM velocity  

		// Compute COM position in the inertial frame 
		// 0_p_G = 0_p_b + 0_R_b * b_p_G
		Link::p_G = link[1].joint.q.tail<3>() + link[1].T.topLeftCorner<3,3>()*Link::b_p_G; 

		// Compute the desired COM values 
		desired.CoM_x_ref_generator(1, 3, 22, 24, 0.8, t); //0.8m/s
		desired.p_G_des << desired.CoM_linear_ref[0], desired.CoM_linear_ref[1], Link::p_G[2];
		desired.v_G_linear_des << desired.CoM_linear_ref[2], desired.CoM_linear_ref[3], Link::v_G[2];
		desired.w_G_des << 0, 0, 0; 
		desired.v_G_des.resize(6); 
		desired.v_G_des << desired.w_G_des, desired.v_G_linear_des; 

		// Compute the angular momentum 
		MatrixXd k_G, k_G_des; 
		k_G = (c_I_G*Link::v_G).head<3>(); 
		k_G_des = (c_I_G*desired.v_G_des).head<3>(); 

		// Compute the torques 
		MatrixXd tau_G_w, tau_G_v; 
		//tau_G_w = (JG.topRows(3)).transpose()*D_k_G*(k_G_des - k_G); // check if J_G_null_c or J_G ? 
		//tau_G_v = (JG.bottomRows(3)).transpose()*(K_G*(desired.p_G_des - Link::p_G) + D_G*(desired.v_G_des.tail<3>() - Link::v_G.tail<3>()));
		tau_G_w = (J_G_null_c.topRows(3)).transpose()*D_k_G*(k_G_des - k_G); // check if J_G_null_c or J_G ? 
		tau_G_v = (J_G_null_c.bottomRows(3)).transpose()*(K_G*(desired.p_G_des - Link::p_G) + D_G*(desired.v_G_des.tail<3>() - Link::v_G.tail<3>()));
		tau_G = tau_G_w + tau_G_v; 
	}
	void get_QR_Projector(std::vector<Link> & link) {
		/* Calculate the QR Decomposition of J_c_T. */
		S.resize(Link::N_actuated, Link::N_DOF); 
		S << MatrixXd::Zero(Link::N_actuated, 6), MatrixXd::Identity(Link::N_actuated, Link::N_actuated);

		QR.compute(Jc.transpose());
		R_full = QR.matrixQR().triangularView<Upper>();
		R = R_full.block(0, 0, Jc.rows(), Jc.rows());
		Q = QR.householderQ();
		S_u.resize(Jc.cols() - Jc.rows(), Jc.cols());
		S_u << Eigen::MatrixXd::Zero(Jc.cols() - Jc.rows(), Jc.rows()),
			   Eigen::MatrixXd::Identity(Jc.cols() - Jc.rows(), Jc.cols() - Jc.rows());
		S_c.resize(Jc.rows(), Jc.cols());
		S_c << Eigen::MatrixXd::Identity(Jc.rows(), Jc.rows()),
			   Eigen::MatrixXd::Zero(Jc.rows(), Jc.cols() - Jc.rows());
		P_QR = S_u * Q.transpose();
		
		QR_Projector = pseudoInverse(P_QR * S.transpose()) * P_QR; 
	}
	void Compute_feedback_torque(std::vector<Link> & link, const double & t) {
		get_Jc_and_BiasForce(link); // Computes Jc and Jc_dot_qd

		b_X_G(link); // Computes b_p_G, b_R_G, b_X_G
		MatrixXd c_I_G; 
		c_I_G = (Link::b_X_G.transpose()*link[1].I_C*Link::b_X_G); // Lambda_G
		JG = c_I_G.inverse()*Link::b_X_G.transpose()*Link::M.topRows<6>(); 

		MatrixXd Minv; Minv = Link::M.inverse(); 
		Jc_bar = Minv * Jc.transpose()*pseudoInverse(Jc*Minv*Jc.transpose());
		JG_bar = Minv * JG.transpose()*pseudoInverse(JG*Minv*JG.transpose());

		MatrixXd I_NN; I_NN = MatrixXd::Identity(Link::N_DOF, Link::N_DOF); 
		N_c = I_NN - Jc_bar * Jc;

		J_G_null_c = JG * N_c; 
		J_G_null_c_bar = Minv * J_G_null_c.transpose() * pseudoInverse(J_G_null_c * Minv * J_G_null_c.transpose()); 

		N_G_null_c = I_NN - Jc_bar * Jc - J_G_null_c_bar * J_G_null_c; 

		Compute_tau_G(link, t); 
	//cout<<tau_G<<endl<<endl;

		Omega = (Jc.transpose() * Jc_bar.transpose() + J_G_null_c.transpose() * J_G_null_c_bar.transpose() + N_G_null_c) * (tau_G); 

		get_QR_Projector(link); 

		tau_feedback = QR_Projector * Omega; 	
	}
	void Compute_feedforward_torque(std::vector<Link> & link) {
		MatrixXd Minv; Minv = Link::M.inverse();
		// Compute Operational space model at contact 
		Lambda_c = pseudoInverse(Jc * Minv * Jc.transpose());  
		mu_c = Jc_bar.transpose() * Link::C - Lambda_c * Jc_dot_qd; // J_c_dot * qd is the contact bias force  
		rho_c = Jc_bar.transpose() * Link::G; 

		// Compute Operational space model at COM
		Lambda_G = Link::b_X_G.transpose()*link[1].I_C*Link::b_X_G; 
		mu_G = SSF(Link::v_G)*Lambda_G*Link::v_G; 
		rho_G = Link::b_X_G.transpose()*Link::G.head<6>(); 
		JG_dot_qd = -Lambda_G.inverse()*(mu_G - Link::b_X_G.transpose()*Link::C.head<6>()); 

		//
		Lambda_G_c = pseudoInverse(J_G_null_c * Minv * J_G_null_c.transpose()); 
		mu_G_c = Lambda_G_c * JG * Minv * N_c.transpose() * Link::C + Lambda_G_c * JG * Jc_bar * Jc_dot_qd - Lambda_G_c * JG_dot_qd; 
		rho_G_c = Lambda_G_c * JG * Minv * N_c.transpose() * Link::G; 

		// v_G_dot_des 
		desired.v_G_dot_des.resize(6);
		desired.v_G_dot_des << 0, 0, 0, desired.CoM_linear_ref[4], desired.CoM_linear_ref[5], 0; 

		// 
		F_G_c = Lambda_G_c * desired.v_G_dot_des + mu_G_c + rho_G_c; 

		tau_feedforward = QR_Projector * (J_G_null_c.transpose() * F_G_c); 
	}
	void Compute_actuation_torque(std::vector<Link> & link, const double & t) {
		Compute_feedback_torque(link, t); // Compute tau_feedback
		Compute_feedforward_torque(link); // Compute tau_feedforward
		tau_act = tau_feedback + tau_feedforward; 
	}

	//////////////////
	void jointState_Callback(const sensor_msgs::JointState::ConstPtr& msg); // Measured q_actuated, qd_actuated, tau_mes 
	void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg); // Measured q_b_angular, w_b, vd_b_linear 
	void GPS_position_Callback(const nav_msgs::Odometry::ConstPtr& msg); // Measured q_b_linear 
	void GPS_velocity_Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg); // Measured v_b_linear 

	//void CoM_linear_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	//void Base_anguler_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg); 

};
