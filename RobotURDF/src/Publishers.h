#pragma once

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>
#include "LINK.h"
#include "JointCallback.h"
#include "Model.h"
#include "MEASURED.h"
#include "DESIRED.h"
//#include "hierarchy_weighted_hierarchical.hpp"
//#include "stiffness_damping.hpp"
//#include "centroidal_orientation.hpp"

using namespace Eigen;

class PUBLISHER
{
public:
	int CoM_initial_publish_number;
	/* ROS Message variables */ 
	// Actuated joint torques
	std_msgs::Float64 tau_RH;
	std_msgs::Float64 tau_RK;
	std_msgs::Float64 tau_RW;
	std_msgs::Float64 tau_FH;
	std_msgs::Float64 tau_FK;
	std_msgs::Float64 tau_FW;
	std_msgs::Float64 tau_LH;
	std_msgs::Float64 tau_LK;
	std_msgs::Float64 tau_LW;

	// COM_x reference 
	//std_msgs::Float64 CoM_x_publish;

	/* Publisher objects */  
	// Actuated joint torques
	ros::Publisher RH;
	ros::Publisher RK;
	ros::Publisher RW;
	ros::Publisher FH;
	ros::Publisher FK;
	ros::Publisher FW;
	ros::Publisher LH;
	ros::Publisher LK;
	ros::Publisher LW;

	// Define publisher for publish an array of data 
	/*ros::Publisher iros_data;
	std_msgs::Float64MultiArray iros_array;
	std::vector<double> iros_vector; */

	// Define the publisher for initial CoM linear position in x direction, but only publish two times.
	//ros::Publisher CoM_x_pub;

	// Define the publisher for Base y direction for ploting.
	//ros::Publisher Base_y_pub;
	//ros::Publisher Base_roll_pub; 



	// Constructor 
	PUBLISHER(ros::NodeHandle & nh); 
	// Destructor 
	~PUBLISHER() {}

	// Function that publishers the torque commands to the joint actuators 
	void torque_publish(VectorXd tau);
}; 

// Constructor Definition 
PUBLISHER::PUBLISHER(ros::NodeHandle & nh)
{
	//CoM_initial_publish_number = 0;

	//iros_data = nh.advertise<std_msgs::Float64MultiArray>("/towr/iros_data", 100);

	// Define the publishers for actuated joints.
	RH = nh.advertise<std_msgs::Float64>("/RobotURDF/RightHip_controller/command", 100);
	RK = nh.advertise<std_msgs::Float64>("/RobotURDF/RightKnee_controller/command", 100);
	RW = nh.advertise<std_msgs::Float64>("/RobotURDF/RightWheel_controller/command", 100);
	FH = nh.advertise<std_msgs::Float64>("/RobotURDF/FrontHip_controller/command", 100);
	FK = nh.advertise<std_msgs::Float64>("/RobotURDF/FrontKnee_controller/command", 100);
	FW = nh.advertise<std_msgs::Float64>("/RobotURDF/FrontWheel_controller/command", 100);
	LH = nh.advertise<std_msgs::Float64>("/RobotURDF/LeftHip_controller/command", 100);
	LK = nh.advertise<std_msgs::Float64>("/RobotURDF/LeftKnee_controller/command", 100);
	LW = nh.advertise<std_msgs::Float64>("/RobotURDF/LeftWheel_controller/command", 100);

	// Define the publisher for initial CoM linear position in x direction, but only publish two times.
	//CoM_x_pub = nh.advertise<std_msgs::Float64>("/towr/CoM_initial_x", 100); 

	// Define the publisher for Base y direction for ploting.
	//Base_y_pub = nh.advertise<std_msgs::Float64>("/towr/Base_y_position", 100);
	//Base_roll_pub = nh.advertise<std_msgs::Float64>("/towr/Base_roll_angle", 100);
}

// Torque publisher function definition 
void PUBLISHER::torque_publish(VectorXd tau)
{
//cout<<tau<<endl<<endl;
	// Store the torques values inside ROS message variables 
	tau_RH.data = tau[0];
	tau_RK.data = tau[1];
	tau_RW.data = tau[2];
	tau_FH.data = tau[3];
	tau_FK.data = tau[4];
	tau_FW.data = tau[5];
	tau_LH.data = tau[6];
	tau_LK.data = tau[7];
	tau_LW.data = tau[8];

	// Publish these torque values to the corresponding ROS topics 
	RH.publish(tau_RH);
	RK.publish(tau_RK);
	RW.publish(tau_RW);
	FH.publish(tau_FH);
	FK.publish(tau_FK);
	FW.publish(tau_FW);
	LH.publish(tau_LH);
	LK.publish(tau_LK);
	LW.publish(tau_LW);



	/*
	std_msgs::Float64 Base_y_publish, Base_roll_publish;
	Base_y_publish.data = Model.measured.q_b_linear[1];
	Base_roll_publish.data = Model.measured.q_b_angular[0];
	Base_y_pub.publish(Base_y_publish);
	Base_roll_pub.publish(Base_roll_publish);*/

	//    int CoM_initial_publish_number = 0; 
	/*
	if (CoM_initial_publish_number < 50)
	{
		CoM_x_publish.data = joint.quadruped.p_IN0[0];
		CoM_x_pub.publish(CoM_x_publish);
		CoM_initial_publish_number++;
	} */ 
}


/*
void PUBLISHER::get_impedance_tau_com_linear_com(std::vector<Link> & link, Gain gain) {
	impedance.tau_com_linear = model.com.J_INc.bottomRows(3).transpose()*
							  (gain.com.K_linear_position.cwiseProduct(joint.com.p_desired - joint.com.p_IN0) +
							   gain.com.D_linear_velocity.cwiseProduct(joint.com.v_desired.tail<3>() - joint.com.v.tail<3>()));
}
*/
/*
void PUBLISHER::get_impedance_tau_com_angular(MODEL & model, Joint & joint, GAIN & gain)
{
	impedance.tau_com_angular = model.com.J_INc.topRows(3).transpose() * gain.com.D_angular_momentum.cwiseProduct(
		(model.com.arc * joint.com.v_desired).head<3>() - (model.com.arc * joint.com.v).head<3>());
}*/
