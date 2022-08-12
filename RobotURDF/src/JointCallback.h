#pragma once

#include <Eigen/Dense>

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

#include <list>
#include <vector>

#include "Model.h"
#include "DESIRED.h"
#include "MEASURED.h"

using namespace Eigen; 

// This header file contains the code for the callback functions 
// Each one of the callback functions is associated with a given parameter 
// it will take a ROS message (msg) and store its content inside of this parameter  

/*### Callback functions used to get the measured values needed to update the joint configuration q and rate qd ###*/
/*####################################################################################################################*/
// Extracts the base orientation, the base angular acceleration, and the base linear acceleration from the IMU message
//
void model::imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{ROS_INFO("IMU Measure"); 
	tf::Quaternion quater; // Create a TF quaternion variable 

	tf::quaternionMsgToTF(msg->orientation, quater); // Store in it the msg->orientation 
	double roll, pitch, yaw; // define variables to store the Euler Angles 
	// Matrix3x3(quater) returns the 3x3 rotation matrix corresponding to the quaternion 
	// .getRPY(roll, pitch, yaw) extract the Euler angles from the rotation matrix and stores them in roll, pitch and yaw 
	tf::Matrix3x3(quater).getRPY(roll, pitch, yaw); 

	// base_q_euler, base_w_euler, base_dv_linear are data members of the Joint class 
	measured.q_b_angular << roll, pitch, yaw; // store the orientation as Euler angles 
	measured.w_b << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z; // store w_b
	measured.vd_b_linear << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; // Store v_b dot  
}
/*####################################################################################################################*/
// Get the joint positions, joint velocities, and joint efforts 
/* The order of the q's, qd's, and tau's inside the JointState message depends on the order in which the <joint> tags were defined 
   We define in the URDF, the joints in the correct order 
   If the order is not correct, then we have to swap the terms in such a way to make them in the correct order. */
void model::jointState_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	ROS_INFO("JointState Measure");
	int i_callback = 0; 
	// q_initial_actuated is read from the joint configuration in SolidWorks 
	VectorXd q_initial_actuated; q_initial_actuated.resize(9); 
	q_initial_actuated << 0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0, 0.896564661, 1.70424217, 0; 
	for (std::vector<double>::const_iterator it1 = msg->position.begin(); it1 != msg->position.end(); ++it1)
	{
		measured.q_actuated[i_callback] = *it1; i_callback++;
	}
	 
	/*
	// swap
	Vector4d temp_q;
	temp_q = q_actuated.segment<4>(8);
	q_actuated.segment<4>(8) = q_actuated.segment<4>(12);
	q_actuated.segment<4>(12) = temp_q; */

	measured.q_actuated = measured.q_actuated + q_initial_actuated;

	int j_callback = 0;
	for (std::vector<double>::const_iterator it2 = msg->velocity.begin(); it2 != msg->velocity.end(); ++it2)
	{
		measured.qd_actuated[j_callback] = *it2; j_callback++;
	} 
	
	/* 
	// swap 
	Vector4d temp_Dq;
	temp_Dq = Dq_actuated.segment<4>(8);
	Dq_actuated.segment<4>(8) = Dq_actuated.segment<4>(12);
	Dq_actuated.segment<4>(12) = temp_Dq;*/

	int k_callback = 0;
	for (std::vector<double>::const_iterator it3 = msg->effort.begin(); it3 != msg->effort.end(); ++it3)
	{
		measured.tau_mes[k_callback] = *it3; k_callback++;
	} 
	
	/*
	// swap 
	Vector4d temp_tau_mea;
	temp_tau_mea = tau_mea.segment<4>(8); // tau_mea[8:11]
	tau_mea.segment<4>(8) = tau_mea.segment<4>(12); // tau_mea[12:15] 
	tau_mea.segment<4>(12) = temp_tau_mea;*/
}
/*####################################################################################################################*/ 
// Get the base position 
void model::GPS_position_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("P3D Position Measure");
	// base_q_linear is a data member of the Joint class used to store base position 
	measured.q_b_linear << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
}
/*####################################################################################################################*/ 
// Get the base linear velocity 
void model::GPS_velocity_Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	ROS_INFO("GPS Velocity Measure");
	measured.v_b_linear << msg->vector.x, msg->vector.y, msg->vector.z;
}


/* ### Callback functions to get the generated reference values ### */
/*####################################################################################################################*/ 
// COM position (position of G) : 
/*
void model::CoM_linear_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i_callback = 0;
	for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		desired.CoM_linear_ref[i_callback] = *it; i_callback++;
	}
}*/
/*####################################################################################################################*/ 
// Base orientation 
/*void model::Base_anguler_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i_callback = 0;
	for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		desired.Base_angular_ref[i_callback] = *it; i_callback++;
	}
} */ 


/*####################################################################################################################*/
// Stores the content of the ROS message msg inside of the q_target parameter 
// Get the desired configuration from the ROS message data and store it in the appropraite variable 
/*void model::q_desired_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i_callback = 0;
	for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		q_target[i_callback] = *it; i_callback++;
	}
}*/
/*####################################################################################################################*/
// Stores the content of the ROS message msg inside of the	Dq_target parameter 
// Get the desired joint velocity from the ROS message data and store it in the appropraite variable 
/*void model::Dq_desired_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i_callback = 0;
	for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		Dq_target[i_callback] = *it; i_callback++;
	}
}*/
/*####################################################################################################################*/
// Stores the content of the ROS message msg inside of the	DDq_target parameter 
// Get the desired joint acceleration from the ROS message data and store it in the appropraite variable 
/*void model::DDq_desired_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i_callback = 0;
	for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		DDq_target[i_callback] = *it; i_callback++;
	}
}*/





/*####################################################################################################################*/
// Stores the content of the ROS message msg inside of the pIN1_target_3D_all parameter 
/*
void Joint::legtip_p_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
int i_callback = 0;
for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
{
legtip.pIN1_target_3D_all[i_callback] = *it; i_callback++;
}
}
*/
/*####################################################################################################################*/
// Stores the content of the ROS message msg inside of the pIN1_target_3D_all parameter 
/*
void Joint::legtip_v_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
int i_callback = 0;
for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
{
legtip.vIN1_target_3D_all[i_callback] = *it; i_callback++;
}
}
*/





/*####################################################################################################################*/
// Contact 
/*
void Joint::FL_Contact_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
contact.states[0] = msg->states.size();
}

void Joint::FR_Contact_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
contact.states[1] = msg->states.size();
}

void Joint::HL_Contact_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
contact.states[3] = msg->states.size();
}

void Joint::HR_Contact_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
contact.states[2] = msg->states.size();
}
*/
/*####################################################################################################################*/
