#pragma once

#include "ros/ros.h"
#include "JointCallback.h"
#include "Model.h"
#include "MEASURED.h"
#include "DESIRED.h"

class SUBSCRIBER
{
public: 
	/* ### Subscriber Objects ### */ 
	/* Subscribers to the sensor topics */
	// Sensor data used to get the joint configuration q and rate qd 
	ros::Subscriber joint_sub;

	ros::Subscriber imu_sub;
	ros::Subscriber GPS_position_sub;
	ros::Subscriber GPS_velocity_sub;

	//
	/*ros::Subscriber lowerleg_p_sub;
	ros::Subscriber lowerleg_v_sub; */
	
	// Contact force sensors 
	/*ros::Subscriber Contact_FL_sub;
	ros::Subscriber Contact_FR_sub;
	ros::Subscriber Contact_HL_sub;
	ros::Subscriber Contact_HR_sub; */ 

	/* Subscribers to the topics that publish the reference value */
	/*
	ros::Subscriber CoM_linear_sub;
	ros::Subscriber Base_anguler_sub;

	ros::Subscriber q_target_sub;
	ros::Subscriber Dq_target_sub;
	ros::Subscriber DDq_target_sub;
	*/ 

	SUBSCRIBER(ros::NodeHandle & nh, model & Model);
	~SUBSCRIBER() {}
};

SUBSCRIBER::SUBSCRIBER(ros::NodeHandle & nh, model & Model)
{
	// Define the subscriber for joint position, velocity and torque/force sensor.
	joint_sub = nh.subscribe("/RobotURDF/joint_states", 100, &model::jointState_Callback, &Model);
	
	// Define the subscribers for base euler angle and angular velocity detection.
	imu_sub = nh.subscribe("/RobotURDF/imu", 100, &model::imu_Callback, &Model);

	// GPS_p_sub = nh.subscribe("/towr/GPS/fix", 100, &GPS_position_Callback); // It is not right to use the GPS directly.
	// Define the subscribers for base linear position and linear velocity detection.
	GPS_position_sub = nh.subscribe("/RobotURDF/base_pose_ground_truth", 100, &model::GPS_position_Callback, &Model);
	GPS_velocity_sub = nh.subscribe("/RobotURDF/GPS/fix_velocity", 100, &model::GPS_velocity_Callback, &Model);

	// Define the subscriber for CoM linear reference and Base angular reference, in which CoM z reference use actual values.
	//CoM_linear_sub = nh.subscribe("/towr/CoM_reference/desire_CoM_p_v_dv", 100, &model::CoM_linear_Callback, &Model);
	//Base_anguler_sub = nh.subscribe("/towr/Base_reference/desire_base_q_w_dw", 100, &model::Base_anguler_Callback, &Model);

	
	// Define the subscribers for listening from motion generator.
	/*lowerleg_p_sub = nh.subscribe("/towr/lowerleg_end_p/desire_ref", 100, &model::legtip_p_ref, &Model);
	lowerleg_v_sub = nh.subscribe("/towr/lowerleg_end_v/desire_ref", 100, &model::legtip_v_ref, &Model); */ 

	/*q_target_sub = nh.subscribe("/towr/joint_position_generation/desire_ref", 100, &model::q_desired_ref, &Model);
	Dq_target_sub = nh.subscribe("/towr/joint_velocity_generation/desire_ref", 100, &model::Dq_desired_ref, &Model);
	DDq_target_sub = nh.subscribe("/towr/joint_acceleration_generation/desire_ref", 100, &model::DDq_desired_ref, &Model); */ 

	// Define the subscribers for contact state detection.
	/*
	Contact_FL_sub = nh.subscribe("/FL_WFB_LINK_contact_sensor_state", 100, &Joint::FL_Contact_Callback, &Model);
	Contact_FR_sub = nh.subscribe("/FR_WFB_LINK_contact_sensor_state", 100, &Joint::FR_Contact_Callback, &Model);
	Contact_HL_sub = nh.subscribe("/HL_WFB_LINK_contact_sensor_state", 100, &Joint::HL_Contact_Callback, &Model);
	Contact_HR_sub = nh.subscribe("/HR_WFB_LINK_contact_sensor_state", 100, &Joint::HR_Contact_Callback, &Model);
	*/


}

void mySigintHandler(int sig)
{
	ROS_INFO("Stop Towr torque controller");
	ros::shutdown();
}
