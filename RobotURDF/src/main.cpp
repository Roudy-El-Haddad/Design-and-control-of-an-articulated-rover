/*This is the main C++ file that corresponds to the Torque Controller Node in ROS*/
#include <iostream>
#include <Eigen/Dense>
#include "string.h"
#include "ros/ros.h"
#include "headers.h"

using namespace std;
using namespace Eigen;

// Global variables
#define b0 0
#define b 1
#define h1 2
#define k1 3
#define w1 4
#define h2 5
#define k2 6
#define w2 7
#define h3 8
#define k3 9
#define w3 10

int main(int argc, char **argv) {
	bool PID=false; // if false use our custom controller otherwise use PID 
	cout<<"Choose the controller: \n0: our custom controller \n1: PID controller \n";
	cin>>PID;

	std::vector<Link> link; // Create a vector of Link objects called link
	// Initially, the vector is empty
	// Add to it N_B Link objects 
	for (int i = 0; i <= Link::N_B; ++i) {
		link.push_back(Link());
	}

	// Initialize the model
	dynamics_model_initialization(link);

	model Model; // Create a model object 

	// Initialize the target equal with the actual values.

	/*####################################################################################################################*/
	// You must call one of the versions of ros::init() before using any other part of the ROS system.
	// Initialize a ROS node named torque_controller_cpp
	ros::init(argc, argv, "torque_controller_cpp", ros::init_options::NoSigintHandler);
	// Create a NodeHandle object 
	ros::NodeHandle nh;
	signal(SIGINT, mySigintHandler);

	// Create an object that contains all the subscribers 
	SUBSCRIBER sub(nh, Model);
	// Create an object that contains the publishers 
	PUBLISHER pub(nh);

	// Print a message stating that the node has started 
	ROS_INFO("Start torque controller");

	/* Define the publish rate. */
	ros::Rate loop_rate(1000); // Run the publishers in fixed rate.
	double Delta_t = loop_rate.expectedCycleTime().toSec();
	/*####################################################################################################################*/
	/*####################################################################################################################*/
	ros::Time time_begin = ros::Time::now(); // Initialize time_begin to the current time 
cout<<time_begin<<endl;
if(!PID){
	while (ros::ok()) {
		float current_time = ros::Time::now().toSec(); // Create a current time variable 
		cout<<"Current time: "<<current_time<<endl;    

		/* Update the configuration using the measured values */
		Link::Q << Model.measured.q_b_angular, Model.measured.q_b_linear, Model.measured.q_actuated; 
		for (int i = 1; i <= Link::N_B; i++) {
			if (i == 1) { link[i].joint.q = Link::Q.head<6>(); }
			else { link[i].joint.q(0) = Link::Q(i + 4); }
		}

		cout<<"Q "<<endl<< Link::Q <<endl<<endl;
		cout<<"Qd "<<endl<<Link::Qd<<endl<<endl;

		/* Update the R matrix between two frames as well as p and X. */
		R_p_update(link);
		/* Update the Homogeneous transformation matrix. */
		HTM_update(link);

		/* Update contact link data */
		contact_link_update(link);

		/* Update configuration rate using the measured values */
		Link::Qd << Model.measured.w_b, link[1].R * Model.measured.v_b_linear, Model.measured.qd_actuated;
		for (int i = 1; i <= Link::N_B; i++) {
			if (i == 1) { link[i].joint.qd = Link::Qd.head<6>(); }
			else { link[i].joint.qd(0) = Link::Qd(i + 4); }
		}

		/*Update the Mass Matrix,  Centrifugal & Coriolis force and gravity force for Joint Space Model*/
		floating_base_RNEA_C(link);
		floating_base_RNEA(link);
		floating_base_CRBA(link);
		Link::G = Link::C_G - Link::C;
		cout<<"M"<<endl<<Link::M<<endl<<endl;
		cout<<"C"<<endl<<Link::C<<endl<<endl;
		cout<<"G"<<endl<<Link::G<<endl<<endl;

		// 
		n_X_1_Cal(link); //cout<< link[3].n_X_1<<endl<<endl;
		v_wrt_base(link); //cout<< link[1].v_wrt_b<<endl<<endl;

		// Compute the actuation torque 
		Model.Compute_actuation_torque(link, current_time);
		cout<<"tau" <<endl<<Model.tau_act<<endl<<endl;
		// Publish the actuation torque 
		pub.torque_publish(Model.tau_act);
		//cout<<2000<<endl<<endl;
}

		double duration = (ros::Time::now() - time_begin).toSec();
		ros::spinOnce(); loop_rate.sleep();
			}
		else{
	std_msgs::Float64 LspeedMsg; 
	std_msgs::Float64 FspeedMsg; 
	std_msgs::Float64 RspeedMsg; 
    std_msgs::Float64 LHpositionMsg; 
    std_msgs::Float64 FHpositionMsg; 
    std_msgs::Float64 RHpositionMsg; 
    std_msgs::Float64 LKpositionMsg; 
    std_msgs::Float64 FKpositionMsg; 
    std_msgs::Float64 RKpositionMsg; 

    LspeedMsg.data=0;
	FspeedMsg.data=0;
	RspeedMsg.data=0;

    LHpositionMsg.data = 0;
    FHpositionMsg.data = 0;
    RHpositionMsg.data = 0;

    LKpositionMsg.data = 0;
    FKpositionMsg.data = 0;
    RKpositionMsg.data = 0;

  while (ros::ok())
  {
    char in;
    std::cout << "Enter command: ";
    std::cin >> in;
    
    switch(in){
	// Wheel speeds 
    case 'a':
    LspeedMsg.data += 0.5;
	FspeedMsg.data += 0.5;
	RspeedMsg.data += 0.5;
    break;
    case 'z':
    LspeedMsg.data -= 0.5;
	FspeedMsg.data -= 0.5;
	RspeedMsg.data -= 0.5;
    break;

	// Left hip configuration 
    case 'e':
    LHpositionMsg.data += 0.1;
    break;
    case 'r':
    LHpositionMsg.data -= 0.1;
    break;

	// Front hip configuration 
    case 'd':
    FHpositionMsg.data += 0.1;
    break;
    case 'f':
    FHpositionMsg.data -= 0.1;
    break;

	// Right hip configuration 
    case 'c':
    RHpositionMsg.data += 0.1;
    break;
    case 'v':
    RHpositionMsg.data -= 0.1;
    break;

	// Left knee configuration 
    case 't':
    LKpositionMsg.data += 0.1;
    break;
    case 'y':
    LKpositionMsg.data -= 0.1;
    break;

	// Front knee configuration 
    case 'g':
    FKpositionMsg.data += 0.1;
    break;
    case 'h':
    FKpositionMsg.data -= 0.1;
    break;

	// Right knee configuration 
    case 'b':
    RKpositionMsg.data += 0.1;
    break;
    case 'n':
    RKpositionMsg.data -= 0.1;
    break;

	// Modify the speed of the left wheel to attempt turning 
	case 'j':
    LspeedMsg.data += 0.2;
    break;
	case 'k':
    LspeedMsg.data = FspeedMsg.data;
    break;
	case 'l':
    LspeedMsg.data -= 0.2;
    break;

} 

    pub.LH.publish(LHpositionMsg); 
    pub.LK.publish(LKpositionMsg); 
    pub.LW.publish(LspeedMsg);

    pub.FH.publish(FHpositionMsg); 
    pub.FK.publish(FKpositionMsg); 
    pub.FW.publish(FspeedMsg);

    pub.RH.publish(RHpositionMsg); 
    pub.RK.publish(RKpositionMsg); 
    pub.RW.publish(RspeedMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }
		}

	return 0;
}
