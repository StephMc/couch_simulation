#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "../include/v_repConst.h"

// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"

// Global variables (modified by topic subscribers):
bool simulationRunning = true;
int frontLeftMotorHandle;
int rearLeftMotorHandle;
int frontRightMotorHandle;
int rearRightMotorHandle;
ros::Publisher motorSpeedPub;

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	//simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
    vrep_common::JointSetStateData motorSpeeds;

    double angle = atan2(msg.linear.y , msg.linear.x);
    double speed = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2));
    double rot = msg.angular.z;

    double wheel_cir = 0.125 * M_PI * 2;

    double vfl = (speed * sin(angle + (M_PI/4))) + rot;
    double vfr = (speed * cos(angle + (M_PI/4))) - rot;
    double vbl = (speed * cos(angle + (M_PI/4))) + rot;
    double vbr = (speed * sin(angle + (M_PI/4))) - rot;

    ROS_WARN("Motor speeds are %lf, %lf, %lf, %lf", vfl, vfr, vbl, vbr);

		// publish the motor speeds:
		motorSpeeds.handles.data.push_back(frontLeftMotorHandle);
		motorSpeeds.handles.data.push_back(rearLeftMotorHandle);
		motorSpeeds.handles.data.push_back(frontRightMotorHandle);
		motorSpeeds.handles.data.push_back(rearRightMotorHandle);
		motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
		motorSpeeds.setModes.data.push_back(2);
		motorSpeeds.setModes.data.push_back(2);
		motorSpeeds.setModes.data.push_back(2);
		motorSpeeds.values.data.push_back((vfl / wheel_cir));
		motorSpeeds.values.data.push_back((vbl / wheel_cir));
		motorSpeeds.values.data.push_back((vfr / wheel_cir));
		motorSpeeds.values.data.push_back((vbr / wheel_cir));
		motorSpeedPub.publish(motorSpeeds);
}

int main(int argc,char* argv[])
{
	// The joint handles are given in the argument list
	// (when V-REP launches this executable, V-REP will also provide the argument list)
    if (argc != 5) {
        printf("Give following arguments: 'frontLeftMotorHandle rearLeftMotorHandle \
                frontRightMotorHandle rearRightMotorHandle'\n");
		sleep(5000);
        return 0;
    }
	frontLeftMotorHandle = atoi(argv[1]);
	rearLeftMotorHandle = atoi(argv[2]);
	frontRightMotorHandle = atoi(argv[3]);
	rearRightMotorHandle = atoi(argv[4]);

    int _argc = 0;
	char** _argv = NULL;
	std::string nodeName("couch");
	ros::init(_argc, _argv, nodeName.c_str());

	if (!ros::master::check()) {
		return(0);
    }
	
	ros::NodeHandle node("~");	
	printf("Couch ros sim node started\n");
	ros::Subscriber subInfo = node.subscribe("/vrep/info", 1, infoCallback);
  ros::Subscriber sub = node.subscribe("/cmd_vel", 1, cmdVelCallback);

    // Start V-REP motor joint subscriber
	ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
	vrep_common::simRosEnableSubscriber srv_enableSubscriber;
	srv_enableSubscriber.request.topicName = "/couch/wheels"; // the topic name
	srv_enableSubscriber.request.queueSize = 1; // the subscriber queue size (on V-REP side)
	srv_enableSubscriber.request.streamCmd = simros_strmcmd_set_joint_state; // the subscriber type
	if (client_enableSubscriber.call(srv_enableSubscriber) && (srv_enableSubscriber.response.subscriberID == -1)) {
	    ros::shutdown();
        printf("Motor subscriber failed\n");
        return 0;
    }

	motorSpeedPub = node.advertise<vrep_common::JointSetStateData>("wheels", 1);
	while (ros::ok() && simulationRunning) {
		ros::spinOnce();
		usleep(50000);
	}
	ros::shutdown();
	printf("Super couch just ended!\n");
	return(0);
}

