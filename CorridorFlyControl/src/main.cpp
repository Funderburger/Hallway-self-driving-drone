#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include "ardrone_autonomy/Navdata.h"
//#include "ardrone_autonomy/ardrone_driver.h"
//#include "ardrone_autonomy/flattrim.h "
#include <geometry_msgs/Twist.h>
#include "drone_control/corridorFlyCommand.h" 
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#define DISTANCE_H 200
#define DISTANCE_W 1400
#define CENTER_X 500
#define CENTER_Y 500
#define SPEED 0.1

using namespace ros;
using namespace std;

Publisher pub;// = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
Publisher pub_land;
float rotationZref=0.0, rotationZ=0.0;
bool setRotationRef=false; // true when the drone took off and stabilized 
bool ctrCall= false;

void controlCallback(const drone_control::corridorFlyCommand::ConstPtr& usm)
{
	// Set default command to hover, unles there is no need for change rotation or corridor fly	
	geometry_msgs::Twist commandTmp;
	commandTmp.linear.x = 0.0;
	commandTmp.linear.y = 0.0;
	commandTmp.linear.z = 0.0;
	commandTmp.angular.z = 0.0;
	// Keep roational angle on Z axes while the ardrone/navrata/rotZ is in 0..360 and not in -180..180
	if (ctrCall){
		cout<<"R "<< rotationZref <<" V " <<rotationZ << endl;  
		if (usm->ahead == "land"){
			// Land command if no VP found
			std_msgs::Empty em;
			pub_land.publish(em);

		}else{
			if (rotationZ>rotationZref+2 || rotationZ<rotationZref-2)
			{
				if ((rotationZref<180 && rotationZ<180)||(rotationZref>=180 && rotationZ>=180))
				{
					if (rotationZ<rotationZref){
						commandTmp.angular.z = 0.15;
					}else{
						commandTmp.angular.z = -0.15;
					}
				}else{
					if (abs(rotationZ-rotationZref)<180){
						commandTmp.angular.z = 0.1;
					}else{
						commandTmp.angular.z = -0.1;
					}	
				}				
				// If the rotation of the plane is OK deal with the coridor flight
			}else{
				float speed = usm->speed;
				if (usm->rotation == "left" ){
					commandTmp.linear.z=-speed;
				}else{ 	
					if (usm->rotation == "right")		
					commandTmp.linear.z=speed;
				}	

				if (usm->direction == "left")
				{
					commandTmp.linear.y=0.015;
				}else{
					if (usm->direction == "right")
					commandTmp.linear.y=-0.015;
				}

				if (usm->ahead == "front")
				{
					commandTmp.linear.x=0.065;
				}else{
					if (usm->ahead == "back"){
						commandTmp.linear.y=-0.065;
					}					
				}
			}
			// Send calculated command (rotation/fly/hover).			
			pub.publish(commandTmp);
		}
	} 
}

void rotStabilizationCallback(const ardrone_autonomy::Navdata::ConstPtr& usm)
{
	if (setRotationRef) 
	{
		rotationZref=(usm->rotZ)+180;
		setRotationRef=false;
	}else{
		rotationZ=(usm->rotZ)+180;
	}
}

int main(int argc, char **argv) {
	//  Initialize the node
	init(argc, argv, "corridor_fly");
	//  Create a node handle
	NodeHandle node;
	//  Subscribe to the corridor fly command data
	Subscriber sub = node.subscribe("/corridor/fly_command", 1, controlCallback);
	//  Subscribe navigational data for stabilizing the rotational around z axes
	Subscriber sub2 = node.subscribe("/ardrone/navdata", 1, rotStabilizationCallback);
	//  A publisher for the movement data 
	pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//  Msg for fly control commands
	geometry_msgs::Twist command;
	command.linear.x = 0.0;
	command.linear.y = 0.0;
	command.linear.z = 0.0;
	command.angular.z = 0.0;
	//  Msg for takeoff and landing 
	std_msgs::Empty msgEmpty;
	Publisher pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);
	pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1, true);  
	//  Service for flat trim and imu calibration, wait 3sec.
	system( "rosservice  call ardrone/imu_recalib" );
	system( "rosservice  call ardrone/flattrim" );
	Duration(3).sleep();
	//  Take off and wait for 10 sec for stabilization and calibration (use landing zone under QC).	
	pub_takeoff.publish(msgEmpty);
	spinOnce();
	Duration(10).sleep();
	setRotationRef  =true;
	Duration(3).sleep();
	ctrCall=true;
	//  Loop at 10Hz, publishing movement commands until we shut down.
	Rate rate(100);
	while (ok()) {
		rate.sleep();
		ros::spinOnce();
	}
	//  Landing commnad
	pub_land.publish(msgEmpty);  
	spinOnce();
	return 0;
}
