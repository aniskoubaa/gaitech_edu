/*
 *  Gaitech Educational Portal
 *
 * Copyright (c) 2016
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "../laserscan/LaserScanner.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>

using namespace std;

sensor_msgs::LaserScan _scanMsg;
ros::Publisher velocity_publisher;
ros::Subscriber scanSubscriber;

ros::Subscriber pose_subscriber;

nav_msgs::Odometry turtlebot_odom_pose;
nav_msgs::Odometry obstacle_pose;


void scanCallback (sensor_msgs::LaserScan scanMessage);
//void reactive_navigation();
void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);
bool GoToGoal(nav_msgs::Odometry  goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);
nav_msgs::Odometry setGoalPosition(double x, double y);
double minDistanceToObstacle();
double avoidObstacle(double speed, double desiredSafeDistanceToObstacle);
void hardSwitches(nav_msgs::Odometry  goal_pose);
bool blendedBehavior(nav_msgs::Odometry  goal_pose, double distance_tolerance);

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "laser_stopper_node");
	ros::NodeHandle n;

	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);
	//subscribe to the odometry topic to get the position of the robot
	pose_subscriber = n.subscribe("/odom", 10, poseCallback);
	//register the velocity publisher
	velocity_publisher =n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);


	ros::spinOnce();
	ros::Rate loop(1);
	loop.sleep();
	ros::spinOnce();
	nav_msgs::Odometry  goal_pose = setGoalPosition(0.45, 2.55);
	//hardSwitches(goal_pose);
	blendedBehavior(goal_pose, 0.75);
	//GoToGoal(goal_pose, 0.4);
	ros::spin();
	return 0;
}

bool blendedBehavior(nav_msgs::Odometry  goal_pose, double distance_tolerance){
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	double Xr=0.0;
	double Yr=0.0;
	double Xg=0.0;
	double Yg=0.0;
	double Xo=0.0;
	double Yo=0.0;

	double Xg_bl=0.0;
	double Yg_bl=0.0;

	double sigma_gtg = 0.65;

	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp_gtg=1.0;
		double Kp_oa=1.2;
		double Kp_gtg_angvel=4.0;
		double Kp_oa_angvel=1.0;
		//double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;

		//robot pose
		Xr = turtlebot_odom_pose.pose.pose.position.x;
		Yr = turtlebot_odom_pose.pose.pose.position.y;
		double theta = tf::getYaw(turtlebot_odom_pose.pose.pose.orientation);

		//goal pose
		Xg = goal_pose.pose.pose.position.x;
		Yg = goal_pose.pose.pose.position.y;

		//obstacle pose
		double distObst = LaserScanner::getFrontRange(_scanMsg);
		//distObst = LaserScanner::getFrontRange(_scanMsg);
		//double angle_to_obstacle = 0;
		Xo=Xr+(distObst*cos(theta));
		Yo=Yr+(distObst*sin(theta));

		//opposite obstacle pose
		double Xop=Xr+(distObst*cos(theta+(PI/2)));
		double Yop=Yr+(distObst*sin(theta+(PI/2)));
		ROS_INFO("xo= %.2f, yo= %.2f | xop= %.2f, yop= %.2f", Xo, Yo,
				Xop, Yop);
		//blended goal position
		Xg_bl=((1.0-sigma_gtg)*Xop)+(sigma_gtg*Xg);
		Yg_bl=((1.0-sigma_gtg)*Yop)+(sigma_gtg*Yg);

		ROS_INFO("xblend= %.2f, yblend= %.2f | xr=%.2f, yr=%.2f, theta=%.2f", Xg_bl, Yg_bl,
				Xr,Yr,theta);
		double e_gtg = getDistance(Xr, Yr, Xg,Yg);
		double e_oa = getDistance(Xr, Yr, Xop,Yop);
		//double E = E+e;


		double ang_vel_oa = Kp_oa_angvel*abs(atan2(Yop-Yr, Xop-Xr)-theta);
		double ang_vel_gtg = Kp_gtg_angvel*(atan2(Yg-Yr, Xg-Xr)-theta);


		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =((1-sigma_gtg)*ang_vel_oa)+((sigma_gtg)*ang_vel_gtg);
		//vel_msg.angular.z = Kp_gtg_angvel*(atan2(Yg-Yr, Xg-Xr)-theta);
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		double linear_vel_oa = (Kp_oa*e_oa)/ang_vel_oa;
		double linear_vel_gtg = (Kp_gtg*e_gtg);
		vel_msg.linear.x = ((1-sigma_gtg)*linear_vel_oa)+((sigma_gtg)*linear_vel_gtg);
		//vel_msg.linear.x = Kp_gtg*e_gtg;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		//ROS_INFO("e= %.2f, angle_error= %.2f ", e, vel_msg.angular.z/4.0);
		velocity_publisher.publish(vel_msg);


		ros::spinOnce();

		loop_rate.sleep();
		/*if (minDistanceToObstacle()<0.7){
			cout<<"end move goal"<<endl;
			vel_msg.linear.x =0;
			vel_msg.angular.z = 0;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			return false;
		}*/

	}while(ros::ok()&&getDistance(Xr, Yr, Xg,Yg)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	return true;

}

void hardSwitches(nav_msgs::Odometry  goal_pose){


	bool destinationReached=false;
	do{
		ROS_INFO("Enter GoToGoal avoid mode\n");
		destinationReached=GoToGoal(goal_pose, 0.3);
		if(!destinationReached){
			ROS_INFO("The robot did reach its destination. Enter ObstacleAvoidance mode\n");
			avoidObstacle(-1, 1.0);
			ROS_INFO("Obstacle avoided\n");
		}
	}while(!destinationReached);
	//reactive_navigation ();
	ROS_INFO("The robot reached its destination\n");

}

nav_msgs::Odometry setGoalPosition(double x, double y){
	nav_msgs::Odometry  goal_pose;
	goal_pose.pose.pose.position.x=0.45;
	goal_pose.pose.pose.position.y=2.55;

	goal_pose.pose.pose.orientation.w=0;
	goal_pose.pose.pose.orientation.x=0;
	goal_pose.pose.pose.orientation.y=0;
	goal_pose.pose.pose.orientation.z=1;

	return goal_pose;
}

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	turtlebot_odom_pose.pose.pose.position.x=pose_message->pose.pose.position.x;
	turtlebot_odom_pose.pose.pose.position.y=pose_message->pose.pose.position.y;
	turtlebot_odom_pose.pose.pose.position.z=pose_message->pose.pose.position.z;

	turtlebot_odom_pose.pose.pose.orientation.w=pose_message->pose.pose.orientation.w;
	turtlebot_odom_pose.pose.pose.orientation.x=pose_message->pose.pose.orientation.x;
	turtlebot_odom_pose.pose.pose.orientation.y=pose_message->pose.pose.orientation.y;
	turtlebot_odom_pose.pose.pose.orientation.z=pose_message->pose.pose.orientation.z;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void scanCallback (sensor_msgs::LaserScan scanMessage){
	_scanMsg = scanMessage;
	//cout<<scanMessage.ranges[0]<<endl;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}

double avoidObstacle(double speed, double desiredSafeDistanceToObstacle){

	/**move(1.0, desiredSafeDistanceToObstacle,false);**/

	int start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(20));
	int end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(25));
	double LR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
	ROS_INFO("LR: %.2f start %d, end %d", LR, start_index, end_index);


	start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-5));
	end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(5));
	double SR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
	ROS_INFO("SR: %.2f start %d, end %d", SR, start_index, end_index);

	start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-25));
	end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-20));
	double RR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
	ROS_INFO("RR: %.2f, start %d, end %d", RR, start_index, end_index);




	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (RR<LR){
		vel_msg.angular.z =1.0;
	}
	else if (RR>LR){
		vel_msg.angular.z =-1.0;
	}
	else
		vel_msg.angular.z =0.0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(minDistanceToObstacle()<desiredSafeDistanceToObstacle);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();

	return minDistanceToObstacle();

}

bool GoToGoal(nav_msgs::Odometry  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	double Xr=0.0;
	double Yr=0.0;
	double Xg=0.0;
	double Yg=0.0;
	ofstream outfile;
	outfile.open("distancelog.cvs");
	do{
		/****** Proportional Controller ******/
		//gains of the proportional controllers
		double Kp_v=1.0;
		double Kp_w=4.0;

		//get coordinates of the robot
		Xr = turtlebot_odom_pose.pose.pose.position.x;
		Yr = turtlebot_odom_pose.pose.pose.position.y;

		//get the coordinates of the goal location
		Xg = goal_pose.pose.pose.position.x;
		Yg = goal_pose.pose.pose.position.y;

		double e = getDistance(Xr, Yr, Xg,Yg);
		//double E = E+e;
		double theta_robot = tf::getYaw(turtlebot_odom_pose.pose.pose.orientation);
		double theta_goal = atan2(Yg-Yr, Xg-Xr);
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);

		//linear velocity control
		vel_msg.linear.x = (Kp_v*e)/abs(Kp_w*(theta_goal-theta_robot));
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		//angular velocity control
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =Kp_w*(theta_goal-theta_robot);

		//publish the velocity message
		velocity_publisher.publish(vel_msg);

		ros::spinOnce();

		loop_rate.sleep();
		//watch dog
		if (minDistanceToObstacle()<0.7){
			ROS_INFO("Goal could not be reached. Stopped because of an obstacle\n");
			vel_msg.linear.x =0;
			vel_msg.angular.z = 0;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			return false;
		}

		//Statistics


		outfile << e << endl;


	}while(getDistance(Xr, Yr, Xg,Yg)>distance_tolerance);
	outfile.close();
	ROS_INFO("Goal is reached. Mission completed\n");
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	return true;
}

double minDistanceToObstacle(){

	return LaserScanner::getMinimumRange(_scanMsg);

}

/*
void reactive_navigation(){

	geometry_msgs::Twist velocity_message;

	ros::Rate loop_rate(10); //in hertz
	loop_rate.sleep();
	ros::spinOnce();
	double last_velocity=0.0;
	while (ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
		int start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(20));
		int end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(25));
		double LR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
		ROS_INFO("LR: %.2f start %d, end %d", LR, start_index, end_index);


		start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-5));
		end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(5));
		double SR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
		ROS_INFO("SR: %.2f start %d, end %d", SR, start_index, end_index);

		start_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-25));
		end_index = LaserScanner::Angle2Index(_scanMsg, degree2radian(-20));
		double RR = LaserScanner::getAverageRange(_scanMsg, start_index, end_index);
		ROS_INFO("RR: %.2f, start %d, end %d", RR, start_index, end_index);


		ROS_INFO("min_range:");
		double w = 2.0;
		if (LaserScanner::getMinimumRange(_scanMsg)<0.7){
			velocity_message.linear.x=-1.0;
			last_velocity = -1.0;
			//velocity_message.angular.z=2*w;
		}else  if (LaserScanner::getMinimumRange(_scanMsg)>1.5){
			velocity_message.linear.x=1.0;
			last_velocity = 1.0;
			//velocity_message.angular.z=2*w;
		}else{
			velocity_message.linear.x=last_velocity;
		}


		velocity_message.linear.y=0;
		velocity_message.linear.z=0;
		velocity_message.angular.x=0;
		velocity_message.angular.y=0;


		if (SR<1.0){
			velocity_message.angular.z=2*w/SR;
		}
		if(LR-RR>1.0){
			ROS_INFO("turn left");
			velocity_message.angular.z=w;
		}else if(LR-RR<1.0){
			ROS_INFO("turn right");
			velocity_message.angular.z=-w;
		}else{
			velocity_message.angular.z=0;
		}

		velocity_publisher.publish(velocity_message);

		loop_rate.sleep();
		ros::spinOnce();
	}


}
 */
