/*
 * PoseMonitor.cpp
 *
 *  Created on: 08 lug 2017
 *      Author: danilo
 */

#include "PoseMonitor.h"
#include <ros/console.h>
#include <ros/ros.h>

PoseMonitor::PoseMonitor() {
	distanceTravelled = 0;
	firstRun = true;

	node = new ros::NodeHandle();
	node->setCallbackQueue(&poseCallbackQueue);
	subscriber = node->subscribe("/odom", 100, &PoseMonitor::poseCallback, this);
	spinner = new ros::AsyncSpinner(1, &poseCallbackQueue);
}


PoseMonitor::~PoseMonitor() {
	delete node;
	delete spinner;
}

float PoseMonitor::getDistanceTravelled(GridMap *map){
	return distanceTravelled;
}

void PoseMonitor::poseCallback(const nav_msgs::OdometryConstPtr &odom){
	float x = odom->pose.pose.position.x;
	float y = odom->pose.pose.position.y;
	if (firstRun){
		lastX = x;
		lastY = y;
		firstRun = false;
	}else{
		distanceTravelled += hypot(x - lastX, y - lastY);
		lastX = x;
		lastY = y;
	}
}


void PoseMonitor::startMonitoring(){
	spinner->start();
}

void PoseMonitor::stopMonitoring(){
	spinner->stop();
}
