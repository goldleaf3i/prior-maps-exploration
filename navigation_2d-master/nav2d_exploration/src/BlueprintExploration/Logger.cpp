/*
 * Logger.cpp
 *
 *  Created on: 01 ago 2017
 *      Author: danilo
 */

#include "Logger.h"
#include <nav2d_exploration/Log.h>

Logger::Logger(){
	beginningTime = ros::Time::now();
	travelledDistance = 0;
	frontierDistance = 0;
	frontierInformationGainFloorplan = 0;
	frontierInformationGainNoFloorplan = 0;
	alpha = 0;
	frontierUtility = 0;
	frontierInformationGainMode = "floorplan";
	coveragePercentage = 0;

	ros::NodeHandle node("~/");
	logPublisher = node.advertise<nav2d_exploration::Log>("log", 1, true);
}


Logger::~Logger() {

}

void Logger::publishLog(){
	nav2d_exploration::Log log;
	double time = computeTime();
	log.time = time;
	log.travelled_distance = this->travelledDistance;
	log.frontier_distance = this->frontierDistance;
	log.alpha = this->alpha;
	log.frontier_information_gain_floorplan = this->frontierInformationGainFloorplan;
	log.frontier_information_gain_no_floorplan = this->frontierInformationGainNoFloorplan;
	log.frontier_utility = this->frontierUtility;
	log.frontier_information_gain_mode = this->frontierInformationGainMode;
	log.coverage_percentage = this->coveragePercentage;
	logPublisher.publish(log);
}

void Logger::resetTime(){
	beginningTime = ros::Time::now();
}

double Logger::computeTime(){
	ros::Duration diff = ros::Time::now() - beginningTime;
	return diff.toSec();
}
