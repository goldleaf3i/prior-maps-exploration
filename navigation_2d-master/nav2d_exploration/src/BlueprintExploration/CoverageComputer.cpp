/*
 * CoverageComputer.cpp
 *
 *  Created on: 20 ago 2017
 *      Author: danilo
 */

#include <BlueprintExploration/CoverageComputer.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

CoverageComputer::CoverageComputer() {
	map_topic = "static_map";
	completeMap = new GridMap();
	coveragePercentage = 0;

	node = new ros::NodeHandle();
	node->setCallbackQueue(&mapCallbackQueue);
	subscriber = node->subscribe(map_topic, 1, &CoverageComputer::mapCallback, this);
	spinner = new ros::AsyncSpinner(1, &mapCallbackQueue);
}

CoverageComputer::CoverageComputer(std::string map_topic){
	this->map_topic = map_topic;
	completeMap = new GridMap();
	coveragePercentage = 0;

	node = new ros::NodeHandle();
	node->setCallbackQueue(&mapCallbackQueue);
	subscriber = node->subscribe(map_topic, 1, &CoverageComputer::mapCallback, this);
	spinner = new ros::AsyncSpinner(1, &mapCallbackQueue);
}

CoverageComputer::~CoverageComputer() {
	delete completeMap;
	delete node;
	delete spinner;
}

void CoverageComputer::mapCallback(const nav_msgs::OccupancyGridConstPtr &grid){
	completeMap->update(*grid);
}

void CoverageComputer::updateCompleteGrid(){
	spinner->start();
}

double CoverageComputer::getCoveragePercentage(){
	return coveragePercentage;
}

double CoverageComputer::computeCoveragePercentage(GridMap *map){
	int actualCoverage = 0;
	int totalCoverage = 0;
	for (int i = 0; i < completeMap->getSize(); i++){
		if (completeMap->getData(i) == -1)
			continue;
		else{
			int mapIndex = getIndexOnActualMap(map, i);
			if(map->getData(mapIndex) != -1){
				actualCoverage++;
				totalCoverage++;
			}else
				totalCoverage++;
		}
	}

	coveragePercentage = (double) actualCoverage / (double) totalCoverage;
	return coveragePercentage;
}

unsigned int CoverageComputer::getIndexOnActualMap(GridMap *map, unsigned int index){
	unsigned int ret;
	unsigned int x,y;

	completeMap->getCoordinates(x, y, index);

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();
	double originXComplete = completeMap->getOriginX();
	double originYComplete = completeMap->getOriginY();

	int xGlobal = x + originXComplete * 1 / map->getResolution();
	int yGlobal = y + originYComplete * 1 / map->getResolution();

	unsigned int xMap = xGlobal - originXMap * 1 / completeMap->getResolution();
	unsigned int yMap = yGlobal - originYMap * 1 / completeMap->getResolution();

	if (!map->getIndex(xMap, yMap, ret))
		ROS_ERROR("[CoverageComputer] Error in getIndexOnActualMap");
	return ret;
}



