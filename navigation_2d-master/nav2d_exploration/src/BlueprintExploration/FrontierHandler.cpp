/*
 * FrontierHandler.cpp
 *
 *  Created on: 06 lug 2017
 *      Author: danilo
 */

#include "FrontierHandler.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

typedef std::pair<int, int> Coordinate;
typedef std::vector<Coordinate> CoordinatesList;
typedef std::vector<CoordinatesList> CoordinatesListsVector;

FrontierHandler::FrontierHandler() {
	frontier_distance_mode = "middle_point";
	infoGainFloorplan = 0;
	infoGainNoFloorplan = 0;
	frontierDistance = 0;
	max_laser_range = 10;
	laser_resolution = 0.25;
	lethal_cost = 60;
	info_gain_mode = "floorplan";

	ros::NodeHandle navigatorNode("~/");
	areaPublisher = navigatorNode.advertise<visualization_msgs::Marker>("infoGainMap", 1, true);
}

FrontierHandler::FrontierHandler(std::string frontier_distance_mode, std::string info_gain_mode ,double max_laser_range, double laser_resolution, double lethal_cost){
	this->frontier_distance_mode = frontier_distance_mode;
	this->max_laser_range = max_laser_range;
	this->laser_resolution = laser_resolution;
	this->lethal_cost = lethal_cost;
	this->info_gain_mode = info_gain_mode;
	infoGainFloorplan = 0;
	infoGainNoFloorplan = 0;
	frontierDistance = 0;
	ros::NodeHandle navigatorNode("~/");
	areaPublisher = navigatorNode.advertise<visualization_msgs::Marker>("infoGainMap", 1, true);
}

FrontierHandler::~FrontierHandler() {

}

void FrontierHandler::setFrontierDistanceMode(std::string mode){
	if (mode != "middle_point" && mode!="nearest_point"){
		frontier_distance_mode = "middle_point";
		ROS_ERROR("[FRONTIER HANDLER] Error in setting frontier distance mode. Set to middle point");
	}else
		frontier_distance_mode = mode;
}

bool FrontierHandler::isCloseToFrontier(GridMap* map, unsigned int index){
	for(Frontier::iterator it = frontier.begin(); it < frontier.end();it++){
		if (areClose(map, index, *it))
			return true;
	}
	return false;
}

void FrontierHandler::addCellToFrontier(unsigned int index){
	frontier.insert(frontier.begin(),index);
}

bool FrontierHandler::areClose(GridMap *map, unsigned int index1, unsigned int index2){
	unsigned int x1,x2,y1,y2;
	if (!map->getCoordinates(x1,y1,index1) || !map->getCoordinates(x2,y2,index2)) {
		ROS_ERROR("[FRONTIER HANDLER] Can't convert index to coordinates");
		return false;
	}
	if (index1 == index2)
		return true;
	if (abs((int) x1 - (int) x2) == 1 && abs((int) y1 - (int) y2) == 0)
		return true;
	if (abs((int) x1 - (int) x2) == 0 && abs((int) y1 - (int) y2) == 1)
		return true;
	if (abs((int) x1 - (int) x2) == 1 && abs((int) y1 - (int) y2) == 1)
		return true;
	return false;
}


int FrontierHandler::size(){
	return frontier.size();
}

int FrontierHandler::getElementByIndex(int i){
	if (i >= 0 && i < frontier.size())
		return frontier.at(i);
	else
		ROS_ERROR("[FRONTIER HANDLER] Cant' get element in position %i", i);
	return -1;
}

void FrontierHandler::add(FrontierHandler frontier){
	for (int i = 0; i < frontier.size(); i++){
		this->addCellToFrontier(frontier.getElementByIndex(i));
	}
}

unsigned int FrontierHandler::getMiddlePoint(GridMap *map){
	unsigned int totx = 0, toty = 0;
	for (Frontier::iterator it = frontier.begin(); it < frontier.end(); it++){
		unsigned int x,y;
		unsigned int index = *it;
		map->getCoordinates(x, y, index);
		totx += x;
		toty += y;
	}
	unsigned int x_ret, y_ret;
	x_ret = totx / frontier.size();
	y_ret = toty / frontier.size();

	unsigned index;
	if (!map->getIndex(x_ret, y_ret, index)){
		ROS_ERROR("[FRONTIER HANDLER] Can't get index of frontier's middle point");
		return -1;
	}else
		return getNearestPointTo(map, index);
}

unsigned int FrontierHandler::getNearestPointTo(GridMap *map, unsigned int index){
	int minDistance, indexMin;
	if (frontier.empty()){
		ROS_ERROR("[FRONTIER HANDLER] Frontier is empty");
		return 0;
	}
	indexMin = frontier.at(0);
	minDistance = distance(map, indexMin, index);
	for (Frontier::iterator it = frontier.begin(); it < frontier.end(); it++){
		int distance;
		unsigned int curIndex = *it;
		distance = this->distance(map, curIndex, index);
		if (distance < minDistance){
			indexMin = curIndex;
			minDistance = distance;
		}
	}
	return indexMin;
}

double FrontierHandler::distance(GridMap *map, unsigned int index1, unsigned int index2){
	unsigned int x1, x2, y1, y2;
	if (!map->getCoordinates(x1, y1, index1) ||	!map->getCoordinates(x2, y2, index2)){
		ROS_ERROR("[FRONTIER HANDLER] Can't compute distance");
		return -1;
	}
	return hypot(abs((int) x1 - (int) x2), abs((int) y1 - (int) y2));
}

double FrontierHandler::distanceFromMiddlePoint(GridMap *map, unsigned int from){
	unsigned int middle = getMiddlePoint(map);
	return distance(map, from, middle);
}

double FrontierHandler::getDistance(){
	return frontierDistance;
}

unsigned int FrontierHandler::getIndex(GridMap *map, unsigned int from){
	if (frontier_distance_mode == "middle_point"){
		return getMiddlePoint(map);
	}
	if (frontier_distance_mode == "nearest_point"){
		return getNearestPointTo(map, from);
	}else
		return getMiddlePoint(map);
}

double FrontierHandler::computeDistance(GridMap *map, unsigned int from){
	if (frontier_distance_mode == "middle_point"){
		frontierDistance = distance(map, from, getMiddlePoint(map));
	}
	if (frontier_distance_mode == "nearest_point"){
		frontierDistance = distance(map, from, getNearestPointTo(map, from));
	}
	return frontierDistance;
}

void FrontierHandler::setInformationGainFloorplan(double infoGainFloor){
	if (infoGainFloor < 0)
		ROS_ERROR("[FRONTIERHANDLER] infoGainFloor cannot be less than zero!");
	infoGainFloorplan = infoGainFloor;
}


void FrontierHandler::setInformationGainNoFloorplan(double infoGainNoFloorplan){
	this->infoGainNoFloorplan = infoGainNoFloorplan;
}

double FrontierHandler::computeInformationGainNoFloorplan(GridMap *map, unsigned int from){
	int totalArea = 0;

	unsigned int indexFrontier = getIndex(map, from);
	unsigned int x,y;
	map->getCoordinates(x, y, indexFrontier);

	bool *alreadyChecked = new bool[map->getSize()];
	for (int i = 0; i < map->getSize(); i++)
		alreadyChecked[i] = false;

	//Rimuovo le coordinate irraggiungibili
	CoordinatesListsVector coordVec = getCoordinatesListsVector(map, x, y);

	std::vector<int> area;
	//Determinare per ogni coordinata se è stata già esplorata o meno, e quindi l'area totale
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;
			map->getIndex(x,y, indexMap);
			if (map->getData(indexMap) > lethal_cost)
				break;
			if(map->getData(indexMap) == -1 && alreadyChecked[indexMap] == false){
				totalArea++;
				area.insert(area.end(), indexMap);
				alreadyChecked[indexMap] = true;
			}
		}
	}

	delete[] alreadyChecked;
	infoGainNoFloorplan = totalArea;
	return infoGainNoFloorplan;
}


CoordinatesListsVector FrontierHandler::getCoordinatesListsVector(GridMap *map, unsigned int originX, unsigned int originY){
	if(!isLegal(map, originX, originY))
		ROS_ERROR("[FrontierHandler] The coordinate passed are not valid");
	static CoordinatesListsVector vec;
	static bool initialized = false;
	if (!initialized){
		CoordinatesListsVector temp;
		for (double angle = 0.0; angle <= 360.0; angle += laser_resolution){
			temp.insert(temp.end(), getCoordinatesList(map, 0, 0, angle));
		}
		vec = temp;
		initialized = true;
	}
	CoordinatesListsVector ret;
	for (int i = 0; i < vec.size(); i++){
		CoordinatesList list = vec.at(i);
		CoordinatesList newList;
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first + originX;
			int y = coord.second + originY;
			if (!isLegal(map, x, y))
				break;
			newList.insert(newList.end(), Coordinate(x, y));
		}
		ret.insert(ret.end(), newList);
	}
	return ret;
}

CoordinatesList FrontierHandler::getCoordinatesList(GridMap *map, unsigned int originX, unsigned int originY, double theta){
	if(!isLegal(map, originX, originY))
		ROS_ERROR("[FrontierHandler] The coordinate passed are not valid");
	if(theta < 0)
		ROS_ERROR("[FrontierHandler] The theta angle must be positive!");
	CoordinatesList ret;
	int xMax, yMax;
	xMax = round( max_laser_range * 1 / map->getResolution() * cos(theta * M_PI / 180.0) + originX);
	yMax = round( max_laser_range * 1 / map->getResolution() * sin(theta * M_PI / 180.0) + originY);

	double angle = theta;
	while (angle >= 360)
		angle -= 360;

	if ((angle >= 0 && angle < 45) || (angle >= 315 && angle <360)){
		for (int x = originX; x <= xMax; x++){
			int y = round(computeLineY(originX, xMax, originY, yMax, x));
				ret.insert(ret.end(), Coordinate(x,y));
		}
	}else if (angle >= 115 && angle < 235){
		for (int x = originX; x >= xMax; x--){
			int y = round(computeLineY(originX, xMax, originY, yMax, x));
					ret.insert(ret.end(), Coordinate(x,y));
		}
	}else if (angle >=45 && angle < 115){
		for (int y = originY; y <= yMax; y++){
			int x = round(computeLineX(originX, xMax, originY, yMax, y));
				ret.insert(ret.end(), Coordinate(x,y));
		}
	}else if (angle >= 235 && angle < 315){
		for (int y = originY; y >= yMax; y--){
			int x = round(computeLineX(originX, xMax, originY, yMax, y));
				ret.insert(ret.end(), Coordinate(x,y));
		}
	}
	return ret;
}


bool FrontierHandler::isLegal(GridMap *map, int x, int y){
	return x >= 0 && x < map->getWidth() && y >= 0 && y < map->getHeight();
}

double FrontierHandler::computeLineX(int x1, int x2, int y1, int y2, int y){
	if (y2 == y1)
		ROS_ERROR("[FrontierHandler] Division by zero");
	double m = ((double) (x2 - x1)) /((double) (y2 - y1));
	return (x1 + m * (y - y1));
}

double FrontierHandler::computeLineY(int x1, int x2, int y1, int y2, int x){
	if (x2 == x1)
		ROS_ERROR("[FrontierHandler] Division by zero");
	double m = ((double)(y2 - y1)) / ((double) (x2 - x1));
	return (m * (x - x1) + y1);
}

double FrontierHandler::computeFrontierUtility(double areaMax, double distanceMax, double alpha){
	double d = (distanceMax - frontierDistance) / distanceMax;
	double i;
	if (info_gain_mode == "floorplan"){
		i = infoGainFloorplan / areaMax;
	}else if(info_gain_mode == "no_floorplan"){
		i = infoGainNoFloorplan / areaMax;
	}else
		ROS_ERROR("[FrontierHandler] The area_gain_mode hasn't a valid value");
	frontierUtility = alpha * d + (1 - alpha) * i;
	return frontierUtility;
}

double FrontierHandler::getFrontierUtility(){
	return frontierUtility;
}

void FrontierHandler::drawInfoGainRVIZ(GridMap *map, unsigned int from){
	unsigned int indexFrontier = getIndex(map, from);
	unsigned int x, y;
	if (!map->getCoordinates(x, y, indexFrontier))
		ROS_ERROR("[FrontierHandler] Error in getting coordinates from frontier's index");

	bool *alreadyChecked = new bool[map->getSize()];
	for (int i = 0; i < map->getSize(); i++)
		alreadyChecked[i] = false;;

	//Rimuovo le coordinate irraggiungibili
	CoordinatesListsVector coordVec = getCoordinatesListsVector(map, x, y);

	std::vector<int> area;
	//Determinare per ogni coordinata se è stata già esplorata o meno, e quindi l'area totale
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;
			map->getIndex(x,y, indexMap);
			if (map->getData(indexMap) > lethal_cost)
				break;
			if(map->getData(indexMap) == -1 && alreadyChecked[indexMap] == false){
				area.insert(area.end(), indexMap);
				alreadyChecked[indexMap] = true;
			}
		}
	}

	drawInfoGainRVIZ(map, area);

	delete[] alreadyChecked;
}

void FrontierHandler::drawInfoGainRVIZ(GridMap *map, std::vector<int> area){
	int numberOfCoord = area.size();
	unsigned int x,y;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = map->getOriginX() + (map->getResolution() / 2);
	marker.pose.position.y = map->getOriginY() + (map->getResolution() / 2);
	marker.pose.position.z = map->getResolution() / 2;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = map->getResolution();
	marker.scale.y = map->getResolution();
	marker.scale.z = map->getResolution();
	marker.color.a = 0.5;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.points.resize(numberOfCoord);
	marker.colors.resize(numberOfCoord);

	srand(1337);
	int r = rand() % 256;
	int g = rand() % 256;
	int b = rand() % 256;


	for(unsigned int i = 0; i < numberOfCoord; i++)
	{
		unsigned int x, y;
		if(!map->getCoordinates(x, y, area.at(i)))
		{
			ROS_ERROR("[FrontierHandler] getCoordinates failed!");
			break;
		}
		marker.points[i].x = x * map->getResolution();
		marker.points[i].y = y * map->getResolution();
		marker.points[i].z = 0;

		marker.colors[i].r = r;
		marker.colors[i].g = g;
		marker.colors[i].b = b;
		marker.colors[i].a = 1.0;

	}
	areaPublisher.publish(marker);
}


double FrontierHandler::getInformationGainFloorplan(){
	return this->infoGainFloorplan;
}

double FrontierHandler::getInformationGainNoFloorplan(){
	return this->infoGainNoFloorplan;
}
