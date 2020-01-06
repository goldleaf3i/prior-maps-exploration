/*
 * FloorPlan.cpp
 *
 *  Created on: 07 lug 2017
 *      Author: danilo
 */

#include <FloorPlan.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

typedef std::pair<int, int> Coordinate;
typedef std::vector<Coordinate> CoordinatesList;
typedef std::vector<CoordinatesList> CoordinatesListsVector;

FloorPlan::FloorPlan() {
	floorplan_map_topic = "static_map";
	max_laser_range = 10;
	laser_resolution = 0.25;
	lethal_cost_floorplan = 60;
	floorplan = new GridMap();
	floorplan->setLethalCost(lethal_cost_floorplan);

	node = new ros::NodeHandle();
	node->setCallbackQueue(&floorplanCallbackQueue);
	subscriber = node->subscribe(floorplan_map_topic, 1, &FloorPlan::floorPlanCallback, this);
	spinner = new ros::AsyncSpinner(1, &floorplanCallbackQueue);

	ros::NodeHandle navigatorNode("~/");
	areaPublisher = navigatorNode.advertise<visualization_msgs::Marker>("infoGainFloorPlan", 1, true);
}

FloorPlan::~FloorPlan() {
	delete node;
	delete spinner;
	delete floorplan;
}

FloorPlan::FloorPlan(std::string floorplan_map_topic, double max_laser_range, double laser_resolution, int lethal_cost_floorplan){
	this->floorplan_map_topic = floorplan_map_topic;
	this->max_laser_range = max_laser_range;
	this->laser_resolution = laser_resolution;
	this->lethal_cost_floorplan = lethal_cost_floorplan;
	floorplan = new GridMap();
	floorplan->setLethalCost(lethal_cost_floorplan);

	node = new ros::NodeHandle();
	node->setCallbackQueue(&floorplanCallbackQueue);
	subscriber = node->subscribe(floorplan_map_topic, 1, &FloorPlan::floorPlanCallback, this);
	spinner = new ros::AsyncSpinner(1, &floorplanCallbackQueue);

	ros::NodeHandle navigatorNode("~/");
	areaPublisher = navigatorNode.advertise<visualization_msgs::Marker>("infoGainFloorPlan", 1, true);
}

void FloorPlan::updateFloorPlanGrid(){
	spinner->start();
}

unsigned int FloorPlan::getIndexOnFloorPlan(GridMap *map, unsigned int index){
	unsigned int x,y;
	unsigned int ret;
	map->getCoordinates(x, y, index);

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();
	double originXFloor = floorplan->getOriginX();
	double originYFloor = floorplan->getOriginY();

	int xGlobal = x + originXMap * 1 / map->getResolution();
	int yGlobal = y + originXMap * 1 / map->getResolution();

	unsigned int xFloor = xGlobal - originXFloor * 1 / floorplan->getResolution();
	unsigned int yFloor = yGlobal - originYFloor * 1 / floorplan->getResolution();

	if(!floorplan->getIndex(xFloor, yFloor, ret))
		ROS_ERROR("[FLOORPLAN] Error in getIndexOnFloorPlan");
	return ret;
}

unsigned int FloorPlan::getIndexOnFloorPlan(GridMap *map, unsigned int x, unsigned int y){
	unsigned int ret;

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();
	double originXFloor = floorplan->getOriginX();
	double originYFloor = floorplan->getOriginY();

	int xGlobal = x + originXMap * 1 / map->getResolution();
	int yGlobal = y + originXMap * 1 / map->getResolution();

	unsigned int xFloor = xGlobal - originXFloor * 1 / floorplan->getResolution();
	unsigned int yFloor = yGlobal - originYFloor * 1 / floorplan->getResolution();

	if (!floorplan->getIndex(xFloor, yFloor, ret))
		ROS_ERROR("[FLOORPLAN] Error in getIndexOnFloorPlan");
	return ret;
}

unsigned int FloorPlan::getIndexOnMap(GridMap *map, unsigned int x, unsigned int y){
	unsigned int ret;

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();
	double originXFloor = floorplan->getOriginX();
	double originYFloor = floorplan->getOriginY();

	int xGlobal = x + originXFloor * 1 / map->getResolution();
	int yGlobal = y + originYFloor * 1 / map->getResolution();

	unsigned int xMap = xGlobal - originXMap * 1 / floorplan->getResolution();
	unsigned int yMap = yGlobal - originYMap * 1 / floorplan->getResolution();

	if (!map->getIndex(xMap, yMap, ret))
		ROS_ERROR("[FLOORPLAN] Error in getIndexOnMap");
	return ret;
}

unsigned int FloorPlan::getIndexOnMap(GridMap *map, unsigned int index){
	unsigned int ret;
	unsigned int x,y;

	floorplan->getCoordinates(x, y, index);

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();
	double originXFloor = floorplan->getOriginX();
	double originYFloor = floorplan->getOriginY();

	int xGlobal = x + originXFloor * 1 / map->getResolution();
	int yGlobal = y + originYFloor * 1 / map->getResolution();

	unsigned int xMap = xGlobal - originXMap * 1 / floorplan->getResolution();
	unsigned int yMap = yGlobal - originYMap * 1 / floorplan->getResolution();

	if (!map->getIndex(xMap, yMap, ret))
		ROS_ERROR("[FLOORPLAN] Error in getIndexOnMap");
	return ret;
}

int FloorPlan::computeInfoGain(GridMap *map, FrontierHandler frontier, unsigned int from){
	int totalArea = 0;

	unsigned int indexFrontier = frontier.getIndex(map, from);
	unsigned int indexFloorPlanFrontier = getIndexOnFloorPlan(map, indexFrontier);
	unsigned int xFloor, yFloor;
	if (!floorplan->getCoordinates(xFloor, yFloor, indexFloorPlanFrontier))
		ROS_ERROR("[FLOORPLAN] Error in getting coordinates from frontier's index");

	CoordinatesListsVector coordVec = getCoordinatesListsVector(map, xFloor, yFloor);
	CoordinatesListsVector newVec;
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		CoordinatesList newList;
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;
			indexMap = getIndexOnMap(map, x, y);
			if (!floorplan->isFree(x, y) || map->getData(indexMap) >= lethal_cost_floorplan)
				break;
			newList.insert(newList.end(), Coordinate(x,y));
		}
		newVec.insert(newVec.end(), newList);
	}
	coordVec = newVec;

	bool *alreadyChecked = new bool[map->getSize()];
	for (int i = 0; i < map->getSize(); i++)
		alreadyChecked[i] = false;

	std::vector<int> area;
	//Determinare per ogni coordinata se è stata già esplorata o meno, e quindi l'area totale
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;

			indexMap = getIndexOnMap(map, x, y);
			unsigned int indexFloor;
			if(!floorplan->getIndex(x, y, indexFloor))
				ROS_ERROR("[FLOORPLAN] Error in getting index from coordinates!");
			if(map->getData(indexMap) == -1 && alreadyChecked[indexFloor] == false){
				totalArea++;
				area.insert(area.end(), indexFloor);
				alreadyChecked[indexFloor] = true;
			}
		}
	}

	if (totalArea == 0)
		totalArea = 1;

	delete[] alreadyChecked;
	return totalArea;
}


void FloorPlan::floorPlanCallback(const nav_msgs::OccupancyGridConstPtr &grid){
	floorplan->update(*grid);
	ROS_DEBUG("SIZE OF FLOORPLAN %i", floorplan->getSize());
	ROS_DEBUG("RESOLUTION OF FLOORPLAN %f", floorplan->getResolution());
//	spinner->stop();
}

CoordinatesListsVector FloorPlan::getCoordinatesListsVector(GridMap *map, unsigned int originX, unsigned int originY){
	if(!isLegal(originX, originY))
		ROS_ERROR("[FLOORPLAN] The coordinate passed are not valid");
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
				if (!isLegal(x, y))
					break;
				newList.insert(newList.end(), Coordinate(x, y));
			}
			ret.insert(ret.end(), newList);
		}
		return ret;
}

CoordinatesList FloorPlan::getCoordinatesList(GridMap *map, unsigned int originX, unsigned int originY, double theta){
	if(!isLegal(originX, originY))
		ROS_ERROR("[FLOORPLAN] The coordinate passed are not valid");
	if(theta < 0)
		ROS_ERROR("[FLOORPLAN] The theta angle must be positive!");
	CoordinatesList ret;
	int xMax, yMax;
	xMax = round( max_laser_range * 1 / floorplan->getResolution() * cos(theta * M_PI / 180.0) + originX);
	yMax = round( max_laser_range * 1 / floorplan->getResolution() * sin(theta * M_PI / 180.0) + originY);

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


bool FloorPlan::isLegal(int x, int y){
	return x >= 0 && x < floorplan->getWidth() && y >= 0 && y < floorplan->getHeight();
}

double FloorPlan::computeLineX(int x1, int x2, int y1, int y2, int y){
	if (y2 == y1)
		ROS_ERROR("[FLOORPLAN] Division by zero");
	double m = ((double) (x2 - x1)) /((double) (y2 - y1));
	return (x1 + m * (y - y1));
}

double FloorPlan::computeLineY(int x1, int x2, int y1, int y2, int x){
	if (x2 == x1)
		ROS_ERROR("[FLOORPLAN] Division by zero");
	double m = ((double)(y2 - y1)) / ((double) (x2 - x1));
	return (m * (x - x1) + y1);
}

void FloorPlan::drawInfoGainRVIZ(GridMap *map, FrontierHandler frontier, unsigned int from){
	unsigned int indexFrontier = frontier.getIndex(map, from);
	unsigned int indexFloorPlanFrontier = getIndexOnFloorPlan(map, indexFrontier);
	unsigned int xFloor, yFloor;
	if (!floorplan->getCoordinates(xFloor, yFloor, indexFloorPlanFrontier))
		ROS_ERROR("[FLOORPLAN] Error in getting coordinates from frontier's index");

	bool *alreadyChecked= new bool[map->getSize()];
	for (int i = 0; i < map->getSize(); i++)
		alreadyChecked[i] = false;


	//Rimuovo le coordinate irraggiungibili
	CoordinatesListsVector coordVec = getCoordinatesListsVector(map, xFloor, yFloor);
	CoordinatesListsVector newVec;
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		CoordinatesList newList;
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;
			indexMap = getIndexOnMap(map, x, y);
			if (!floorplan->isFree(x, y) || map->getData(indexMap) >= lethal_cost_floorplan)
				break;
			newList.insert(newList.end(), Coordinate(x,y));
		}
		newVec.insert(newVec.end(), newList);
	}
	coordVec = newVec;

	std::vector<int> area;
	//Determinare per ogni coordinata se è stata già esplorata o meno, e quindi l'area totale
	for (int i = 0; i < coordVec.size(); i++){
		CoordinatesList list = coordVec.at(i);
		for (int j = 0; j < list.size(); j++){
			Coordinate coord = list.at(j);
			int x = coord.first;
			int y = coord.second;
			unsigned int indexMap;

			indexMap = getIndexOnMap(map, x, y);
			unsigned int indexFloor;
			if(!floorplan->getIndex(x, y, indexFloor))
				ROS_ERROR("[FLOORPLAN] Error in getting index from coordinates!");

			if(map->getData(indexMap) == -1 && !alreadyChecked[indexFloor]){
				alreadyChecked[indexFloor] = true;
				area.insert(area.end(), indexFloor);
			}
		}
	}

	drawInfoGainRVIZ(map, area);
	delete[] alreadyChecked;
}

void FloorPlan::drawInfoGainRVIZ(GridMap *map, std::vector<int> area){
	int numberOfCoord = area.size();
	unsigned int x,y;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = floorplan->getOriginX() + (floorplan->getResolution() / 2);
	marker.pose.position.y = floorplan->getOriginY() + (floorplan->getResolution() / 2);
	marker.pose.position.z = floorplan->getResolution() / 2;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = floorplan->getResolution();
	marker.scale.y = floorplan->getResolution();
	marker.scale.z = floorplan->getResolution();
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
		if(!floorplan->getCoordinates(x, y, area.at(i)))
		{
			ROS_ERROR("[FLOORPLAN] getCoordinates failed!");
			break;
		}
		marker.points[i].x = x * floorplan->getResolution();
		marker.points[i].y = y * floorplan->getResolution();
		marker.points[i].z = 0;

		marker.colors[i].r = r;
		marker.colors[i].g = g;
		marker.colors[i].b = b;
		marker.colors[i].a = 1.0;

	}
	areaPublisher.publish(marker);
}
