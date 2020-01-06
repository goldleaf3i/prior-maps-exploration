/*
 * FrontierHandler.cpp
 *
 *  Created on: 06 lug 2017
 *      Author: danilo
 */

#include <BlueprintExploration/FrontierListHandler.h>

typedef std::vector<unsigned int> Queue;

FrontierListHandler::FrontierListHandler() {
	frontier_distance_mode = "middle_point";
	max_laser_range = 10;
	laser_resolution = 0.25;
	lethal_cost = 60;
	info_gain_mode = "floorplan";
}

FrontierListHandler::FrontierListHandler(std::string frontier_distance_mode, std::string info_gain_mode, double max_laser_range, double laser_resolution, double lethal_cost){
	this->frontier_distance_mode = frontier_distance_mode;
	this->max_laser_range = max_laser_range;
	this->laser_resolution = laser_resolution;
	this->lethal_cost = lethal_cost;
	this->info_gain_mode = info_gain_mode;
}

FrontierListHandler::~FrontierListHandler() {

}

void FrontierListHandler::deleteElementByIndex(int index){
	frontierList.erase(frontierList.begin() + index);
}

void FrontierListHandler::updateFrontierList(GridMap* map, unsigned int start){
	//Initialization
	frontierList.clear();
	unsigned int mapSize = map->getSize();
	bool *alreadyChecked = new bool[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)
	{
		alreadyChecked[i] = false;
	}


//	Queue queue;
//	queue.insert(queue.begin(), start);
	alreadyChecked[start] = true;

	unsigned int *queue = new unsigned int[map->getSize()];
	int curIndex = 0;
	int curSize = 1;
	queue[0] = start;

	for (int i = 0; i < curSize && i < map->getSize(); i++){
		unsigned int index = queue[i];

		if (map->isFrontier(index))
			addToFrontierList(map, index);

		//Add nearby cells
		unsigned int ind[4];
		ind[0] = index - 1;               // left
		ind[1] = index + 1;               // right
		ind[2] = index - map->getWidth(); // upa
		ind[3] = index + map->getWidth(); // down
		for(unsigned int it = 0; it < 4; it++)
		{
			unsigned int i = ind[it];
			if(map->isFree(i) && alreadyChecked[i] == false)
			{
				queue[curSize] = i;
				curSize++;
				alreadyChecked[i] = true;
			}
		}
	}
	delete[] alreadyChecked;
	delete[] queue;
}

void FrontierListHandler::addToFrontierList(GridMap *map, unsigned int index){
	bool found = false;
	std::vector<int> nearbyFrontiers;
	for(int i = 0 ; i < frontierList.size(); i++){
		FrontierHandler front = frontierList.at(i);
		if (front.isCloseToFrontier(map, index)){
			found = true;
			nearbyFrontiers.insert(nearbyFrontiers.begin(), i);
		}
	}
	if (nearbyFrontiers.size() == 1){
		FrontierHandler frontier = frontierList.at(nearbyFrontiers.at(0));
		frontier.addCellToFrontier(index);
		frontierList[nearbyFrontiers.at(0)] = frontier;
		return;
	}
	//If there are two nearby frontiers, concatenate them
	if(nearbyFrontiers.size() > 1){
		FrontierHandler newFrontier(frontier_distance_mode, info_gain_mode, max_laser_range, laser_resolution, lethal_cost);
		newFrontier.addCellToFrontier(index);
		for (int i = 0; i < nearbyFrontiers.size(); i ++){
			FrontierHandler front = frontierList.at(nearbyFrontiers.at(i));
			newFrontier.add(front);
		}
		FrontierList newFrontierList;
		for (int i = 0; i < frontierList.size(); i++){
			if(std::find(nearbyFrontiers.begin(), nearbyFrontiers.end(), i) == nearbyFrontiers.end()){
				newFrontierList.insert(newFrontierList.begin(), frontierList.at(i));
			}
		}
		newFrontierList.insert(newFrontierList.begin(), newFrontier);
		frontierList = newFrontierList;
		return;
	}
	if(!found){
		FrontierHandler frontier(frontier_distance_mode, info_gain_mode, max_laser_range, laser_resolution, lethal_cost);
		frontier.addCellToFrontier(index);
		frontierList.insert(frontierList.begin(),frontier);
	}
}

int FrontierListHandler::numberOfFrontiersCells(){
	int num = 0;
	for(FrontierList::iterator it = frontierList.begin(); it < frontierList.end(); it++){
		num += it->size();
	}
	return num;
}

FrontierHandler FrontierListHandler::getElementByIndex(int i){
	return frontierList.at(i);
}

int FrontierListHandler::size(){
	return frontierList.size();
}


FrontierHandler FrontierListHandler::getNearestFrontier(GridMap *map, unsigned int from){
	FrontierHandler frontier;
	int minDistance;

	frontier = getElementByIndex(0);
	minDistance = frontier.computeDistance(map, from);

	for (FrontierList::iterator it = frontierList.begin(); it < frontierList.end(); it++){
		FrontierHandler cur = *it;
		int curDistance = cur.computeDistance(map, from);
		if (curDistance < minDistance){
			frontier = cur;
			minDistance = curDistance;
		}
	}

	return frontier;
}

bool FrontierListHandler::empty(){
	return frontierList.size() == 0;
}

void FrontierListHandler::setElementByIndex(FrontierHandler frontier, int index){
	if (index < 0 || index >= frontierList.size())
		ROS_ERROR("[FrontierListHandler] Cannot set element by index. The index is not allowed");
	frontierList[index] = frontier;
}
