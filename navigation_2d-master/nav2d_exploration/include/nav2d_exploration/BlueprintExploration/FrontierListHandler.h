/*
 * FrontierListHandler.h
 *
 *  Created on: 06 lug 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERLISTHANDLER_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERLISTHANDLER_H_

#include <BlueprintExploration/FrontierHandler.h>
#include <nav2d_navigator/GridMap.h>

class FrontierListHandler {
public:
	typedef std::vector<FrontierHandler> FrontierList;

	FrontierListHandler();
	FrontierListHandler(std::string frontier_distance_mode, std::string info_gain_mode, double max_laser_range, double laser_resolution, double lethal_cost);
	virtual ~FrontierListHandler();

	void updateFrontierList(GridMap *map, unsigned int start);
	FrontierHandler getElementByIndex(int index);
	void setElementByIndex(FrontierHandler frontier, int index);
	void deleteElementByIndex(int index);
	int numberOfFrontiersCells();
	int size();
	FrontierHandler getNearestFrontier(GridMap *map, unsigned int from);
	bool empty();

private:
	typedef std::vector<int> DistanceVector;

	//Attributes
	FrontierList frontierList;
	DistanceVector frontierDistance;
	std::string frontier_distance_mode;
	std::string info_gain_mode;
	double max_laser_range;
	double laser_resolution;
	int lethal_cost;

	//Methods
	void addToFrontierList(GridMap* map, unsigned int index);
};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERLISTHANDLER_H_ */
