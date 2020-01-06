/*
 * Floorprint.h
 *
 *  Created on: 07 lug 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FLOORPRINT_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FLOORPRINT_H_

#include <BlueprintExploration/FrontierListHandler.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>

class FloorPlan {

public:
	FloorPlan();
	FloorPlan(std::string floorplan_map_topic, double max_laser_range, double laser_resolution, int lethal_cost_floorplan);
	virtual ~FloorPlan();
	int computeInfoGain(GridMap *map, FrontierHandler frontier, unsigned int from);
	void updateFloorPlanGrid();
	void drawInfoGainRVIZ(GridMap *map, FrontierHandler frontier, unsigned int from);

private:
	typedef std::pair<int, int> Coordinate;
	typedef std::vector<Coordinate> CoordinatesList;
	typedef std::vector<CoordinatesList> CoordinatesListsVector;

	//Parameters
	std::string floorplan_map_topic;
	double max_laser_range;
	double laser_resolution;
	int lethal_cost_floorplan;

	//Attributes
	GridMap *floorplan;
	ros::NodeHandle *node;
	ros::Subscriber subscriber;
	ros::AsyncSpinner *spinner;
	ros::CallbackQueue floorplanCallbackQueue;
	ros::Publisher areaPublisher;

	//Methods
	void floorPlanCallback(const nav_msgs::OccupancyGridConstPtr &grid);
	unsigned int getIndexOnFloorPlan(GridMap *map, unsigned int index);
	unsigned int getIndexOnFloorPlan(GridMap *map, unsigned int x, unsigned int y);
	unsigned int getIndexOnMap(GridMap *map, unsigned int index);
	unsigned int getIndexOnMap(GridMap *map, unsigned int x, unsigned int y);
	CoordinatesList getCoordinatesList(GridMap *map, unsigned int originX, unsigned int originY, double theta);
	CoordinatesListsVector getCoordinatesListsVector(GridMap *map, unsigned int originX, unsigned int originY);
	double computeLineX(int x1, int x2, int y1, int y2, int y);
	double computeLineY(int x1, int x2, int y1, int y2, int x);
	bool isLegal(int x, int y);
	void drawInfoGainRVIZ(GridMap *map, std::vector<int> area);

};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FLOORPRINT_H_ */
