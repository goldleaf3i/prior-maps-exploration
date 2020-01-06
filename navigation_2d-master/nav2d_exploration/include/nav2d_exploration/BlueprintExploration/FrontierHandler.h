/*
 * FrontierHandler.h
 *
 *  Created on: 06 lug 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERHANDLER_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERHANDLER_H_

#include <nav2d_navigator/GridMap.h>

class FrontierHandler {

public:
	FrontierHandler();
	FrontierHandler(std::string frontier_distance_mode, std::string area_gain_mode ,double max_laser_range, double laser_resolution, double lethal_cost);
	virtual ~FrontierHandler();
	void setFrontierDistanceMode(std::string mode);
	bool isCloseToFrontier(GridMap* map, unsigned int index);
	void addCellToFrontier(unsigned int index);
	int size();
	int getElementByIndex(int index);
	void add(FrontierHandler frontier);
	double getDistance();
	double computeDistance(GridMap *map, unsigned int from);
	unsigned int getIndex(GridMap *map, unsigned int from);

	void setInformationGainNoFloorplan(double infoGainNoFloorplan);
	double getInformationGainFloorplan();
	double getInformationGainNoFloorplan();
	void setInformationGainFloorplan(double infoGainFloor);
	double computeInformationGainNoFloorplan(GridMap *map, unsigned int from);

	double computeFrontierUtility(double areaMax, double distanceMax, double alpha);
	double getFrontierUtility();
	void drawInfoGainRVIZ(GridMap *map, unsigned int from);

private:
	typedef std::vector<unsigned int> Frontier;
	typedef std::pair<int, int> Coordinate;
	typedef std::vector<Coordinate> CoordinatesList;
	typedef std::vector<CoordinatesList> CoordinatesListsVector;

	//Params
	std::string frontier_distance_mode;
	std::string info_gain_mode;
	double lethal_cost;
	double laser_resolution;
	double max_laser_range;

	//Attributes
	Frontier frontier;
	double frontierDistance;
	double infoGainFloorplan;
	double infoGainNoFloorplan;
	double frontierUtility;
	ros::Publisher areaPublisher;

	//Methods
	bool areClose(GridMap *map, unsigned int index1, unsigned int index2);
	double distance(GridMap *map, unsigned int index1, unsigned int index2);
	unsigned int getMiddlePoint(GridMap *map);
	unsigned int getNearestPointTo(GridMap *map, unsigned int index);
	double distanceFromMiddlePoint(GridMap *map, unsigned int from);

	CoordinatesListsVector getCoordinatesListsVector(GridMap *map, unsigned int originX, unsigned int originY);
	CoordinatesList getCoordinatesList(GridMap *map, unsigned int originX, unsigned int originY, double theta);
	double computeLineX(int x1, int x2, int y1, int y2, int y);
	double computeLineY(int x1, int x2, int y1, int y2, int x);
	bool isLegal(GridMap *map, int x, int y);


	void drawInfoGainRVIZ(GridMap *map, std::vector<int> area);

};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_FRONTIERHANDLER_H_ */
