/*
 * CoverageComputer.h
 *
 *  Created on: 20 ago 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_INCLUDE_NAV2D_EXPLORATION_BLUEPRINTEXPLORATION_COVERAGECOMPUTER_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_INCLUDE_NAV2D_EXPLORATION_BLUEPRINTEXPLORATION_COVERAGECOMPUTER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav2d_navigator/GridMap.h>

class CoverageComputer {
public:
	CoverageComputer();
	CoverageComputer(std::string map_topic);
	virtual ~CoverageComputer();

	void updateCompleteGrid();
	double getCoveragePercentage();
	double computeCoveragePercentage(GridMap *map);

private:
	//Attributes
	GridMap *completeMap;
	ros::NodeHandle *node;
	ros::Subscriber subscriber;
	ros::AsyncSpinner *spinner;
	ros::CallbackQueue mapCallbackQueue;
	std::string map_topic;

	double coveragePercentage;

	void mapCallback(const nav_msgs::OccupancyGridConstPtr &grid);
	unsigned int getIndexOnActualMap(GridMap *map, unsigned int index);
};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_INCLUDE_NAV2D_EXPLORATION_BLUEPRINTEXPLORATION_COVERAGECOMPUTER_H_ */
