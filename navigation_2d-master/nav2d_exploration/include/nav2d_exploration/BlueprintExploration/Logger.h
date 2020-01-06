/*
 * Logger.h
 *
 *  Created on: 01 ago 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_LOGGER_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_LOGGER_H_

#include <ros/ros.h>

class Logger {
private:
	float travelledDistance;
	float frontierDistance;
	float frontierInformationGainFloorplan;
	float frontierInformationGainNoFloorplan;
	float alpha;
	float frontierUtility;
	float coveragePercentage;
	std::string frontierInformationGainMode;

	ros::Time beginningTime;

	ros::Publisher logPublisher;
public:
	Logger();
	virtual ~Logger();

	void publishLog();
	void resetTime();
	double computeTime();

	//Setter
	void setTravelledDistance(float travelledDistance) {
		this->travelledDistance = travelledDistance;
	}

	void setAlpha(float alpha) {
		this->alpha = alpha;
	}

	void setFrontierInformationGainFloorplan(float frontierInfoGainFloorplan) {
		this->frontierInformationGainFloorplan = frontierInfoGainFloorplan;
	}

	void setFrontierInformatinoGainNoFloorplan(float frontierInformationGainNoFloorplan) {
		this->frontierInformationGainNoFloorplan = frontierInformationGainNoFloorplan;
	}

	void setFrontierDistance(float frontierDistance) {
		this->frontierDistance = frontierDistance;
	}

	void setFrontierUtility(float frontierUtility) {
		this->frontierUtility = frontierUtility;
	}

	void setFrontierInformationGainMode(std::string frontierInformationGainMode){
		this->frontierInformationGainMode = frontierInformationGainMode;
	}

	void setCoveragePercentage(float coveragePercentage){
		this->coveragePercentage = coveragePercentage;
	}
};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_LOGGER_H_ */
