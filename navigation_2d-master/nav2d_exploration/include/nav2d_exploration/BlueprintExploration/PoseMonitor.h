/*
 * PoseMonitor.h
 *
 *  Created on: 08 lug 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_POSEMONITOR_H_
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_POSEMONITOR_H_

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav2d_navigator/GridMap.h>

class PoseMonitor {
public:
	PoseMonitor();
	virtual ~PoseMonitor();

	float getDistanceTravelled(GridMap *map);
	void startMonitoring();
	void stopMonitoring();

private:
	//Attributes
	float distanceTravelled;
	bool firstRun;
	ros::NodeHandle *node;
	ros::Subscriber subscriber;
	ros::AsyncSpinner *spinner;
	ros::CallbackQueue poseCallbackQueue;

	float lastX;
	float lastY;

	//Methods
	void poseCallback(const nav_msgs::OdometryConstPtr &odom);
};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_MASTER_NAV2D_EXPLORATION_SRC_BLUEPRINTEXPLORATION_POSEMONITOR_H_ */
