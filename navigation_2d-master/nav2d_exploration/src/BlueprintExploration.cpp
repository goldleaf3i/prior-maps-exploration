/*
 * BlueprintExploration.cpp
 *
 *  Created on: 15 giu 2017
 *      Author: danilo
 */

#include "BlueprintExploration.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <chrono>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


BlueprintExploration::BlueprintExploration() {
	ros::NodeHandle navigatorNode("~/");
	navigatorNode.param("floorplan_filename", this->floorplan_filename, std::string(""));
	navigatorNode.param("floorplan_map_topic", this->floorplan_map_topic, std::string("map_static"));
	navigatorNode.param("frontier_distance_mode", this->frontier_distance_mode, std::string("middle_point"));
	navigatorNode.param("lethal_cost", this->lethal_cost, 33);
	navigatorNode.param("alpha", this->alpha, 0.1);
	navigatorNode.param("max_laser_range", this->max_laser_range, 10.0);
	navigatorNode.param("laser_resolution", this->laser_resolution, 0.1);
	navigatorNode.param("lethal_cost_floorplan", this->lethal_cost_floorplan, 60);
	navigatorNode.param("information_gain", this->information_gain, std::string("floorplan"));
	navigatorNode.param("complete_map_topic", this->complete_map_topic, std::string("blueprint_map_server_coverage/map"));
	navigatorNode.param("publish_goal_to_move_base", this->publish_to_move_base, false);
	frontierPublisher = navigatorNode.advertise<visualization_msgs::Marker>("frontiers", 1, true);

	frontierList = FrontierListHandler(frontier_distance_mode, information_gain, max_laser_range, laser_resolution, 50);
	poseMonitor.startMonitoring();
	floorplan = new FloorPlan(floorplan_map_topic, max_laser_range, laser_resolution, lethal_cost_floorplan);
	floorplan->updateFloorPlanGrid();

	logger.resetTime();
	timerInitialized = false;

	coverageComputer = new CoverageComputer(this->complete_map_topic);
	coverageComputer->updateCompleteGrid();

	//View debug messages
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	updatedMap = new GridMap();

	if (publish_to_move_base){
		node = new ros::NodeHandle();
		node->setCallbackQueue(&globalCostmapCallbackQueue);
		globalCostmapSubscriber = node->subscribe("/move_base/global_costmap/costmap", 1, &BlueprintExploration::globalCostmapCallback, this);

		globalCostmap = new GridMap();
		globalCostmap->setLethalCost(lethal_cost);
		ac = new MoveBaseClient("move_base", true);

		spinner = new ros::AsyncSpinner(1, &globalCostmapCallbackQueue);
		spinner->start();
	}
}

BlueprintExploration::~BlueprintExploration() {
	delete floorplan;
	delete coverageComputer;
	delete updatedMap;
	if (publish_to_move_base){
		delete ac;
		delete spinner;
		delete globalCostmap;
		delete node;
	}
}

void BlueprintExploration::globalCostmapCallback(const nav_msgs::OccupancyGridConstPtr &grid){
	globalCostmapGrid = *grid;
	globalCostmap->update(globalCostmapGrid);
	globalCostmapUpdateSubscriber = node->subscribe("/move_base/global_costmap/costmap_updates", 20, &BlueprintExploration::globalCostmapUpdatesCallback, this);
}

void BlueprintExploration::globalCostmapUpdatesCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg){
	int index = 0;
	for(int y=msg->y; y< msg->y+msg->height; y++){
		for(int x=msg->x; x< msg->x+msg->width; x++){
			int sx = globalCostmapGrid.info.width;
			int temp = y * sx + x;
			globalCostmapGrid.data[ temp ] = msg->data[ index++ ];
		}
	}
	globalCostmap->update(globalCostmapGrid);
}


int BlueprintExploration::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal){
	GridMap *actualMap = new GridMap();
	actualMap->update(map->getMap());
	actualMap->setLethalCost(lethal_cost);

	if(publish_to_move_base){
		actualMap->update(globalCostmap->getMap());
		actualMap->setLethalCost(lethal_cost);
	}

	unsigned int x,y;
	actualMap->getCoordinates(x, y, start);
	unsigned int originX, originY;
	originX = actualMap->getOriginX();
	originY = actualMap->getOriginY();

	frontierList.updateFrontierList(actualMap, start);
	drawFrontiersRVIZ(actualMap);
	ROS_DEBUG("Number of frontiers detected: %i", frontierList.size());
	if (!frontierList.empty()){
		unsigned int index = computeNextFrontier(actualMap, start);
		FrontierHandler frontier = frontierList.getElementByIndex(index);
		goalFindTarget = frontier.getIndex(actualMap, start);

		//Draw area gain
		if (information_gain == "floorplan")
			floorplan->drawInfoGainRVIZ(actualMap, frontier, start);
		else
			frontier.drawInfoGainRVIZ(actualMap, start);

		if (!timerInitialized){
			logger.resetTime();
			timerInitialized = true;
		}
		//Publish log
		logger.setFrontierInformationGainFloorplan(frontier.getInformationGainFloorplan());
		logger.setFrontierInformatinoGainNoFloorplan(frontier.getInformationGainNoFloorplan());
		logger.setFrontierDistance(frontier.getDistance());
		logger.setFrontierUtility(frontier.getFrontierUtility());
		float travelledDistance = poseMonitor.getDistanceTravelled(actualMap);
		ROS_INFO("Distance travelled so far: %f", travelledDistance);
		logger.setAlpha(alpha);
		logger.setTravelledDistance(travelledDistance);
		logger.setFrontierInformationGainMode(this->information_gain);
		double cov = coverageComputer->computeCoveragePercentage(actualMap);
		ROS_INFO("Amount of explored area so far: %f percent", cov);
		logger.setCoveragePercentage(cov);
		logger.publishLog();
		returnFindTarget = EXPL_TARGET_SET;
	}else{
		logger.setFrontierInformationGainFloorplan(0);
		logger.setFrontierInformatinoGainNoFloorplan(0);
		logger.setFrontierDistance(0);
		logger.setFrontierUtility(0);
		float travelledDistance = poseMonitor.getDistanceTravelled(actualMap);
		ROS_INFO("Distance travelled so far: %f", travelledDistance);
		logger.setAlpha(alpha);
		logger.setTravelledDistance(travelledDistance);
		logger.setFrontierInformationGainMode(this->information_gain);
		logger.setCoveragePercentage(coverageComputer->computeCoveragePercentage(actualMap));
		logger.publishLog();
		returnFindTarget = EXPL_FINISHED;
	}
	delete actualMap;

	goal = goalFindTarget;
	if (publish_to_move_base && returnFindTarget == EXPL_TARGET_SET)
		publishGoalToMoveBase(map, goal);
	return returnFindTarget;

}

void BlueprintExploration::publishGoalToMoveBase(GridMap *map, unsigned int goal){
	unsigned int x,y;

	map->getCoordinates(x, y, goal);

	double originXMap = map->getOriginX();
	double originYMap = map->getOriginY();

	double xGlobal = x * map->getResolution() + originXMap;
	double yGlobal = y * map->getResolution() + originXMap;

	move_base_msgs::MoveBaseGoal goalM;
	goalM.target_pose.header.frame_id = "map";
	goalM.target_pose.header.stamp = ros::Time::now();
	goalM.target_pose.pose.position.x = xGlobal;
	goalM.target_pose.pose.position.y = yGlobal;
	goalM.target_pose.pose.orientation.w = 1.0;
	ac->sendGoal(goalM);
}

unsigned int BlueprintExploration::computeNextFrontier(GridMap *map, unsigned int from){
	if (frontierList.empty()){
		ROS_ERROR("[BlueprintExploration] The frontiers' list is empty!");
		return -1;
	}

//	for (int i = 0; i < frontierList.size(); i++){
//		FrontierHandler cur = frontierList.getElementByIndex(i);
//		if (information_gain == "floorplan"){
//			double a = floorplan->computeInfoGain(map, cur, from);
//			if (a <= 100){
//				frontierList.deleteElementByIndex(i);
//			}
//		}
//	}

	double areaMax = -1, distanceMax = -1;
	for (int i = 0; i < frontierList.size(); i++){
		FrontierHandler cur = frontierList.getElementByIndex(i);
		double a = 0;
		if (information_gain == "floorplan"){
			a = floorplan->computeInfoGain(map, cur, from);
			cur.setInformationGainFloorplan(a);
		}else{
			a = cur.computeInformationGainNoFloorplan(map, from);
		}
		double d = cur.computeDistance(map, from);
		if (a > areaMax)
			areaMax = a;
		if (d > distanceMax)
			distanceMax = d;
		frontierList.setElementByIndex(cur, i);
	}

	int maxIndex = 0;
	double maxUtility = frontierList.getElementByIndex(0).computeFrontierUtility(areaMax, distanceMax, alpha);
	for (int i = 0; i < frontierList.size(); i++){
		FrontierHandler cur = frontierList.getElementByIndex(i);
		double curUtility = cur.computeFrontierUtility(areaMax, distanceMax, alpha);
		frontierList.setElementByIndex(cur, i);
		if (curUtility > maxUtility){
			maxIndex = i;
			maxUtility = curUtility;
		}
	}

	ROS_DEBUG("Selected Frontier %i with utilty %f", maxIndex, maxUtility);

	return maxIndex;
}

void BlueprintExploration::drawFrontiersRVIZ(GridMap *map){
	int mFrontierCells = frontierList.numberOfFrontiersCells();
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
	marker.points.resize(mFrontierCells);
	marker.colors.resize(mFrontierCells);

	unsigned int p = 0;
	srand(1337);
	for(unsigned int i = 0; i < frontierList.size(); i++)
	{
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		for(unsigned int j = 0; j < frontierList.getElementByIndex(i).size(); j++)
		{
			if(p < mFrontierCells)
			{
				unsigned int index = frontierList.getElementByIndex(i).getElementByIndex(j);
				if(!map->getCoordinates(x, y, index))
				{
					ROS_ERROR("[BLUEPRINTEXPLORATION] getCoordinates failed!");
					break;
				}
				marker.points[p].x = x * map->getResolution();
				marker.points[p].y = y * map->getResolution();
				marker.points[p].z = 0;

				marker.colors[p].r = r;
				marker.colors[p].g = g;
				marker.colors[p].b = b;
				marker.colors[p].a = 1.0;
			}else
			{
				ROS_ERROR("[BLUEPRINTEXPLORATION] SecurityCheck failed! (Asked for %d / %d)", p, mFrontierCells);
			}
			p++;
		}
	}
	frontierPublisher.publish(marker);
}
