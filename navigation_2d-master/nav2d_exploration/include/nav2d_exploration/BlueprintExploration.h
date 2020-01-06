/*
 * BlueprintExploration.h
 *
 *  Created on: 15 giu 2017
 *      Author: danilo
 */

#ifndef BLUEPRINTEXPLORATION_NAVIGATION_2D_EXPLORATION
#define BLUEPRINTEXPLORATION_NAVIGATION_2D_EXPLORATION

#include <nav2d_navigator/ExplorationPlanner.h>
#include <BlueprintExploration/FrontierListHandler.h>
#include <BlueprintExploration/PoseMonitor.h>
#include <BlueprintExploration/Logger.h>
#include <BlueprintExploration/CoverageComputer.h>
#include <FloorPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map_msgs/OccupancyGridUpdate.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class BlueprintExploration : public ExplorationPlanner {
public:
	BlueprintExploration();
	~BlueprintExploration();

	int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

private:
	typedef std::vector<double> FrontierCostList;

	//Attributes
	FrontierListHandler frontierList;
	FrontierCostList frontierCostList;
	FloorPlan *floorplan;
	Logger logger;
	CoverageComputer *coverageComputer;
	bool timerInitialized;
	GridMap *updatedMap;
	int returnFindTarget;
	unsigned int goalFindTarget;

	ros::Publisher frontierPublisher;
	PoseMonitor poseMonitor;

	//Move_base attributes
	MoveBaseClient *ac;
	ros::NodeHandle *node;
	ros::Subscriber globalCostmapSubscriber;
	ros::AsyncSpinner *spinner;
	ros::CallbackQueue globalCostmapCallbackQueue;
	ros::Subscriber globalCostmapUpdateSubscriber;
	nav_msgs::OccupancyGrid globalCostmapGrid;
	GridMap *globalCostmap;


	//Parameters
	std::string floorplan_filename;
	std::string frontier_distance_mode;
	std::string floorplan_map_topic;
	std::string information_gain;
	std::string complete_map_topic;
	int lethal_cost;
	int lethal_cost_floorplan;
	double alpha;
	double max_laser_range;
	double laser_resolution;
	bool publish_to_move_base;


	//Methods
	void drawFrontiersRVIZ(GridMap *map);
	unsigned int computeNextFrontier(GridMap *map, unsigned int from);
	void publishGoalToMoveBase(GridMap *map, unsigned int goal);

	void globalCostmapCallback(const nav_msgs::OccupancyGridConstPtr &grid);
	void globalCostmapUpdatesCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg);
};

#endif /* BLUEPRINTEXPLORATION_NAVIGATION_2D_EXPLORATION */
