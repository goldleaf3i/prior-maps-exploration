#include <pluginlib/class_list_macros.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include "NearestFrontierPlanner.h"
#include "MultiWavefrontPlanner.h"
#include "MinPosPlanner.h"
#include "BlueprintExploration.h"

PLUGINLIB_EXPORT_CLASS(NearestFrontierPlanner, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(MultiWavefrontPlanner, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(MinPosPlanner, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(BlueprintExploration, ExplorationPlanner)
