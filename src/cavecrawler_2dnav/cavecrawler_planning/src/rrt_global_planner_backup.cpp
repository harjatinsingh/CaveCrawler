#include <ros/ros.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <visualization_msgs/Marker.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
// using namespace ca;

// bool myStateValidityCheckerFunction(const oc::SpaceInformation *si,const ob::State *state)
bool myStateValidityCheckerFunction(const ob::State *state)
{
    return true;

    //  if (!si->satisfiesBounds(state))
    // {
    //     return false;
    // }
    // else return true;

    // Get the cost of the footprint at the current location:
    // double cost = calc_cost(state);

    // // std::cout << cost << std::endl;
    // // Too high cost:
    // if (cost > 90)
    // {
    //     return false;
    // }

    // // Error? Unknown space?
    // if (cost < 0)
    // {
    // }

    // return true;

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "rrt_global_planner_node");
  ros::NodeHandle n("~");

  ompl::msg::noOutputHandler();
  ros::Publisher pub_path_marker = n.advertise<visualization_msgs::Marker>("path", 1);
  ros::Publisher pub_graph_marker = n.advertise<visualization_msgs::Marker>("graph", 1);
  ros::Duration(0.1).sleep();

  // Create R3 space
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
  ob::RealVectorBounds bounds(3);  // We are going to create a unit box from (-1,-1,-1) to (1,1,1)
  bounds.setLow(0, -1.0);
  bounds.setLow(1, -1.0);
  bounds.setLow(2, -1.0);
  bounds.setHigh(0, 1.0);
  bounds.setHigh(1, 1.0);
  bounds.setHigh(2, 1.0);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));



  // Set start and goal
  ob::ScopedState<ob::RealVectorStateSpace> start(si), goal(si); // Planning from (0,0,0) to (0.8, 0.8, 0.8)
  start->values[0] = 0;
  start->values[1] = 0;
  start->values[2] = 0;
  goal->values[0] = 0.8;
  goal->values[1] = 0.8;
  goal->values[2] = 0.8;

  // Create some obstacles
  // ShapeSet obstacle_set;
  // obstacle_set.AddShape(boost::shared_ptr<Shape>(new Cuboid(0.4, 0.3, 0.4, 1.6, -1.1, 1.1))); // x_com,  y_com,  x_size,  y_size,  z_lower,  z_upper
  // obstacle_set.AddShape(boost::shared_ptr<Shape>(new Cylinder(0.8, 0.4, 0.35, 0.2, 1.0))); //  x_com, y_com, z_com, radius, height

  // Tell the planner how to collision check by creating a function handle that takes a state as input and returns feasible or not
  // boost::function<bool(const ob::State*)> valid_fn = [&](const ob::State* s) {
  //   Eigen::Vector3d pos = planning_common::workspace_utils::GetTranslationVector3d(si, s); // Extract 3d workspace information (would work for non R3 spaces too!)
  //   return !obstacle_set.InShapeSet(pos);
  // };
  // si->setStateValidityChecker(valid_fn);
  si->setStateValidityChecker(myStateValidityCheckerFunction);
  si->setStateValidityCheckingResolution(0.001); // dimensionless number as a fraction of the workspace size
  si->setup();

  // Setup problem
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start.get(), goal.get());
  pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

  // Setup planner
  // boost::shared_ptr<ompl::geometric::RRTstar> rrtstar(new ompl::geometric::RRTstar(si));
  // ob::PlannerPtr planner = rrtstar;

  auto planner(std::make_shared<og::RRTConnect>(si));

  while (ros::ok()) 
  {
    //ROS_INFO_STREAM("No Solution Found");
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    // if(solved == ob::PlannerStatus::EXACT_SOLUTION) 
    if (solved)
    {
      // boost::shared_ptr<og::PathGeometric> path = boost::static_pointer_cast<og::PathGeometric>(planner->getProblemDefinition()->getSolutionPath()); 
      ob::PathPtr path = pdef->getSolutionPath();
      // ob::Cost cost = planner->getProblemDefinition()->getOptimizationObjective()->pathCost(path.get());
      ROS_INFO_STREAM("Found solution gghghghg");// with cost: "<<cost.value());
      // pub_path_marker.publish(planning_common::visualization_utils::GetMarker(*path, 0.01));
      ob::PlannerData data(si);
      planner->getPlannerData(data);
      // pub_graph_marker.publish(planning_common::visualization_utils::GetGraph(data, 100, 0.01, 0, 0, 1, 0.3));
    }
    else  
    {
      ROS_INFO_STREAM("No Solution Found");
      ob::PlannerData data(si);
      planner->getPlannerData(data);
      // pub_graph_marker.publish(planning_common::visualization_utils::GetGraph(data, 100, 0.01, 0, 0, 1, 0.3));
    }
    // pub_obstacles_marker_array.publish(obstacle_set.GetMarkerArray(1, 0, 0, 0.7));

    planner->clear();
    pdef->clearSolutionPaths();
    ros::Duration(0.5).sleep();
  }
}
