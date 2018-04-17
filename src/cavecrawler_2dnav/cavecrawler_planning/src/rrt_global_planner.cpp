#include <pluginlib/class_list_macros.h>
#include <rrt_global_planner.h>
#include <tf/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(ompl_global_planner::OmplGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace ompl_global_planner {

OmplGlobalPlanner::OmplGlobalPlanner () : _costmap_ros(NULL), _initialized(false), _allow_unknown(true),_space(new ob::SE2StateSpace),_costmap_model(NULL)
{
	ROS_INFO_STREAM("constructor 1");
}

OmplGlobalPlanner::OmplGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	ROS_INFO_STREAM("constructor 2");
    initialize(name, costmap_ros);
}


void OmplGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
   if(!_initialized)
   {
        ros::NodeHandle private_nh("~/" + name);
        _costmap_ros = costmap_ros;
        _frame_id = "world";
        max_footprint_cost = 256;
        _costmap_model = new base_local_planner::CostmapModel(*_costmap_ros->getCostmap());

        // _plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
        // private_nh.param("global_frame",  _frame_id, world);
        private_nh.param("allow_unknown", _allow_unknown, true);
		// private_nh.param("max_footprint_cost", max_footprint_cost, 256);
        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        _initialized = true;
        ROS_INFO_STREAM("Ompl global planner initialized!");
    }
    else
    {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

bool OmplGlobalPlanner::isStateValid(const oc::SpaceInformation *si,const ob::State *state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    double x,y,theta,cost;
    // Get the values:
    x = state->as<ob::SE2StateSpace::StateType>()->getX();
    y = state->as<ob::SE2StateSpace::StateType>()->getY();
    theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

	cost = _costmap_model->footprintCost(x, y, theta, _costmap_ros->getRobotFootprint());

	if( (cost >= 0) && (cost < max_footprint_cost) )
	{
		return true;
	}
	return false;
}


bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

 //  plan.clear();
 //  ros::NodeHandle n();
 //  std::string global_frame = _frame_id;
 //  ROS_INFO_STREAM("Main");
 //  // ompl::msg::noOutputHandler();
 //  // Create R3 space
 //  // ob::RealVectorBounds bounds(3);  // We are going to create a unit box from (-1,-1,-1) to (1,1,1)
 //  // bounds.setLow(0, -1.0);
 //  // bounds.setLow(1, -1.0);
 //  // bounds.setLow(2, -1.0);
 //  // bounds.setHigh(0, 1.0);
 //  // bounds.setHigh(1, 1.0);
 //  // bounds.setHigh(2, 1.0);
 //       ob::RealVectorBounds bounds(2);
 //      bounds.setLow(-1);
 //      bounds.setHigh(1);
 //  _space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
 //  ob::SpaceInformationPtr si(new ob::SpaceInformation(_space));



 //  // Set start and goal
 //  ob::ScopedState<ob::SE2StateSpace> robot_start(_space);//, // Planning from (0,0,0) to (0.8, 0.8, 0.8)
 //   robot_start->setX(-0.5);
 //  robot_start->setY(0.0);
 //  robot_start->setYaw(0.0);
 //  ob::ScopedState<ob::SE2StateSpace> robot_goal(_space);//, // Planning from (0,0,0) to (0.8, 0.8, 0.8)
 //   robot_goal->setX(0.5);
 //  robot_goal->setY(1.0);
 //  robot_start->setYaw(0.0);

 // si->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid,this,_1));
 //  //si->setStateValidityChecker([&si](const ob::State *state) { return isStateValid(si.get(), state); });
 //  si->setStateValidityCheckingResolution(0.001); // dimensionless number as a fraction of the workspace size
 //  si->setup();

 //  // Setup problem
 //  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
 //  pdef->setStartAndGoalStates(robot_start.get(), robot_goal.get());
 //  pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

 //  // Setup planner
 //  // boost::shared_ptr<ompl::geometric::RRTstar> rrtstar(new ompl::geometric::RRTstar(si));
 //  // ob::PlannerPtr planner = rrtstar;

 //  auto planner(std::make_shared<og::RRTConnect>(si));

 //    planner->setProblemDefinition(pdef);
 //    planner->setup();
 //    ob::PlannerStatus solved = planner->ob::Planner::solve(0.0);

 //    // if(solved == ob::PlannerStatus::EXACT_SOLUTION) 
 //    if (solved)
 //    {
 //      // boost::shared_ptr<og::PathGeometric> path = boost::static_pointer_cast<og::PathGeometric>(planner->getProblemDefinition()->getSolutionPath()); 
 //      ob::PathPtr path = pdef->getSolutionPath();
 //      // ob::Cost cost = planner->getProblemDefinition()->getOptimizationObjective()->pathCost(path.get());
 //      ROS_INFO_STREAM("Found solution haha");// with cost: "<<cost.value());
 //      // pub_path_marker.publish(planning_common::visualization_utils::GetMarker(*path, 0.01));
 //      ob::PlannerData data(si);
 //      planner->getPlannerData(data);
 //      // pub_graph_marker.publish(planning_common::visualization_utils::GetGraph(data, 100, 0.01, 0, 0, 1, 0.3));
 //    }
 //    else  
 //    {
 //      ROS_INFO_STREAM("No Solution Found");
 //      ob::PlannerData data(si);
 //      planner->getPlannerData(data);
 //      // pub_graph_marker.publish(planning_common::visualization_utils::GetGraph(data, 100, 0.01, 0, 0, 1, 0.3));
 //    }
 //    // pub_obstacles_marker_array.publish(obstacle_set.GetMarkerArray(1, 0, 0, 0.7));

 //    planner->clear();
 //    pdef->clearSolutionPaths();
 //    ros::Duration(0.5).sleep();
  return true;

}
};
