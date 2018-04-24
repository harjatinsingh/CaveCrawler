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
    // private_nh_.param("max_dist_between_pathframes", max_dist_between_pathframes_, 0.10);
    //   private_nh_.param("max_footprint_cost", max_footprint_cost_, 256);
    //   private_nh_.param("relative_validity_check_resolution", relative_validity_check_resolution_, 0.05);
    //   private_nh_.param("interpolate_path", interpolate_path_, true);
    //   private_nh_.param("publish_diagnostics", publish_diagnostics_, false);
    //   private_nh_.param("Inscribed_radius", inscribed_radius_, 0.2);				// 6/27/2014
    //   private_nh_.param("Circumscribed_radius", circumscribed_radius_, 0.25);		// 6/27/2014

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

bool OmplGlobalPlanner::isStateValid(const ob::State *state)
{
    bool inBound = _space->satisfiesBounds(state);
    if (!inBound)
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

	void OmplGlobalPlanner::PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D)
	{
		// use tf-pkg to convert angles
		tf::Pose pose_tf;
		
		// convert geometry_msgs::PoseStamped to tf::Pose
		tf::poseMsgToTF(pose, pose_tf);
		
		// now get Euler-Angles from pose_tf
		double useless_pitch, useless_roll, yaw;
		pose_tf.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
		// normalize angle
		yaw = angles::normalize_angle(yaw);
		
		// and set to pose2D
		pose2D.x = pose.position.x;
		pose2D.y = pose.position.y;
		pose2D.theta = yaw;
		
		return;
	}

bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{

	// ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
	// ROS_INFO_STREAM("start and goal is "<<start.pose.position.x<<" "<<start.pose.position.y<<" "<<goal.pose.position.x<<" "<<goal.pose.position.y);
	plan.clear();
	ros::NodeHandle n();
	std::string global_frame = _frame_id;

	costmap_2d::Costmap2D* costmap_ =  _costmap_ros->getCostmap(); 

	tf::Stamped<tf::Pose> goal_tf;
	tf::Stamped<tf::Pose> start_tf;

	poseStampedMsgToTF(goal,goal_tf);
	poseStampedMsgToTF(start,start_tf);


	// get bounds from worldmap and set it to bounds for the planner
	// as goal and map are set in same frame (checked above) we can directly
	// get the extensions of the statespace from the map-prms		
	ob::RealVectorBounds bounds(2);
	double map_upperbound, map_lowerbound;

	//CHECK THIS IN THE FINAL START GOAL CONFIG. MIGHT HAVE TO CHANGE THE CONFIG STUFF
	// get bounds for x coordinate
	map_upperbound = costmap_->getSizeInMetersX() - costmap_->getOriginX();	// costmap_->getOriginX();
	map_lowerbound = costmap_->getOriginX();											// map_upperbound - costmap_->getSizeInMetersX();
	bounds.setHigh(0, map_upperbound);
	bounds.setLow(0, map_lowerbound);
	// ROS_WARN("Setting upper bound and lower bound of map x-coordinate to (%f, %f).", map_upperbound, map_lowerbound);

	// get bounds for y coordinate
	map_upperbound = costmap_->getSizeInMetersY() - costmap_->getOriginY();
	map_lowerbound = costmap_->getOriginY();
	bounds.setHigh(1, map_upperbound);
	bounds.setLow(1, map_lowerbound);
	// ROS_WARN("Setting upper bound and lower bound of map y-coordinate to (%f, %f).", map_upperbound, map_lowerbound);

	// now set it to the planner
	_space->as<ob::SE2StateSpace>()->setBounds(bounds);
	// ROS_INFO_STREAM("SizeInMetersX "<<costmap_->getSizeInMetersX()<<" SizeInMetersY "<<costmap_->getSizeInMetersY());
	// ROS_INFO_STREAM("OriginX "<<costmap_->getOriginX()<<" OriginY() "<<costmap_->getOriginY());											// map_upperbound - costmap_->getSizeInMetersY();		
	// now create instance to ompl setup
	og::SimpleSetup ss(_space);
	// get SpaceInformationPointer from simple_setup (initialized in makePlan routine)
	ob::SpaceInformationPtr si = ss.getSpaceInformation();
 	// ob::SpaceInformationPtr si(new ob::SpaceInformation(_space));
	// set state validity checker
	si->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid,this,_1));
	// // set validity checking resolution
	si->setStateValidityCheckingResolution(0.05); // dimensionless number as a fraction of the workspace size
  	// si->setup();
    double yaw, pitch, roll;
    tf::Pose pose_tf;

    // Define problem:
    ob::ScopedState<> ompl_start(_space);
    ompl_start[0] = start.pose.position.x;
    ompl_start[1] = start.pose.position.y;

    tf::poseMsgToTF(start.pose, pose_tf);
    pose_tf.getBasis().getEulerYPR(yaw, pitch, roll);
    
    ompl_start[2] = yaw;
	// check whether this satisfies the bounds of the manifold
	bool inBound = _space->satisfiesBounds(ompl_start->as<ob::SE2StateSpace::StateType>());
	if(!inBound)
	{
		ROS_ERROR("Start Pose lies outside the bounds of the map - Aborting Planner ");
		return false;
	}

    ob::ScopedState<> ompl_goal(_space);
    ompl_goal[0] = goal.pose.position.x;
    ompl_goal[1] = goal.pose.position.y;

    tf::poseMsgToTF(goal.pose, pose_tf);
    pose_tf.getBasis().getEulerYPR(yaw, pitch, roll);

    ompl_goal[2] = yaw;
	// before starting planner -> check whether target configuration is free
	// Goal position check
	int sample_costs =  _costmap_model->footprintCost(goal.pose.position.x, goal.pose.position.y, yaw,_costmap_ros->getRobotFootprint());

	if ((sample_costs < 0.0) || (sample_costs > max_footprint_cost))
	{
		ROS_ERROR("Collision on target: Planning aborted! Change target position.");
		return false;
	}
	
	// check whether this satisfies the bounds of the manifold
	inBound = _space->satisfiesBounds(ompl_goal->as<ob::SE2StateSpace::StateType>());
	if(!inBound)
	{
		ROS_ERROR("Target Pose lies outside the bounds of the map - Aborting Planner ");
		return false;
	}
	
	// set start and goal state to planner
	ss.setStartAndGoalStates(ompl_start, ompl_goal);
	ROS_INFO_STREAM("going to setup problem");
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
