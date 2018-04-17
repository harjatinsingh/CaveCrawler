/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#ifndef RRT_GLOBAL_PLANNER_CPP
#define RRT_GLOBAL_PLANNER_CPP

#include <ompl/geometric/PathGeometric.h>
#include <visualization_msgs/Marker.h>

using std::string;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_global_planner {

class OmplGlobalPlanner : public nav_core::BaseGlobalPlanner 
{

	public:

	OmplGlobalPlanner();
	OmplGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool isStateValid(const ob::State *state);
	bool makePlan(const geometry_msgs::PoseStamped& start,
	            const geometry_msgs::PoseStamped& goal,
	            std::vector<geometry_msgs::PoseStamped>& plan
	           );
	

    private:

    costmap_2d::Costmap2DROS* _costmap_ros;
    std::string _frame_id;
    ros::Publisher _plan_pub;
    bool _initialized;
    bool _allow_unknown;

    std::string tf_prefix_;
    // boost::mutex _mutex;
    base_local_planner::CostmapModel* _costmap_model;

    // State spaces:
    // ob::StateSpacePtr _se2_space;
    // ob::StateSpacePtr _velocity_space;
    ob::StateSpacePtr _space;

};
};
#endif