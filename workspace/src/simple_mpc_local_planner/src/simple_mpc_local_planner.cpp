#include <pluginlib/class_list_macros.h>
#include "simple_mpc_local_planner/simple_mpc_local_planner.h"

PLUGINLIB_EXPORT_CLASS(simple_mpc_local_planner::SimpleMPCLocalPlanner, nav_core::BaseLocalPlanner)

namespace simple_mpc_local_planner{

   SimpleMPCLocalPlanner::SimpleMPCLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

   SimpleMPCLocalPlanner::SimpleMPCLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros): costmap_ros_(NULL), tf_(NULL), initialized_(false)
   {
      initialize(name, tf, costmap_ros);
   }

   SimpleMPCLocalPlanner::~SimpleMPCLocalPlanner() {}

   // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
   void SimpleMPCLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                 costmap_2d::Costmap2DROS* costmap_ros)
   {
      if(!initialized_)
      {
         tf_ = tf;
         costmap_ros_ = costmap_ros;
         initialized_ = true;
      }
   }

   bool SimpleMPCLocalPlanner::setPlan(
      const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
   )
   {
      if(!initialized_)
      {
         ROS_ERROR("This planner has not been initialized");
         return false;
      }
      return true;
   }

   bool SimpleMPCLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
   {
      if(!initialized_)
      {
         ROS_ERROR("This planner has not been initialized");
         return false;
      }
      return true;
   }

   bool SimpleMPCLocalPlanner::isGoalReached()
   {
      if(!initialized_)
      {
         ROS_ERROR("This planner has not been initialized");
         return false;
      }
      return false;
   }
}