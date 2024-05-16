#ifndef SIMPLE_MPC_LOCAL_PLANNER_H
#define SIMPLE_MPC_LOCAL_PLANNER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <simple_mpc_local_planner/SimpleMPCLocalPlannerConfig.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <simple_mpc_local_planner/dwa_planner.h>

namespace simple_mpc_local_planner {
  /**
   * @class SimpleMPCLocalPlanner
   * @brief SimpleMPC implements BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class SimpleMPCLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for SimpleMPCLocalPlanner
       */
      SimpleMPCLocalPlanner();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~SimpleMPCLocalPlanner();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }

    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(SimpleMPCLocalPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      //My methods
      bool initMyParams(ros::NodeHandle nh);

      /**
       * @brief check if point is inside a corridor
       */
      bool isCorridorInRange(geometry_msgs::Polygon corridor);

      static bool isPointInCorridor(geometry_msgs::Point point, geometry_msgs::Polygon corridor);

      static bool isPathInCorridor(nav_msgs::Path path, geometry_msgs::Polygon corridor, double resolution, double meter_step=1.0);
      
      static bool isPathInCorridor(const std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::Polygon corridor, double resolution, double meter_step=1.0);

      static geometry_msgs::Polygon inflateCorridor(geometry_msgs::Polygon corridor, double amount);

      static geometry_msgs::Point getCorridorCentroid(geometry_msgs::Polygon corridor);
      //

      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<SimpleMPCLocalPlannerConfig> *dsrv_;
      simple_mpc_local_planner::SimpleMPCLocalPlannerConfig default_config_;
      bool setup_;
      geometry_msgs::PoseStamped current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;

      // -- My params --
      int priority_; //Low value equals higher priority
      int consecutive_points_dist_;
      int current_corridor_idx_ = -1;
      bool successful_initialization_ = false;
      bool exited_corridor_ = false;
      bool stop_ = false;
      bool entered_corridor_ = false;
      double corridor_inflation_amount_;
      double global_costmap_resolution_;
      ros::Publisher corridor_pub_; //visualization purpose
      ros::Publisher inflated_corridor_pub_; //visualization purpose
      ros::Publisher inflated_corridor2_pub_; //visualization purpose
      std::string tf_prefix_;
      geometry_msgs::Polygon centroid_square_;
      std::vector<ros::Subscriber> subscribers_;
      std::vector<geometry_msgs::Polygon> corridors_; 
      //std::vector<geometry_msgs::Point> inflated_corridor_vertices_;
      nav_msgs::Path global_plan_;
      geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
      std::vector<geometry_msgs::PoseStamped> my_local_plan_;
      // --           --
      
      bool initialized_;

      
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
  };
};
#endif