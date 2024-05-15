#include <simple_mpc_local_planner/simple_mpc_local_planner.h>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Point.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_mpc_local_planner::SimpleMPCLocalPlanner, nav_core::BaseLocalPlanner)

namespace simple_mpc_local_planner {

  void SimpleMPCLocalPlanner::reconfigureCB(SimpleMPCLocalPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  SimpleMPCLocalPlanner::SimpleMPCLocalPlanner() : initialized_(false), odom_helper_("odom"), setup_(false) {
  }

  bool SimpleMPCLocalPlanner::initMyParams(ros::NodeHandle nh) {
    //tf_prefix
    tf_prefix_ = nh.param("tf_prefix", std::string("/"));

    //global costmap resolution
    std::string resolution_str;
    nh.searchParam("global_costmap/resolution", resolution_str);
    global_costmap_resolution_ = nh.param(resolution_str, 0.05);

    //priority
    priority_ = nh.param("priority_level", 0);

    //inflation amount
    corridor_inflation_amount_ = nh.param("corridor_inflation_amount", 1.0);

    //distance from two consecutive points in global plans
    consecutive_points_dist_ = nh.param("consecutive_points_dist", 1.0);

    //corridor vertices
    int corridor_num = 1;
    XmlRpc::XmlRpcValue vertices_vec;
    while(nh.getParam("corridor" + std::to_string(corridor_num), vertices_vec)) {
      std::vector<geometry_msgs::Point> corridor;
      for (int i=0; i<vertices_vec.size(); i++) {
        double x, y;
        geometry_msgs::Point vertex;
        try {
          vertex.x = vertices_vec[i][0];
          vertex.y = vertices_vec[i][1];
        } catch (...) {
          ROS_ERROR("Unable to fetch vertex.");
          return false;
        }
        corridor.push_back(vertex);
      }

      corridors_.push_back(corridor);
      corridor_num++;
    }

    ROS_INFO("Found %d corridors", corridor_num);

    if (priority_ == 0) return true; //Run as DWA

    //global plan topic
    XmlRpc::XmlRpcValue global_plan_topic;
    XmlRpc::XmlRpcValue odom_topic;
    if (!nh.getParam(std::string("robot1/global_plan_topic"), global_plan_topic) && global_plan_topic.getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR("Couldn't get global plan topic.");
      return false;
    }

    if (!nh.getParam(std::string("robot1/amcl_topic"), odom_topic) && odom_topic.getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR("Couldn't get amcl pose topic.");
      return false;
    }

    //Set odom callback and subscriber
    const std::string s_odom = static_cast<std::string>(odom_topic);
    boost::function<void (const geometry_msgs::PoseWithCovarianceStamped&)> odom_cb = [this](const geometry_msgs::PoseWithCovarianceStamped& odom){
      amcl_pose_ = odom;
    };
    subscribers_.push_back(nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(s_odom, 1, odom_cb));

    //Set global plan callback and subscriber
    const std::string s_plan = static_cast<std::string>(global_plan_topic);
    boost::function<void (const nav_msgs::Path&)> plan_cb = [this](const nav_msgs::Path& plan) {
      global_plan_ = plan;
    };
    subscribers_.push_back(nh.subscribe<nav_msgs::Path>(s_plan, 1, plan_cb));


    return true;
  }

  void SimpleMPCLocalPlanner::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }

      // Initialize my parameters
      
      successful_initialization_ = initMyParams(private_nh);
      if (!successful_initialization_) ROS_ERROR("Initialization failed, local planner will run as DWA Local Planner");
      else ROS_INFO("SimpleMPCLocalPlanner initialized successfully!");
      
      initialized_ = true;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<SimpleMPCLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<SimpleMPCLocalPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  //Raytrace algo from the internet to check if point is inside the corridor
  bool SimpleMPCLocalPlanner::isPointInCorridor(geometry_msgs::Point point, std::vector<geometry_msgs::Point> corridor) {
    bool inside = false;
    geometry_msgs::Point p1 = corridor[0], p2;

    for (int i=1; i<=corridor.size(); i++) {
     
      p2 = corridor[i%corridor.size()];

      //Point.y inside edge.y and point.x to the left of edge.x 
      if (point.y > std::min(p1.y, p2.y) && point.y <= std::max(p1.y, p2.y) && point.x <= std::max(p1.x, p2.x)) {
        double x_intersection = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
        if (p1.x == p2.x || point.x <= x_intersection) inside = !inside;
      }
      
      p1 = p2;
    }

    return inside;
  }
  
  //Check if local plan furherst point is inside corridor
  bool SimpleMPCLocalPlanner::isCorridorInRange(std::vector<geometry_msgs::Point> corridor) {
    if (my_local_plan_.size() == 0) return false;

    geometry_msgs::TransformStamped odom_to_map;
    geometry_msgs::Pose target_pose;

    odom_to_map = (*tf_).lookupTransform("map", tf_prefix_ + "/odom", ros::Time(0), ros::Duration(3.0));
    
    tf2::doTransform(my_local_plan_[my_local_plan_.size()-1].pose, target_pose, odom_to_map);
    if (SimpleMPCLocalPlanner::isPointInCorridor(target_pose.position, corridor)) return true;
    
    return false;
  }

  bool SimpleMPCLocalPlanner::isPathInCorridor(nav_msgs::Path path, std::vector<geometry_msgs::Point> corridor, double resolution, double meter_step) {
    if (meter_step < 1) meter_step = 1;

    for (int i=0; i<path.poses.size(); i+=std::max(1, int(meter_step/resolution))) {
      if (SimpleMPCLocalPlanner::isPointInCorridor(path.poses[i].pose.position, corridor)) return true;
    }

    return false;
  }

  bool SimpleMPCLocalPlanner::isPathInCorridor(const std::vector<geometry_msgs::PoseStamped>& path, std::vector<geometry_msgs::Point> corridor, double resolution, double meter_step) {
    if (meter_step < 1) meter_step = 1;

    for (int i=0; i<path.size(); i+=std::max(1, int(meter_step/resolution))) {
      if (SimpleMPCLocalPlanner::isPointInCorridor(path[i].pose.position, corridor)) return true;
    }

    return false;    
  }

  geometry_msgs::Point SimpleMPCLocalPlanner::getCorridorCentroid(std::vector<geometry_msgs::Point> corridor) {
    geometry_msgs::Point centroid;
    
    if (corridor.size()==0) return centroid;

    for (geometry_msgs::Point p : corridor) {
      centroid.x += p.x;
      centroid.y += p.y;
    }

    centroid.x /= corridor.size();
    centroid.y /= corridor.size();

    return centroid;
  }

  std::vector<geometry_msgs::Point> SimpleMPCLocalPlanner::inflateCorridor(std::vector<geometry_msgs::Point> corridor, double amount) {
    std::vector<geometry_msgs::Point> inflated_corridor;
    
    for (geometry_msgs::Point vertex : corridor) {
      double angle = std::atan2(vertex.y, vertex.x);
      
      geometry_msgs::Point new_point;
      new_point.x = vertex.x + amount * std::cos(angle);
      new_point.y = vertex.y + amount * std::sin(angle);

      inflated_corridor.push_back(new_point);
    }

    return inflated_corridor;
  }

  bool SimpleMPCLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    //My code
    //Find corridor idx of corridor which global plan traverses
    current_corridor_idx_ = -1;
    for (int i = 0; i < corridors_.size(); i++) {
      if (SimpleMPCLocalPlanner::isPathInCorridor(orig_global_plan, corridors_[i], global_costmap_resolution_, consecutive_points_dist_)) current_corridor_idx_ = i;
    }

    //Build center of corridor
    if (current_corridor_idx_ != -1) {
      double radius = 1.0;
      geometry_msgs::Point centroid = SimpleMPCLocalPlanner::getCorridorCentroid(corridors_[current_corridor_idx_]);
      for (int i=0; i<4; i++) {
        geometry_msgs::Point p;
        p.x = centroid.x + radius*(i<2?1:-1);
        p.y = centroid.y + radius*(i==0||i==3?-1:1);

        centroid_square_.push_back(p);
      }
    }

    //

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool SimpleMPCLocalPlanner::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void SimpleMPCLocalPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void SimpleMPCLocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  SimpleMPCLocalPlanner::~SimpleMPCLocalPlanner(){
    //make sure to clean things up
    delete dsrv_;
  }

  bool SimpleMPCLocalPlanner::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    
    // call with updated footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("simple_mpc_local_planner", "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      my_local_plan_ = local_plan;
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("simple_mpc_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    //publish information to the visualizer
    my_local_plan_ = local_plan;
    publishLocalPlan(local_plan);
    return true;
  }

  bool SimpleMPCLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    //My code
    if (current_corridor_idx_ != -1) { //my path passes through a corridor if such value is not -1

      std::vector<geometry_msgs::Point> corridor = corridors_[current_corridor_idx_];
      std::vector<geometry_msgs::Point> inflated_corridor = inflateCorridor(corridor, corridor_inflation_amount_);
      std::vector<geometry_msgs::Point> inflated_corridor2 = inflateCorridor(corridor, corridor_inflation_amount_+2.0);
      
      //Check if any other path passes through corridor
      bool is_in_corridor = SimpleMPCLocalPlanner::isPathInCorridor(global_plan_, corridor, global_costmap_resolution_, consecutive_points_dist_);

      if (is_in_corridor && isCorridorInRange(inflated_corridor) && priority_ > 0) {
        ROS_WARN_ONCE("Corridor in range while other robot plans on traversing it: stop and wait");
        stop_ = true;
      }

      if (stop_) {
        //Check if other robot passed through center of corridor
        if (isPointInCorridor(amcl_pose_.pose.pose.position, centroid_square_)) {
          entered_corridor_ = true;
          ROS_INFO_ONCE("Other robot has reached center of corridor");
        }
        
        //Check if other robot exited corridor
        exited_corridor_ = entered_corridor_ && !isPointInCorridor(amcl_pose_.pose.pose.position, inflated_corridor2);

        //ROS_WARN("%f, %f, %d", amcl_pose_.pose.pose.position.x, amcl_pose_.pose.pose.position.y, isPointInCorridor(amcl_pose_.pose.pose.position, inflated_corridor2));

        if (exited_corridor_) {
          ROS_INFO_ONCE("Other robot has exited corridor: continue");
          
          //Reset path
          nav_msgs::Path path;
          global_plan_ = path;

          //Reset entered and exited corridor
          entered_corridor_ = false;
          exited_corridor_ = false;
          stop_ = false;
        }
        
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishLocalPlan(empty_plan);

        return true;
      }
    }
    //

    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("simple_mpc_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("simple_mpc_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      my_local_plan_ = local_plan;
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          [this](auto pos, auto vel, auto vel_samples){ return dp_->checkTrajectory(pos, vel, vel_samples); });
    } else {
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);


      if (!isOk) {
        //Try to clear costmaps and recompute
        costmap_ros_->resetLayers(); //My code

        ROS_WARN_NAMED("simple_mpc_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        transformed_plan = empty_plan;
      }
      
      publishGlobalPlan(transformed_plan);
      
      return isOk;
    }
  }


};