//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obstacle_avoidance/car_params.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),//angle w.r.t global x- axis (rad)
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  vel_commands_ = std::vector<CommandStamped>(10);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel,
                                uint64_t time) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    odom_stamp_ = time - car_params::sensing_latency;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;

  //std::cout << "omega, vel, loc, angle: " << ang_vel << ", " << robot_vel_[0] << ", " << loc << ", " << angle << std::endl;

  odom_stamp_ = time - car_params::sensing_latency;
  last_odom_stamp_ = odom_stamp_;
  //std::cout << "odom stamp: " << odom_stamp_ << std::endl;
  has_new_odom_ = true;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   uint64_t time) {
  point_cloud_ = cloud;
  point_cloud_stamp_ = time - car_params::sensing_latency;
  has_new_points_= true;
}

//Solve the TOC Problem
void Navigation::TimeOptimalControl(const PathOption& path) {
    double current_speed = odom_state_tf.speed;
    double min_stop_distance = car_params::safe_distance-0.5*current_speed*current_speed/car_params::min_acceleration; //calculate the minimum stopping distance at current velocity
    double set_speed = (path.free_path_length>min_stop_distance)?car_params::max_velocity:0; //decelerate if free path is is smaller than minimum stopping distance otherwise accelerate
    
    //std::cout << "free path: " << path.free_path_length << std::endl;
    //std::cout << "min stop: " << min_stop_distance << std::endl;
    
    // Publish command to topic 
    drive_msg_.header.seq++;
    drive_msg_.header.stamp = ros::Time::now();
    drive_msg_.curvature = path.curvature;
    drive_msg_.velocity = set_speed;

    //std::cout << path.free_path_length << std::endl;

    drive_pub_.publish(drive_msg_);
    //TODO: record the commands used for latency compensation
}

void Navigation::TransformPointCloud(TimeShiftedTF transform){

  Eigen::Matrix2f R;
  R << cos(transform.theta), sin(transform.theta), -sin(transform.theta), cos(transform.theta);

  transformed_point_cloud_.resize(point_cloud_.size());

  for(std::size_t i = 0; i < point_cloud_.size(); i++){
    transformed_point_cloud_[i] =  R*(point_cloud_[i] - transform.position);
  }
}

void Navigation::Run(){
  //This function gets called 20 times a second to form the control loop.
  uint64_t start_loop_time = ros::Time::now().toNSec();
  uint64_t actuation_time = start_loop_time + car_params::actuation_latency;
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  //Latency Compensation
  obstacle_avoidance::CleanVelocityBuffer(vel_commands_, std::min(odom_stamp_, point_cloud_stamp_));

  if(has_new_odom_){
    odom_state_tf.position = Eigen::Vector2f(0, 0);
    odom_state_tf.speed = robot_vel_[0];
    odom_state_tf.theta = 0;
    odom_state_tf.stamp = odom_stamp_;
    has_new_odom_ = false;
  }

  // Find drive cmd directly before odom msg
  int cmd_start_index = std::lower_bound(vel_commands_.begin(), vel_commands_.end(), odom_state_tf.stamp) - vel_commands_.begin() - 1;

  // Integrate odometry into the future across the vector of vehicle commands
  for(std::size_t i = cmd_start_index; i < vel_commands_.size() - 1; i++){
    odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_[i], vel_commands_[i+1].stamp - odom_state_tf.stamp);
  }
  odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_.back(), actuation_time - odom_state_tf.stamp);

  // odom_state_tf is now transform from last odom msg to future actuation time
  
  // get transform from point cloud msg to odom msg
  uint64_t latest_sensor_msg_stamp; // whichever msg is newer
  TimeShiftedTF pc_to_odom_transform; // Empty transform to populate as we integrate between sensor msgs
  pc_to_odom_transform.speed = robot_vel_[0];

  if(point_cloud_stamp_ > odom_stamp_){
    latest_sensor_msg_stamp = point_cloud_stamp_;
    pc_to_odom_transform.stamp = odom_stamp_;
  }
  else{
    latest_sensor_msg_stamp = odom_stamp_;
    pc_to_odom_transform.stamp = point_cloud_stamp_;
  }

  // Integrates from older sensor message to newer message
  int pc_to_odom_index = 0;
  while(vel_commands_[pc_to_odom_index+1].stamp < latest_sensor_msg_stamp){
    pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], vel_commands_[pc_to_odom_index+1].stamp - pc_to_odom_transform.stamp);
    pc_to_odom_index++;
  }
  pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], latest_sensor_msg_stamp - pc_to_odom_transform.stamp);

  // With odom->future_odom and point_cloud->odom, we can find transform for point cloud at future actuation time
  if(point_cloud_stamp_ > odom_stamp_){
    pc_to_odom_transform.theta = odom_state_tf.theta - pc_to_odom_transform.theta;
    pc_to_odom_transform.position = odom_state_tf.position - pc_to_odom_transform.position;
  }
  else{
    pc_to_odom_transform.theta += odom_state_tf.theta;
    pc_to_odom_transform.position += odom_state_tf.position;
  }

  // Transform point cloud into future actuation time, storing as transformed_point_cloud_
  TransformPointCloud(pc_to_odom_transform);

  // Visualize Latency Compensation
  //obstacle_avoidance::LatencyPointCloud(local_viz_msg_, transformed_point_cloud_);
  //obstacle_avoidance::DrawCarLocal(local_viz_msg_, odom_state_tf.position, odom_state_tf.theta);

  // "Carrot on a stick" goal point, and resulting goal curvature
  Eigen::Vector2f goal_point(4, 0.0);
  float goal_curvature = obstacle_avoidance::GetCurvatureFromGoalPoint(goal_point);
  goal_curvature = Clamp(goal_curvature, car_params::min_curvature, car_params::max_curvature);

  // 4) Generate range of possible paths centered on goal_curvature, using std::vector<struct PathOption>
  static std::vector<struct PathOption> path_options(car_params::num_curves);

  // 5) For possible paths and point_cloud:
  for(std::size_t curve_index = 0; curve_index < path_options.size(); curve_index++){
    float curvature = obstacle_avoidance::GetCurvatureOptionFromRange(curve_index, goal_curvature, car_params::min_curvature, car_params::curvature_increment);
    
    // Initialize path_option and collision bounds for curvature
    obstacle_avoidance::PathBoundaries collision_bounds(abs(curvature));
    path_options[curve_index] = navigation::PathOption{
      curvature,                    // curvature
      10,                           // default clearance
      car_params::max_path_length,  // free Path Length
      Eigen::Vector2f(0, 0),        // obstacle point
      Eigen::Vector2f(0, 0)};       // closest point

    obstacle_avoidance::EvaluatePathWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);
    obstacle_avoidance::LimitFreePath(path_options[curve_index], goal_point);
    obstacle_avoidance::EvaluateClearanceWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);

    // Visualization test code
    // visualization::DrawPathOption(path_options[curve_index].curvature, path_options[curve_index].free_path_length, path_options[curve_index].clearance, local_viz_msg_);
    // visualization::DrawCross(path_options[curve_index].obstruction, 0.1,  0x0046FF, local_viz_msg_);
  }

  // 6) Select best path from scoring function
  struct PathOption best_path = obstacle_avoidance::ChooseBestPath(path_options,goal_point);
  //obstacle_avoidance::VisualizeObstacleAvoidanceInfo(goal_point,path_options,best_path,local_viz_msg_);
  
  // 7) Publish commands with 1-D TOC, update vector of previous vehicle commands
  TimeOptimalControl(best_path);

    // static double start_timer;
    // if(first_cycle){
    //   first_cycle = false;
    //   start_timer = GetMonotonicTime();
    //   std::cout << "Start Time: " << start_timer << std::endl;
    //   std::cout << "Start Time + 2: " << start_timer + 2<< std::endl;
    // }

    // // Remove for obstacle avoidance
    // std::cout << "Curr Time: " << GetMonotonicTime() << std::endl;
    // if(GetMonotonicTime() < start_timer + 1.0){
    // drive_msg_.curvature = 0.0;
    // drive_msg_.velocity = 1.0;
    // }
    // else{
    //   drive_msg_.curvature = 0.0;
    //   drive_msg_.velocity = 0.0;
    // }

    drive_pub_.publish(drive_msg_);
    // Remove for obstacle avoidance
    
  CommandStamped drive_cmd(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toNSec() + car_params::actuation_latency);
  vel_commands_.push_back(drive_cmd);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
