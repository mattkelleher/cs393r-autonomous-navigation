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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obstacle_avoidance/car_params.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
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
    nav_complete_(false),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    map_file_(map_file) {
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
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = false; 
  std::cout << "New Nav Goal: (" << nav_goal_loc_.x() << ", " << nav_goal_loc_.y() << ")" << std::endl; 
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
   if(dist_point_to_point(nav_goal_loc_, robot_loc_) < 0.15) {
     std::cout << "***************Goal Reached!***************" << std::endl;
     nav_complete_ = true;
   }   
   if(nav_complete_) {
     return; 
   }
  
  //This function gets called 20 times a second to form the control loop.
  uint64_t start_loop_time = ros::Time::now().toNSec();
  uint64_t actuation_time = start_loop_time + car_params::actuation_latency;
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_); 
  //visualization::ClearVisualizationMsg(global_viz_msg_); TODO undo

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
  Vector2f goal_point = get_local_goal();
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
//##########################################################################################

float Navigation::dist_point_to_line(float point_x, float point_y, Vector2f map_line_point1, Vector2f map_line_point2){
  float a;  //  line is defined as ax+by+c=0
  float b;
  float c;

  a = map_line_point1.y() - map_line_point2.y();
  b = map_line_point2.x() - map_line_point1.x();
  c = map_line_point1.x() * map_line_point2.y() - map_line_point2.x() * map_line_point1.y();

  return abs(a * point_x + b * point_y + c) / sqrt(pow(a,2) + pow(b,2));
}

float Navigation::dist_point_to_point(Vector2f p1, Vector2f p2) {
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
 }

void Navigation::make_graph(){
  std::cout << "Entering make graph" << std::endl;
  vector<Vector2f> sample_points;
  map_.Load(map_file_);  
  float xmin, xmax, ymin, ymax;
  xmin = -44.0;
  xmax = 11.8;
  ymin = -32.75;
  ymax = -8.5;
  for(int i = 0; i < 3 * 1400; i++) { //drop points in GDC1 "lower" x = [-44, 11.8] y=[-32.75, -8.5]
    Vector2f sample_point;
    sample_point.x() = rng_.UniformRandom(xmin, xmax);
    sample_point.y() = rng_.UniformRandom(ymin, ymax);
    sample_points.push_back(sample_point); 
  }
  xmin = -42.33;
  xmax = 44.26;
  ymin = 2.70;
  ymax = 22.12;
  for(int i = 0; i < 3 * 1740; i++) { //drop points in GDC1 "upper main"
    Vector2f sample_point;
    sample_point.x() = rng_.UniformRandom(xmin, xmax);
    sample_point.y() = rng_.UniformRandom(ymin, ymax);
    sample_points.push_back(sample_point); 
  }
  xmin = 6.40;
  xmax = 26.80;
  ymin = 22.12;
  ymax = 34.30;
  for(int i = 0; i < 3 * 256; i++) { //drop points in GDC1 "upper top"
    Vector2f sample_point;
    sample_point.x() = rng_.UniformRandom(xmin, xmax);
    sample_point.y() = rng_.UniformRandom(ymin, ymax);
    sample_points.push_back(sample_point); 
  }
  std::cout << "Inital sampling complete" << std::endl;
  vector<Vector2f> sample_points_filtered;
  for (auto sample_point : sample_points){
    bool point_valid = true;
    for (size_t j = 0; j < map_.lines.size(); j++){
      Vector2f m_point1;
      Vector2f m_point2;
      
      m_point1.x() = map_.lines[j].p0.x();
      m_point1.y() = map_.lines[j].p0.y();
      m_point2.x() = map_.lines[j].p1.x();
      m_point2.y() = map_.lines[j].p1.y();

      if (dist_point_to_line(sample_point.x(), sample_point.y(), m_point1, m_point2) < 0.2){ //TODO 0.3 
  //      point_valid = false; TODO undo? this will simply give us all sampled points
        break;
      }
    }
    if(point_valid) {
      sample_points_filtered.push_back(sample_point);
    }  
  }
  std::cout << "Vertice filtering complete" << std::endl;
  vector<Vector2i> edges;
  for(size_t i = 0; i < sample_points_filtered.size(); i++) {
    for(size_t j = i + 1; j < sample_points_filtered.size(); j++) {
      if(0 < dist_point_to_point(sample_points_filtered[i], sample_points_filtered[j]) && dist_point_to_point(sample_points_filtered[i], sample_points_filtered[j]) < 2){ //TODO 0 and 0.1
        edges.push_back(Vector2i(int(i), int(j)));
      }
    } 
  }
  std::cout << "Edge indentifying complete" << std::endl;
  vector<Vector2i> edges_filtered;
  for (size_t i = 0; i<edges.size(); i++){
    bool edge_valid = true;
    Vector2i edge = edges[i];
    line2f edge_line(sample_points_filtered[edge.x()], sample_points_filtered[edge.y()]); 
    for (size_t j = 0; j < map_.lines.size(); j++){ //TODO make map_ part of navigation class and initalize
      const line2f map_line = map_.lines[j];
      Vector2f intrsctn_pt;
      if(map_line.Intersection(edge_line, &intrsctn_pt)){
        edge_valid = false;
        break;       
      }
    }
    if(edge_valid) {
      edges_filtered.push_back(edge);
    }
  }
  std::cout<< "Edge filtering complete" << std::endl;
  // Write vertices to a file
  FILE* vertex_fid = fopen("vertices.txt", "w");
  if(vertex_fid == NULL) {
    fprintf(stderr, "ERROR: Unable to open vertices.txt");
    exit(1);
  }
  for(size_t i = 0; i < sample_points_filtered.size(); i++) {
    float point_x = sample_points_filtered[i].x();
    float point_y = sample_points_filtered[i].y();
    fprintf(vertex_fid, "%f, %f \n", point_x, point_y);
  }
  fclose(vertex_fid);

  // Write edges to a file
  FILE* edge_fid = fopen("edges.txt", "w");
  if(edge_fid == NULL) {
    fprintf(stderr, "ERROR: Unable to open edge.txt");
    exit(1);
  }
  for(size_t i = 0; i < edges_filtered.size(); i++) {
    fprintf(vertex_fid, "%i, %i \n", edges_filtered[i].x(), edges_filtered[i].y());
  }
  fclose(edge_fid);
}

void Navigation::load_graph(){
  
  FILE* vertex_fid = fopen("vertices.txt", "r");
  if(vertex_fid == NULL) {
    fprintf(stderr, "ERROR: Unable to open vertices.txt");
    exit(1);
  }
  v_.clear();
  neighbors_.clear();
  float x(0), y(0);
  while(fscanf(vertex_fid, "%f, %f \n", &x, &y) == 2) {
    v_.push_back(Vector2f(x,y));
    neighbors_.push_back({});
    visualization::DrawPoint(Vector2f(x,y), 0x1644db, global_viz_msg_);
    visited_.push_back(0);
  }
  fclose(vertex_fid);

  
  FILE* edge_fid = fopen("edges.txt", "r");
  if(edge_fid == NULL) {
    fprintf(stderr, "ERROR: Unable to open edge.txt");
    exit(1);
  }
  int p0(0), p1(0);
  while(fscanf(edge_fid, "%i, %i \n", &p0, &p1) == 2) {
    neighbors_[p0].push_back(p1); 
    neighbors_[p1].push_back(p0);
  }
}

void Navigation::reset_graph(){
  for(size_t i = 0; i < visited_.size(); i++) {
    visited_[i] = 0;
  }
}

Vector2f Navigation::get_local_goal() {
  // Check if v_ and neighbors_ are empty if empty:
  if(v_.empty() || neighbors_.empty()){
    // Check if vertices.txt and edges.txt exist, 
    if(FILE* v_fid = fopen("vertices.txt", "r")) {
      fclose(v_fid);
      if(FILE* e_fid = fopen("edges.txt", "r")) {
        fclose(e_fid);
        load_graph();
      }
      else{
        make_graph();
      }
    }
    else{
      make_graph();
    }
  }  
  // find intersection of plan and 4m radius around current location, this is local_goal
  Vector2f carrot(4,0);
  bool intersection_found = 0;
  while(!intersection_found) {
    intersection_found = find_carrot(&carrot); 
    if(!intersection_found){
      make_plan();
    }
  }
     // if no intersection -> plan is not valid -> run generate_plan() and repeat 
  return carrot;
}

float ParticleFilter::_Distance(Vector2f p1, Vector2f p2) {
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

bool Navigation::find_carrot(Vector2f* carrot){
  // Using plan_ and robot_loc_  (these are both in map frame)
  //   -> plan_ is a vector of ints (coresponding to verticies) (just the indices)
  // 1. find potential carrot locations in map frame  (intersection of plan and 4m circle around robot
  // 2. transform carrot location from map frame to robot frame
  // 3. pick best potential carrot
  //     note we there may be 2+ intersection points, pick the one clostest to the nav_goal_loc_
  //     AND in front of the robot

  float deg_res = 360;
  Vector2f starting_point(2,0); 
  //vector<line2f> circle;
  line2f circle_line(Vector2f(starting_point.x(),-2*sin(M_PI/deg_res)/sin(M_PI*(90-180/deg_res)/180)),
                     Vector2f(starting_point.x(),2*sin(M_PI/deg_res)/sin(M_PI*(90-180/deg_res)/180)));
  //line2f circle = vector containing lines
  Vector2f intersection_point;
  Vector2f potential_carrot;
  float min_goal_dist = 4;

  for(size_t n = 0; n < plan_.size(); n++) {
    line2f plan_line(Vector2f(robot_loc_[plan_[n]].x(),robot_loc_[plan_[n]].y()),
                     Vector2f(robot_loc_[plan_[n+1]].x(),robot_loc_[plan_[n+1]].y()));
    for(int theta = -90; theta < 90; theta++) {
      line2f circle_line(robot_loc_.x()+circle_line.p0.x()*cos(theta*M_PI/180)-circle_line.p0.y()*sin(theta*M_PI/180),
                         robot_loc_.y()+circle_line.p0.x()*sin(theta*M_PI/180)+circle_line.p0.y()*cos(theta*M_PI/180),
                         robot_loc_.x()+circle_line.p1.x()*cos(theta*M_PI/180)-circle_line.p1.y()*sin(theta*M_PI/180),
                         robot_loc_.y()+circle_line.p1.x()*sin(theta*M_PI/180)+circle_line.p1.y()*cos(theta*M_PI/180));
      bool intersects = plan_line.Intersects(circle_line,&intersection_point);
      if(intersects && _Distance(intersection_point,nav_goal_loc_) < min_goal_dist) {
        min_goal_dist = _Distance(intersection_point,nav_goal_loc_);
        potential_carrot = intersection_point;
      }
    }
  
  carrot = potential_carrot;

  }
}

void Navigation::make_plan(){ 
  reset_graph();
  // Add start and goal to graph
  // run dijkstra 
  // add vertice indicies to plan_ in order
  // remove start and goal vertices from v_, neighbors_, and visited_ 
}

}  // namespace navigation
