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
    robot_angle_(0),
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
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation

// attempt to generate vertices & edges for PRM
// 100000*100000 points on the map based on the map given
// try generating 2000 * 2000 sample points
// any point that is closer to a line in map

// float Navigation::dist_point_to_line(float point_x, float point_y, Vector2f map_line_point1, Vector2f map_line_point2){
//   float dist;
//   dist = abs((map_line_point2.x - map_line_point1.x)*(map_line_point1.y-point_y) - (map_line_point1.x - map_line_point0.x) * (map_line_point2.y-map_line_point1.y))/sqrt((map_line_point2.x-map_line_point1.x)^2+(map_line_point2.y-map_line_point1.y)^2);
//   return dist;
// }
// float Navigation::dist_point_to_point(Vector2f p1, Vector2f p2) {
//  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
// }

// std::vector< Vector2f > sample_points;
//not sure if this is a bug since i is not used
// for (int i = 0 ; i < 4000000; i++){
//  Vector2f sample_point;
//  float x = rng_.UniformRandom(0, 1);
// 100 = 100000 / 1000
//  rand_x = -50 + (100 * x);
//  sample_point.x = rand_x;
//  float y = rng_.UniformRandom(0, 1);
//  rand_y = -50 + (100 * y);
//  sample_point.y = rand_y;
//  sample_points.push_back(sample_point);
// }

// std::vector< int > sample_points_filtered;
// for (Vector2f sample_point : sample_points){
//   for (Line line : map){
//     if (dist_point_to_line(sample_point.x, sample_point.y, line.point1, line.point2) > 0.3){
//	  sample_points_filtered.push_back(sample_point);
//	}
//   }
// }
// std::vector <vector<Vector2f>> edges;
// for (Vector2f sample_point1 : sample_points_filtered){
//   for (Vector2f sample_point2 : sample_points_filtered){
//   	if (0 < dist_point_to_point(sample_point1, sample_point2) < 0.1){
//wrong syntax
//	  edges.push_back(sample_point1, sample_point2);
//	}
//   }
// }
//
// std::vector <vector<Vector2f>> edges;
// for (vector<Vector2f> edge : edges){
//   for (Line line : map){
//   	if (line !intersect edge){
//       edges_filtered.push_back(edge);
//      }
//   }
// }
//
// add starting point and ending point
// dijkstra
