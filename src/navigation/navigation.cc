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

//}  // namespace navigation 
//#######################################################################################################################################

// attempt to generate vertices & edges for PRM
// 100000*100000 points on the map based on the map given
// try generating 2000 * 2000 sample points
// any point that is closer to a line in map

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
  vector<Vector2f> sample_points;

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

  vector<Vector2f> sample_points_filtered;
  for (auto sample_point : sample_points){
    for (size_t j = 0; j < map_.lines.size(); j++){
      Vector2f m_point1;
      Vector2f m_point2;
      
      m_point1.x() = map_.lines[j].p0.x();
      m_point1.y() = map_.lines[j].p0.y();
      m_point2.x() = map_.lines[j].p1.x();
      m_point2.y() = map_.lines[j].p1.y();

      if (dist_point_to_line(sample_point.x(), sample_point.y(), m_point1, m_point2) > 0.3){ //TODO 0.3 
        sample_points_filtered.push_back(sample_point);
      }
    }  
  }

  vector<Vector2i> edges;
  for(size_t i = 0; i < sample_points_filtered.size(); i++) {
    for(size_t j = i + 1; j < sample_points_filtered.size(); j++) {
      if(0 < dist_point_to_point(sample_points_filtered[i], sample_points_filtered[j]) && dist_point_to_point(sample_points_filtered[i], sample_points_filtered[j]) < 2){ //TODO 0 and 0.1
        edges.push_back(Vector2i(int(i), int(j)));
      }
    } 
  }
  vector<Vector2i> edges_filtered;
  for (size_t i = 0; i<edges.size(); i++){
    Vector2i edge = edges[i];
    line2f edge_line(sample_points_filtered[edge.x()], sample_points_filtered[edge.y()]); 
    for (size_t j = 0; j < map_.lines.size(); j++){ //TODO make map_ part of navigation class and initalize
      const line2f map_line = map_.lines[j];
      Vector2f intrsctn_pt;
      if(!map_line.Intersection(edge_line, &intrsctn_pt)){ //TODO
        edges_filtered.push_back(edge);
      }
    }
  }
  
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
  FILE* edge_fid = fopen("edge.txt", "w");
  if(edge_fid == NULL) {
    fprintf(stderr, "ERROR: Unable to open edge.txt");
    exit(1);
  }
  for(size_t i = 0; i < edges_filtered.size(); i++) {
    fprintf(vertex_fid, "%i, %i \n", edges_filtered[i].x(), edges_filtered[i].y());
  }
  fclose(edge_fid);
}

}  // namespace navigation
