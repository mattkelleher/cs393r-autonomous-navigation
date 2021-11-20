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
\file     obstacle_avoidance.h
\brief    Function interfaces for obstacle avoidance 
\authors  Melissa Cruz, Yuhong Kan, Maxx Wilson (C) 2021
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "navigation/navigation.h"
#include "visualization/visualization.h"
#include "obstacle_avoidance/car_params.h"

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

namespace obstacle_avoidance{

/**
 * struct to contain the radii which define different collision zones.
 * 
 * While not required by the structure, the curvature values used are always positive for our use.
 */
struct PathBoundaries{
    float curvature = 0.0;
    float min_radius = 0.0;
    float max_radius = 0.0;
    float boundary_radius = 0.0;

    PathBoundaries(float curvature){
        this->curvature = curvature;

        if(abs(curvature) > 1e-5){
            this->min_radius = 1/this->curvature - car_params::width/2 - car_params::safety_margin;

            this->boundary_radius = hypot(
            car_params::dist_to_front_bumper,
            1/this->curvature - car_params::width/2 - car_params::safety_margin);
            
            this->max_radius = hypot(
            car_params::dist_to_front_bumper,
            1/this->curvature + car_params::width/2 + car_params::safety_margin);
        }
    }
};

/**
 * Implements computationally cheap methods to rule out certain points, saving more expensive searches
 * Includes skipping points behind the car, points that the car will turn away from, and points outside the search radius.
 * 
 * @param curvature signed curvature defining path 
 * @param point point to check
 * @returns bool where point can either be skipped immediately or requires further computation
 */
bool IsPointCollisionPossible(float curvature, const Eigen::Vector2f &point);

/**
 * Evaluates path (defined mainly by curvature) over vector of points in point cloud,
 * updating path_option (including free_path_length, etc.) based on sensor data.
 * 
 * @param path_option       updated by function with path values determined from point cloud
 * @param collision_bounds  contains parameters defining radial boundaries for collision regions of a path
 * @param point_cloud_      reference to navigation.point_cloud_, vector of 2D obstacle points
 */
void EvaluatePathWithPointCloud(navigation::PathOption &path_option, const PathBoundaries &collision_bounds, const std::vector<Eigen::Vector2f> &point_cloud_);

/**
 * Evaluates path with specific point, checking for collision, clearance, and distance to goal.
 * 
 * @param collision_bounds  contains parameters defining radial boundaries for collision regions of a path
 * @param point             obstacle point to evaluate
 * @returns PathOption struct containing calculated path parameters
 */
navigation::PathOption EvaluatePathWithPoint(const PathBoundaries &collision_bounds, Eigen::Vector2f point);

/**
 * Given a target point in base_link frame, return a curvature path that intersects the point
 * 
 * @param point target point in base_link frame (X Axis forward, Y-Axis left)
 * @returns curvature of the arc that passes through target point
 */
float GetCurvatureFromGoalPoint(Eigen::Vector2f point);

/**
 * For Side Collision, calculate Free Path Length to obstacle.
 * 
 * Always returns a positive path length, regardless of signs of input parameters.
 * 
 * @param radius_to_collision   distance from center of turning to collision point, same as radius to obstacle
 * @param angle_to_obstacle     angle between base link and point obstacle in radians
 * @param min_collision_radius  smallest radius traced by a point on the car, lower bound for collision
 * @returns arc length of path to obstacle in meters, always positive
 */
float GetPathLengthToSideCollision(float radius_to_collision, float angle_to_obstacle, float min_collision_radius);

/**
 * For Front Collision, calculate Free Path Length to obstacle.
 * 
 * Always returns a positive path length, regardless of signs of input parameters.
 * 
 * @param radius_to_collision   distance from center of turning to collision point, same as radius to obstacle
 * @param angle_to_obstacle     angle between base link and point obstacle in radians
 * @param dist_to_front         distance in meters between base_link and front bumper
 * @returns arc length of path to obstacle in meters, always positive
 */
float GetPathLengthToFrontCollision(float radius_to_collision, float angle_to_obstacle, float dist_to_front);

/**
 * Returns a curvature value an integer multiple of increments away from the goal curvature
 * with min_val as a lower bound. Essentially, this pulls curvature values from the valid curvature
 * range, while ensuring curves are spaced nicely from the current goal curvature.
 * 
 * Assumes the request value index will not exceed the range maximum (i.e. max curvature).
 * 
 * Example:
 * Inputs are Index = 3, Req = 0.7, Min = -2.0, Max = 2.0, Increment = 0.6
 * Range is [-1.7, -1.1, -0.5, 0.1, 0.7, 1.3, 1.9]
 * Output is 0.1
 * 
 * @param desired_val_index Index of value in virtual range
 * @param req_val           Required value in range that defines offset from min_val
 * @param min_val           lower bound to values in range
 * @param increment         increment between adjacent range values
 * @returns curvature as requested by index from possible curvature range
 */
float GetCurvatureOptionFromRange(float desired_val_index, float req_val, float min_val, float increment);

void LimitFreePath(navigation::PathOption& path,const Eigen::Vector2f& goal);
struct navigation::PathOption ChooseBestPath(std::vector<navigation::PathOption>& paths, const Eigen::Vector2f& goal);
void EvaluateClearanceWithPointCloud(navigation::PathOption &path_option, const PathBoundaries &collision_bounds, const std::vector<Eigen::Vector2f> &point_cloud_);
float EvaluateClearanceWithPoint(const PathBoundaries &collision_bounds, const navigation::PathOption &path_option, Eigen::Vector2f point);

// Visualization functions //
/**
 * Call the other visualization functions to visualize all the information needed for obstacle avoidance
 * 
 * @param goal 2D goal point in car frame
 * @param paths vector of possible path options
 * @param selected_path final path selected by scoring function
 * @param msg visualization msg to draw (local)
 */
void VisualizeObstacleAvoidanceInfo(Eigen::Vector2f& goal,
                           std::vector<navigation::PathOption>& paths,
                           const navigation::PathOption& selected_path,
                           amrl_msgs::VisualizationMsg &msg);
                           
void CarOutliner(amrl_msgs::VisualizationMsg& msg);
void PossiblePathsOutliner(const std::vector<navigation::PathOption>& paths,amrl_msgs::VisualizationMsg& msg);
void SelectedPathOutliner(const navigation::PathOption& selected_path,amrl_msgs::VisualizationMsg& msg);
void GoalOutliner(Eigen::Vector2f& goal, amrl_msgs::VisualizationMsg& msg);

/**
 * Clears values from the velocity command buffer up to to the value before timestamp
 * 
 * @param v vector of previous vehicle commands
 * @param time timestamp in nano seconds, prior to which commands should be cleared
 */
void CleanVelocityBuffer(std::vector<navigation::CommandStamped> &v, uint64_t time);

/**
 * Given a goal point in base_link frame, return a curvature path that intersects the point
 * 
 * @param point 2D Point representing immediate target
 */
float GetCurvatureFromGoalPoint(Eigen::Vector2f point);

/**
 * Function to integrate the vehicle's state forward in time given the previous curvature and velocity commands.
 * 
 * @param curr_state contains current estimate of vehicle position, velocity, angle
 * @param last_cmd contains previous command issues to car
 * @param timestep timestep in nanoseconds
 * 
 * @returns Time shifted transform representing the new robot state/transform between previous and next states.
 */
navigation::TimeShiftedTF IntegrateState(navigation::TimeShiftedTF curr_state, navigation::CommandStamped last_cmd, uint64_t timestep);

/**
 * Models vehicle dynamics, where the car cannot instantly achieve commanded velocity
 * but must accelerate from the current velocity.
 * 
 * @param current_velocity m/s
 * @param command_velocity m/s
 * @param timestep timestep in seconds to integrate velocity across
 * @returns estimated speed value for next timestep
 */
double getNewSpeed(double current_velocity, double command_velocity, double timestep);

void VisualizeLatencyInfo(amrl_msgs::VisualizationMsg &msg, std::vector<Eigen::Vector2f> &point_cloud, Eigen::Vector2f odom_loc, float odom_angle); 

/**
 * Outline forward-predicted position of point cloud [blue]
 * 
 * @param msg local viz message
 * @param point_cloud reference to point cloud to be displayed
 */
void LatencyPointCloud(amrl_msgs::VisualizationMsg &msg, std::vector<Eigen::Vector2f>& point_cloud);

/**
 * Outline forward-predicted position of car [blue]. Draws car at given angle and position
 * relative to base link.
 * 
 * @param msg local viz message
 * @param position position in m/s, x-forward y-left
 * @param theta angle in radians
 */
void DrawCarLocal(amrl_msgs::VisualizationMsg &msg, Eigen::Vector2f position, float theta);
} // namespace obstacle_avoidance             

#endif // OBSTACLE_AVOIDANCE_H