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
\file    obstacle_avoidance.h
\brief   Function implementations for obstacle avoidance 
\authors  Melissa Cruz, Yuhong Kan, Maxx Wilson (C) 2021
*/
//========================================================================

#include <iostream>

#include "shared/math/math_util.h"

#include "obstacle_avoidance/obstacle_avoidance.h"


#include <float.h>
#include "shared/math/line2d.h"

#include "gflags/gflags.h"


// line arguments used in obstacle avoidance function
DEFINE_double(clearance_param, car_params::clearance_gain, "clearance parameter used in scoring function");
DEFINE_double(distance_goal_param, car_params::dist_goal_gain, "distance to goal parameter used in scoring function");

using namespace math_util;

namespace obstacle_avoidance{

bool IsPointCollisionPossible(float curvature, const Eigen::Vector2f &point){
    //return true;
    
    if(point[0] < 0){
        // Ignore points behind car
        return false;
    }
    else if(Sign(curvature)*point[1] < -car_params::clearance_factor){ //-(car_params::width/2 + car_params::safety_margin + car_params::clearance_factor)){
        // Ignore points in the direction opposite of curvature
        // (Can't hit any point to the left when turning right)
        return false;
    }
    else if(point.norm() > car_params::max_path_length){
        // Ignore points outside of max search area
        // This will change latency depending on environment,
        // longer but consistent might be preferable to sometimes long sometimes short
        return false;
    }
    else{
        return true;
    }
}
   
void EvaluatePathWithPointCloud(navigation::PathOption &path_option, const PathBoundaries &collision_bounds, const std::vector<Eigen::Vector2f> &point_cloud_){
    for(std::size_t point_index = 0; point_index < point_cloud_.size(); point_index++){
        Eigen::Vector2f point = point_cloud_[point_index];

        if(!obstacle_avoidance::IsPointCollisionPossible(path_option.curvature, point)){
          continue;
        }

        // Easier to mirror a point for negative curvature than to make everything conditional on sign
        if(path_option.curvature < 0){
          point[1] *= -1;
        }

        navigation::PathOption path_result = EvaluatePathWithPoint(collision_bounds, point);

        // Update path_option with shorter paths or smaller clearances
        if(path_result.free_path_length < path_option.free_path_length){
          path_option.free_path_length = path_result.free_path_length;
          path_option.obstruction = point_cloud_[point_index];
        }
        // if(abs(path_result.clearance) < abs(path_option.clearance)){
        //   path_option.clearance = path_result.clearance;
        // }
    }
    
}


navigation::PathOption EvaluatePathWithPoint(const PathBoundaries &collision_bounds, Eigen::Vector2f point){
        navigation::PathOption path_result{0, 1000, car_params::max_path_length};

        // Zero Curvature Collision
        if(abs(collision_bounds.curvature) < 1e-5 && abs(point[1]) <= (car_params::safety_margin + car_params::width / 2)){
            path_result.free_path_length = point[0] - car_params::dist_to_front_bumper;
        }

        float radius_to_obstacle = (point - Eigen::Vector2f(0, 1/collision_bounds.curvature)).norm();
        float angle_to_obstacle = atan2(point[0], 1/collision_bounds.curvature - point[1]);

        // Dont check points larger than current collision angle TODO

        // Evaluate Collision Region
        if(radius_to_obstacle < collision_bounds.min_radius){
            // No Collision, Inner Miss
            // path_result.clearance = collision_bounds.min_radius - radius_to_obstacle;
        }
        else if(collision_bounds.min_radius <= radius_to_obstacle && radius_to_obstacle <= collision_bounds.boundary_radius){
            // Side Collision
            path_result.free_path_length = obstacle_avoidance::GetPathLengthToSideCollision(radius_to_obstacle, angle_to_obstacle, collision_bounds.min_radius);
        }
        else if(collision_bounds.boundary_radius <= radius_to_obstacle && radius_to_obstacle <= collision_bounds.max_radius){
            // Front Collision
            path_result.free_path_length = obstacle_avoidance::GetPathLengthToFrontCollision(radius_to_obstacle, angle_to_obstacle, car_params::dist_to_front_bumper);
        }
        else{
            // No Collision, Outer Miss
            // path_result.clearance = radius_to_obstacle - collision_bounds.max_radius;
        }
        return path_result;
}

void EvaluateClearanceWithPointCloud(navigation::PathOption &path_option, const PathBoundaries &collision_bounds, const std::vector<Eigen::Vector2f> &point_cloud_){
    path_option.clearance = car_params::clearance_factor;
    
    for(std::size_t point_index = 0; point_index < point_cloud_.size(); point_index++){
        Eigen::Vector2f point = point_cloud_[point_index];
        // if(!obstacle_avoidance::IsPointCollisionPossible(path_option.curvature, point)){
        //   continue;
        // }
        // Easier to mirror a point for negative curvature than to make everything conditional on sign
        if(path_option.curvature < 0){
          point[1] *= -1;
        }
        float clearance = EvaluateClearanceWithPoint(collision_bounds, path_option, point);
        
        if(clearance<path_option.clearance){
            path_option.clearance = clearance;
            path_option.closest_point = point_cloud_[point_index];
            
        }
    }
}
float EvaluateClearanceWithPoint(const PathBoundaries &collision_bounds, const navigation::PathOption &path_option, Eigen::Vector2f point){
    // zero curvature
    if(abs(path_option.curvature)<1e-3) {
        if(path_option.obstruction[0]>=point[0]&&point[0]>=0) {
            return abs(point[1]);
        }
        return car_params::clearance_factor;
    }
    float radius = abs(1/path_option.curvature);
    float radius_to_point = (point - Eigen::Vector2f(0, abs(1/collision_bounds.curvature))).norm();
    float angle_to_point = atan2(point[0], abs(1/collision_bounds.curvature) - point[1]);
    if(angle_to_point<0) angle_to_point = -angle_to_point + M_PI;
    float theta = atan2(path_option.obstruction[0], abs(1/collision_bounds.curvature) - path_option.obstruction[1]);
    if(theta<0) theta = -theta + M_PI;
    if(angle_to_point>theta) return car_params::clearance_factor;
    // Evaluate Collision Region
    if(radius_to_point < collision_bounds.min_radius || radius_to_point > collision_bounds.max_radius){
        // No Collision, Inner Miss
        return abs(radius_to_point - radius);
    }
    return car_params::clearance_factor;
}


float GetPathLengthToSideCollision(float radius_to_collision, float angle_to_obstacle, float min_collision_radius){
    float angle_to_collision = atan2(
        sqrt(radius_to_collision*radius_to_collision - min_collision_radius*min_collision_radius),
        abs(min_collision_radius));
    return abs(radius_to_collision * (abs(angle_to_obstacle) - angle_to_collision));
}

float GetPathLengthToFrontCollision(float radius_to_collision, float angle_to_obstacle, float dist_to_front){
        float angle_to_collision = atan2(
            dist_to_front,
            sqrt(radius_to_collision*radius_to_collision - dist_to_front*dist_to_front));
    return abs(radius_to_collision * (abs(angle_to_obstacle) - angle_to_collision));
}

float GetCurvatureFromGoalPoint(Eigen::Vector2f point){
    float x = point[0];
    float y = point[1];

    if(abs(y)>1e-5){ // Non-Zero Curvature
        return 2/(y + Sq(x)/y);
    }
    else{
        return 0.0;
    }
}

float GetCurvatureOptionFromRange(float desired_val_index, float req_val, float min_val, float increment){
    float offset = - floor((req_val - min_val)/increment) * increment;
    float curvature = req_val + offset + desired_val_index*increment;
    return curvature;
}

//helper function: get the angle between vector[p_middle,p_left] and vector[p_middle,p_right]
double getAngle(const Eigen::Vector2f& p_middle, const Eigen::Vector2f& p_left, const Eigen::Vector2f& p_right){
    Eigen::Vector2f v1 = p_middle - p_left;
    Eigen::Vector2f v2 = p_middle - p_right;
    double angle = acos((v1/v1.norm()).dot(v2/v2.norm()));
    return angle;
}

// limit free path length and calculate closest point to goal 
void LimitFreePath(navigation::PathOption& path,const Eigen::Vector2f& goal){
    // zero curvature
    if(abs(path.curvature)<1e-3) {
        if(goal[0]>=0&&goal[0]<path.free_path_length) path.free_path_length = goal[0];
        path.closest_point.x() = path.free_path_length + (car_params::wheel_base+car_params::length)/2;
        path.closest_point.y() = 0;
        path.obstruction = path.closest_point;
        return;
    }
    Eigen::Vector2f rotate_center(0, 1/path.curvature);
    Eigen::Vector2f car_odem(0,0);
    double angle = getAngle(rotate_center,goal,car_odem);
    if (goal[0] < 0) angle = 2*M_PI - angle; // if goal is at the back of the car
    // check whether the free path need to be trimmed
    if(abs(angle/path.curvature)<path.free_path_length){
        // trim free path and change obstruction point to the goal
        path.free_path_length = abs(angle/path.curvature);
        // set the obstruction to the closet point
        path.obstruction = rotate_center + 1/path.curvature*((goal-rotate_center)/(goal-rotate_center).norm());
    }
    double theta = path.free_path_length * path.curvature;
    
    path.closest_point.x() = 1/path.curvature*sin(theta);
    path.closest_point.y() = 1/path.curvature*(1-cos(theta));
}

//calculate distance to goal of the specific path
double GetDistanceToGoal(const navigation::PathOption& path,const Eigen::Vector2f& goal){
    return (path.closest_point-goal).norm();
}

//Scoring function
struct navigation::PathOption ChooseBestPath(std::vector<navigation::PathOption>& paths, const Eigen::Vector2f& goal){
    navigation::PathOption* bestPath = NULL;
    double best_score = -DBL_MAX;
    for(auto& path:paths){
        double distance_to_goal = GetDistanceToGoal(path,goal);
        // if(abs(path.curvature)<1e-3) {
        //     std::cout<<"curvature: "<<path.curvature<<", free_path_length: "<<path.free_path_length<<", clearance:"<<path.clearance<<", distance_to_goal"<<distance_to_goal<<std::endl;
        //     std::cout<<"closest point: ("<<path.closest_point[0]<<","<<path.closest_point[1]<<")"<<std::endl;
        // }
        double score = path.free_path_length + FLAGS_clearance_param * path.clearance + FLAGS_distance_goal_param * distance_to_goal;
        if(score>best_score){
            best_score = score;
            bestPath = &path;
        }
    }
    return *bestPath;
}

// Visualization for needed for obstacle avoidance
void VisualizeObstacleAvoidanceInfo(Eigen::Vector2f& goal,
                           std::vector<navigation::PathOption>& paths,
                           const navigation::PathOption& selected_path,
                           amrl_msgs::VisualizationMsg &msg){
    CarOutliner(msg);
    PossiblePathsOutliner(paths,msg);
    SelectedPathOutliner(selected_path,msg);
    GoalOutliner(goal,msg);
}

// Outline the car[Yellow Line] and safety margin
void CarOutliner(amrl_msgs::VisualizationMsg &msg){
    Eigen::Vector2f front_left((car_params::wheel_base+car_params::length)/2,car_params::width/2);
    Eigen::Vector2f front_right((car_params::wheel_base+car_params::length)/2,-car_params::width/2);
    Eigen::Vector2f back_left(-(car_params::length-car_params::wheel_base)/2,car_params::width/2);
    Eigen::Vector2f back_right(-(car_params::length-car_params::wheel_base)/2,-car_params::width/2);
    //car front
    visualization::DrawLine(front_left,front_right,0xfcd703,msg);
    //car back
    visualization::DrawLine(back_left,back_right,0xfcd703,msg);
    //car left
    visualization::DrawLine(front_left,back_left,0xfcd703,msg);
    //car right
    visualization::DrawLine(front_right,back_right,0xfcd703,msg);
    //TODO: safety margin
}
// Draw all the possible path options
void PossiblePathsOutliner(const std::vector<navigation::PathOption>& paths,amrl_msgs::VisualizationMsg& msg){
    for(auto& path:paths){
        visualization::DrawPathOption(path.curvature,path.free_path_length,path.clearance,msg);
    }
}
// Draw the selected path
void SelectedPathOutliner(const navigation::PathOption& selected_path,amrl_msgs::VisualizationMsg& msg){
    visualization::DrawPathOption(selected_path.curvature,selected_path.free_path_length,selected_path.clearance,msg);
    //draw the closest point [Blue Cross]
    visualization::DrawCross(selected_path.closest_point,0.25,0x1e9aa8,msg);
    //draw the obstruction point [Yellow Cross]
    visualization::DrawCross(selected_path.obstruction,0.25,0xfcba03,msg);
}
// Draw goal [Red Cross]
void GoalOutliner(Eigen::Vector2f& goal, amrl_msgs::VisualizationMsg& msg){
    visualization::DrawCross(goal,0.5,0xfc4103,msg);
}

void CleanVelocityBuffer(std::vector<navigation::CommandStamped> &v, uint64_t time){
  auto it = std::lower_bound(v.begin(), v.end(), time);
  v.erase(v.begin(), it-1);
}

//Integrates from one time stamp to the next
//Returns float &point_cloud_stamp_ failed linker????
Eigen::Vector2f Integrate(uint64_t stamp, std::vector<navigation::CommandStamped> &v, float angle){
    int it = std::lower_bound(v.begin(), v.end(), stamp) - v.begin(); // Index of command before pc
    //std::cout << v[it].velocity << std::endl;
    // std::cout << "    stamp: " <<  stamp << "\n";
    // std::cout << "    ros::Time::now(): " <<  ros::Time::now().toNSec() << "\n";
    // std::cout << "    del_t(ns): " << ros::Time::now().toNSec() -  stamp << "\n";
    float del_x = (v[it].velocity * cos(angle)) * pow(10.0, -9.0) * (car_params::sys_latency);//sys_latency in ns
    float del_y = (v[it].velocity) * sin(angle) * pow(10.0, -9.0) * (car_params::sys_latency);
    return Eigen::Vector2f(del_x, del_y);
}


// Outline forward-predicted position of car [blue]
void DrawCarLocal(amrl_msgs::VisualizationMsg &msg, Eigen::Vector2f position, float theta){

    float fwd = car_params::dist_to_front_bumper;
    float side = car_params::dist_to_side_bumper;
    float rear = car_params::dist_to_rear_bumper;
    uint32_t color = 0x1e9aa8;

    Eigen::Vector2f front_left = position + Eigen::Vector2f(-side*sin(theta) + fwd*cos(theta), side*cos(theta) + fwd*sin(theta));
    Eigen::Vector2f front_right = position + Eigen::Vector2f(side*sin(theta) + fwd*cos(theta), -side*cos(theta) + fwd*sin(theta));
    Eigen::Vector2f back_left = position + Eigen::Vector2f(-side*sin(theta) - rear*cos(theta), -rear*sin(theta) + side*cos(theta));
    Eigen::Vector2f back_right = position + Eigen::Vector2f(side*sin(theta) - rear*cos(theta), -rear*sin(theta) - side*cos(theta));

    visualization::DrawLine(front_left, front_right, color, msg); // front
    visualization::DrawLine(back_left, back_right, color, msg); // back
    visualization::DrawLine(front_left, back_left, color, msg); // left
    visualization::DrawLine(front_right, back_right, color, msg); // right
    visualization::DrawLine(front_right, back_left, color,msg);
}

navigation::TimeShiftedTF IntegrateState(navigation::TimeShiftedTF curr_state, navigation::CommandStamped last_cmd, uint64_t timestep){
    navigation::TimeShiftedTF next_state;

    double dt = ((double) timestep) / 1e9;

    next_state.stamp = curr_state.stamp + timestep;
    next_state.speed = getNewSpeed(curr_state.speed, last_cmd.velocity, dt);
    next_state.theta = curr_state.theta + curr_state.speed * last_cmd.curvature * dt;

    if(last_cmd.curvature == 0){
        next_state.position = curr_state.position + Eigen::Vector2f(curr_state.speed * dt, 0);
    }
    else{
        next_state.position = curr_state.position + Eigen::Vector2f(
            sin(curr_state.speed * last_cmd.curvature * dt)/last_cmd.curvature,
            (1-cos(curr_state.speed * last_cmd.curvature * dt))/last_cmd.curvature);
    }
    return next_state;
}

double getNewSpeed(double current_velocity, double command_velocity, double timestep){
    float accel = (command_velocity >= current_velocity) ? car_params::max_acceleration : car_params::min_acceleration;
    if(Sign(accel) == 1){ // accelerating
        return std::min(current_velocity + accel*timestep, command_velocity);
    }
    else{ // deccelerating
        return std::max(current_velocity + accel*timestep, command_velocity);
    }

}

// Outline forward-predicted position of point cloud [blue]
void LatencyPointCloud(amrl_msgs::VisualizationMsg &msg, std::vector<Eigen::Vector2f> &point_cloud){
  for(std::size_t i = 0; i < point_cloud.size(); i++){
    visualization::DrawPoint(point_cloud[i], 0x1e9aa8, msg);
  }
}

// Visualization for needed for latency
void VisualizeLatencyInfo(amrl_msgs::VisualizationMsg &msg, std::vector<Eigen::Vector2f> &point_cloud, Eigen::Vector2f odom_loc, float odom_angle){
    DrawCarLocal(msg, odom_loc, odom_angle);
    LatencyPointCloud(msg, point_cloud);
}
} //namespace obstacle_avoidance

