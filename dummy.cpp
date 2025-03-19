#include <chrono>
#include <thread>

#include "skidsteer.h"
#include "audi.h"
#include "pfmsconnector.h"

#include "pfms_types.h"
#include "pfmshog.h"


SkidSteer::SkidSteer() {


     //setting default movemnt charcteritics
    
    rotate_right_ = -1.0;
    rotate_left_ = 1.0;
    move_foward_ = 1.0;
    move_reverse_ = -1.0;

    platform_type_ = pfms::PlatformType::SKIDSTEER; // Set platform to SKIDSTEER.
    ROS_communication_ = std::make_shared<PfmsConnector>(); // Initialize ROS communication.
    
   

}

// Method to set goal points the skid steer will go to.
//Stores the provided goal point in the target_goal_ member variable.
// Returns true to indicate success
bool SkidSteer::setGoal(pfms::geometry_msgs::Point goal){

    target_goal_ = goal;
        return true;

}

// Driving the skidsteer to the goals
bool SkidSteer::reachGoal(void){ 


    //Reads the current odometry and updates distance traveled
    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    distanceToGoal(); 
    distance_travelled_by_platform_ = distance_travelled_by_platform_ + distance_platform_to_goal_; 

    
    bool orientation = false; 
    unsigned long count = 1;
    double angle = std::atan2(target_goal_.y - current_platform_odometry_.position.y, target_goal_.x - current_platform_odometry_.position.x); // Desired orientation angle.
    double rotatingAngleError = current_platform_odometry_.yaw - angle; // Calculate initial angle error.
    auto start = std::chrono::high_resolution_clock::now(); // Start time for performance measurement.

    // Adjusts the vehicle’s orientation to face the goal.
    while(!orientation){
        // accounts and adjusts for angle error depending on which direction it is out by
        if(rotatingAngleError > 0){
            
            //error to the right
            
            command_ = {count, rotate_right_, 0.0}; 
            ROS_communication_->send(command_);
            readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
            rotatingAngleError = current_platform_odometry_.yaw - angle - 0.25; 
                
                if(rotatingAngleError < 0.1){
                    orientation = true; 
                }
        }   
            //error to the left
        else if(rotatingAngleError < 0){
            command_ = {count, rotate_left_, 0.0}; // Command for counter-clockwise turn.
            ROS_communication_->send(command_);
            readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
            rotatingAngleError = current_platform_odometry_.yaw - angle + 0.2; // Update angle error.
            
                if(rotatingAngleError > M_PI){
                    
                    orientation = true; // Correct orientation achieved, considering angle wrap-around.
                }
        }
    }

    command_ = {1, 0.0, 0.0};
    ROS_communication_->send(command_);
    
    count = 1;

    
    bool arrivedAtGoal = false;
    count = 1;
    
    while(!arrivedAtGoal){
        // Drives the skidsteer towards the goal
        command_ = {count, 0.0, move_foward_};
        ROS_communication_->send(command_);
        count++;
        
        
        readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
        
        double currentDistanceToGoal = std::sqrt(std::pow(target_goal_.x - current_platform_odometry_.position.x, 2) + std::pow(target_goal_.y - current_platform_odometry_.position.y, 2));
        
            if(currentDistanceToGoal < tolerance_value_){
                
                arrivedAtGoal = true; 
            }
    }
    
    //stop the skidsteer when at goal
    bool stopped = false;
    count = 1;
    double velocity = 0.0;

    while(!stopped){
        command_ = {count, 0.0, 0.0};
        
        ROS_communication_->send(command_);
        count++; 
        readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
        // Calculate the vehicle's current velocity to determine if it has stopped.
        double velocity = std::sqrt(std::pow(current_platform_odometry_.linear.x, 2) + std::pow(current_platform_odometry_.linear.y, 2));
        if(velocity < 1e-1){
            stopped = true; // Vehicle has stopped if velocity is below threshold.
        }
    }

    // Final check after stopping to ensure the goal is still within tolerance.
    readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    double currentDistanceToGoal = std::sqrt(std::pow(target_goal_.x - current_platform_odometry_.position.x, 2) + std::pow(target_goal_.y - current_platform_odometry_.position.y, 2));
    if(currentDistanceToGoal < tolerance_value_){

        //track time taken to drive to the goal
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double elapsed_time_seconds = elapsed.count();
        time_in_motion_by_platform_ += elapsed_time_seconds; 
        
        return true; 
    }
    
    else{
        
        return false;
    }
}

// Calculate the distance to the goal based on current position.
double SkidSteer::distanceToGoal() {

    // Calculate the Euclidean distance between the current position and the goal.
    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    double distance = std::sqrt(std::pow(current_platform_odometry_.position.x - target_goal_.x, 2) + std::pow(current_platform_odometry_.position.y - target_goal_.y, 2));
    
        return distance;

}

// using the atan fun. to guess the time tacken to get to the goals from current point
double SkidSteer::timeToGoal(void) {

    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    
    double rotatingAngleError;
    
    double angle = std::atan2(target_goal_.y - current_platform_odometry_.position.y, target_goal_.x - current_platform_odometry_.position.x);
    
    rotatingAngleError = current_platform_odometry_.yaw - angle;
    
    rotatingAngleError = abs(rotatingAngleError);
    
    double distanceToGoal = std::sqrt(std::pow(current_platform_odometry_.position.x - target_goal_.x, 2) + std::pow(current_platform_odometry_.position.y - target_goal_.y, 2));
    
    double timeToGoal = rotatingAngleError/rotate_left_ + distanceToGoal/move_foward_;
    
    return timeToGoal;

}

// Set the tolerance 
bool SkidSteer::setTolerance(double tolerance) {

    if (tolerance < 0) {
        //negative tolrence are irlative
        return false;
    }
    tolerance_value_ = tolerance; 
    return true; 

}

// get the distance travelled by the skidsteer to the goal
double SkidSteer::platformDistanceTravelled(void) {

    return distance_travelled_by_platform_;

}

// get the time the skidsteer took to the get the goal.
double SkidSteer::timeInMotion(void) {

    return time_in_motion_by_platform_;

}

// get the current odometry of the skidsteer.
pfms::nav_msgs::Odometry SkidSteer::getOdometry(void) {

    ROS_communication_->read(current_platform_odometry_, platform_type_);
    
        return current_platform_odometry_;

}