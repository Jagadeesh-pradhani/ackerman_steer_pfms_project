#include "skidsteer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

//-----------------------------------------------------------------------
// Anonymous Namespace: Constant Parameters for SkidSteer Control
// Description: Holds various constant parameters used to configure the 
//              SkidSteer control algorithm.
//-----------------------------------------------------------------------
namespace {
    const double DEFAULT_TOLERANCE       = 0.5;   // Default distance tolerance
    const double ANGULAR_ERROR_THRESHOLD = 0.1;   // Acceptable angular error (radians)
    const double KP_ANG                  = 3.0;   // Proportional gain for angular control
    const double KP_MOVE                 = 0.2;   // Proportional gain for forward motion
    const double KP_COAST                = 0.2;   // Proportional gain for coasting
    const double MAX_MOVE_SPEED          = 1.0;   // Maximum speed in MOVE state
    const double MAX_COAST_SPEED         = 0.3;   // Maximum speed in COAST state
    const double COAST_MULTIPLIER        = 2.0;   // Multiplier to determine coast threshold
    const int    LOOP_SLEEP_MS           = 20;    // Loop cycle time in milliseconds
}

//-----------------------------------------------------------------------
// Class: SkidSteerInternal
// Description: Maintains the internal state and configuration parameters 
//              for SkidSteer control.
//-----------------------------------------------------------------------
class SkidSteerInternal {
public:
    pfms::nav_msgs::Odometry odom;          // Latest odometry reading
    pfms::geometry_msgs::Point goal;          // Target goal position
    double tolerance;                         // Distance tolerance for reaching goal
    double totalDistance;                     // Accumulated distance traveled
    double distanceToGoalVal;                 // Computed distance to the goal
    double totalTime;                         // Accumulated time in motion
    double timeToGoalVal;                     // Computed estimated time to goal
    
    // Movement scaling parameters.
    double rotate_right_ = -1.0;
    double rotate_left_  = 1.0;
    double move_foward_  = 1.0;
    double move_reverse_ = -1.0;
    
    pfms::commands::SkidSteer cmd;            // Command message for SkidSteer
    std::shared_ptr<PfmsConnector> connector; // Communication connector
    
    // Constructor initializes default values.
    SkidSteerInternal() 
      : tolerance(DEFAULT_TOLERANCE), totalDistance(0.0), distanceToGoalVal(0.0),
        timeToGoalVal(0.0), totalTime(0.0) {
        cmd.seq = 0;
    }
};

//-----------------------------------------------------------------------
// Global instance of SkidSteerInternal for this module.
//-----------------------------------------------------------------------
static SkidSteerInternal skidData;

//-----------------------------------------------------------------------
// Constructor: SkidSteer
// Description: Initializes the SkidSteer module and sets the platform type.
//-----------------------------------------------------------------------
SkidSteer::SkidSteer() {
    skidData.tolerance = DEFAULT_TOLERANCE;
    if (!skidData.connector)
        skidData.connector = std::make_shared<PfmsConnector>(pfms::PlatformType::SKIDSTEER);
}

//-----------------------------------------------------------------------
// Method: setGoal
// Description: Sets the desired target position for the SkidSteer vehicle.
// Returns: true after the goal is set.
//-----------------------------------------------------------------------
bool SkidSteer::setGoal(pfms::geometry_msgs::Point goal) {
    skidData.goal = goal;
    return true;
}

//-----------------------------------------------------------------------
// Method: reachGoal
// Description: Executes the state machine that drives the vehicle to the 
//              target goal. The states include IDLE, ROTATE, MOVE, COAST, 
//              and STOPPING.
// Returns: true if the goal is reached successfully; false otherwise.
//-----------------------------------------------------------------------
bool SkidSteer::reachGoal(void) {
    bool goalReached = false;
    double distance = 0.0;
    double angularError = 0.0;
    
    // Define state machine states.
    enum class State { IDLE, ROTATE, MOVE, COAST, STOPPING };
    State state = State::IDLE;
    
    // Calculate the coast threshold based on tolerance.
    const double coastThreshold = COAST_MULTIPLIER * skidData.tolerance;
    
    // Main control loop.
    while (!goalReached) {
        // Update odometry; if reading fails, print error and exit.
        if (!skidData.connector->read(skidData.odom)) {
            std::cout << "SkidSteer: Unable to read odometry." << std::endl;
            return false;
        }
        
        // Calculate Euclidean distance to the goal.
        distance = std::hypot(skidData.goal.x - skidData.odom.position.x,
                              skidData.goal.y - skidData.odom.position.y);
        
        // Compute the desired heading.
        double desiredHeading = std::atan2(skidData.goal.y - skidData.odom.position.y,
                                           skidData.goal.x - skidData.odom.position.x);
        double currentHeading = skidData.odom.yaw;
        angularError = desiredHeading - currentHeading;
        
        // Normalize angular error to the range [-pi, pi].
        while (angularError > M_PI)  angularError -= 2 * M_PI;
        while (angularError < -M_PI) angularError += 2 * M_PI;
        
        // State transitions based on angular error and distance.
        switch(state) {
            case State::IDLE:
                if (std::fabs(angularError) > ANGULAR_ERROR_THRESHOLD)
                    state = State::ROTATE;
                else if (distance > coastThreshold)
                    state = State::MOVE;
                else if (distance > skidData.tolerance)
                    state = State::COAST;
                else
                    state = State::STOPPING;
                break;
                
            case State::ROTATE:
                if (std::fabs(angularError) <= ANGULAR_ERROR_THRESHOLD) {
                    if (distance > coastThreshold)
                        state = State::MOVE;
                    else if (distance > skidData.tolerance)
                        state = State::COAST;
                    else
                        state = State::STOPPING;
                }
                break;
                
            case State::MOVE:
                if (distance <= coastThreshold)
                    state = State::COAST;
                break;
                
            case State::COAST:
                if (distance <= skidData.tolerance)
                    state = State::STOPPING;
                else if (std::fabs(angularError) > ANGULAR_ERROR_THRESHOLD)
                    state = State::ROTATE;
                break;
                
            case State::STOPPING:
                skidData.cmd.move_f_b = 0.0;
                skidData.cmd.turn_l_r = 0.0;
                goalReached = true;
                break;
        }
        
        // Issue commands based on the current state.
        if (state == State::ROTATE) {
            skidData.cmd.move_f_b = 0.0;
            skidData.cmd.turn_l_r = KP_ANG * angularError;
        } else if (state == State::MOVE) {
            double forwardSpeed = KP_MOVE * distance;
            if (forwardSpeed > MAX_MOVE_SPEED)
                forwardSpeed = MAX_MOVE_SPEED;
            skidData.cmd.move_f_b = forwardSpeed;
            skidData.cmd.turn_l_r = 0.0;
        } else if (state == State::COAST) {
            double forwardSpeed = KP_COAST * distance;
            if (forwardSpeed > MAX_COAST_SPEED)
                forwardSpeed = MAX_COAST_SPEED;
            skidData.cmd.move_f_b = forwardSpeed;
            skidData.cmd.turn_l_r = 0.0;
        } else if (state == State::IDLE) {
            skidData.cmd.move_f_b = 0.0;
            skidData.cmd.turn_l_r = 0.0;
        }
        // In STOPPING state, commands remain zero.
        
        // Send the command to the vehicle.
        skidData.connector->send(skidData.cmd);
        skidData.cmd.seq++;  // Increment command sequence.
        
        // Sleep for a fixed loop interval.
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
    }
    
    // Update the total distance traveled.
    skidData.totalDistance += distance;
    return goalReached;
}

//-----------------------------------------------------------------------
// Method: distanceToGoal
// Description: Computes the remaining distance to the target goal.
// Returns: The calculated distance value.
//-----------------------------------------------------------------------
double SkidSteer::distanceToGoal(void) {
    bool OK = skidData.connector->read(skidData.odom);
    if (!OK)
        std::cout << "No odom" << std::endl;
    
    // Update the distance and time estimates using the helper method.
    OK = checkOriginToDestination(skidData.odom, skidData.goal,
                                   skidData.distanceToGoalVal, skidData.timeToGoalVal,
                                   skidData.odom);
    return skidData.distanceToGoalVal;
}

//-----------------------------------------------------------------------
// Method: timeToGoal
// Description: Estimates the total time to reach the goal by combining 
//              rotational and forward movement times.
// Returns: The estimated time.
//-----------------------------------------------------------------------
double SkidSteer::timeToGoal(void) {
    bool OK = skidData.connector->read(skidData.odom);
    
    // Compute the required rotation.
    double angle = std::atan2(skidData.goal.y - skidData.odom.position.y,
                              skidData.goal.x - skidData.odom.position.x);
    double rotatingAngleError = std::fabs(skidData.odom.yaw - angle);
    
    // Calculate the straight-line distance.
    double distanceToGoal = std::sqrt(std::pow(skidData.odom.position.x - skidData.goal.x, 2) +
                                      std::pow(skidData.odom.position.y - skidData.goal.y, 2));
    
    // Estimate time: rotation time plus forward travel time.
    double timeToGoal = rotatingAngleError / skidData.rotate_left_ + distanceToGoal / skidData.move_foward_;
    skidData.timeToGoalVal = timeToGoal;
    
    return skidData.timeToGoalVal;
}

//-----------------------------------------------------------------------
// Method: setTolerance
// Description: Updates the tolerance used to determine goal proximity.
// Returns: true after the tolerance is set.
//-----------------------------------------------------------------------
bool SkidSteer::setTolerance(double tolerance) {
    skidData.tolerance = tolerance;
    return true;
}

//-----------------------------------------------------------------------
// Method: distanceTravelled
// Description: Retrieves the total distance the vehicle has traveled.
//-----------------------------------------------------------------------
double SkidSteer::distanceTravelled(void) {
    return skidData.totalDistance;
}

//-----------------------------------------------------------------------
// Method: timeInMotion
// Description: Retrieves the total time the vehicle has been in motion.
//-----------------------------------------------------------------------
double SkidSteer::timeInMotion(void) {
    return skidData.totalTime;
}

//-----------------------------------------------------------------------
// Method: getOdometry
// Description: Retrieves the current odometry data from the vehicle.
//-----------------------------------------------------------------------
pfms::nav_msgs::Odometry SkidSteer::getOdometry(void) {
    return skidData.odom;
}

//-----------------------------------------------------------------------
// Method: checkOriginToDestination
// Description: Computes the Euclidean distance between the origin and the 
//              goal and estimates the time required to reach the destination.
// Parameters:
//  - origin: The current odometry data.
//  - goal: The target goal position.
//  - distance: (Output) The computed distance between origin and goal.
//  - time: (Output) The estimated time based on a constant speed model.
//  - estimatedGoalPose: (Output) The estimated odometry at the goal.
// Returns: true if the computation is successful.
//-----------------------------------------------------------------------
bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    // Compute the Euclidean distance.
    distance = std::hypot(goal.x - origin.position.x, goal.y - origin.position.y);
    
    // Estimate travel time using a constant speed factor.
    time = distance / (1.0 - 0.06);
    
    // Set the estimated goal pose.
    estimatedGoalPose = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw = std::atan2(goal.y - origin.position.y, goal.x - origin.position.x);
    return true;
}
