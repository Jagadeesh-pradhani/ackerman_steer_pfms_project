#include "ackerman.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

//-----------------------------------------------------------------------
// Namespace: control
// Description: Enumerates the possible states for the Ackerman vehicle 
//              control state machine.
//-----------------------------------------------------------------------
namespace control {
    enum State {
        IDLE,         // Vehicle is idle
        ACCELERATE,   // Vehicle is accelerating
        COAST,        // Vehicle is coasting
        STOPPING      // Vehicle is stopping
    };
}

//-----------------------------------------------------------------------
// Overloaded operator<< 
// Description: Allows easy printing of the control::State for debugging.
//-----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, control::State state) {
    switch (state) {
        case control::IDLE:
            os << "IDLE ";
            break;
        case control::ACCELERATE:
            os << "ACCELERATE ";
            break;
        case control::COAST:
            os << "COAST ";
            break;
        case control::STOPPING:
            os << "STOPPING ";
            break;
        default:
            break;
    }
    return os;
}

//-----------------------------------------------------------------------
// Class: AckermanInternal
// Description: Holds the internal state, configuration, and command 
//              information for the Ackerman control module.
//-----------------------------------------------------------------------
class AckermanInternal {
public:
    pfms::nav_msgs::Odometry odom;          // Latest odometry reading
    pfms::geometry_msgs::Point goal;          // Target goal position
    double tolerance;                         // Tolerance threshold for goal achievement
    double distanceToGoalVal = 0.0;             // Computed distance to goal
    double timeToGoalVal = 0.0;                 // Computed time to goal
    double totalDistance;                     // Accumulated distance traveled
    double totalTime;                         // Accumulated time in motion
    pfms::commands::Ackerman cmd;             // Command message to be sent to the platform
    std::shared_ptr<PfmsConnector> connector; // Connector for communication with the control system
    Audi audi;                                // Object to compute steering commands
    
    // Constructor initializes key parameters with default values.
    AckermanInternal() 
      : tolerance(0.5), distanceToGoalVal(0.0), timeToGoalVal(0.0),
        totalDistance(0.0), totalTime(0.0) {
        cmd.seq = 0;
    }
};

//-----------------------------------------------------------------------
// Global instance of AckermanInternal used in this module.
//-----------------------------------------------------------------------
static AckermanInternal ackermanData;

//-----------------------------------------------------------------------
// Constructor: Ackerman
// Description: Initializes the Ackerman control module and sets the 
//              platform type.
//-----------------------------------------------------------------------
Ackerman::Ackerman() {
    // Set platform type for this Ackerman vehicle.
    setPlatformType(pfms::PlatformType::ACKERMAN);
    
    // Initialize the connector if it hasn't been already.
    if (!ackermanData.connector)
        ackermanData.connector = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
}

//-----------------------------------------------------------------------
// Method: setGoal
// Description: Sets the target goal for the Ackerman vehicle and computes 
//              the initial steering parameters.
// Returns: true if steering computation is successful; false otherwise.
//-----------------------------------------------------------------------
bool Ackerman::setGoal(pfms::geometry_msgs::Point goal) {
    double steering;
    double distance;
    
    // Update current odometry data.
    bool OK = ackermanData.connector->read(ackermanData.odom);
    ackermanData.goal = goal;
    
    // Compute steering angle and distance using the Audi object.
    OK = ackermanData.audi.computeSteering(ackermanData.odom, ackermanData.goal, steering, distance);
    if (!OK) 
        return false;
        
    return true;
}

//-----------------------------------------------------------------------
// Method: reachGoal
// Description: Uses a state machine to drive the vehicle toward the goal.
//              Transitions through IDLE, ACCELERATE, COAST, and STOPPING states.
// Returns: true if the goal is reached; false if an error occurs.
//-----------------------------------------------------------------------
bool Ackerman::reachGoal(void) {
    // Initialize state machine to IDLE.
    control::State state = control::IDLE;
    bool goalReached = false;
    double distance = 0.0;
    double steering = 0.0;
    
    // Main control loop.
    while (!goalReached) {
        // Update odometry; exit if reading fails.
        if (!ackermanData.connector->read(ackermanData.odom)) {
            std::cout << "Ackerman: Unable to read odometry. Check simulation." << std::endl;
            return false;
        }
        
        // Compute the required steering and distance to the goal.
        bool ok = ackermanData.audi.computeSteering(ackermanData.odom, ackermanData.goal, steering, distance);
        if (!ok) {
            std::cout << "Ackerman: Unable to compute steering. Switching to STOPPING state." << std::endl;
            state = control::STOPPING;
        }
        
        // Process state transitions and command generation.
        switch(state) {
            case control::IDLE:
                // If far enough, switch to accelerating.
                if (distance > ackermanData.tolerance)
                    state = control::ACCELERATE;
                break;
                
            case control::ACCELERATE:
                // Transition to coast state when within a near range.
                if (distance < 5.0 * ackermanData.tolerance)
                    state = control::COAST;
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.1;
                }
                break;
                
            case control::COAST:
                // If very close, switch to stopping.
                if (distance < 2.0 * ackermanData.tolerance)
                    state = control::STOPPING;
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.05;
                }
                break;
                
            case control::STOPPING:
                // Apply maximum brake if within tolerance.
                if (distance < ackermanData.tolerance) {
                    goalReached = true;
                    ackermanData.cmd.brake = 9000; // Maximum braking torque.
                    ackermanData.cmd.throttle = 0.0;
                    ackermanData.cmd.steering = 0.0;
                }
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.1;
                }
                break;
                
            default:
                break;
        }
        
        // Send the computed command to the vehicle.
        ackermanData.connector->send(ackermanData.cmd);
        ackermanData.cmd.seq++;  // Increment command sequence number.
        
        // Wait 20 milliseconds before next loop iteration.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        // Debug output showing current state, distance, and steering.
        // std::cout << state << " Distance: " << distance << " steering: " << steering << std::endl;
    }
    
    // Update total distance traveled.
    ackermanData.totalDistance += distance;
    return goalReached;
}

//-----------------------------------------------------------------------
// Method: distanceToGoal
// Description: Computes the remaining distance to the target goal.
// Returns: The computed distance value.
//-----------------------------------------------------------------------
double Ackerman::distanceToGoal(void) {
    // Update the odometry reading.
    bool OK = ackermanData.connector->read(ackermanData.odom);
    
    // Compute the distance and estimated time to the goal.
    OK = ackermanData.audi.checkOriginToDestination(ackermanData.odom, 
                                                      ackermanData.goal, 
                                                      ackermanData.distanceToGoalVal, 
                                                      ackermanData.timeToGoalVal, 
                                                      ackermanData.odom);
    // std::cout << "distance : " << ackermanData.distanceToGoalVal << std::endl;
    return ackermanData.distanceToGoalVal;
}

//-----------------------------------------------------------------------
// Method: timeToGoal
// Description: Estimates the time required to reach the goal based on a 
//              constant speed model.
// Returns: The estimated time.
//-----------------------------------------------------------------------
double Ackerman::timeToGoal(void) {
    ackermanData.timeToGoalVal = ackermanData.distanceToGoalVal / 2.91;
    return ackermanData.timeToGoalVal;
}

//-----------------------------------------------------------------------
// Method: setTolerance
// Description: Sets the tolerance level for goal attainment.
// Returns: true after setting the value.
//-----------------------------------------------------------------------
bool Ackerman::setTolerance(double tolerance) {
    ackermanData.tolerance = tolerance;
    return true;
}

//-----------------------------------------------------------------------
// Method: distanceTravelled
// Description: Returns the cumulative distance traveled by the vehicle.
//-----------------------------------------------------------------------
double Ackerman::distanceTravelled(void) {
    return ackermanData.totalDistance;
}

//-----------------------------------------------------------------------
// Method: timeInMotion
// Description: Returns the cumulative time the vehicle has been in motion.
//-----------------------------------------------------------------------
double Ackerman::timeInMotion(void) {
    return ackermanData.totalTime;
}

//-----------------------------------------------------------------------
// Method: getOdometry
// Description: Retrieves the latest odometry data.
//-----------------------------------------------------------------------
pfms::nav_msgs::Odometry Ackerman::getOdometry(void) {
    return ackermanData.odom;
}

//-----------------------------------------------------------------------
// Method: checkOriginToDestination
// Description: Computes the Euclidean distance between the origin and the 
//              goal, estimates the travel time, and sets the estimated goal pose.
// Parameters:
//  - origin: Current odometry data.
//  - goal: Target position.
//  - distance: (Output) Computed distance value.
//  - time: (Output) Estimated time based on constant speed.
//  - estimatedGoalPose: (Output) Estimated pose at the goal.
// Returns: true if the computation is successful.
//-----------------------------------------------------------------------
bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    // Compute the Euclidean distance.
    distance = std::hypot(goal.x - origin.position.x, goal.y - origin.position.y);
    
    // Estimate the time using a constant speed (2.91 m/s).
    time = distance / 2.91;
    
    // Update the estimated goal pose.
    estimatedGoalPose = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw = std::atan2(goal.y - origin.position.y, goal.x - origin.position.x);
    return true;
}
