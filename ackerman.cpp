#include "ackerman.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "pfmsconnector.h"

//------------------------------------------------------------------------------
// Namespace: control
// Description: Enumerates possible states for the Ackerman vehicle control.
//------------------------------------------------------------------------------
namespace control {
    enum State {
        IDLE,         // Vehicle is idle
        ACCELERATE,   // Vehicle is accelerating
        COAST,        // Vehicle is coasting
        STOPPING      // Vehicle is stopping
    };
}

//------------------------------------------------------------------------------
// Overloaded operator<< for printing control::State values for debugging.
//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, control::State state) {
    switch (state) {
        case control::IDLE:         os << "IDLE "; break;
        case control::ACCELERATE:   os << "ACCELERATE "; break;
        case control::COAST:        os << "COAST "; break;
        case control::STOPPING:     os << "STOPPING "; break;
        default: break;
    }
    return os;
}

//------------------------------------------------------------------------------
// Ackerman Constructor
// Description: Initializes default internal variables and sets up the
//              communication connector.
//------------------------------------------------------------------------------
Ackerman::Ackerman()
  : tolerance(0.5), distanceToGoalVal(0.0), timeToGoalVal(0.0),
    totalDistance(0.0), totalTime(0.0)
{
    // Initialize the command sequence number.
    cmd.seq = 0;
    
    // Create the connector for the Ackerman platform.
    connector = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
    
    // Set platform type using Controller method.
    setPlatformType(pfms::PlatformType::ACKERMAN);
}

//------------------------------------------------------------------------------
// Ackerman Destructor
// Description: Cleans up allocated resources (connector, etc.).
//------------------------------------------------------------------------------
Ackerman::~Ackerman() {
    // Explicitly reset the connector.
    if (connector) {
        connector.reset();
    }
}

//------------------------------------------------------------------------------
// Method: setGoal
// Description: Sets the target goal and computes initial steering parameters.
// Returns: true if steering computation is successful; false otherwise.
//------------------------------------------------------------------------------
bool Ackerman::setGoal(pfms::geometry_msgs::Point goalPoint) {
    double steering = 0.0;
    double distance = 0.0;
    
    // Update the current odometry.
    bool OK = connector->read(odom);
    
    // Set the goal position.
    goal = goalPoint;
    
    // Compute the initial steering angle and distance using the Audi object.
    OK = audi.computeSteering(odom, goal, steering, distance);
    if (!OK)
        return false;
        
    return true;
}

//------------------------------------------------------------------------------
// Method: reachGoal
// Description: Uses a state machine to drive the vehicle toward the goal.
// Returns: true if the goal is reached; false if an error occurs.
//------------------------------------------------------------------------------
bool Ackerman::reachGoal(void) {
    // Initialize state machine to IDLE.
    control::State state = control::IDLE;
    bool goalReached = false;
    double distance = 0.0;
    double steering = 0.0;
    
    // Main control loop.
    while (!goalReached) {
        // Update odometry; exit if reading fails.
        if (!connector->read(odom)) {
            std::cout << "Ackerman: Unable to read odometry. Check simulation." << std::endl;
            return false;
        }
        
        // Compute required steering angle and distance to the goal.
        bool ok = audi.computeSteering(odom, goal, steering, distance);
        if (!ok) {
            std::cout << "Ackerman: Unable to compute steering. Switching to STOPPING state." << std::endl;
            state = control::STOPPING;
        }
        
        // Process state transitions and generate commands.
        switch(state) {
            case control::IDLE:
                if (distance > tolerance)
                    state = control::ACCELERATE;
                break;
                
            case control::ACCELERATE:
                if (distance < 5.0 * tolerance)
                    state = control::COAST;
                else {
                    cmd.brake = 0.0;
                    cmd.steering = steering;
                    cmd.throttle = 0.1;
                }
                break;
                
            case control::COAST:
                if (distance < 2.0 * tolerance)
                    state = control::STOPPING;
                else {
                    cmd.brake = 0.0;
                    cmd.steering = steering;
                    cmd.throttle = 0.05;
                }
                break;
                
            case control::STOPPING:
                if (distance < tolerance) {
                    goalReached = true;
                    cmd.brake = 9000; // Maximum braking torque.
                    cmd.throttle = 0.0;
                    cmd.steering = 0.0;
                } else {
                    cmd.brake = 0.0;
                    cmd.steering = steering;
                    cmd.throttle = 0.1;
                }
                break;
                
            default:
                break;
        }
        
        // Send the command to the vehicle.
        connector->send(cmd);
        cmd.seq++;  // Increment command sequence number.
        
        // Wait for 20 milliseconds before the next iteration.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // Update total distance traveled.
    totalDistance += distance;
    return goalReached;
}

//------------------------------------------------------------------------------
// Method: distanceToGoal
// Description: Computes the remaining distance to the target goal.
// Returns: The computed distance.
//------------------------------------------------------------------------------
double Ackerman::distanceToGoal(void) {
    // Update odometry.
    bool OK = connector->read(odom);
    
    // Compute distance and estimated time using Audi helper method.
    OK = audi.checkOriginToDestination(odom, goal, distanceToGoalVal, timeToGoalVal, odom);
    return distanceToGoalVal;
}

//------------------------------------------------------------------------------
// Method: timeToGoal
// Description: Estimates the time required to reach the goal based on a 
//              constant speed model.
// Returns: The estimated time.
//------------------------------------------------------------------------------
double Ackerman::timeToGoal(void) {
    // For simplicity, a fixed speed model is used.
    timeToGoalVal = distanceToGoalVal / 2.91;
    return timeToGoalVal;
}

//------------------------------------------------------------------------------
// Method: setTolerance
// Description: Sets the tolerance level for goal attainment.
// Returns: true after the tolerance is set.
//------------------------------------------------------------------------------
bool Ackerman::setTolerance(double tol) {
    tolerance = tol;
    return true;
}

//------------------------------------------------------------------------------
// Method: distanceTravelled
// Description: Returns the total distance traveled.
//------------------------------------------------------------------------------
double Ackerman::distanceTravelled(void) {
    return totalDistance;
}

//------------------------------------------------------------------------------
// Method: timeInMotion
// Description: Returns the total time the vehicle has been in motion.
//------------------------------------------------------------------------------
double Ackerman::timeInMotion(void) {
    return totalTime;
}

//------------------------------------------------------------------------------
// Method: getOdometry
// Description: Retrieves the latest odometry reading.
//------------------------------------------------------------------------------
pfms::nav_msgs::Odometry Ackerman::getOdometry(void) {
    return odom;
}
