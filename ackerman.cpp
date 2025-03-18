#include "ackerman.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// For state machine control
namespace control {
    enum State {
        IDLE,
        ACCELERATE,
        COAST,
        STOPPING
    };
}

// Overload operator<< for state debugging.
std::ostream& operator<<(std::ostream& os, control::State state) {
    switch (state) {
        case control::IDLE: os << "IDLE "; break;
        case control::ACCELERATE: os << "ACCELERATE "; break;
        case control::COAST: os << "COAST "; break;
        case control::STOPPING: os << "STOPPING "; break;
    }
    return os;
}

// Internal helper class to hold Ackerman-specific state.
class AckermanInternal {
public:
    pfms::nav_msgs::Odometry odom;
    pfms::geometry_msgs::Point goal;
    double tolerance;
    double distanceToGoalVal;
    double timeToGoalVal;
    double totalDistance;
    double totalTime;
    pfms::commands::Ackerman cmd;
    std::shared_ptr<PfmsConnector> connector;
    Audi audi;
    
    AckermanInternal() : tolerance(0.5), distanceToGoalVal(0.0), timeToGoalVal(0.0),
                           totalDistance(0.0), totalTime(0.0) {
        cmd.seq = 0;
    }
};

// Single instance for this example.
static AckermanInternal ackermanData;

Ackerman::Ackerman() {
    // Set platform type to ACKERMAN.
    setPlatformType(pfms::PlatformType::ACKERMAN);
    if (!ackermanData.connector)
        ackermanData.connector = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
}

bool Ackerman::setGoal(pfms::geometry_msgs::Point goal) {
    ackermanData.goal = goal;
    return true;
}

bool Ackerman::reachGoal(void) {
    control::State state = control::IDLE;
    bool goalReached = false;
    double distance = 0.0;
    double steering = 0.0;
    
    while (!goalReached) {
        if (!ackermanData.connector->read(ackermanData.odom)) {
            std::cout << "Ackerman: Unable to read odometry. Check simulation." << std::endl;
            return false;
        }
        
        bool ok = ackermanData.audi.computeSteering(ackermanData.odom, ackermanData.goal, steering, distance);
        if (!ok) {
            std::cout << "Ackerman: Unable to compute steering. Switching to STOPPING state." << std::endl;
            state = control::STOPPING;
        }
        
        switch(state) {
            case control::IDLE:
                if (distance > ackermanData.tolerance)
                    state = control::ACCELERATE;
                break;
            case control::ACCELERATE:
                if (distance < 5.0 * ackermanData.tolerance)
                    state = control::COAST;
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.1;
                }
                break;
            case control::COAST:
                if (distance < 2.0 * ackermanData.tolerance)
                    state = control::STOPPING;
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.05;
                }
                break;
            case control::STOPPING:
                // Check both distance and near-zero forward velocity.
                if (distance < std::fabs(ackermanData.odom.linear.x) < 0.1) {
                    goalReached = true;
                    ackermanData.cmd.brake = 9000; // maximum braking torque
                    ackermanData.cmd.throttle = 0.0;
                    ackermanData.cmd.steering = 0.0;
                }
                else {
                    ackermanData.cmd.brake = 0.0;
                    ackermanData.cmd.steering = steering;
                    ackermanData.cmd.throttle = 0.05;
                }
                break;
            default:
                break;
        }
        
        ackermanData.connector->send(ackermanData.cmd);
        ackermanData.cmd.seq++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        // std::cout << state << " Distance: " << distance << std::endl;
        // std::cout << state << " callback: " << goalReached << std::endl;
    }
    
    ackermanData.totalDistance += distance;
    return goalReached;
}

double Ackerman::distanceToGoal(void) {
    return ackermanData.distanceToGoalVal;
}

double Ackerman::timeToGoal(void) {
    return ackermanData.timeToGoalVal;
}

bool Ackerman::setTolerance(double tolerance) {
    ackermanData.tolerance = tolerance;
    return true;
}

double Ackerman::distanceTravelled(void) {
    return ackermanData.totalDistance;
}

double Ackerman::timeInMotion(void) {
    return ackermanData.totalTime;
}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void) {
    return ackermanData.odom;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    // Compute Euclidean distance between origin and goal.
    distance = std::hypot(goal.x - origin.position.x, goal.y - origin.position.y);
    // Estimate time using a constant speed (e.g., 2.91 m/s).
    time = distance / 2.91;
    // Set estimated goal pose.
    estimatedGoalPose = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw = std::atan2(goal.y - origin.position.y, goal.x - origin.position.x);
    return true;
}
