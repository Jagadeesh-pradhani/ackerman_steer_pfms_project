#include "skidsteer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// Internal helper class for SkidSteer state.
class SkidSteerInternal {
public:
    pfms::nav_msgs::Odometry odom;
    pfms::geometry_msgs::Point goal;
    double tolerance;
    double totalDistance;
    double totalTime;
    pfms::commands::SkidSteer cmd;
    std::shared_ptr<PfmsConnector> connector;
    
    SkidSteerInternal() : tolerance(0.5), totalDistance(0.0), totalTime(0.0) {
        cmd.seq = 0;
    }
};

// Single instance for this example.
static SkidSteerInternal skidData;

SkidSteer::SkidSteer() {
    skidData.tolerance = 0.5;
    if (!skidData.connector)
        skidData.connector = std::make_shared<PfmsConnector>(pfms::PlatformType::SKIDSTEER);
}

bool SkidSteer::setGoal(pfms::geometry_msgs::Point goal) {
    skidData.goal = goal;
    return true;
}

bool SkidSteer::reachGoal(void) {
    bool goalReached = false;
    double distance = 0.0;
    double angularError = 0.0;
    double kP = 1.0; // Proportional gain for angular control.
    
    while (!goalReached) {
        if (!skidData.connector->read(skidData.odom)) {
            std::cout << "SkidSteer: Unable to read odometry." << std::endl;
            return false;
        }
        
        // Compute distance using odom.position
        distance = std::hypot(skidData.goal.x - skidData.odom.position.x,
                              skidData.goal.y - skidData.odom.position.y);
        
        // Compute desired heading.
        double desiredHeading = std::atan2(skidData.goal.y - skidData.odom.position.y,
                                           skidData.goal.x - skidData.odom.position.x);
        double currentHeading = skidData.odom.yaw;
        angularError = desiredHeading - currentHeading;
        while (angularError > M_PI) angularError -= 2 * M_PI;
        while (angularError < -M_PI) angularError += 2 * M_PI;
        
        if (std::fabs(angularError) > 0.1) {
            skidData.cmd.move_f_b = 0.0;
            skidData.cmd.turn_l_r = kP * angularError;
        } else {
            skidData.cmd.move_f_b = 0.1;
            skidData.cmd.turn_l_r = 0.0;
        }
        
        if (distance < skidData.tolerance) {
            goalReached = true;
            skidData.cmd.move_f_b = 0.0;
            skidData.cmd.turn_l_r = 0.0;
        }
        
        skidData.connector->send(skidData.cmd);
        skidData.cmd.seq++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::cout << "SkidSteer: Distance: " << distance << ", Angular Error: " << angularError << std::endl;
    }
    
    skidData.totalDistance += distance;
    return goalReached;
}

double SkidSteer::distanceToGoal(void) {
    return std::hypot(skidData.goal.x - skidData.odom.position.x,
                      skidData.goal.y - skidData.odom.position.y);
}

double SkidSteer::timeToGoal(void) {
    // Placeholder for time estimation.
    return 0.0;
}

bool SkidSteer::setTolerance(double tolerance) {
    skidData.tolerance = tolerance;
    return true;
}

double SkidSteer::distanceTravelled(void) {
    return skidData.totalDistance;
}

double SkidSteer::timeInMotion(void) {
    return skidData.totalTime;
}

pfms::nav_msgs::Odometry SkidSteer::getOdometry(void) {
    return skidData.odom;
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    distance = std::hypot(goal.x - origin.position.x, goal.y - origin.position.y);
    time = distance / 2.91; // constant speed estimate
    estimatedGoalPose = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw = std::atan2(goal.y - origin.position.y, goal.x - origin.position.x);
    return true;
}
