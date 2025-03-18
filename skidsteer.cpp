#include "skidsteer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// All numeric constants are declared here.
namespace {
    const double DEFAULT_TOLERANCE       = 0.5;
    const double ANGULAR_ERROR_THRESHOLD = 0.1;
    const double KP_ANG                  = 3.0;   // Gain for angular control.
    const double KP_MOVE                 = 0.2;   // Gain for forward speed in MOVE state.
    const double KP_COAST                = 0.2;   // Gain for forward speed in COAST state.
    const double MAX_MOVE_SPEED          = 0.5;   // Maximum forward speed in MOVE state.
    const double MAX_COAST_SPEED         = 0.3;   // Maximum forward speed in COAST state.
    const double COAST_MULTIPLIER        = 2.0;   // Multiplier for tolerance to determine coast threshold.
    const int    LOOP_SLEEP_MS           = 20;    // Loop sleep time in milliseconds.
}

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
    
    SkidSteerInternal() 
      : tolerance(DEFAULT_TOLERANCE), totalDistance(0.0), totalTime(0.0) {
        cmd.seq = 0;
    }
};

// Single instance for this example.
static SkidSteerInternal skidData;

SkidSteer::SkidSteer() {
    skidData.tolerance = DEFAULT_TOLERANCE;
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
    
    // Define state machine states.
    enum class State { IDLE, ROTATE, MOVE, COAST, STOPPING };
    State state = State::IDLE;
    
    // Calculate coast threshold.
    const double coastThreshold = COAST_MULTIPLIER * skidData.tolerance;
    
    while (!goalReached) {
        if (!skidData.connector->read(skidData.odom)) {
            std::cout << "SkidSteer: Unable to read odometry." << std::endl;
            return false;
        }
        
        // Compute distance to goal.
        distance = std::hypot(skidData.goal.x - skidData.odom.position.x,
                              skidData.goal.y - skidData.odom.position.y);
        
        // Compute desired heading and angular error.
        double desiredHeading = std::atan2(skidData.goal.y - skidData.odom.position.y,
                                           skidData.goal.x - skidData.odom.position.x);
        double currentHeading = skidData.odom.yaw;
        angularError = desiredHeading - currentHeading;
        while (angularError > M_PI)  angularError -= 2 * M_PI;
        while (angularError < -M_PI) angularError += 2 * M_PI;
        
        // Update state machine transitions.
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
        
        skidData.connector->send(skidData.cmd);
        skidData.cmd.seq++;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
        
        // std::cout << "SkidSteer State: " 
        //           << (state == State::IDLE ? "IDLE" : state == State::ROTATE ? "ROTATE" : state == State::MOVE ? "MOVE" : state == State::COAST ? "COAST" : "STOPPING")
        //           << ", Distance: " << distance 
        //           << ", Angular Error: " << angularError << std::endl;
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
