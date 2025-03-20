#include "skidsteer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "pfmsconnector.h"

//------------------------------------------------------------------------------
// Anonymous Namespace: Constant Parameters for SkidSteer Control
//------------------------------------------------------------------------------
namespace {
    const double DEFAULT_TOLERANCE       = 0.5;   // Default tolerance (meters)
    const double ANGULAR_ERROR_THRESHOLD = 0.1;   // Acceptable angular error (radians)
    const double KP_ANG                  = 3.0;   // Proportional gain for angular control
    const double KP_MOVE                 = 0.2;   // Proportional gain for forward motion
    const double KP_COAST                = 0.2;   // Proportional gain for coasting
    const double MAX_MOVE_SPEED          = 1.0;   // Maximum speed in MOVE state
    const double MAX_COAST_SPEED         = 0.3;   // Maximum speed in COAST state
    const double COAST_MULTIPLIER        = 2.0;   // Multiplier to determine coast threshold
    const int    LOOP_SLEEP_MS           = 20;    // Loop cycle time in milliseconds
}

//------------------------------------------------------------------------------
// SkidSteer Constructor
// Description: Initializes default parameters and sets up communication.
//------------------------------------------------------------------------------
SkidSteer::SkidSteer()
  : tolerance(DEFAULT_TOLERANCE), totalDistance(0.0),
    distanceToGoalVal(0.0), totalTime(0.0), timeToGoalVal(0.0),
    rotate_right_(-1.0), rotate_left_(1.0), move_foward_(1.0), move_reverse_(-1.0)
{
    // Initialize command sequence.
    cmd.seq = 0;
    
    // Create the connector for the SkidSteer platform.
    connector = std::make_shared<PfmsConnector>(pfms::PlatformType::SKIDSTEER);
}

//------------------------------------------------------------------------------
// SkidSteer Destructor
// Description: Cleans up allocated resources.
//------------------------------------------------------------------------------
SkidSteer::~SkidSteer() {
    if (connector) {
        connector.reset();
    }
}

//------------------------------------------------------------------------------
// Method: setGoal
// Description: Sets the target goal for the vehicle.
// Returns: true after the goal is set.
//------------------------------------------------------------------------------
bool SkidSteer::setGoal(pfms::geometry_msgs::Point goalPoint) {
    goal = goalPoint;
    return true;
}

//------------------------------------------------------------------------------
// Method: reachGoal
// Description: Uses a state machine to drive the vehicle toward the target goal.
// Returns: true if the goal is reached successfully; false otherwise.
//------------------------------------------------------------------------------
bool SkidSteer::reachGoal(void) {
    bool goalReached = false;
    double distance = 0.0;
    double angularError = 0.0;

    // Define the control states.
    enum class State { IDLE, ROTATE, MOVE, COAST, STOPPING };
    State state = State::IDLE;

    // Calculate coast threshold based on tolerance.
    const double coastThreshold = COAST_MULTIPLIER * tolerance;

    // Main control loop.
    while (!goalReached) {
        // Read the latest odometry; exit if unsuccessful.
        if (!connector->read(odom)) {
            std::cout << "SkidSteer: Unable to read odometry." << std::endl;
            return false;
        }

        // Compute the Euclidean distance to the goal.
        distance = std::hypot(goal.x - odom.position.x, goal.y - odom.position.y);

        // Compute desired heading toward the goal.
        double desiredHeading = std::atan2(goal.y - odom.position.y, goal.x - odom.position.x);
        double currentHeading = odom.yaw;
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
                else if (distance > tolerance)
                    state = State::COAST;
                else
                    state = State::STOPPING;
                break;

            case State::ROTATE:
                if (std::fabs(angularError) <= ANGULAR_ERROR_THRESHOLD) {
                    if (distance > coastThreshold)
                        state = State::MOVE;
                    else if (distance > tolerance)
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
                if (distance <= tolerance)
                    state = State::STOPPING;
                else if (std::fabs(angularError) > ANGULAR_ERROR_THRESHOLD)
                    state = State::ROTATE;
                break;

            case State::STOPPING:
                // Zero out commands to stop the vehicle.
                cmd.move_f_b = 0.0;
                cmd.turn_l_r = 0.0;
                goalReached = true;
                break;
        }

        // Issue commands based on current state.
        if (state == State::ROTATE) {
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = KP_ANG * angularError;
        } else if (state == State::MOVE) {
            double forwardSpeed = KP_MOVE * distance;
            if (forwardSpeed > MAX_MOVE_SPEED)
                forwardSpeed = MAX_MOVE_SPEED;
            cmd.move_f_b = forwardSpeed;
            cmd.turn_l_r = 0.0;
        } else if (state == State::COAST) {
            double forwardSpeed = KP_COAST * distance;
            if (forwardSpeed > MAX_COAST_SPEED)
                forwardSpeed = MAX_COAST_SPEED;
            cmd.move_f_b = forwardSpeed;
            cmd.turn_l_r = 0.0;
        } else if (state == State::IDLE) {
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = 0.0;
        }
        // In STOPPING state, commands remain zero.

        // Send the command to the vehicle.
        connector->send(cmd);
        cmd.seq++;  // Increment command sequence number.

        // Sleep for a fixed loop interval.
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
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
double SkidSteer::distanceToGoal(void) {
    bool OK = connector->read(odom);
    if (!OK)
        std::cout << "SkidSteer: Unable to read odometry." << std::endl;

    distanceToGoalVal = std::sqrt(std::pow(odom.position.x - goal.x, 2) + std::pow(odom.position.y - goal.y, 2));
    return distanceToGoalVal;
}

//------------------------------------------------------------------------------
// Method: timeToGoal
// Description: Estimates the total time to reach the goal by combining rotation
//              and forward travel times.
// Returns: The estimated time.
//------------------------------------------------------------------------------
double SkidSteer::timeToGoal(void) {
    bool OK = connector->read(odom);

    // Compute the required rotation to align with the goal.
    double angle = std::atan2(goal.y - odom.position.y, goal.x - odom.position.x);
    double rotatingAngleError = std::fabs(odom.yaw - angle);

    // Compute the straight-line distance to the goal.
    double distanceToGoalCalc = std::hypot(odom.position.x - goal.x, odom.position.y - goal.y);

    // Estimate time: rotation time plus forward travel time.
    double timeEstimate = rotatingAngleError / rotate_left_ + distanceToGoalCalc / move_foward_;
    timeToGoalVal = timeEstimate;

    return timeToGoalVal;
}

//------------------------------------------------------------------------------
// Method: setTolerance
// Description: Updates the tolerance for determining goal proximity.
// Returns: true after the tolerance is set.
//------------------------------------------------------------------------------
bool SkidSteer::setTolerance(double tol) {
    tolerance = tol;
    return true;
}

//------------------------------------------------------------------------------
// Method: distanceTravelled
// Description: Returns the total distance traveled by the vehicle.
//------------------------------------------------------------------------------
double SkidSteer::distanceTravelled(void) {
    return totalDistance;
}

//------------------------------------------------------------------------------
// Method: timeInMotion
// Description: Returns the total time the vehicle has been in motion.
//------------------------------------------------------------------------------
double SkidSteer::timeInMotion(void) {
    return totalTime;
}

//------------------------------------------------------------------------------
// Method: getOdometry
// Description: Retrieves the latest odometry reading.
//------------------------------------------------------------------------------
pfms::nav_msgs::Odometry SkidSteer::getOdometry(void) {
    return odom;
}

