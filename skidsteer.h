#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"
#include <memory>

//------------------------------------------------------------------------------
// Class: SkidSteer
// Description: Implements control for a skid-steer vehicle.
//------------------------------------------------------------------------------
class SkidSteer: public Controller
{
public:
  // Constructor: Initializes default parameters and communication resources.
  SkidSteer();
  
  // Destructor: Cleans up resources.
  ~SkidSteer() override;

  // Sets the desired target position.
  bool setGoal(pfms::geometry_msgs::Point goal) override;
  
  // Drives the vehicle toward the goal using a state machine.
  bool reachGoal(void) override;
  
  // Computes and returns the remaining distance to the goal.
  double distanceToGoal(void) override;
  
  // Estimates the total time required to reach the goal.
  double timeToGoal(void) override;
  
  // Sets the tolerance threshold for goal proximity.
  bool setTolerance(double tolerance) override;
  
  // Returns the total distance traveled.
  double distanceTravelled(void) override;
  
  // Returns the total time the vehicle has been in motion.
  double timeInMotion(void) override;
  
  // Retrieves the latest odometry data.
  pfms::nav_msgs::Odometry getOdometry(void) override;



private:
  // Latest odometry reading.
  pfms::nav_msgs::Odometry odom;
  
  // Target goal position.
  pfms::geometry_msgs::Point goal;
  
  // Tolerance for goal attainment.
  double tolerance;
  
  // Total distance traveled.
  double totalDistance;
  
  // Computed distance to the goal.
  double distanceToGoalVal;
  
  // Total time in motion.
  double totalTime;
  
  // Estimated time to reach the goal.
  double timeToGoalVal;
  
  // Movement scaling parameters.
  double rotate_right_;   // Scale factor for right rotation.
  double rotate_left_;    // Scale factor for left rotation.
  double move_foward_;    // Scale factor for forward motion.
  double move_reverse_;   // Scale factor for reverse motion.
  
  // Command message for skid-steer control.
  pfms::commands::SkidSteer cmd;
  
  // Connector for communication with the vehicle.
  std::shared_ptr<PfmsConnector> connector;
};

#endif // SKIDSTEER_H
