#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"
#include <memory>

//------------------------------------------------------------------------------
// Class: Ackerman
// Description: Implements vehicle control using an Ackerman steering model.
//------------------------------------------------------------------------------
class Ackerman: public Controller
{
public:
  // Constructor: Initializes default parameters and communication resources.
  Ackerman();
  
  // Destructor: Cleans up resources.
  ~Ackerman() override;

  // Sets the target goal for the vehicle.
  bool setGoal(pfms::geometry_msgs::Point goal) override;
  
  // Drives the vehicle toward the goal using a state machine.
  bool reachGoal(void) override;
  
  // Computes and returns the remaining distance to the goal.
  double distanceToGoal(void) override;
  
  // Estimates and returns the time required to reach the goal.
  double timeToGoal(void) override;
  
  // Sets the tolerance threshold for goal attainment.
  bool setTolerance(double tolerance) override;
  
  // Returns the cumulative distance traveled.
  double distanceTravelled(void) override;
  
  // Returns the cumulative time the vehicle has been in motion.
  double timeInMotion(void) override;
  
  // Retrieves the latest odometry data.
  pfms::nav_msgs::Odometry getOdometry(void) override;

private:
  // Latest odometry reading.
  pfms::nav_msgs::Odometry odom;
  
  // Target goal position.
  pfms::geometry_msgs::Point goal;
  
  // Tolerance threshold for determining goal achievement.
  double tolerance;
  
  // Computed distance to the goal.
  double distanceToGoalVal;
  
  // Estimated time to reach the goal.
  double timeToGoalVal;
  
  // Total distance traveled.
  double totalDistance;
  
  // Total time the vehicle has been in motion.
  double totalTime;
  
  // Command message to be sent to the platform.
  pfms::commands::Ackerman cmd;
  
  // Connector for communication with the control system.
  std::shared_ptr<PfmsConnector> connector;
  
  // Object for computing steering commands.
  Audi audi;
};

#endif // ACKERMAN_H
