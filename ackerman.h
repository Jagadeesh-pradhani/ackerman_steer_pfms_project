#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  // Default constructor should set all attributes to a default value
  Ackerman();

  // Override virtual functions from ControllerInterface
  virtual bool reachGoal(void) override;
  virtual bool setGoal(pfms::geometry_msgs::Point goal) override;
  virtual double distanceToGoal(void) override;
  virtual double timeToGoal(void) override;
  virtual bool setTolerance(double tolerance) override;
  virtual double distanceTravelled(void) override;
  virtual double timeInMotion(void) override;
  virtual pfms::nav_msgs::Odometry getOdometry(void) override;

  // Added: Override checkOriginToDestination from ControllerInterface
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) override;
};

#endif // ACKERMAN_H
