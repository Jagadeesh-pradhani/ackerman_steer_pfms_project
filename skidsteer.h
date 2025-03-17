#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  // Default constructor should set all attributes to a default value
  SkidSteer();

  // Override virtual functions from ControllerInterface
  virtual bool reachGoal(void) override;
  virtual bool setGoal(pfms::geometry_msgs::Point goal) override;
  virtual double distanceToGoal(void) override;
  virtual double timeToGoal(void) override;
  virtual bool setTolerance(double tolerance) override;
  virtual double distanceTravelled(void) override;
  virtual double timeInMotion(void) override;
  virtual pfms::nav_msgs::Odometry getOdometry(void) override;

  // Added: Override checkOriginToDestination
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) override;
};

#endif // SKIDSTEER_H
