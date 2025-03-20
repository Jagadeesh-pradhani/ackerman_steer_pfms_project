#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"

class Controller: public ControllerInterface
{
public:
  // Default constructor should set all attributes to a default value
  Controller();
  
  // Virtual destructor to ensure proper cleanup in derived classes
  virtual ~Controller();

  /**
  * Reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  * @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  virtual bool reachGoal(void) = 0;

  /**
  * Setter for goal, the function will update internal variables associated with @sa timeToGoal
  * and @sa distanceToGoal
  * @return goal reachable
  */
  virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;

  /**
  * Getter for platform type
  * @return PlatformType
  */
  pfms::PlatformType getPlatformType(void) override; // in controller

  /**
  * Getter for distance to be travelled to reach goal, updates as the platform moves to current goal
  * @return distance to be travelled to goal [m]
  */
  virtual double distanceToGoal(void) = 0;

  /**
  * Getter for time to reach goal, updates as the platform moves to current goal
  * @return time to travel to goal [s]
  */
  virtual double timeToGoal(void) = 0;

  /**
  * Set tolerance when reaching goal
  * @return tolerance accepted [m]
  */
  virtual bool setTolerance(double tolerance) = 0;  // in controller

  /**
  * Returns total distance travelled by platform
  * @return total distance travelled since started [m]
  */
  virtual double distanceTravelled(void) = 0;  // in controller

  /**
  * Returns total time in motion by platform, time when stationary not included
  * @return total time in motion since started [s]
  */
  virtual double timeInMotion(void) = 0;  // in controller?

  /**
  * Returns current odometry information
  * @return odometry - current pose (x, y, yaw) and velocity (vx, vy)
  */
  virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

  // See controllerinterface.h for more information

private:
  pfms::PlatformType platform_type_;

protected:
  // Allow derived classes to set the platform type.
  void setPlatformType(pfms::PlatformType type) { platform_type_ = type; }
};

#endif // CONTROLLER_H