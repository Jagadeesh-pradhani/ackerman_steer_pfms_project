#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"

class Mission: public MissionInterface
{
public:
    /**
    * The Default constructor
    * @sa ControllerInterface and @sa MissionInterface for more information
    */
    Mission(std::vector<ControllerInterface*> controllers);

    // Overridden functions from MissionInterface
    virtual void setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
    virtual bool runMission() override;
    virtual void setMissionObjective(mission::Objective objective) override;
    virtual std::vector<double> getDistanceTravelled() override;
    virtual std::vector<double> getTimeMoving() override;
    virtual std::vector<unsigned int> getPlatformGoalAssociation() override;

private:
    std::vector<ControllerInterface*> controllers_; // copy of ControllerInterfaces
    std::vector<pfms::geometry_msgs::Point> goals_;   // copy of goals
};

#endif // MISSION_H
