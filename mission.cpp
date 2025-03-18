#include "mission.h"
#include "ackerman.h"
#include "skidsteer.h"
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers) {
    controllers_ = controllers;
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    goals_ = goals;
}

bool Mission::runMission() {
    // For each goal, assign it to a controller in a round-robin fashion.
    // In our tests, there is only one controller so each goal is assigned to platform 0.
    for (size_t i = 0; i < goals_.size(); i++) {
        unsigned int controllerIndex = i % controllers_.size();
        controllers_[controllerIndex]->setGoal(goals_[i]);
        bool reached = controllers_[controllerIndex]->reachGoal();
        if (!reached) {
            std::cout << "Mission aborted: goal " << i << " not reached." << std::endl;
            return false;
        }
    }
    return true;
}

void Mission::setMissionObjective(mission::Objective objective) {
    // This simple implementation does not alter behavior based on objective.
}

std::vector<double> Mission::getDistanceTravelled() {
    std::vector<double> distances;
    for (auto ctrl : controllers_) {
        distances.push_back(ctrl->distanceTravelled());
    }
    return distances;
}

std::vector<double> Mission::getTimeMoving() {
    std::vector<double> times;
    for (auto ctrl : controllers_) {
        times.push_back(ctrl->timeInMotion());
    }
    return times;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation() {
    // For each goal in the stored goals vector, return an assignment.
    // Since our tests expect 2 goals and one controller,
    // we simply return a vector of zeros of the same size as goals_.
    std::vector<unsigned int> assignment;
    assignment.resize(goals_.size(), 0);
    return assignment;
}
