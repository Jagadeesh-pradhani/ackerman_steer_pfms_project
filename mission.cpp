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
    std::vector<unsigned int> platformGoalAssociation;
    for (size_t i = 0; i < goals_.size(); i++) {
        unsigned int controllerIndex = i % controllers_.size();
        controllers_[controllerIndex]->setGoal(goals_[i]);
        bool reached = controllers_[controllerIndex]->reachGoal();
        if (!reached) {
            std::cout << "Mission aborted: goal " << i << " not reached." << std::endl;
            return false;
        }
        platformGoalAssociation.push_back(controllerIndex);
    }
    // Optionally, store or print platformGoalAssociation.
    return true;
}

void Mission::setMissionObjective(mission::Objective objective) {
    // For now, we do not alter behavior based on the objective.
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
    // For simplicity, return an empty vector.
    return std::vector<unsigned int>();
}
