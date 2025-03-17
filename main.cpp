#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include <vector>
#include "pfms_types.h"
#include <iostream>
#include <cstdlib>

using std::vector;
using std::cout;
using std::endl;

int main(int argc, char *argv[]) {

    double x1 = 2;
    double y1 = 5;
    double x2 = 4;
    double y2 = -5;

    if(argc != 3){
         cout << "Not enough arguments provided." << endl;
         cout << "Usage: " << argv[0] << " <x> <y>" << endl;
         cout << "Defaulting to: " << x1 << " " << y1  << " " << x2 << " " << y2 << endl;
    }
    else{
         x1 = atof(argv[1]);
         y1 = atof(argv[2]);
         x2 = atof(argv[1]);
         y2 = atof(argv[2]);

         cout << "Goals: " << x1 << " " << y1  << " " << x2 << " " << y2 << endl;
    }
    
    vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new SkidSteer());

    // Setting tolerance to reach goals
    controllers[0]->setTolerance(0.5);
    controllers.at(0)->setTolerance(0.5);
    controllers.at(1)->setTolerance(0.5);

    // Goals
    pfms::geometry_msgs::Point goal0 { x1, y1, 0 };
    pfms::geometry_msgs::Point goal1 { x2, y2, 0 };

    vector<pfms::geometry_msgs::Point> goals;
    goals.push_back(goal0);
    goals.push_back(goal1);

    // Mission coordination
    Mission mission(controllers);
    mission.setGoals(goals);
    mission.setMissionObjective(mission::Objective::DISTANCE);

    vector<unsigned int> assignment = mission.getPlatformGoalAssociation();
    for(unsigned int i = 0; i < assignment.size(); i++){
        cout << i << " : " << assignment.at(i) << endl;
    }

    bool OK = mission.runMission();

    if(OK){
        cout << "Controllers reached goals." << endl;
    }
    else {
        cout << "Controllers could NOT reach goals." << endl;
    }

    // Cleanup: Free allocated controllers.
    for(auto ctrl : controllers) {
        delete ctrl;
    }

    return 0;
}
