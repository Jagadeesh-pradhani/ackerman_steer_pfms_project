#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"

#include "pfms_types.h"
#include "pfmshog.h"
#include "a1_test_helper.h"
#include <cmath>

using std::cout;
using std::endl;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(MissionTest, Ackerman) {

    ///We will just move the husky out of the path of the audi
    std::shared_ptr<PfmsHog> pfmsHogSkidPtr = std::make_shared<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(-20,0,0);
        pfmsHogSkidPtr->teleport(odo);
    }    

    // Move Ackermna to Strating position
    pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsHog> pfmsHogAckPtr = std::make_shared<PfmsHog>(platform,true);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogAckPtr->teleport(odo);
    }



    //Set-up the controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);

    //Goals
    pfms::geometry_msgs::Point goal0{8,-1};
    pfms::geometry_msgs::Point goal1{-5,-2};

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back(goal0);
    goals.push_back(goal1);

    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::DISTANCE);
    mission.setGoals(goals);

    std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();
    ASSERT_EQ(assignment.size(),goals.size());//Need to have assigned a platform for each goal
    ASSERT_EQ(assignment.at(0),0); // Platform 0 should be going to goal 0
    ASSERT_EQ(assignment.at(1),0); // Platform 0 should be going to goal 1

    // We use pfmsHog to check the goal has been reached, and do so by firsts 
    // setting the goal for the PfmsHog to assess
    pfmsHogAckPtr->setGoals(goals);

    // This function will block (not return) until the function has finished
    bool missionOK = mission.runMission();
    // The mission should succseed
    EXPECT_TRUE(missionOK);

    {
        //We now check that the goal has been reached using the PfmsHog for Ackerman
        std::vector<double> stats;
        bool reachedCheck = pfmsHogAckPtr->checkGoalsReached(stats);
        ASSERT_TRUE(reachedCheck);

        //We can also check the distances as it passed each goal reported by PfmsHog
        ASSERT_EQ(stats.size(),3);
        ASSERT_NEAR(stats.at(0),0,1.0);
        ASSERT_NEAR(stats.at(1),0,1.0);
    }

}

TEST(MissionTest, Husky) {

    ///We will just move the husky out of the path of the audi
    std::shared_ptr<PfmsHog> pfmsHogSkidPtr = std::make_shared<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0,0,0);
        pfmsHogSkidPtr->teleport(odo);
    }    

    // Move Ackermna to Strating position
    pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsHog> pfmsHogAckPtr = std::make_shared<PfmsHog>(platform,true);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogAckPtr->teleport(odo);
    }



    //Set-up the controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);

    //Goals
    pfms::geometry_msgs::Point goal0{1,-1};
    pfms::geometry_msgs::Point goal1{-1,-1};

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back(goal0);
    goals.push_back(goal1);

    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::DISTANCE);
    mission.setGoals(goals);

    std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();
    ASSERT_EQ(assignment.size(),goals.size());//Need to have assigned a platform for each goal
    ASSERT_EQ(assignment.at(0),0); // Platform 0 should be going to goal 0
    ASSERT_EQ(assignment.at(1),0); // Platform 0 should be going to goal 1

    // We use pfmsHog to check the goal has been reached, and do so by firsts 
    // setting the goal for the PfmsHog to assess
    pfmsHogSkidPtr->setGoals(goals);

    // This function will block (not return) until the function has finished
    bool missionOK = mission.runMission();
    // The mission should succseed
    EXPECT_TRUE(missionOK);

    {
        //We now check that the goal has been reached using the PfmsHog for Ackerman
        std::vector<double> stats;
        bool reachedCheck = pfmsHogSkidPtr->checkGoalsReached(stats);
        ASSERT_TRUE(reachedCheck);

        //We can also check the distances as it passed each goal reported by PfmsHog
        ASSERT_EQ(stats.size(),3);
        ASSERT_NEAR(stats.at(0),0,1.0);
        ASSERT_NEAR(stats.at(1),0,1.0);
    }

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
