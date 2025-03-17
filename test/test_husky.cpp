#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "skidsteer.h"
#include "pfms_types.h"
#include <cmath>


// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "skidsteer_test.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST_F(SkidSteerTest, Simple) {

    //Starting point for husky vehicle
    Odometry odo = populateOdo(0,2,0);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    pfms::geometry_msgs::Point pt{10,0};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "SkidSteer: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    EXPECT_NEAR(dist,10.2646,0.2);
    EXPECT_NEAR(t,10.395,0.1);
}


TEST_F(SkidSteerTest, Simple2) {

    //Starting point for vehicle
    Odometry odo = populateOdo(0,2,0);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    pfms::geometry_msgs::Point pt{0,-6.5};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "SkidSteer: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    EXPECT_NEAR(dist,8.499,0.2);
    EXPECT_NEAR(t,10.07,0.2);

}

TEST_F(SkidSteerTest, Simple3) {

    //Starting point for vehicle
    Odometry odo = populateOdo(0,-6,-M_PI/2);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    pfms::geometry_msgs::Point pt {0,-5};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "SkidSteer: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    EXPECT_NEAR(dist,1.00,0.2);
    EXPECT_NEAR(t,4.1414,0.2);    
}


TEST_F(SkidSteerTest, Simple4) {

    Odometry odo = populateOdo(0.225,-5.306,127.825*M_PI/180);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    pfms::geometry_msgs::Point pt {1,4};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    ASSERT_NEAR(dist,9.33806,0.2);
    ASSERT_NEAR(t,10.0813,0.2);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
