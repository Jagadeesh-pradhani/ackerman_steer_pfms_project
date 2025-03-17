#ifndef MISION_TEST_H
#define MISION_TEST_H

#include "gtest/gtest.h"
#include "pfmshog.h"

// The below is to setup our Unit Tests, Not required to understand, skip to wheer tests start
class MissionTest : public testing::Test {
 protected:
  // Per-test-suite set-up.
  // Called before the first test in this test suite.
  // Can be omitted if not needed.
  static void SetUpTestCase() {
    // Avoid reallocating static objects if called in subclasses of FooTest.
    if (pfmsHogPtr_ == nullptr) {
      pfmsHogPtr_ = new PfmsHog(pfms::PlatformType::ACKERMAN);
    }
  }

  // Per-test-suite tear-down.
  // Called after the last test in this test suite.
  // Can be omitted if not needed.
  static void TearDownTestCase() {
      if (pfmsHogPtr_ != nullptr){
        delete pfmsHogPtr_;
        pfmsHogPtr_ = nullptr;
      }
  }

  static PfmsHog* pfmsHogPtr_;

};

PfmsHog* MissionTest::pfmsHogPtr_ = nullptr;


#endif // MISSION_TEST_H
