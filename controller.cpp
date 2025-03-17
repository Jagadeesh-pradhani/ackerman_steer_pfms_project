#include "controller.h"

// Constructor: Default, platform_type_ will be set by derived classes.
Controller::Controller() {
    // No initialization for platform_type_ here.
}

// Destructor
Controller::~Controller() {
    // Clean-up, if needed.
}

// This getter returns the stored platform type.
pfms::PlatformType Controller::getPlatformType(void) {
    return platform_type_;
}
