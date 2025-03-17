set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(MYDEBUG CACHE STRING "Additional definition" ON) # Needed to get RCLCPP headers in

find_package(pfms2ros 3.0.7 REQUIRED)

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp" "../mission.cpp")
message ( STATUS "Compiling student with following files  ${SRC_LIST}" )
add_library(example SHARED ${SRC_LIST})
target_link_libraries(example pfmsconnector audi)

add_dependencies(example pfms2ros::pfmsconnector)
link_directories(/usr/local/lib)

# This uses the complete student's library
# CMake will create the cell library that you will need 
add_library(mission_test "../mission.cpp")
target_link_libraries(mission_test pfmsconnector)

# Now we make the gtests
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

message(STATUS "Found include dir ${GTEST_INCLUDE_DIRS}")
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(testAudi test_audi.cpp)
target_link_libraries(testAudi ${GTEST_LIBRARIES} pfmshog example pthread)

add_executable(testHusky test_husky.cpp)
target_link_libraries(testHusky ${GTEST_LIBRARIES} pfmshog example pthread)

add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pfmshog example pthread)

add_executable(testHuskyReachGoals test_husky_reach_goals.cpp)
target_link_libraries(testHuskyReachGoals ${GTEST_LIBRARIES} pfmshog example pthread)

add_executable(testMission test_mission.cpp)
target_link_libraries(testMission ${GTEST_LIBRARIES} pfmshog example pthread mission_test)

