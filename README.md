# ackerman_steer_pfms_project

The assignment uses a physics simulator called gazebo which is integrated into ROS. Students are provided with the simulator which is installed on your system as part of customisation of ubuntu for subject. Students do not need ROS knowledge for the assessment.  

Please upgrade the packages before starting to work on the assignment, executing two commands in the terminal 

```bash
sudo apt update
sudo apt install pfms-ros-integration ros-humble-pfms ros-humble-audibot-* ros-humble-husky-*
```

Students are provided skeleton code in this folder. This includes the structure of classes,  including interface classes that govern the behaviour of some classes. Unit tests that are used for marking are also provided.

Activities in week 4 tutorial will revolve around interacting with the platform, sending commands, as well as getting information from the platforms and sensors on them.  

When you run/test your code, you need to have the simulator running. This is done by executing the following command in the terminal. 

```bash
ros2 launch pfms a1.launch.py
```

Terminate the simulation by executing CTRL+C in the terminal windows.

If you ever need to reset the simulator to the starting point execute below in a new terminal window. 

```
ros2 service call /reset_world std_srvs/srv/Empty 
```

## build
```bash
cd ackerman_steer_pfms_project
mkdir build
cd build
cmake ..
make
```

## Test
```bash
cd build
./test/testAudiReachGoals
```


