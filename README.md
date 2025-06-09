# Franka_RealSense_data_collection
Installing RealSense and Franka repositories and code for LfD data collection

## ABOUT
**This tutorial explains how to install Franka, RealSense, and aruco_ros for your Ubuntu Version**

This repo is made by Venkatesh. It contains the instrcutions to install:
  - [Libfranka](#libfranka)
  - [franka_ros](#franka_ros)
  - [moveit](#moveit)
  - [RealSense](#realsense)
  - [Aruco_ros](#aruco_ros)
  - [Cartesian_Impedance_control](Cartesian-Impedance-control-zero-torque-mode)

Tested on Ubuntu 16.04 (ROS Kinetic) /20.04 (ROS Noetic)

**Note: This repository has one Python file only to check the working of the system**


If you are working with strawberries for your dataset, you can have a look at Franceso and Chiara's code for Python files and dataset https://github.com/imanlab/Franka_datacollection_pipeline. The Python file in this repository is taken from the above repository

### Libfranka

1) You need to know which version your franka controller currently is on. To do this open the desk interface of franka i.e., go to https://172.16.0.2/desk. 
2) Once you are on the interface click on the 3 lines on the top right of the screen to view a dropdown
3) Click on settings. You will see a dashboard that has the franka version on it. Ex: 4.0.3, 2.1.1, etc...

4) Now that you know the version of your franka conrtoller, go to https://frankaemika.github.io/docs/compatibility.html to check the compatible versions of libfranka and franka_ros. Note that this is very important to get right. EX: Our lab's ROS Kinetic system has franka_version 2.1.1. So, the libfranka version is 0.5.0 and franka_ros is 0.6.0

5) Now that you know the libfranka version (for IML, it will either be version 0.5.0 or 0.8.0 based on the franka controller), you can install from source libfranka using the following commands

**Note:** As on 06/06/2025, the lab is using ROS Noetic and libfranka version 0.9.0

```
cd

sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

git clone --recursive https://github.com/frankaemika/libfranka # only for panda

git checkout <version>

git submodule update

cd libfranka

mkdir build

cd build

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..

cmake --build .
```

Congratulations!!! You finished step 1, that is installing libfranka. 

The next step is installing franka_ros

### franka_ros

1) First set up your catkin workspace, where you will have all your repositories. To do this use the following commands

```
cd

mkdir -p catkin_ws/src

cd catkin_ws

source /opt/ros/noetic/setup.sh

catkin_init_workspace src

```
2) Now have your franka_ros version ready (For IML it will be either 0.6.0 or 0.8.0)
   
**Note:** Instead of specifying the excact version as mentioned above just choosing "noetic-devel" branch from the repo works for ROS Noetic as on 06/06/2025. So, `<version>` during `git checkout` below will be `noetic-devel`

```
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

git checkout <version>

rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka

catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

source devel/setup.sh
```

Note: If you are on kinetic and get an error about unmet dependencies. Make sure you are using python 2.7 and do the following
```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

sudo apt-get install ros-kinetic-gazebo-ros-control
```

3) Finally you need to allow your user account to set real-time permissions for its processes
```
sudo addgroup realtime

sudo usermod -a -G realtime $(whoami)
```
Congratulations!! You installed franka_ros as well. Before going to step 3, that is installing Moveit, restart your PC. This is important as it resets your user account to work with real-time kernel

You can check working of franka_ros using the following commands

```
cd ~/catkin_ws

source devel/setup.bash

roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2
```
You will see a bunch of topics when you do _**rostopic_list**_ :

![alt text](https://github.com/imanlab/Franka_RealSense_data_collection/blob/main/franka_control.png)

if you see an error when you enter the roslaunch command above
```
      [ERROR]: libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]
```
this would mostly be because of the mismatch in libfranka and franka_ros versions. 

This error can also appear if the emergency stop is not released. So, make sure the lights on franka panda are blue and not white.

### Moveit

The following steps have been tested on ROS Noetic and are suggested to use. If that does not work only then use panda_moveit_config from this repo

```
cd ~/catkin_ws/src

git clone -b noetic-devel https://github.com/moveit/panda_moveit_config.git

cd ..

catkin_make

source devel/setup.bash

roslaunch panda_moveit_config move_group.launch

roslaunch panda_moveit_config moveit_rviz.launch
```

You should be able to see in the rostopics that move_group is installed. The pose of the robot in the real world should be mimicked by rviz and get continually updated. That's it!

### RealSense

Now for the final step. The RealSense camera we have in IML is IntelRealSense D415. The following steps work for ROS1 and my reference for this tutorial is the realsense [github repo](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy). This couild become tricky part as RealSense is moving to ROS2.

**Note: Install RealSense from source. Do not install using apt-get**

1) Check the branches in the Realsense github repo I mentioned above. The branch we want for ROS1 is _ros1-legacy_
```
cd ~/catkin_ws/src/

git clone https://github.com/IntelRealSense/realsense-ros.git

git checkout <branch or tag name>
```
2) Now clone the ddynamic_reconfigure [github repo](https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel) (tested on ROS kinetic. Use dynamic_reconficgure [repo](https://github.com/ros/dynamic_reconfigure) if you are on Noetic). 
```
cd ~/catkin_ws/src/

git clone https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel

git checkout kinetic-devel
```
3) Now you have realsense and ddynamic reconfigure in your catkin workspace
```
cd ~/catkin_ws/src/

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
```
Congratulations you have now installed RealSense camera as well

 Check the working using the following commands

 ```
roslaunch realsense2_camera rs_camera.launch
```
and check the list of topics published using _**rostopic_list**_

### aruco_ros

To use the aruco marker and communicate with the aruco marker via ROS do the following

```
cd ~/catkin_ws/src

git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git

cd ~/catkin_ws

catkin_make -DCMAKE_PREFIX_PATH=/opt/ros/noetic -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4

source devel/setup.bash
```

**Note:** It is important to build the package using the above command i.e with `-DOpenCV_DIR=` and not a simple `catkin_make`

Once the aruco_ros is built and source without errors or warnings, make the following changes

1) Within the newly installed aruco_ros package go to the aruco_ros folder and open the single.launch or double.launch
2) Change the default values of the `markerId` and `markerSize` arguments. I am using markerId 8 and markerSize 0.05
3) Change `remap from="/camera_info" to ="/camera/color/camera_info"`
4) Change `remap from="/image" to="/camera/color/image_raw"`
5) Change `param name="camera_frame" value="camera_color_optical_frame"`

Save these changes and run

```
cd ~/catkin_ws

source devel/setup.bash

roslaunch aruco_ros single.launch
```

You can see that the aruco marker is being detected. You can verify by checking `rostopic echo /aruco_simple/pose` or using rviz


### Cartesian Impedance control - zero torque mode

To have all the joints freed up similar to when the emergency stop button is pressed down, do the following

1) Go to the franka_example_controllers package in your franka_ros i.e
```
cd ~/catkin_ws/src/franka_ros/franka_example_controllers
```
2) Go to the _**config**_ folder. Replace the franka_example_controllers.yaml file with the one present in this repo
3) Go to the _**cfg**_ folder. Replace the compliance_param.cfg with the one present in this repo
4) Go to the _**include**_ folder. Replace the pseudo_inversion.h with the one present in this repo.
5) Go to the _**src**_ folder. Replace the cartesian_impedance_example_controller.cpp file with the one present in this repo.
6) Now add the _**panda_moveit_config**_ package from this repository into your **franka_ros** folder, i.e., you will see the panda_moveit_config folder if you do
```
ls ~/catkin_ws/src/franka_ros
```

7) Now run the following commands
```
cd ~/catkin_ws

catkin_make

source devel/setup.bash
```

That's it! You can check the working of the controller doing the following:

1) Add the franka_moveit.launch file from this repo in your main package's _**launch**_ folder. As an example, I am adding it to the lanuch folder of the franka_example_controllers package. Now, I can see a file named franka_moveit.launch when I do 
```
ls ~/catkin_ws/src/franka_ros/franka_example_controllers/launch
```
Now run the launch file 
```
cd ~/catkin_ws

source devel/setup.bash

roslaunch franka_example_controllers franka_moveit.launch
```

2) Add the data_collection_top.py file from this repo in you _**srcipt**_ folder. Again as an example I am adding it to the franka_example_controllers package's **script** folder.
Open a new terminal and do the following
```
cd ~/catkin_ws

source devel/setup.bash

cd ~/catkin_ws/src/franka_ros/franka_example_controllers/script

chmod +x data_collection_top.py

rosrun franka_example_controllers data_collection_top.py
```
The Franka Panda robot will by default be in position control mode and you cannot move it manually. 
  - It should go to home position if you press **"h"** in the terminal.
  - Now press **"t"**. The robot will use the zero torque cartesian impedance controller and you can freely move the robot manually. 

**That's it. Enjoy!!**

### Author:
Venkatesh Sripada: s.venkatesh779@gmail.com
