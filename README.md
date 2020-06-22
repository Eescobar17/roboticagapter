Gapter: Set-up GapterSim Environment

In order to launch the Gapter simulation on the computer, it is
necessary to first configure the working environment, installing some
necessary tools and their dependencies to achieve a correct operation
of the simulation.

A. Prerequisites

1. Ubuntu 14.04 LTS
2. Ros indigo (http://wiki.ros.org/indigo/Installation/Ubuntu)


Configure Ubuntu:

sudo apt-get update

sudo apt-get install gawk make git curl cmake -y


ROS dependencies and requided packages:

sudo apt-get install python-rosinstall ros-indigo-octomap-msgs
ros-indigo-joy ros-indigo-geodesy ros-indigo-octomap-ros
ros-indigo-mavlink  ros-indigo-control-toolbox
ros-indigo-transmission-interface ros-indigo-joint-limits-interface
unzip ros-indigo-mavros-msgs ros-indigo-mavros-extras
ros-indigo-mavros -y



Mavproxy:

Install dependencies:

sudo apt-get install g++ python-pip python-matplotlib python-serial
python-wxgtk2.8 python-scipy python-opencv python-numpy
python-pyparsing ccache realpath libopencv-dev -y


Install MavProxy:

sudo pip install future

sudo apt-get install libxml2-dev libxslt1-dev -y

sudo pip2 install pymavlink catkin_pkg --upgrade

sudo pip install MAVProxy==1.5.2


B. Ardupilot

mkdir -p ~/gapter_sim

cd gapter_sim

git clone https://github.com/gaitech-robotics/gapter_ardupilot -b sim_ros_gazebo


C. Creating ros work space

mkdir -p ~/gapter_sim/ros_ws/src

cd ~/gapter_sim/ros_ws/src

source /opt/ros/indigo/setup.bash

catkin_init_workspace

cd ..

catkin_make

source devel/setup.bash

Download all repositories required:

cd src/

git clone https://github.com/gaitech-robotics/gapter_ardupilot_sitl_gazebo_plugin

git clone https://github.com/gaitech-robotics/gapter_rotors_simulator
-b sonar_plugin

git clone https://github.com/PX4/mav_comm.git

git clone https://github.com/ethz-asl/glog_catkin.git

git clone https://github.com/catkin/catkin_simple.git

git clone https://github.com/gaitech-robotics/mavros

git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/

git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b indigo-devel

D. Installing Gazebo

Setup your computer:

sudo sh -c 'echo "deb
http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release
-cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'


Setup Keys:

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -


Install gazebo:

sudo apt-get update
sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*'
sudo apt-get update
sudo apt-get install gazebo4 libgazebo4-dev drcsim -y


E. Compile workspace

source ~/gapter_sim/ros_ws/devel/setup.bash

cd ~/gapter_sim/ros_ws

sudo apt-get update

sudo apt install libgoogle-glog-dev

catkin_make --pkg mav_msgs mavros_msgs gazebo_msgs

source devel/setup.bash

catkin_make -j 4


If you have an error, type the following:

cd ~/gapter_sim/ros_ws/src/glog_catkin

cmake ..



Once this is done, let’s try again

cd ~/gapter_sim/ros_ws/

source devel/setup.bash

catkin_make -j 4             Repeat if is necessary

Once all the necessary tools for the simulation have been installed
and configured the environment work, we are already able to launch the
Gapter simulation in our computer.


F. launch Gapter Simulation

Open the terminal and type following:

source ~/gapter_sim/ros_ws/devel/setup.bash

cd ~/gapter_sim/gapter_ardupilot/ArduCopter

../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo


In another terminal launch following:

source ~/gapter_sim/ros_ws/devel/setup.bash

roslaunch ardupilot_sitl_gazebo_plugin gapter_spawn.launch

Once mavproxy launched completely in the first terminal load
parameters from Ardupilot directory

param load /your home
directory/gapter_sim/gapter_ardupilot/Tools/Frame_params/gapter.param

param set ARMING_CHECK 0







Trajectory simulation using ROS

Once the simulation is launched, open another terminal and type the following:

cd ~/gapter_sim/ros_ws/src

source ~/gapter_sim/ros_ws/devel/setup.bash

catkin_create_pkg rombo mavros

cd rombo/

Once created, the CMakeLists.txt file needs to be modified.

rm CMakeLists.txt

wget https://raw.githubusercontent.com/Eescobar17/roboticagapter/master/CMakeLists.txt

if a src folder didn’t create, type:

mkdir src/

Let's continue:

cd src/

wget https://raw.githubusercontent.com/Eescobar17/roboticagapter/master/main.cpp

cd ~/gapter_sim/ros_ws/

catkin_make --pkg rombo

rosrun rombo rombo

Gapter simulation using joystick

Configuring and Using a Linux-Supported Joystick with ROS

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

Once the joystick is configured, launch the simulation as specified
before and run the joy node in another terminal:

rosparam set joy_node/dev "/dev/input/jsX"

rosrun joy joy_node

Open another terminal and type the following:

cd ~/gapter_sim/ros_ws/src

catkin_create_pkg gapter_py rospy

cd ~/gapter_sim/ros_ws/src/gapter_py

rm CMakeLists.txt

gedit CMakeLists.txt

Now, change the first line for:

cmake_minimum_required(VERSION 2.8.3)

if a src folder didn’t create, type:

mkdir src/

Let's continue:

cd src/

wget https://raw.githubusercontent.com/Eescobar17/roboticagapter/master/gapter_pilot.py

chmod a+x gapter_pilot.py

cd ~/gapter_sim/ros_ws/

catkin_make --pkg gapter_py

Finally, open another terminal and type

source ~/gapter_sim/ros_ws/devel/setup.bash

rosrun gapter_py gapter_pilot.py
