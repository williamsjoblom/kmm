# KMM
Konstruktion med mikrodatorer, projekt.

## Tools, libraries and dependencies
### ROS Kinetic (Robot Operating System)

Supported on: Ubuntu 15.10 (Wily), Ubuntu 16.04 (Xenial), Debian 8 (Jessie)

http://wiki.ros.org/ROS/Tutorials

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### AVR compiler

```
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
```

### SFML (Simple and Fast Multimedia Library)

https://www.sfml-dev.org/tutorials/2.4/

```
sudo apt-get install libsfml-dev
```
## Setup project

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_PACKAGE_PATH
cd ~/catkin_ws/src/
git clone git@github.com:williamsjoblom/kmm.git
~/catkin_ws/
catkin_make
```
