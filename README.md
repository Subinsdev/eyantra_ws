# EYANTRA SAHAYAK BOT
![](./images/logo.png)

# Basic ROS Installation Guide

## Prerequisites
- Ubuntu 16.04 or newer (Ubuntu 18.04 recommended)
- [ROS Kinetic ](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 16.04) or [ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 16.04)
- [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

## Before Cloning
Create a ROS workspace and a src folder, then run catkin build
Note: use only small alphabets without spaces, use underscore instead.
```sh
$ mkdir cyborg_ws
$ cd cyborg_ws
$ mkdir src
$ catkin build
```
No need to source the workspace everytime, run this command to source the workspace whenever you open a new terminal
```sh
$ echo 'source ~/cyborg_ws/devel/setup.bash' >> ~/.bashrc
```
## Clone
Clone this repository inside src folder
```sh
$ cd src
$ git clone https://github.com/arnabGudu/eyantra_ws.git
```

## Build
Build all the packages with
```sh
$ cd ..
$ catkin build
```
or build specific packages with
```sh
$ cd ..
$ catkin build [package_name]
```

License
----

MIT
