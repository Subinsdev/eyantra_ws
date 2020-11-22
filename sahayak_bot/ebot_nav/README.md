# EYANTRA SAHAYAK BOT
![](../../images/logo.png)

# Mobile Robot 2D Mapping and Navigation
## Prerequisites
- Joint State Publisher:
```sh
sudo apt-get install ros-melodic-joint-state-publisher-gui
```
- Gmapping package:
```sh
sudo apt-get install ros-melodic-gmapping
```
- Teleop keyboard package: (Ignore if already installed)
```sh
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
- Navigation package:
```sh
sudo apt-get install ros-melodic-navigation
```
- Tf2 package:
```sh
sudo apt-get install ros-melodic-tf2-sensor-msgs
```
- AMCL package:
```sh
sudo apt-get install ros-melodic-amcl
```
- Map server package:
```sh
sudo apt-get install ros-melodic-map-server
```
Now build your catkin workspace and source the setup bash file.

## Clone
Clone this repository inside src folder. Copy sahayak_bot folder to elsewhere before cloning the 2nd repository.
```sh
$ git clone https://github.com/arnabGudu/eyantra_ws.git
$ git clone https://github.com/vishalgpt579/sahayak_bot.git
```

## Build
Build all the packages with
```sh
$ catkin build
```
or build specific packages with
```sh
$ cd ..
$ catkin build [package_name]
```

## Run Modules:
#### Gazebo & RViz:
```sh
$ roslaunch ebot_description nav_test.launch
$ roslaunch ebot_description ebot_visualize.launch
```
#### Teleop Twist Keyboard:
```sh
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
#### map_saver:
```sh
$ rosrun map_server map_saver -f sample_world
```
#### Gmapping:
```sh
$ rosrun gmapping slam_gmapping scan:=/ebot/laser/scan _base_frame:=ebot_base
```
#### AMCL:
```sh
$ roslaunch ebot_nav amcl.launch
```
#### Gmapping + AMCL:
```sh
$ roslaunch ebot_nav gmapping_amcl.launch
```
#### move_base:
```sh
$ roslaunch ebot_nav move_base.launch
```
#### task2
```sh
$ roslaunch ebot_nav task2.launch
```
License
----

MIT
