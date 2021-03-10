# ROS package for TeraRanger array solutions by Terabee
[![Build Status](https://travis-ci.org/Terabee/teraranger_array.svg?branch=master)](https://travis-ci.org/Terabee/teraranger_array)

This package is a collection of nodes for TeraRanger array solutions:
* [TeraRanger Tower Evo](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower-evo/)
* [TeraRanger Hub Evo](https://www.terabee.com/shop/accessories/teraranger-hub-evo/)
* [TeraRanger Tower](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower/)
* [TeraRanger Multiflex](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-multiflex/)
* [TeraRanger Hub](https://www.terabee.com/shop/accessories/teraranger-hub/)

## Building and running the package from source

To clone and build the package in your workspace follow these steps:

* If you have ssh key setup for your github account:

```
cd ~/ros_ws/src
git clone git@github.com:Terabee/teraranger_array.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

* If you prefer to use https use this set of commands:

```
cd ~/ros_ws/src
git clone https://github.com/Terabee/teraranger_array.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Dependencies
This package depends on this [serial](http://wiki.ros.org/serial) library. To get it, execute the following command:

```
sudo apt-get install ros-<your_distro>-serial
```

where <your_distro> is your ROS distribution (e.g. kinetic, lunar, indigo).

If it's not available for your distribution, clone https://github.com/wjwwood/serial into your workspace, then build and source your workspace.

## Running TeraRanger Hub Evo & Tower Evo

**TeraRanger Hub Evo/Tower Evo** uses **TeraRanger Evo 60m**, **TeraRanger Evo 40m**, **TeraRanger Evo 15m**, **TeraRanger Evo 600Hz** or **TeraRanger Evo 3m** sensors for the distance measurements. To use them please run **teraranger_evo** node.

After your workspace is built and sourced:
If you are using **USB**:
```
rosrun teraranger_array teraranger_evo _portname:=/dev/ttyACM0
```
If you are using **UART Daughterboard**:
```
rosrun teraranger_array teraranger_evo _portname:=/dev/ttyACM0 _baudrate:=921600
```

## Running TeraRanger Tower & TeraRanger Hub

**TeraRanger Tower** and TeraRanger Hub uses **TeraRanger One** sensors for the distance measurements. To use them please run **teraranger_one** node.

After your workspace is built and sourced:
```
rosrun teraranger_array teraranger_one _portname:=/dev/ttyACM0
```

## Running TeraRanger Multiflex

To use TeraRanger Multiflex please run **teraranger_multiflex** node.

After your workspace is built and sourced:
```
rosrun teraranger_array teraranger_multiflex _portname:=/dev/ttyACM0
```

## Changing Sensor Configuration

You can change the operating mode of the sensors by running **rqt_reconfigure**:

```
rosrun rqt_reconfigure rqt_reconfigure
```

**IMPORTANT: Please notice that for the Hub Evo you are able to set the sensor type of each sensor (Evo 60m, Evo 40m, Evo 15m, Evo 600Hz, Evo 3m or Evo Mini) with a dynamic_reconfigure for each port**

By default sensors are set to Evo 60m type.

## Product pictures and where to get the sensors

### Teraranger Tower Evo

<img src="https://www.terabee.com/wp-content/uploads/2019/03/TeraRanger-Tower-Evo.png" width="400"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower-evo/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2019/03/TeraRanger-Tower-Evo-Specification-sheet-.pdf)|

### TeraRanger Hub Evo

<img src="https://www.terabee.com/wp-content/uploads/2019/03/Whats-in-the-pack.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/shop/accessories/teraranger-hub-evo/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2019/03/TeraRanger-Hub-Evo-Specification-sheet.pdf)|


### TeraRanger Tower

<img src="https://www.terabee.com/wp-content/uploads/2019/03/Teraranger_tower_typeB-1.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2019/03/Towerspecificationsheet-1.pdf)|


### TeraRanger Multiflex

<img src="https://www.terabee.com/wp-content/uploads/2019/03/TeraRanger-Multiflex-e1553710124882.png" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-multiflex/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2019/03/MultiflexSpecificationSheet.pdf)|
