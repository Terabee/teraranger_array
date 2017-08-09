# ROS package for TeraRanger array solutions by Terabee

This package is a collection of nodes for TeraRanger array solutions:
* [TeraRanger Tower](http://www.teraranger.com/teraranger-tower/)
* [TeraRanger Multiflex](http://www.teraranger.com/products/teraranger-multiflex/)
* [TeraRanger Hub](http://www.teraranger.com/products/teraranger-hub/)

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

## Running the TeraRanger Tower & TeraRanger Hub

TeraRanger Tower and TeraRanger Hub utilize **TeraRanger One** sensors for the distance measurements. To use them please run **teraranger_one node**.

After your workspace is built and sourced:
```
rosrun teraranger_array teraranger_one _portname:=/dev/ttyACM0
```

## Running TeraRanger Multiflex

To use TeraRanger Multiflex please execute the following command after building and sourcing your workspace:

```
rosrun teraranger_array teraranger_multiflex _portname:=/dev/ttyACM0
``` 

## Changing Sensor Configuration

You can change the operating mode of the sensors by running **rqt_reconfigure**:

```
rosrun rqt_reconfigure rqt_reconfigure
```

## Product pictures and where to get the sensors

### TeraRanger Tower

<img src="http://www.teraranger.com/wp-content/uploads/2016/03/Teraranger_tower_typeB-1.png" width="300"/>

| Information |
| -------------- |
|[Product page](http://www.teraranger.com/teraranger-tower/)| 
|[Specification sheet](http://www.teraranger.com/wp-content/uploads/2016/03/Towerspecificationsheet.pdf)|
|[Online shop](http://www.teraranger.com/product/teraranger-tower/) |


### TeraRanger Multiflex

<img src="http://www.teraranger.com/wp-content/uploads/2017/01/multiflx-on-kobuki.jpg" width="300"/>


| Information |
| -------------- |
|[Product page](http://www.teraranger.com/products/teraranger-multiflex/)| 
|[Specification sheet](http://www.teraranger.com/wp-content/uploads/2017/04/MultiflexSpecificationSheet.pdf)|
|[Online shop](http://www.teraranger.com/product/teraranger-multiflex/) |