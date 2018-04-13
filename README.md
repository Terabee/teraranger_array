# ROS package for TeraRanger array solutions by Terabee

This package is a collection of nodes for TeraRanger array solutions:
* [TeraRanger Hub Evo](https://www.terabee.com/portfolio-item/teraranger-hub-evo/)
* [TeraRanger Tower](https://www.terabee.com/portfolio-item/teraranger-tower/)
* [TeraRanger Multiflex](https://www.terabee.com/portfolio-item/teraranger-multiflex/)
* [TeraRanger Hub](https://www.terabee.com/portfolio-item/teraranger-hub/)

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

## Running TeraRanger Hub Evo

TeraRanger Hub Evo uses **TeraRanger Evo 60m** or **TeraRanger Evo 600Hz** sensors for the distance measurements. To use them please run **teraranger_evo** node.

After your workspace is built and sourced:
```
rosrun teraranger_array teraranger_evo _portname:=/dev/ttyACM0
```

## Running TeraRanger Tower & TeraRanger Hub

TeraRanger Tower and TeraRanger Hub uses **TeraRanger One** sensors for the distance measurements. To use them please run **teraranger_one** node.

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

**IMPORTANT: Please notice that for the Hub Evo you are able to set the sensor type of each sensor (Evo 60m or Evo 600Hz) with a dynamic_reconfigure for each port**

By default Evo 60m are set on the Hub.

## Product pictures and where to get the sensors

### TeraRanger Hub Evo

<img src="https://www.terabee.com/wp-content/uploads/2017/12/TeraRanger-Hub-Evo.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/portfolio-item/teraranger-hub-evo/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2018/02/TeraRanger-Hub-Evo-Specification-sheet.pdf)|
|[Online shop](http://www.teraranger.com/product/teraranger-hub-evo/) |

### TeraRanger Tower

<img src="http://www.teraranger.com/wp-content/uploads/2016/03/Teraranger_tower_typeB-1.png" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/portfolio-item/teraranger-tower/)|
|[Specification sheet](https://www.terabee.com/portfolio-item/teraranger-tower/#tower-specifications)|
|[Online shop](http://www.teraranger.com/product/teraranger-tower/) |


### TeraRanger Multiflex

<img src="https://www.terabee.com/wp-content/uploads/2017/08/DSC0311-Editar-3.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/portfolio-item/teraranger-multiflex/)|
|[Specification sheet](https://www.terabee.com/portfolio-item/teraranger-multiflex/#teraranger-specifications)|
|[Online shop](http://www.teraranger.com/product/teraranger-multiflex/) |
