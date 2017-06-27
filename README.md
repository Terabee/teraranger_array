# ROS module for Teraranger-Hub and Teraranger-Tower
TeraRanger Hub/Tower ROS Module

This is the ROS module for the TeraRanger Tower or Hub (www.teraranger.com).
## Using module

To use the ROS node you just need to:

    Create a ROS Workspace
    Copy the node 'tr_hub_parser' package into the workspace src directory
    Compile using: catkin_make
    Setup environment: source devel/setup.sh
    Run using: rosrun tr_hub_parser tr_hub_parser _portname:=/dev/ttyACM0

If you want to change the operating mode, run

    rosrun rqt_reconfigure rqt_reconfigure

NB: remember to execute the daemon roscore before running the rosrun command
