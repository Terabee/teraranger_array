#pragma once
#include <string>
#include <limits>

#include <ros/console.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <serial/serial.h>
#include <teraranger_array/helper_lib.h>
#include <sensor_msgs/Range.h>
#include <teraranger_array/RangeArray.h>


class RosSimpleNode
{
  protected:
    ros::NodeHandle nh_;
    virtual void spin() = 0;

  public:
    RosSimpleNode()
    {
      nh_ = ros::NodeHandle("~");
    }
    virtual ~RosSimpleNode();
};


template <class M>
class RosSimplePublisher : public RosSimpleNode
{
  protected:
    M msg_;
    std::string topic_name_;
    ros::Publisher publisher_;
    void spin(){while(ros::ok()) ros::spinOnce();}
    virtual void init_msg() = 0;

  public:
    RosSimplePublisher()
    {
      topic_name_ = "node_topic";
      publisher_ = nh_.advertise<M>(topic_name_, 1);
    }
    RosSimplePublisher(std::string topic_name)
    {
      topic_name_ = topic_name;
      publisher_ = nh_.advertise<M>(topic_name, 1);
    }
    virtual ~RosSimplePublisher();
};

template<typename C>
class RosDynReconfWrapper
{
  private:
    typename dynamic_reconfigure::Server<C> dyn_param_server_;
    typename dynamic_reconfigure::Server<C>::CallbackType dyn_param_server_callback_function_;
  public:
    RosDynReconfWrapper();
    virtual ~RosDynReconfWrapper();

    virtual void dynParamCallback(const C &config, uint32_t level) = 0;
};

// TODO Add generic serial class [NO ROS INSIDE]
// TODO Add generic range sensor [NO ROS INSIDE]
// TODO Separate no-ros classes in another package
// TODO Separate templates classes in another package

// Hierarchy test
// #include <teraranger_array/TerarangerHubOneConfig.h>
class TerarangerOneRosDriver : public RosSimplePublisher<sensor_msgs::Range>//, public RosDynReconfWrapper<teraranger_one_cfg::TerarangerHubOneConfig>
{
public:
  TerarangerOneRosDriver() : RosSimplePublisher("range"){}
  virtual ~TerarangerOneRosDriver();
};
