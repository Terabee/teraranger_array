#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <teraranger_array/TerarangerHubOneConfig.h>
#include <teraranger_array/RangeArray.h>
#include <serial/serial.h>

#define BUFFER_SIZE 19
#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000
#define CMD_BYTE_LENGTH 3

#define OUT_OF_RANGE_VALUE 0
#define TOO_CLOSE_VALUE 200
#define INVALID_MEASURE_VALUE -1
#define VALUE_TO_METER_FACTOR 0.001

namespace teraranger_array
{

static const char PRECISE_MODE[] = "PPP";
static const char FAST_MODE[] = "FFF";
static const char OUTDOOR_MODE[] = "OOO";

static const char BINARY_MODE[] = "BBB";
static const char TEXT_MODE[] = "TTT";
static const char FORCE_8_SENSORS[] = "CFF";

class TerarangerHubOne
{
public:
  TerarangerHubOne();
  virtual ~TerarangerHubOne();

  void serialDataCallback(uint8_t data);

  void dynParamCallback(const teraranger_one_cfg::TerarangerHubOneConfig &config, uint32_t level);

  bool loadParameters();
  void setMode(const char *c);

  ros::NodeHandle nh_;
  ros::Publisher range_publisher_;

  dynamic_reconfigure::Server<teraranger_one_cfg::TerarangerHubOneConfig> dyn_param_server_;
  dynamic_reconfigure::Server<teraranger_one_cfg::TerarangerHubOneConfig>::CallbackType dyn_param_server_callback_function_;

  serial::Serial serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  std::string ns_;

  void spin();

private:
  float field_of_view;
  float max_range;
  float min_range;
  uint8_t number_of_sensors;
  std::string frame_id;

  teraranger_array::RangeArray measure;
};

} // namespace teraranger_array
