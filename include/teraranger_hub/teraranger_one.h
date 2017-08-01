#ifndef TERARANGER_HUB_ONE_H
#define TERARANGER_HUB_ONE_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <string>

#include "serial_port.h"
#include <dynamic_reconfigure/server.h>
#include <teraranger_hub/TerarangerHubOneConfig.h>
#include <teraranger_hub/RangeArray.h>

#define BUFFER_SIZE 19

namespace teraranger_hub
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

  SerialPort *serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  std::string ns_;

private:
  const float field_of_view = 0.0593;
  const float max_range = 14.0;
  const float min_range = 0.2;
  const uint8_t number_of_sensors = 8;
  const std::string frame_id = "base_range_";

  teraranger_hub::RangeArray measure;
};

} // namespace teraranger_hub

#endif // TERARANGER_HUB_ONE_H
