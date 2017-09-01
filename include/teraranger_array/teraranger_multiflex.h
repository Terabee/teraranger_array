#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <teraranger_array/TerarangerHubMultiflexConfig.h>
#include <teraranger_array/RangeArray.h>
#include <serial/serial.h>

#define BUFFER_SIZE 20

#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 2000 	// Value adapted to avoid timeout when reconfiguring
#define CMD_BYTE_LENGTH 4

#define OUT_OF_RANGE_VALUE -1

namespace teraranger_array
{

// TODO: Re-enable if multiple modes are supported by Multiflex
// static const char PRECISE_MODE[] = {0x00, 0x21, 0x02, 0xB5};
// static const char FAST_MODE[] = {0x00, 0x21, 0x01, 0xBC};
static const char LONG_RANGE_MODE[] = {(char)0x00, (char)0x21, (char)0x03, (char)0xB2};

static const char BINARY_MODE[] = {(char)0x00, (char)0x11, (char)0x02, (char)0x4C};
static const char TEXT_MODE[] = {(char)0x00, (char)0x11, (char)0x01, (char)0x45};

static const float MIN_RANGE = 0.05;
static const float MAX_RANGE = 2.0;
static const float FIELD_OF_VIEW = 0.2967;

static const uint8_t SENSOR_COUNT = 8;
static const uint8_t BITMASK_POS = 18; //Position of bitmask in message frame

static const uint8_t REPLY_MSG_LEN = 5;
static const char REPLY_CHAR = 'R'; //First character of Multiflex response message

static const uint8_t MF_MSG_LEN = 20;
static const uint8_t MF_CHAR = 'M'; //First character of Multiflex message

class TerarangerHubMultiflex
{
public:
  TerarangerHubMultiflex();
  std::string IntToString(int number);

  virtual ~TerarangerHubMultiflex();
  void serialDataCallback(uint8_t data);
  void spin();

  void dynParamCallback(const teraranger_mutliflex_cfg::TerarangerHubMultiflexConfig &config, uint32_t level);

  void parseCommand(uint8_t *input_buffer, uint8_t len);
  std::string arrayToString(uint8_t *input_buffer, uint8_t len);

  bool loadParameters();
  void setMode(const char *c);
  void setSensorBitMask(int *sensor_bit_mask_ptr);
  int *sensor_bit_mask_ptr;
  int sensor_bit_mask[8];

  ros::NodeHandle nh_;
  ros::Publisher range_publisher_;

  dynamic_reconfigure::Server<teraranger_mutliflex_cfg::TerarangerHubMultiflexConfig> dyn_param_server_;
  dynamic_reconfigure::Server<teraranger_mutliflex_cfg::TerarangerHubMultiflexConfig>::CallbackType dyn_param_server_callback_function_;

  serial::Serial serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  std::string ns_;
private:
  teraranger_array::RangeArray range_array_msg;
};

} // namespace teraranger_array
