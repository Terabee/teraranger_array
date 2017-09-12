#pragma once
#include <limits>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/Imu.h>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <teraranger_array/TerarangerHubEvoConfig.h>
#include <teraranger_array/RangeArray.h>
#include <serial/serial.h>

#define BUFFER_SIZE 38 //max frame length
#define RANGES_FRAME_LENGTH 20
#define RANGE_CRC_POS 19
#define IMU_QUAT_FRAME_LENGTH 12
#define IMU_EULER_FRAME_LENGTH 10
#define IMU_QUATLIN_FRAME_LENGTH 18

#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000

#define OUT_OF_RANGE_VALUE -1
#define TOO_CLOSE_VALUE 0
#define INVALID_MEASURE_VALUE 1
#define VALUE_TO_METER_FACTOR 0.001

namespace teraranger_array
{
// Protocol commands
static const char ENABLE_CMD[5] = {(char)0x00, (char)0x52, (char)0x02, (char)0x01, (char)0xDF};
static const char DISABLE_CMD[5] = {(char)0x00, (char)0x52, (char)0x02, (char)0x00, (char)0xD8};

static const char TEXT_MODE[4] = {(char)0x00, (char)0x11, (char)0x01, (char)0x45};
static const char BINARY_MODE[4] = {(char)0x00, (char)0x11, (char)0x02, (char)0x4C};

static const char LONG_RANGE[4] = {(char)0x00, (char)0x21, (char)0x04, (char)0xA7}; // for long range evo
static const char SHORT_RANGE[4] = {(char)0x00, (char)0x21, (char)0x02, (char)0xB5}; // for short range evo

static const char CROSSTALK_MODE[4] = {(char)0x00, (char)0x31,(char)0x01,(char)0xEB}; // All sensors in parallel
static const char NONCROSSTALK_MODE[4] = {(char)0x00, (char)0x31, (char)0x02, (char)0xE2}; // All sensors sequentially
static const char TOWER_MODE[4] = {(char)0x00, (char)0x31, (char)0x03, (char)0xE5}; // 4 by 4 in a cross manner

static const char IMU_OFF[4] = {(char)0x00, (char)0x41, (char)0x01, (char)0x49};
static const char IMU_QUAT[4] = {(char)0x00, (char)0x41, (char)0x02, (char)0x40};
static const char IMU_EULER[4] = {(char)0x00, (char)0x41, (char)0x03, (char)0x47};
static const char IMU_QUATLIN[4] = {(char)0x00, (char)0x41, (char)0x04, (char)0x52};

static const char RATE_ASAP[5] = {(char)0x00, (char)0x52, (char)0x03,(char)0x01, (char)0xCA};
static const char RATE_700[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x02,(char)0xC3};
static const char RATE_600[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x03,(char)0xC4};
static const char RATE_500[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x04,(char)0xD1};
static const char RATE_250[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x05,(char)0xD6};
static const char RATE_100[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x06,(char)0xDF};
static const char RATE_50[5] = {(char)0x00, (char)0x52, (char)0x03, (char)0x07,(char)0xD8};

enum imu_mode{
  off,
  quat,
  euler,
  quatlin
};

class TerarangerHubEvo
{
public:
  TerarangerHubEvo();
  virtual ~TerarangerHubEvo();


  void serialDataCallback(uint8_t data);

  void dynParamCallback(const teraranger_evo_cfg::TerarangerHubEvoConfig &config, uint32_t level);

  bool loadParameters();
  void setMode(const char *c, int length);
  void spin();

  ros::NodeHandle nh_;
  ros::Publisher range_publisher_;
  ros::Publisher imu_publisher_;

  dynamic_reconfigure::Server<teraranger_evo_cfg::TerarangerHubEvoConfig> dyn_param_server_;
  dynamic_reconfigure::Server<teraranger_evo_cfg::TerarangerHubEvoConfig>::CallbackType dyn_param_server_callback_function_;

  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  std::string ns_;
private:
  float field_of_view ;
  float max_range;
  float min_range;
  int number_of_sensor;
  std::string frame_id;
  serial::Serial serial_port_;
  imu_mode imu_status;
  int current_imu_frame_length;

  teraranger_array::RangeArray range_array_msg;
  sensor_msgs::Imu imu_msg;

  void processRangeFrame(uint8_t* input_buffer, int seq_ctr);
  void processImuFrame(uint8_t* input_buffer, int seq_ctr);
};

} // namespace teraranger_array
