#include <string>
#include <sstream>
#include <iomanip>
#include <teraranger_array/teraranger_multiflex.h>
#include <teraranger_array/helper_lib.h>
#include <ros/console.h>

namespace teraranger_array
{

TerarangerHubMultiflex::TerarangerHubMultiflex()
{
  // Get parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyACM0"));

  // Publishers
  range_publisher_ = nh_.advertise<teraranger_array::RangeArray>("ranges", 1);

  // Create serial port
  serial_port_.setPort(portname_);
  serial_port_.setBaudrate(SERIAL_SPEED);
  serial_port_.setParity(serial::parity_none);
  serial_port_.setStopbits(serial::stopbits_one);
  serial_port_.setBytesize(serial::eightbits);
  serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
  serial_port_.setTimeout(to);

  serial_port_.open();

  if(!serial_port_.isOpen())
  {
    ROS_ERROR("Could not open : %s ", portname_.c_str());
    ros::shutdown();
    return;
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  if (ns_ != "" && ns_[0] == '/')
  { // Remove first backslash if needed
    ns_.erase(0,1);
  }
  ROS_INFO("node namespace: [%s]", ns_.c_str());

  std::string frame_id = "base_range_";

  // Initialize rangeArray
  for (size_t i=0; i < SENSOR_COUNT; i++)
  {
    sensor_msgs::Range range;
    range.field_of_view = FIELD_OF_VIEW;
    range.max_range = MAX_RANGE;
    range.min_range = MIN_RANGE;
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.range = 0.0;
    // set the right range frame depending of the namespace
    if (ns_ == "")
    {
     range.header.frame_id = frame_id + boost::lexical_cast<std::string>(i);
    }
    else
    {
     range.header.frame_id = ns_ + '_'+ frame_id + boost::lexical_cast<std::string>(i);
    }
    range_array_msg.ranges.push_back(range);
  }

  // set the right RangeArray frame depending of the namespace
  if (ns_ == "")
  {
    range_array_msg.header.frame_id = "base_hub";
  }
  else
  {
    range_array_msg.header.frame_id = "base_" + ns_;
  }

  // Set operation Mode
  setMode(BINARY_MODE);

  // Initialize all active sensors

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&TerarangerHubMultiflex::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerHubMultiflex::~TerarangerHubMultiflex()
{
}

void TerarangerHubMultiflex::parseCommand(uint8_t *input_buffer, uint8_t len)
{
  static int int_min_range = (int)(MIN_RANGE * 1000);
  static int int_max_range = (int)(MAX_RANGE * 1000);
  static int seq_ctr = 0;

  uint8_t crc = HelperLib::crc8(input_buffer, len);

  if (crc == input_buffer[len])
  {

    int16_t ranges[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      ranges[i] = input_buffer[i * 2 + 2] << 8;
      ranges[i] |= input_buffer[i * 2 + 3];
    }

    uint8_t bitmask = input_buffer[BITMASK_POS];
    uint8_t bit_compare = 1;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      range_array_msg.ranges.at(i).header.stamp = ros::Time::now();
      range_array_msg.ranges.at(i).header.seq = seq_ctr++;
      float final_range;
      if ((bitmask & bit_compare) == bit_compare)
      {
        if(ranges[i] == OUT_OF_RANGE_VALUE)
        {
          final_range = std::numeric_limits<float>::infinity();
        }
        else if (ranges[i] > int_max_range)
        {
          final_range = std::numeric_limits<float>::infinity(); // convert to m
        }
        else if(ranges[i] < int_min_range)
        {
          final_range = -std::numeric_limits<float>::infinity();
        }
        else
        {
          final_range = ranges[i] * 0.001; // convert to m
        }
      }
      else
      {
        final_range = std::numeric_limits<float>::quiet_NaN();
        ROS_WARN_ONCE("Not all sensors activated set proper bitmask using rosrun rqt_reconfigure rqt_reconfigure");
      }
      bit_compare <<= 1;
      range_array_msg.ranges.at(i).range = final_range;
    }
    range_array_msg.header.seq = (int)seq_ctr/8;
    range_array_msg.header.stamp = ros::Time::now();
    range_publisher_.publish(range_array_msg);
  }
  else
  {
    ROS_ERROR("[%s] crc missmatch", ros::this_node::getName().c_str());
  }
}

std::string TerarangerHubMultiflex::arrayToString(uint8_t *input_buffer, uint8_t len)
{
  std::ostringstream convert;
  for (int a = 0; a < len; a++)
  {
    convert << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (int)input_buffer[a];
    convert << std::uppercase << std::hex << " ";
  }
  std::string str = convert.str();
  return str;
}

void TerarangerHubMultiflex::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int size_frame = REPLY_MSG_LEN;
  static char first_char = REPLY_CHAR;

  if (single_character == MF_CHAR && buffer_ctr == 0)
  {
    size_frame = MF_MSG_LEN;
    first_char = MF_CHAR;
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
  }

  else if (single_character == REPLY_CHAR && buffer_ctr == 0)
  {
    size_frame = REPLY_MSG_LEN;
    first_char = REPLY_CHAR;
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
  }

  else if (first_char == REPLY_CHAR && buffer_ctr < size_frame)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;

    if (buffer_ctr == size_frame)
    {
      std::string str = arrayToString(input_buffer, size_frame);
      ROS_DEBUG_STREAM("Respond frame received... : " << str);
      // reset
      buffer_ctr = 0;
      // clear struct
      bzero(&input_buffer, BUFFER_SIZE);
    }
  }
  else if (first_char == MF_CHAR && buffer_ctr < size_frame)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    if (buffer_ctr == size_frame)
    {
      std::string str = arrayToString(input_buffer, size_frame);
      ROS_DEBUG_STREAM("Frame received... : " << str);

      parseCommand(input_buffer, 19);
      // reset
      buffer_ctr = 0;
      // clear struct
      bzero(&input_buffer, BUFFER_SIZE);
    }
  }
  else
  {
    ROS_DEBUG("Received uknown character %x", single_character);
    // reset
    buffer_ctr = 0;
    // clear struct
    bzero(&input_buffer, BUFFER_SIZE);
  }
}

void TerarangerHubMultiflex::setMode(const char *c)
{
  if(!serial_port_.write((uint8_t*)c, CMD_BYTE_LENGTH))
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerHubMultiflex::setSensorBitMask(int *sensor_bit_mask_ptr)
{

  uint8_t bit_mask_hex = 0x00;
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    bit_mask_hex |= *(sensor_bit_mask_ptr + 7 - i) << (7 - i);
  }

  // calculate crc

  uint8_t command[4] = {0x00, 0x52, 0x03, bit_mask_hex};
  int8_t crc = HelperLib::crc8(command, 4);

  //send command
  char full_command[5] = {(char)0x00, (char)0x52, (char)0x03, (char)bit_mask_hex, (char)crc};

  if(!serial_port_.write((uint8_t*)full_command, 5))
  {
    ROS_ERROR("Timeout or error while sending command");
  }
  serial_port_.flushOutput();
}

void TerarangerHubMultiflex::dynParamCallback(const teraranger_mutliflex_cfg::TerarangerHubMultiflexConfig &config, uint32_t level)
{

  if (level == 1 || level == -1)
  {
    sensor_bit_mask[0] = config.Sensor_0 ? 1 : 0;
    sensor_bit_mask[1] = config.Sensor_1 ? 1 : 0;
    sensor_bit_mask[2] = config.Sensor_2 ? 1 : 0;
    sensor_bit_mask[3] = config.Sensor_3 ? 1 : 0;
    sensor_bit_mask[4] = config.Sensor_4 ? 1 : 0;
    sensor_bit_mask[5] = config.Sensor_5 ? 1 : 0;
    sensor_bit_mask[6] = config.Sensor_6 ? 1 : 0;
    sensor_bit_mask[7] = config.Sensor_7 ? 1 : 0;

    sensor_bit_mask_ptr = sensor_bit_mask;

    setSensorBitMask(sensor_bit_mask_ptr);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      ROS_INFO("Sensor %d is set to %d", i, sensor_bit_mask[i]);
    }
  }

  // TODO: Re-enable when multiple modes are supported by Multiflex
  // else if (level == 0)
  // {
  //  if (config.Mode == teraranger_mutliflex_cfg::TerarangerHubMultiflex_Fast)
  //  {
  //    setMode(FAST_MODE);
  // ROS_INFO("Fast mode set");
  //  }
  //
  //  if (config.Mode == teraranger_mutliflex_cfg::TerarangerHubMultiflex_Precise)
  //  {
  //    setMode(PRECISE_MODE);
  // ROS_INFO("Precise mode set");
  //  }
  //
  //  if (config.Mode == teraranger_mutliflex_cfg::TerarangerHubMultiflex_LongRange)
  //  {
  //    setMode(LONG_RANGE_MODE);
  // ROS_INFO("Long range mode set");
  //  }
  // }
  else
  {
    ROS_DEBUG("Dynamic reconfigure, got %d", level);
  }
}

std::string TerarangerHubMultiflex::IntToString(int number)
{
  std::ostringstream oss;
  oss << number;
  return oss.str();
}

void TerarangerHubMultiflex::spin()
{
  static uint8_t buffer[1];
  while(ros::ok())
  {
    if(serial_port_.read(buffer, 1))
    {
      serialDataCallback(buffer[0]);
    }
    else
    {
      ROS_ERROR("Timeout or error while reading serial");
    }
    ros::spinOnce();
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teraranger_hub_multiflex");
  teraranger_array::TerarangerHubMultiflex multiflex;
  multiflex.spin();

  return 0;
}
