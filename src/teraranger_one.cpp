#include <ros/console.h>
#include <string>

#include <teraranger_array/RangeArray.h>
#include <teraranger_array/teraranger_one.h>
#include <teraranger_array/helper_lib.h>


namespace teraranger_array
{

TerarangerHubOne::TerarangerHubOne()
{
  // Get parameters and namespace
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_,
                             std::string("/dev/ttyACM0"));
  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  if (ns_ != "" && ns_[0] == '/'){ // Remove first backslash if needed
    ns_.erase(0,1);
  }
  ROS_INFO("node namespace: [%s]", ns_.c_str());

  // Publishers
  range_publisher_ = nh_.advertise<teraranger_array::RangeArray>("ranges", 8);

  // Serial Port init
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
  ROS_INFO("[%s] is up and running with the following parameters:",
           ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(),
           portname_.c_str());

  // Set operation Mode
  setMode(BINARY_MODE);

  //Initialize  members
  field_of_view = 0.0593;
  max_range = 14.0;
  min_range = 0.2;
  number_of_sensors = 8;
  frame_id = "base_range_";

  // Initialize data structure
  for (int i = 0; i < number_of_sensors; i++)
  {
    sensor_msgs::Range range;
    range.field_of_view = field_of_view;
    range.max_range = max_range;
    range.min_range = min_range;
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.range = 0.0;
    // set the right range frame depending of the namespace
    if (ns_ == ""){
      range.header.frame_id = frame_id + boost::lexical_cast<std::string>(i);
    }
    else{
      range.header.frame_id = ns_ + '_'+ frame_id + boost::lexical_cast<std::string>(i);
    }
    measure.ranges.push_back(range);
  }
  // set the right RangeArray frame depending of the namespace
  if (ns_ == ""){
    measure.header.frame_id = "base_hub";
  }
  else{
    measure.header.frame_id = "base_" + ns_;// Remove first slash
  }

  // Dynamic reconfigure
  dyn_param_server_callback_function_ =
      boost::bind(&TerarangerHubOne::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerHubOne::~TerarangerHubOne() {}

void TerarangerHubOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character != 'T' && buffer_ctr < 19)
  {
    // not the beginning of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }
  else if (single_character == 'T')
  {

    if (buffer_ctr == 19)
    {
      // end of feed, calculate
      int16_t crc = HelperLib::crc8(input_buffer, 18);

      if (crc == input_buffer[18])
      {
        ROS_DEBUG("Frame: %s ", input_buffer);

        for (size_t i = 0; i < measure.ranges.size(); i++)
        {
          measure.ranges.at(i).header.stamp = ros::Time::now();
          measure.ranges.at(i).header.seq = seq_ctr++;

          // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
          char c1 = input_buffer[2 * (i + 1)];
          char c2 = input_buffer[2 * (i + 1) + 1];
          int16_t current_range = (c1 & 0x0FF) << 8;
          current_range |= (c2 & 0x0FF);

          float float_range = HelperLib::two_chars_to_float(c1, c2) * VALUE_TO_METER_FACTOR;
          float final_range;

          if(current_range == TOO_CLOSE_VALUE)// Too close, 255 is for short range
          {
            final_range = -std::numeric_limits<float>::infinity();
          }
          else if(current_range == OUT_OF_RANGE_VALUE)// Out of range
          {
            final_range = std::numeric_limits<float>::infinity();
          }
          else if(current_range == INVALID_MEASURE_VALUE)// Not connected
          {
            final_range = std::numeric_limits<float>::quiet_NaN();
          }
          // Enforcing min and max range
          else if(float_range > max_range)
          {
            final_range = std::numeric_limits<float>::infinity();
          }
          else if(float_range < min_range)
          {
            final_range = -std::numeric_limits<float>::infinity();
          }
          else// Convert to meters
          {
            final_range = float_range;
          }
          measure.ranges.at(i).range = final_range;
        }
        measure.header.seq = (int) seq_ctr / 8;
        measure.header.stamp = ros::Time::now();
        range_publisher_.publish(measure);
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] received T but did not expect it, reset buffer without "
                "evaluating data",
                ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer",
              ros::this_node::getName().c_str());
  }

  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}

void TerarangerHubOne::setMode(const char *c)
{
  if(!serial_port_.write((uint8_t*)c, CMD_BYTE_LENGTH))
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerHubOne::dynParamCallback(
    const teraranger_one_cfg::TerarangerHubOneConfig &config, uint32_t level)
{
  if (config.Mode == teraranger_one_cfg::TerarangerHubOne_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == teraranger_one_cfg::TerarangerHubOne_Precise)
  {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == teraranger_one_cfg::TerarangerHubOne_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }
}

void TerarangerHubOne::spin()
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
  ros::init(argc, argv, "teraranger_hub_one");
  teraranger_array::TerarangerHubOne teraranger_one;
  teraranger_one.spin();

  return 0;
}
