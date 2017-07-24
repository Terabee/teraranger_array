#include <ros/console.h>
#include <string>
#include <teraranger_hub/RangeArray.h>
#include <teraranger_hub/teraranger_one.h>
#include <teraranger_hub/crc_lib.h>


namespace teraranger_hub
{

TerarangerHubOne::TerarangerHubOne()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_,
                             std::string("/dev/ttyACM0"));

  for (uint8_t i = 0; i < number_of_sensors; i++)
  {
    sensor_msgs::Range range;
    range.field_of_view = field_of_view;
    range.max_range = max_range;
    range.min_range = min_range;
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.range = 0.0;
    range.header.frame_id =
        ros::names::append(ns_, frame_id + boost::lexical_cast<std::string>(i));

    measure.ranges.push_back(range);
  }

  // Publishers
  range_publisher_ = nh_.advertise<teraranger_hub::RangeArray>("teraranger_hub_one", 8);

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ =
      boost::bind(&TerarangerHubOne::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
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

  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  std::string str = ns_.c_str();
  ROS_INFO("node namespace: [%s]", ns_.c_str());

  // Set operation Mode
  setMode(BINARY_MODE);

  // Force using 8 sensors
  // setMode(FORCE_8_SENSORS);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ =
      boost::bind(&TerarangerHubOne::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerHubOne::~TerarangerHubOne() {}

float two_chars_to_float(uint8_t c1, uint8_t c2)
{
  int16_t current_range = c1 << 8;
  current_range |= c2;

  float res = (float)current_range;
  return res;
}

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
      int16_t crc = Crc::crc8(input_buffer, 18);

      if (crc == input_buffer[18])
      {
        ROS_DEBUG("Frame: %s ", input_buffer);

        for (size_t i = 0; i < measure.ranges.size(); i++)
        {
          measure.ranges.at(i).header.stamp = ros::Time::now();
          measure.ranges.at(i).header.seq = seq_ctr++;

          // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
          float float_range = two_chars_to_float(input_buffer[2 * (i + 1)], input_buffer[2 * (i + 1) + 1]);

          if ((float_range * 0.001 < min_range) && (float_range > 0))
          { //check for hardware cut-off
            float_range = min_range;
          }
          else if ((float_range * 0.001 > max_range) || (float_range < 0))
          { //software cut-off should be adapted to sensor
            float_range = -1.0;
          }
          else
          {
            float_range = float_range * 0.001;
          }
          measure.ranges.at(i).range = float_range;
        }
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

void TerarangerHubOne::setMode(const char *c) { serial_port_->sendChar(c, 3); }

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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teraranger_hub_one");
  teraranger_hub::TerarangerHubOne teraranger_one;
  ros::spin();

  return 0;
}
