#include <ros/console.h>
#include <string>

#include <teraranger_array/RangeArray.h>
#include <teraranger_array/teraranger_evo.h>
#include <teraranger_array/helper_lib.h>


namespace teraranger_array
{

TerarangerHubEvo::TerarangerHubEvo()
{
  // Get parameters and namespace
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_,
                             std::string("/dev/ttyACM0"));
  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  if (ns_ != "" && ns_[0] == '/')
  { // Remove first backslash if needed
    ns_.erase(0,1);
  }
  ROS_INFO("node namespace: [%s]", ns_.c_str());

  // Publishers
  range_publisher_ = nh_.advertise<teraranger_array::RangeArray>("ranges", 10);
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);

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

  //Initialize local parameters and measurement array
  field_of_view = 0.03491;
  max_range = 60.0;
  min_range = 0.5;
  number_of_sensor = 8;
  frame_id = "base_range_";

  // Initialize rangeArray
  for (size_t i=0; i < number_of_sensor; i++)
  {
    sensor_msgs::Range range;
    range.field_of_view = field_of_view;
    range.max_range = max_range;
    range.min_range = min_range;
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

  // Initialize IMU message
  boost::array<float,9> lincov = {{0.01, 0.0, 0.0,0.0,0.01,0.0,0.0,0.0,0.01}}; // This is to avoid compiler warning
  boost::array<float,9> oricov = {{0.001, 0.0, 0.0, 0.0,0.001,0.0,0.0,0.0,0.001}}; // This is to avoid compiler warning
  imu_msg.linear_acceleration_covariance = lincov;
  imu_msg.orientation_covariance = oricov;


  // set the right RangeArray and IMU frame depending of the namespace
  if (ns_ == "")
  {
    range_array_msg.header.frame_id = "base_hub";
    imu_msg.header.frame_id = "hub_gyro_link";
  }
  else
  {
    range_array_msg.header.frame_id = "base_" + ns_;
    imu_msg.header.frame_id = ns_ + "_gyro_link";
  }

  // This line is needed to start measurements on the hub
  setMode(BINARY_MODE, 4);
  setMode(TOWER_MODE, 4);
  setMode(RATE_ASAP, 5);
  setMode(IMU_QUAT,4);
  imu_status = quat;
  current_imu_frame_length = IMU_QUAT_FRAME_LENGTH;

  // Enable output
  setMode(ENABLE_CMD, 5);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ =
    boost::bind(&TerarangerHubEvo::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerHubEvo::~TerarangerHubEvo() {}

void TerarangerHubEvo::setMode(const char *c, int length)
{
  if(!serial_port_.write((uint8_t*)c, length))
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerHubEvo::dynParamCallback(
    const teraranger_evo_cfg::TerarangerHubEvoConfig &config, uint32_t level)
{
  switch(level)
  {
    case -1:// Catching first reconfigure call
      break;
    case 0:// Set the mode dynamically
      ROS_INFO("Reconfigure call: Output mode");
      if (config.Output_Mode == teraranger_evo_cfg::TerarangerHubEvo_Binary)
      {
        setMode(BINARY_MODE, 4);
      }
      else if (config.Output_Mode == teraranger_evo_cfg::TerarangerHubEvo_Text)
      {
        setMode(TEXT_MODE, 4);
      }
      else ROS_ERROR("Invalid reconfigure option");
      break;
    case 1:// Set the rate dynamically
      ROS_INFO("Reconfigure call: Rate");
      if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_ASAP)
      {
        setMode(RATE_ASAP, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_700)
      {
        setMode(RATE_700, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_600)
      {
        setMode(RATE_600, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_500)
      {
        setMode(RATE_500, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_250)
      {
        setMode(RATE_250, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_100)
      {
        setMode(RATE_100, 5);
      }
      else if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_50)
      {
        setMode(RATE_50, 5);
      }
      else ROS_ERROR("Invalid reconfigure option");
      break;
    case 2:// Set range mode
      ROS_INFO("Reconfigure call: Range mode");
      if (config.Range_mode == teraranger_evo_cfg::TerarangerHubEvo_Long_range)
      {
        setMode(LONG_RANGE, 4);
        max_range = 60.0;
        min_range = 0.5;
        for (size_t i=0; i < number_of_sensor; i++)
        {
          range_array_msg.ranges[i].max_range = max_range;
          range_array_msg.ranges[i].min_range = min_range;
        }

      }
      else if (config.Range_mode == teraranger_evo_cfg::TerarangerHubEvo_Short_range)
      {
        setMode(SHORT_RANGE, 4);
        max_range = 2.00;
        min_range = 0.00;
        for (size_t i=0; i < number_of_sensor; i++)
        {
          range_array_msg.ranges[i].max_range = max_range;
          range_array_msg.ranges[i].min_range = min_range;
        }
      }
      else ROS_ERROR("Invalid reconfigure option");
      break;
    case 3:// Set the IMU mode dynamically
      ROS_INFO("Reconfigure call: IMU mode");
      if (config.IMU_mode == teraranger_evo_cfg::TerarangerHubEvo_OFF)
      {
        setMode(IMU_OFF,4);
        imu_status = off;
        current_imu_frame_length = 0;
      }
      else if (config.IMU_mode == teraranger_evo_cfg::TerarangerHubEvo_QUAT)
      {
        setMode(IMU_QUAT,4);
        imu_status = quat;
        current_imu_frame_length = IMU_QUAT_FRAME_LENGTH;
      }
      else if (config.IMU_mode == teraranger_evo_cfg::TerarangerHubEvo_EULER)
      {
        setMode(IMU_EULER,4);
        imu_status = euler;
        current_imu_frame_length = IMU_EULER_FRAME_LENGTH;
      }
      else if (config.IMU_mode == teraranger_evo_cfg::TerarangerHubEvo_QUATLIN)
      {
        setMode(IMU_QUATLIN,4);
        imu_status = quatlin;
        current_imu_frame_length = IMU_QUATLIN_FRAME_LENGTH;
      }
      else ROS_ERROR("Invalid reconfigure option");
      break;
    case 4://Set the sequence mode dynamically
      ROS_INFO("Reconfigure call: Sequence mode");
      if(config.Sequence_mode == teraranger_evo_cfg::TerarangerHubEvo_Crosstalk)
      {
        setMode(CROSSTALK_MODE,4);
      }
      else if(config.Sequence_mode == teraranger_evo_cfg::TerarangerHubEvo_Non_crosstalk)
      {
        setMode(NONCROSSTALK_MODE,4);
      }
      else if(config.Sequence_mode == teraranger_evo_cfg::TerarangerHubEvo_Tower_mode)
      {
        setMode(TOWER_MODE,4);
      }
      else ROS_ERROR("Invalid reconfigure option");
      break;
    default:
      ROS_ERROR("Invalid reconfigure level : %d", level);
      break;
  }
}

void TerarangerHubEvo::processRangeFrame(uint8_t* input_buffer, int seq_ctr)
{
  //Processing full range frame
  uint8_t crc = HelperLib::crc8(input_buffer, RANGES_FRAME_LENGTH-1);

  if (crc == input_buffer[RANGE_CRC_POS])
  {
    for (size_t i=0; i < range_array_msg.ranges.size(); i++)
    {
      range_array_msg.ranges.at(i).header.stamp = ros::Time::now();
      range_array_msg.ranges.at(i).header.seq = seq_ctr++;

      // Convert bytes to range
      // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
      char c1 = input_buffer[2 * (i + 1)];
      char c2 = input_buffer[2 * (i + 1) + 1];
      //ROS_DEBUG("c1 : %x, c2 : %x", (c1 & 0x0FF), (c2 & 0x0FF));
      int16_t current_range = (c1 & 0x0FF) << 8;
      current_range |= (c2 & 0x0FF);

      // Checking for hardware extreme values
      float float_range = (float)current_range * VALUE_TO_METER_FACTOR;
      float final_range;
      if(current_range == TOO_CLOSE_VALUE || current_range == 255)// Too close, 255 is for short range
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
      ROS_DEBUG("Value int : %d | float : %f | final_range : %f", current_range, float_range, final_range);

      range_array_msg.ranges.at(i).range = final_range;
    }
    range_array_msg.header.seq = (int)seq_ctr/8;
    range_array_msg.header.stamp = ros::Time::now();
    range_publisher_.publish(range_array_msg);
  }
  else
  {
    ROS_ERROR("[%s] Range frame crc missmatch", ros::this_node::getName().c_str());
  }
}

void TerarangerHubEvo::processImuFrame(uint8_t* input_buffer, int seq_ctr)
{
  int imu_length = (int)(current_imu_frame_length-4)/2;
  int16_t imu[imu_length]; // create array with right number of 16bits values
  uint8_t crc = 0;
  //ROS_DEBUG("Current buffer : %s", input_buffer);
  crc = HelperLib::crc8(input_buffer, current_imu_frame_length-1);
  //ROS_DEBUG("IMU frame length : [%d]", current_imu_frame_length);

  if (crc == input_buffer[current_imu_frame_length-1])
  {
    for (int i = 0; i < imu_length; i++)
    {
        imu[i] = input_buffer[2*(i+1)+1] << 8;
        imu[i] |= input_buffer[2*(i+1)+2];
    }

    if (imu_status == euler)// euler YY PP RR
    {
      imu_msg.orientation.x = imu[2]/16384.0;
      imu_msg.orientation.y = imu[1]/16384.0;
      imu_msg.orientation.z = imu[0]/16384.0;
      imu_msg.orientation.w = 1.0;

      //Resetting acceleration
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 0.0;
    }
    else if(imu_status == quat)// quaternion WW XX YY ZZ
    {
      imu_msg.orientation.w = imu[0]/16384.0;
      imu_msg.orientation.x = imu[1]/16384.0;
      imu_msg.orientation.y = imu[2]/16384.0;
      imu_msg.orientation.z = imu[3]/16384.0;

      //Resetting acceleration
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 0.0;
    }
    else if(imu_status == quatlin)// quaternion WW XX YY ZZ + lin. accel. XX YY ZZ
    {
      imu_msg.orientation.w = imu[0]/16384.0;
      imu_msg.orientation.x = imu[1]/16384.0;
      imu_msg.orientation.y = imu[2]/16384.0;
      imu_msg.orientation.z = imu[3]/16384.0;

      //linear acceleration
      imu_msg.linear_acceleration.x = imu[4]/100.0;
      imu_msg.linear_acceleration.y = imu[5]/100.0;
      imu_msg.linear_acceleration.z = imu[6]/100.0;
    }

    imu_msg.header.seq = seq_ctr;
    imu_msg.header.stamp = ros::Time::now();
    imu_publisher_.publish(imu_msg);
  }
  else
  {
    ROS_ERROR("[%s] Imu frame crc missmatch : computed was : %d and expected was: %d", ros::this_node::getName().c_str(), crc, (uint8_t)input_buffer[current_imu_frame_length-1]);
  }
}

void TerarangerHubEvo::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  //ROS_DEBUG("Buffer counter %d : %s | current char : %c", buffer_ctr, input_buffer, (char)single_character);

  if (buffer_ctr == 0)
  {
    if (single_character == 'T' || single_character == 'I')
    {
      // Waiting for T or an I
      input_buffer[buffer_ctr++] = single_character;
      //ROS_DEBUG("Waiting for T or I | current char : %c", (char)single_character);
      return;
    }
  }
  else if (buffer_ctr == 1)
  {
    if (single_character == 'H' || single_character == 'M')
    {
      // Waiting for H after a T or an M after an I
      input_buffer[buffer_ctr++] = single_character;
      //ROS_DEBUG("Waiting for H or M | current char : %c", (char)single_character);
      return;
    }
  }

  else if (buffer_ctr > 1)
  {
    if (input_buffer[0] == 'T')// Parsing ranges
    {
      // Gathering after-header range data
      if (buffer_ctr < RANGES_FRAME_LENGTH)
      {
        //ROS_DEBUG("Gathering range data | current char : %c", (char)single_character);
        input_buffer[buffer_ctr++] = single_character;
        return;
      }
      else if (buffer_ctr == RANGES_FRAME_LENGTH)
      {
        processRangeFrame(input_buffer, seq_ctr);
      }
      else if (buffer_ctr > RANGES_FRAME_LENGTH)
      {
        ROS_DEBUG("[%s] : Buffer overflow, resetting buffer without "
                  "evaluating data",
                  ros::this_node::getName().c_str());
      }
    }
    else if (input_buffer[0] == 'I')// Parsing Imu
    {
      // ROS_INFO("%d", imu_status);
      //ROS_DEBUG("IMU frame length : [%d]", current_imu_frame_length);
      //ROS_DEBUG("Gathering imu data | current char : %c", (char)single_character);
      // Gathering after-header imu data
      if (buffer_ctr < current_imu_frame_length)
      {
        input_buffer[buffer_ctr++] = single_character;
        return;
      }
      else if (buffer_ctr == current_imu_frame_length)
      {
        processImuFrame(input_buffer, seq_ctr);
      }
      else if (buffer_ctr > current_imu_frame_length)
      {
        ROS_DEBUG("[%s] : Buffer overflow, resetting buffer without "
                  "evaluating data",
                  ros::this_node::getName().c_str());
      }
    }
    //ROS_DEBUG("Resetting buffer | current char : %c", (char)single_character);
    // resetting buffer and ctr
    buffer_ctr = 0;
    bzero(&input_buffer, BUFFER_SIZE);

    // Appending current char to hook next frame
    if (single_character == 'T' || single_character == 'I')
    {
      input_buffer[buffer_ctr++] = single_character;
    }
  }
}

void TerarangerHubEvo::spin()
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
  setMode(DISABLE_CMD, 5);
}

}// end of namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "teraranger_hub_evo");
  teraranger_array::TerarangerHubEvo node;
  node.spin();

  return 0;
}
