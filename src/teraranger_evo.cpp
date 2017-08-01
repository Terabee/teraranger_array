#include <ros/console.h>
#include <string>

#include <teraranger_hub/RangeArray.h>
#include <teraranger_hub/teraranger_evo.h>
#include <teraranger_hub/helper_lib.h>

namespace teraranger_hub
{

TerarangerHubEvo::TerarangerHubEvo()
{
    // Get parameters and namespace
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("portname", portname_,
                               std::string("/dev/ttyACM0"));
    ns_ = ros::this_node::getNamespace();
    ns_ = ros::names::clean(ns_);
    ROS_INFO("node namespace: [%s]", ns_.c_str());

    // Publishers
    range_publisher_ = nh_.advertise<teraranger_hub::RangeArray>("teraranger_evo", 8);

    // Serial Port init
    serial_port_ = new SerialPort();
    if (!serial_port_->connect(portname_)) {
      ROS_ERROR("Could not open : %s ", portname_.c_str());
      ros::shutdown();
      return;
    }
    else {
      serial_data_callback_function_ =
          boost::bind(&TerarangerHubEvo::serialDataCallback, this, _1);
      serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);
    }

    // Output loaded parameters to console for double checking
    ROS_INFO("[%s] is up and running with the following parameters:",
             ros::this_node::getName().c_str());
    ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(),
             portname_.c_str());

    //Initialize local parameters and measurement array
    field_of_view = 0.0593;
    max_range = 14.0;
    min_range = 0.2;
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
     if (ns_ == ""){
       range.header.frame_id = frame_id + boost::lexical_cast<std::string>(i);
     }
     else{
       range.header.frame_id = ns_.erase(0,1) + '_'+ frame_id + boost::lexical_cast<std::string>(i);
     }
     measure.ranges.push_back(range);
   }

   // set the right RangeArray frame depending of the namespace
   if (ns_ == ""){
     measure.header.frame_id = "base_hub";
   }
   else{
     measure.header.frame_id = "base_" + ns_.erase(0,1);// Remove first slash
   }

    // This line is needed to start measurements on the hub
    setMode(ENABLE_CMD, 5);
    setMode(BINARY_MODE, 4);
    setMode(TOWER_MODE, 4);
    setMode(RATE_ASAP, 5);

    // Dynamic reconfigure
    dyn_param_server_callback_function_ =
        boost::bind(&TerarangerHubEvo::dynParamCallback, this, _1, _2);
    dyn_param_server_.setCallback(dyn_param_server_callback_function_);
  }

  TerarangerHubEvo::~TerarangerHubEvo() {}

  void TerarangerHubEvo::setMode(const char *c, int length) {
    serial_port_->sendChar(c, length);
  }

  void TerarangerHubEvo::dynParamCallback(
      const teraranger_evo_cfg::TerarangerHubEvoConfig &config, uint32_t level) {

    // Set the mode dynamically
    if (config.Mode == teraranger_evo_cfg::TerarangerHubEvo_Binary) {
      setMode(BINARY_MODE, 4);
    }
    if (config.Mode == teraranger_evo_cfg::TerarangerHubEvo_Text) {
      setMode(TEXT_MODE, 4);
    }

    if (config.Range_mode == teraranger_evo_cfg::TerarangerHubEvo_Long_range) {
      setMode(LONG_RANGE, 4);
    }
    if (config.Range_mode == teraranger_evo_cfg::TerarangerHubEvo_Short_range) {
      setMode(SHORT_RANGE, 4);
    }

    // Set the rate dynamically
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_ASAP){
      setMode(RATE_ASAP, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_700){
      setMode(RATE_700, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_600){
      setMode(RATE_600, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_500){
      setMode(RATE_500, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_250){
      setMode(RATE_250, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_100){
      setMode(RATE_100, 5);
    }
    if (config.Rate == teraranger_evo_cfg::TerarangerHubEvo_50){
      setMode(RATE_50, 5);
    }

  }

  void TerarangerHubEvo::serialDataCallback(uint8_t single_character) {
    static uint8_t input_buffer[BUFFER_SIZE];
    static int buffer_ctr = 0;
    static int seq_ctr = 0;

    ROS_DEBUG("Buffer of size %d : %s | current char : %c", buffer_ctr, input_buffer, (char)single_character);

    if (single_character == 'T' && buffer_ctr == 0) {
      // Waiting for T
      input_buffer[buffer_ctr++] = single_character;
      return;
    }
    else if (single_character == 'H' && buffer_ctr == 1) {
      // Waiting for H after a T
      input_buffer[buffer_ctr++] = single_character;
      return;
    }

    // Gathering after-header range data
    if (buffer_ctr > 1 && buffer_ctr < RANGES_FRAME_LENGTH) {
      input_buffer[buffer_ctr++] = single_character;
      return;
    }
    else if (buffer_ctr == RANGES_FRAME_LENGTH) {
      //Processing full range frame
      int16_t crc = HelperLib::crc8(input_buffer, 19);

      if (crc == input_buffer[RANGE_CRC_POS]) {
          //ROS_DEBUG("Frame of size %d : %s ", buffer_ctr, input_buffer);

          for (size_t i=0; i < measure.ranges.size(); i++) {
            measure.ranges.at(i).header.stamp = ros::Time::now();
            measure.ranges.at(i).header.seq = seq_ctr++;

            // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
            char c1 = input_buffer[2 * (i + 1)];
            char c2 = input_buffer[2 * (i + 1) + 1];
            ROS_DEBUG("c1 : %x, c2 : %x", (c1 & 0x0FF), (c2 & 0x0FF));
            int16_t current_range = (c1 & 0x0FF) << 8;
            current_range |= (c2 & 0x0FF);

            float float_range = (float)current_range;
            ROS_DEBUG("Value int : %d | float : %f", current_range, float_range);

            if (current_range <= 1 || current_range == 255) {
              float_range = -1.0;
            } else {
              float_range = float_range * 0.001;
            }
            measure.ranges.at(i).range = float_range;
          }
          measure.header.seq = (int) seq_ctr / 8;
          measure.header.stamp = ros::Time::now();
          range_publisher_.publish(measure);
      } else {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    } else if (buffer_ctr > RANGES_FRAME_LENGTH){
        ROS_DEBUG("[%s] : Buffer overflow, resetting buffer without "
                  "evaluating data",
                  ros::this_node::getName().c_str());
    }
    // resetting buffer and ctr
    buffer_ctr = 0;
    bzero(&input_buffer, BUFFER_SIZE);

    // Appending current char to hook next frame
    input_buffer[buffer_ctr++] = single_character;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teraranger_hub_evo");
  teraranger_hub::TerarangerHubEvo node;
  ros::spin();

  return 0;
}
