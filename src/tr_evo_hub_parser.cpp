#include <ros/console.h>
#include <string>

#include <tr_hub_parser/RangeArray.h>
#include <tr_hub_parser/tr_evo_hub_parser.h>

namespace tr_hub_parser {

Tr_hub_parser::Tr_hub_parser() {

    // Get parameters
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("portname", portname_,
                               std::string("/dev/ttyACM0"));
    // Publishers
    range_publisher_ = nh_.advertise<tr_hub_parser::RangeArray>("tr_hub_parser", 8);

    // Serial Port init
    serial_port_ = new SerialPort();
    if (!serial_port_->connect(portname_)) {
      ROS_ERROR("Could not open : %s ", portname_.c_str());
      ros::shutdown();
      return;
    }
    else {
      serial_data_callback_function_ =
          boost::bind(&Tr_hub_parser::serialDataCallback, this, _1);
      serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);
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

    //Initialize local parameters and measurement array
    field_of_view = 0.0593;
    max_range = 14.0;
    min_range = 0.2;
    number_of_sensor = 8;
    frame_id = "base_range_";

    // Initialize rangeArray
    for (size_t i=0; i < number_of_sensor; i++) {
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

    // This line is needed to start measurements on the hub
    setMode(ENABLE_CMD, 5);
    setMode(BINARY_MODE, 4);
    setMode(CROSSTALK_MODE, 4);
    setMode(RATE_700, 5);

    // Dynamic reconfigure
    dyn_param_server_callback_function_ =
        boost::bind(&Tr_hub_parser::dynParamCallback, this, _1, _2);
    dyn_param_server_.setCallback(dyn_param_server_callback_function_);
  }

  Tr_hub_parser::~Tr_hub_parser() {}

  uint8_t crc8(uint8_t *p, uint8_t len) {
    uint16_t i;
    uint16_t crc = 0x0;

    while (len--) {
      i = (crc ^ *p++) & 0xFF;
      crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
    }
    return crc & 0xFF;
  }

  float two_chars_to_float(uint8_t c1, uint8_t c2){
    int16_t current_range = c1 << 8;
    current_range |= c2;

    float res = (float)current_range;
    return res;
  }

  void Tr_hub_parser::setMode(const char *c, int length) {
    serial_port_->sendChar(c, length);
  }

  void Tr_hub_parser::dynParamCallback(
      const tr_hub_parser::Tr_hub_parserConfig &config, uint32_t level) {
    if (config.Mode == tr_hub_parser::Tr_hub_parser_Fast) {
      // setMode(FAST_MODE);
    }

    if (config.Mode == tr_hub_parser::Tr_hub_parser_Precise) {
      // setMode(PRECISE_MODE);
    }

    if (config.Mode == tr_hub_parser::Tr_hub_parser_Outdoor) {
      // setMode(OUTDOOR_MODE);
    }
  }

  void Tr_hub_parser::serialDataCallback(uint8_t single_character) {
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
      int16_t crc = crc8(input_buffer, 19);

      if (crc == input_buffer[RANGE_CRC_POS]) {
          ROS_DEBUG("Frame of size %d : %s ", buffer_ctr, input_buffer);

          for (size_t i=0; i < measure.ranges.size(); i++) {
            measure.ranges.at(i).header.stamp = ros::Time::now();
            measure.ranges.at(i).header.seq = seq_ctr++;

            // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
            float float_range = two_chars_to_float(input_buffer[2 * (i + 1)],input_buffer[2 * (i + 1) + 1]);

            if ((float_range * 0.001 < min_range) && (float_range > 0)) { //check for hardware cut-off
              float_range = min_range;
            } else if ((float_range * 0.001 > max_range) || (float_range < 0)) { //software cut-off should be adapted to sensor
              float_range = -1.0;
            } else {
              float_range = float_range * 0.001;
            }
            measure.ranges.at(i).range = float_range;
          }
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

void Tr_hub_parser::setMode(const char *c) { serial_port_->sendChar(c); }

void Tr_hub_parser::dynParamCallback(
    const tr_hub_parser::Tr_hub_parserConfig &config, uint32_t level) {
  if (config.Mode == tr_hub_parser::Tr_hub_parser_Fast) {
    // setMode(FAST_MODE);
  }

  if (config.Mode == tr_hub_parser::Tr_hub_parser_Precise) {
    // setMode(PRECISE_MODE);
  }

  if (config.Mode == tr_hub_parser::Tr_hub_parser_Outdoor) {
    // setMode(OUTDOOR_MODE);
  }
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tr_hub_parser_node");
  tr_hub_parser::Tr_hub_parser tr_hub_parser_node;
  ros::spin();

  return 0;
}
