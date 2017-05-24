/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name Teraranger_tower nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/console.h>
#include <string>

#include <tr_hub_parser/RangeArray.h>
#include <tr_hub_parser/tr_hub_parser.h>

namespace tr_hub_parser {

Tr_hub_parser::Tr_hub_parser() {
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_,
                             std::string("/dev/ttyACM0"));
  //Initialize local parameters and measurement array
  field_of_view = 0.0593;
  max_range = 14.0;
  min_range = 0.2;
  number_of_sensor = 8;
  frame_id = "base_range_";

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

  // Publishers
  range_publisher_ = nh_.advertise<tr_hub_parser::RangeArray>("tr_hub_parser", 8);

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ =
      boost::bind(&Tr_hub_parser::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_)) {
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

  float res = (float)current_range * 0.001;
  return res;
}

void Tr_hub_parser::serialDataCallback(uint8_t single_character) {
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character != 'T' && buffer_ctr < 19) {
    // not the beginning of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }
  else if (single_character == 'T') {
    //ROS_INFO("%s\n", reinterpret_cast<const char*>(single_character));

    if (buffer_ctr == 19) {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 18);

      if (crc == input_buffer[18]) {
        ROS_DEBUG("Frame: %s ", input_buffer);

        for (size_t i=0; i < measure.ranges.size(); i++) {
          measure.ranges.at(i).header.stamp = ros::Time::now();
          measure.ranges.at(i).header.seq = seq_ctr++;

          float float_range = two_chars_to_float(input_buffer[2 * (i + 1)],input_buffer[2 * (i + 1) + 1]);

          if (float_range < min_range) {
            float_range = min_range;
          } else if (float_range > max_range) {
            float_range = max_range;
          }
          measure.ranges.at(i).range = float_range;
        }
        range_publisher_.publish(measure);

      } else {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    } else {
      ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without "
                "evaluating data",
                ros::this_node::getName().c_str());
    }
  } else {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer",
              ros::this_node::getName().c_str());
  }

  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
} //

// set mode original

void Tr_hub_parser::setMode(const char *c) { serial_port_->sendChar(c); }

void Tr_hub_parser::dynParamCallback(
    const tr_hub_parser::Tr_hub_parserConfig &config, uint32_t level) {
  if (config.Mode == tr_hub_parser::Tr_hub_parser_Fast) {
    setMode(FAST_MODE);
  }

  if (config.Mode == tr_hub_parser::Tr_hub_parser_Precise) {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == tr_hub_parser::Tr_hub_parser_Outdoor) {
    setMode(OUTDOOR_MODE);
  }
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tr_hub_parser_node");
  tr_hub_parser::Tr_hub_parser tr_hub_parser_node;
  ros::spin();

  return 0;
}
