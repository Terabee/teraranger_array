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
	// Get paramters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("portname", portname_, std::string("/dev/ttyACM0"));

	// Publishers
	range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_hub_multiflex", 8);

	// Create serial port
	serial_port_ = new SerialPort();

	// Set callback function for the serial ports
	serial_data_callback_function_ = boost::bind(&TerarangerHubMultiflex::serialDataCallback, this, _1);
	serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

	// Connect serial port
	if (!serial_port_->connect(portname_))
	{
		ros::shutdown();
		return;
	}

	// Output loaded parameters to console for double checking
	ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
	ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

	ns_ = ros::this_node::getNamespace();
	ns_ = ros::names::clean(ns_);
	std::string str = ns_.c_str();
	ROS_INFO("node namespace: [%s]", ns_.c_str());

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

	sensor_msgs::Range sensors[SENSOR_COUNT];
	for (uint8_t i = 0; i < SENSOR_COUNT; i++)
	{
		sensors[i].field_of_view = FIELD_OF_VIEW;
		sensors[i].max_range = MAX_RANGE;
		sensors[i].min_range = MIN_RANGE;
		sensors[i].radiation_type = sensor_msgs::Range::INFRARED;

		std::string frame = "base_range_";
		std::string frame_id = frame + IntToString(i);
		sensors[i].header.frame_id = ros::names::append(ns_, frame_id);
	}

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
			if ((bitmask & bit_compare) == bit_compare)
			{
				if (ranges[i] < int_max_range && ranges[i] > int_min_range)
				{
					sensors[i].header.stamp = ros::Time::now();
					sensors[i].header.seq = seq_ctr++;
					sensors[i].range = ranges[i] * 0.001; // convert to m
					range_publisher_.publish(sensors[i]);
				}
				else
				{
					sensors[i].header.stamp = ros::Time::now();
					sensors[i].header.seq = seq_ctr++;
					sensors[i].range = -1;
					range_publisher_.publish(sensors[i]);
				}
			}
			else
			{
				ROS_WARN_ONCE("Not all sensors activated set proper bitmask using rosrun rqt_reconfigure rqt_reconfigure");
			}
			bit_compare <<= 1;
		}
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
	serial_port_->sendChar(c, 4);
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
	char full_command[5] = {0x00, 0x52, 0x03, (char)bit_mask_hex, (char)crc};

	serial_port_->sendChar(full_command, 5);
}

void TerarangerHubMultiflex::dynParamCallback(const teraranger_mutliflex_cfg::TerarangerHubMultiflexConfig &config, uint32_t level)
{

	if (level == 1)
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teraranger_hub_multiflex");
	teraranger_array::TerarangerHubMultiflex multiflex;
	ros::spin();

	return 0;
}
