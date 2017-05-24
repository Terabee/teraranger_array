#include <gtest/gtest.h>
#include <tr_hub_parser/tr_hub_parser.h>

class HubParserTest : public ::testing::Test{
protected:
  uint8_t input_buffer[BUFFER_SIZE] = {0x54,0x48,0x08,0xa7,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0b};//expected result is one 2.22 meter measurement
};

TEST_F(HubParserTest, crc8Test){
  int16_t crc = tr_hub_parser::crc8(input_buffer, 18);
  EXPECT_EQ(crc, input_buffer[18]);
}

// TEST_F(HubParserTest, parsingTest){
//   // for(size_t i = 0; i < BUFFER_SIZE; i++){
//   //   parser->serialDataCallback(input_buffer[i]);
//   // }
//   parser->build_frame(input_buffer);
//   float result = parser->getMeasure()->ranges.at(1).range;
//   ASSERT_FLOAT_EQ(result, 2.22);
// }

TEST_F(HubParserTest, parsingTest){
  float result = tr_hub_parser::two_chars_to_float(input_buffer[2],input_buffer[3]);
  ASSERT_FLOAT_EQ(result, 2.214999999);
}

int main(int argc, char **argv) {
  // static uint8_t input_buffer[BUFFER_SIZE] = {0x54,0x48,0x08,0xa7,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0b};//expected result is one 2.22 meter measurement
  // tr_hub_parser::Tr_hub_parser parser;
  //ros::init(argc, argv, "tr_hub_parser_node");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
