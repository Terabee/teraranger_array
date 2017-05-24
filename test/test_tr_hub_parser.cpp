#include <gtest/gtest.h>
#include <tr_hub_parser/tr_hub_parser.h>

class HubParserTest : public ::testing::Test{
protected:
  tr_hub_parser::Tr_hub_parser *parser;
  uint8_t input_buffer[BUFFER_SIZE] = {0x54,0x48,0x08,0xa7,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0b};//expected result is one 2.22 meter measurement
  virtual void SetUp(){
    parser = new tr_hub_parser::Tr_hub_parser;
  }

  virtual void TearDown(){
    delete parser;
  }
};

TEST_F(HubParserTest, crc8Test){
  int16_t crc = parser->crc8(input_buffer, 18);
  EXPECT_EQ(crc, input_buffer[18]);
}

TEST_F(HubParserTest, parsingTest){
  // for(size_t i = 0; i < BUFFER_SIZE; i++){
  //   parser->serialDataCallback(input_buffer[i]);
  // }
  parser->build_frame(input_buffer);
  float result = parser->getMeasure()->ranges.at(1).range;
  ASSERT_FLOAT_EQ(result, 2.22);
}

int main(int argc, char **argv) {
  // static uint8_t input_buffer[BUFFER_SIZE] = {0x54,0x48,0x08,0xa7,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0b};//expected result is one 2.22 meter measurement
  // tr_hub_parser::Tr_hub_parser parser;
  //ros::init(argc, argv, "tr_hub_parser_node");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
