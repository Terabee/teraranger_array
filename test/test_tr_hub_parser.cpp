#include <gtest/gtest.h>
#include <tr_hub_parser/tr_hub_parser.h>

TEST (HubParserTest, callbackTest){
    printf("%s\n", "Test is running");
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
