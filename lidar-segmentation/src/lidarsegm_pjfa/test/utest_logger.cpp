#include <iostream>

// Bring in my package's API, which is what I'm testing
#include "../include/logger.h"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuiteLogger, testCaseSetSeverityLevel)
{
  lipe::logging::Logger logger_;
  lipe::logging::SeverityLevel value;
  
  value = lipe::logging::SeverityLevel::trace;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());

  value = lipe::logging::SeverityLevel::debug;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());

  value = lipe::logging::SeverityLevel::info;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());  

  value = lipe::logging::SeverityLevel::warning;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());  

  value = lipe::logging::SeverityLevel::error;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());  

  value = lipe::logging::SeverityLevel::fatal;
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(value, logger_.GetSeverityLevel());  
}

TEST(TestSuiteLogger, testCaseSetSeverityLevel_OutOfRange)
{
  lipe::logging::Logger logger_;
  lipe::logging::SeverityLevel value;
  lipe::logging::SeverityLevel return_value = lipe::logging::SeverityLevel::error;
  
  value = lipe::logging::SeverityLevel(int(lipe::logging::SeverityLevel::fatal) + 1);
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(return_value, logger_.GetSeverityLevel()); 

  value = lipe::logging::SeverityLevel(int(lipe::logging::SeverityLevel::trace) - 1);
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(return_value, logger_.GetSeverityLevel());   

  value = lipe::logging::SeverityLevel(100);
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(return_value, logger_.GetSeverityLevel());   

  value = lipe::logging::SeverityLevel(-100);
  logger_.SetSeverityLevel(value);
  EXPECT_EQ(return_value, logger_.GetSeverityLevel());   
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}