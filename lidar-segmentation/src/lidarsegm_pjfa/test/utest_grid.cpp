#include <iostream>

// Bring in my package's API, which is what I'm testing
#include "../include/grid.h"

// Bring in gtest
#include <gtest/gtest.h>
#include <logger.h>

// Declare a test
TEST(TestSuiteGrid_GetCellForPoint, testCaseCentralCell)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int exp_i, exp_j;
  lipe::PCLMyPointType point;

  // central cell
  point.x = 0.0;
  point.y = 0.0;
  exp_i = 100;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = 0.01;
  point.y = -0.01;
  exp_i = 100;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = -0.01;
  point.y = 0.01;
  exp_i = 100;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
}


TEST(TestSuiteGrid_GetCellForPoint, testCaseNextCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int exp_i, exp_j;
  lipe::PCLMyPointType point;

  // next cells:
  point.x = -0.5;
  point.y = 0.5;
  exp_i = 101;
  exp_j = 99;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = 0.5;
  point.y = -0.5;
  exp_i = 99;
  exp_j = 101;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
  
  point.x = -0.5;
  point.y = 0.5;
  exp_i = 101;
  exp_j = 99;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = 0.5;
  point.y = -0.5;
  exp_i = 99;
  exp_j = 101;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
}


TEST(TestSuiteGrid_GetCellForPoint, testCaseCellBorders)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int exp_i, exp_j;
  lipe::PCLMyPointType point;

  // cell borders:
  point.x = 0.25001;
  point.y = 0.25001;
  exp_i = 99;
  exp_j = 99;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
  
  point.x = 0.24999;
  point.y = 0.24999;
  exp_i = 100;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = -0.25001;
  point.y = -0.25001;
  exp_i = 101;
  exp_j = 101;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
  
  point.x = -0.24999;
  point.y = -0.24999;
  exp_i = 100;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );  
}

TEST(TestSuiteGrid_GetCellForPoint, testCaseGridBorders)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int exp_i, exp_j;
  lipe::PCLMyPointType point;

  // grid border:
  point.x = +50.2499;
  point.y = 0.0;
  exp_i = 0;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = -50.2499;
  point.y = 0.0;
  exp_i = 200;
  exp_j = 100;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = 0.0;
  point.y = +50.2499;
  exp_i = 100;
  exp_j = 0;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
  
  point.x = 0.0;
  point.y = -50.2499;
  exp_i = 100;
  exp_j = 200;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
}


TEST(TestSuiteGrid_GetCellForPoint, testCaseFurtherCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int exp_i, exp_j;
  lipe::PCLMyPointType point;

  // further cells:
  point.x = 10.0;
  point.y = -3.5;
  exp_i = 80;
  exp_j = 107;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );

  point.x = -50.01;
  point.y = 1.51;
  exp_i = 200;
  exp_j = 97;
  EXPECT_EQ( grid.GetCellForPoint(point), &(grid.cells_[exp_i][exp_j]) );
}


TEST(TestSuiteGrid_GetCellForPoint, testCaseOutOfGridBorder)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;

  point.x = +50.25001;
  point.y = 0;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );

  point.x = -50.25001;
  point.y = 0;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );

  point.x = 0;
  point.y = +50.25001;  
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );

  point.x = 0;
  point.x = -50.25001;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );      
}


TEST(TestSuiteGrid_GetCellForPoint, testCaseOutOfGridFar)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;

  point.x = +123;
  point.y = 0;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );

  point.x = -34234;
  point.y = +34545;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );

  point.x = -342304;
  point.y = -345145;
  EXPECT_EQ( grid.GetCellForPoint(point), nullptr );
}

TEST(TestSuiteGrid_GetCellDistanceFromEgo, testCaseGetCellDistanceFromEgo)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int i, j;
  float exp_d;
  
  i = 100;
  j = 100;
  exp_d = 0.0;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );
}

TEST(TestSuiteGrid_GetCellDistanceFromEgo, testCaseNearbyCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int i, j;
  float exp_d;

  i = 101;
  j = 100;
  exp_d = 0.5;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );

  i = 99;
  j = 100;
  exp_d = 0.5;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 100;
  j = 101;
  exp_d = 0.5;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 100;
  j = 99;
  exp_d = 0.5;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );    
}

TEST(TestSuiteGrid_GetCellDistanceFromEgo, testCaseDiagonallyNearbyCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int i, j;
  float exp_d;
  
  i = 101;
  j = 101;
  exp_d = 0.5 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j) , exp_d );

  i = 99;
  j = 99;
  exp_d = 0.5 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 99;
  j = 101;
  exp_d = 0.5 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 101;
  j = 99;
  exp_d = 0.5 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );    
}


TEST(TestSuiteGrid_GetCellDistanceFromEgo, testCaseBorderCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int i, j;
  float exp_d;
  
  i = 0;
  j = 100;
  exp_d = 50.0;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j) , exp_d );

  i = 200;
  j = 100;
  exp_d = 50.0;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 100;
  j = 0;
  exp_d = 50.0;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 100;
  j = 200;
  exp_d = 50.0;
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );    
}


TEST(TestSuiteGrid_GetCellDistanceFromEgo, testCaseCornerCells)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  int i, j;
  float exp_d;
  
  i = 0;
  j = 0;
  exp_d = 50 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j) , exp_d );

  i = 200;
  j = 0;
  exp_d = 50 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 0;
  j = 200;
  exp_d = 50 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );  

  i = 200;
  j = 200;
  exp_d = 50 * sqrt(2);
  EXPECT_FLOAT_EQ( grid.GetCellDistanceFromEgo(i,j), exp_d );    
}


// check if GetCellForPoint and checkIfInputPointInRange functions are consistent.
TEST(TestSuiteGrid_AddPoint_Consistency, testCaseInsideOfGrid)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;
  point.z = 0.0f;
  bool answer1;
  bool answer2;

  point.x = 0;
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = -1;
  point.y = 1;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = -1;
  point.y = -1;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 1;
  point.y = -1;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );      

  point.x = 1;
  point.y = 1;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );      
}

TEST(TestSuiteGrid_AddPoint_Consistency, testCaseInsideOfGridBorder)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;
  point.z = 0.0f;
  const float eps = 0.001f;
  bool answer1;
  bool answer2;

  point.x = 50.25 - eps;      // 50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = -50.25 + eps;     // -50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = 50.25 - eps;        // 50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = -50.25 + eps;       // -50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );      
}

TEST(TestSuiteGrid_AddPoint_Consistency, testCaseExactlyOnOfGridBorder)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;
  point.z = 0.0f;
  bool answer1;
  bool answer2;

  point.x = 50.25;      // 50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = -50.25;     // -50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = 50.25;        // 50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = -50.25;       // -50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );     
}

TEST(TestSuiteGrid_AddPoint_Consistency, testCaseOutsideOfGridBorder)
{
  lipe::LidarConfig lidar_config;
  lipe::GridConfig grid_config(
    201,  // rows
    201,  // columns
    lipe::default_values::grid_config::dist_for_extra_min_points,                         // dist
    0.5,                                                                                  // c_size
    lipe::default_values::grid_config::input_point_lower_threshold_relative_to_ground,    // input_point_lower_threshold_relative_to_ground_
    lipe::default_values::grid_config::input_point_higher_threshold_relative_to_ground    // input_point_higher_threshold_relative_to_ground_
  );
  lipe::Grid grid(grid_config, lidar_config);

  lipe::PCLMyPointType point;
  point.z = 0.0f;
  const float eps = 0.001f;
  bool answer1;
  bool answer2;


  point.x = 50.25 + eps;      // 50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = -50.25 - eps;     // -50.25 is the grid border.
  point.y = 0;
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = 50.25 + eps;        // 50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );

  point.x = 0;    
  point.y = -50.25 - eps;       // -50.25 is the grid border.
  answer1 = (grid.GetCellForPoint(point) == nullptr);
  answer2 = (grid.checkIfInputPointInRange(point.x, point.y) == false);
  EXPECT_EQ( answer1, answer2 );     
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
