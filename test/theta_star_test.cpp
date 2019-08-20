/**
 * MIT License
 *
 * Copyright (c) 2019 Sean Crutchlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file theta_star_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include "theta_star/theta_star.h"

namespace theta_star {
class ThetaStarTest : public ::testing::Test {
 protected:
  std::vector<ThetaStar::coord_t>* moves_ptr;
  std::vector<double>* costs_ptr;

  virtual void SetUp() {
    moves_ptr = new std::vector<ThetaStar::coord_t>{{-1,  0},    /// DOWN
                                                     { 0, -1},    /// RIGHT
                                                     { 1,  0},    /// UP
                                                     { 0,  1}};   /// LEFT

    costs_ptr = new std::vector<double>{1.0, 1.0, 1.0, 1.0};
  }

  virtual void TearDown() {
    delete moves_ptr;
    delete costs_ptr;
  }
};

TEST_F(ThetaStarTest, StartIsGoal) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{0, 0}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(0, 0));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, OutOfBoundsStart) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(5, 6), std::make_pair(5, 5));

  EXPECT_TRUE(path.empty());
}

TEST_F(ThetaStarTest, OutOfBoundsGoal) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(2, 8));

  EXPECT_TRUE(path.empty());
}

TEST_F(ThetaStarTest, OccupiedStart) {
  std::vector<std::vector<int>> grid{{1, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));

  EXPECT_TRUE(path.empty());
}

TEST_F(ThetaStarTest, OccupiedGoal) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 1}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));

  EXPECT_TRUE(path.empty());
}

TEST_F(ThetaStarTest, OpenOppositeEnds) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{0, 0}, {5, 5}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, OpenLShape) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{2, 1}, {3, 4}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(2, 1), std::make_pair(3, 4));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, BlockedLShape1) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 1, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{2, 1}, {3, 1}, {3, 4}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(2, 1), std::make_pair(3, 4));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, BlockedLShape2) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 1, 1, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{2, 1}, {3, 1}, {3, 4}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(2, 1), std::make_pair(3, 4));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, ManhattanOppositeEnds1) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 0},
                                     {0, 0, 1, 1, 0, 0},
                                     {0, 1, 1, 0, 0, 0},
                                     {0, 1, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{0, 0}, {0, 5}, {5, 5}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, ManhattanOppositeEnd2) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 1, 0},
                                     {0, 0, 0, 1, 0, 0},
                                     {0, 0, 1, 0, 0, 0},
                                     {0, 1, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{0, 0}, {5, 0}, {5, 5}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, ManhattanOppositeEnds3) {
  std::vector<std::vector<int>> grid{{0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 1, 0},
                                     {0, 0, 0, 1, 1, 0},
                                     {0, 0, 1, 1, 0, 0},
                                     {0, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0}};

  std::vector<ThetaStar::coord_t> optimal_coords{{0, 0}, {5, 0}, {5, 5}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 5));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, Spiral) {
  std::vector<std::vector<int>> grid{
               // 0  1  2  3  4  5  6  7  8  9  10 11
                 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 0
                 {0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0},   // 1
                 {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},   // 2
                 {0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0},   // 3
                 {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0},   // 4
                 {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0},   // 5
                 {0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0},   // 6
                 {0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0},   // 7
                 {0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0},   // 8
                 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},   // 9
                 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},   // 10
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};  // 11

  std::vector<ThetaStar::coord_t> optimal_coords{
    {0, 0}, {11, 0}, {11, 11}, {0, 11},
    {0, 2}, {9, 2},  {9, 9},   {2, 9},
    {2, 4}, {7, 4},  {7, 7},   {5, 7},
    {5, 6}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 6));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, Maze) {
  std::vector<std::vector<int>> grid{
               // 0  1  2  3  4  5  6  7  8  9  10 11
                 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 0
                 {0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0},   // 1
                 {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},   // 2
                 {0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0},   // 3
                 {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0},   // 4
                 {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0},   // 5
                 {0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0},   // 6
                 {0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},   // 7
                 {0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0},   // 8
                 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},   // 9
                 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},   // 10
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};  // 11

  std::vector<ThetaStar::coord_t> optimal_coords{
    {0, 0}, {11, 0}, {11, 11}, {7, 11},
    {7, 9}, {9, 9},  {9, 5},   {7, 5},
    {7, 7}, {5, 7},  {5, 6}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(0, 0), std::make_pair(5, 6));
  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, Funnel1) {
  std::vector<std::vector<int>> grid{
               // 0  1  2  3  4  5  6  7  8  9  10 11
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 0
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 1
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 2
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 3
                 {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},   // 4
                 {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},   // 5
                 {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},   // 6
                 {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},   // 7
                 {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0},   // 8
                 {0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},   // 9
                 {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},   // 10
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}};  // 11

  std::vector<ThetaStar::coord_t> optimal_coords{{10, 8}, {10, 4},
                                                  {3, 4},  {3, 6},
                                                  {7, 6},  {7, 11}, {10, 11}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(10, 8),
                                   std::make_pair(10, 11));

  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}

TEST_F(ThetaStarTest, Funnel2) {
  std::vector<std::vector<int>> grid{
               // 0  1  2  3  4  5  6  7  8  9  10 11
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 0
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 1
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 2
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // 3
                 {0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0},   // 4
                 {0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0},   // 5
                 {0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0},   // 6
                 {0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0},   // 7
                 {0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0},   // 8
                 {1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0},   // 9
                 {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0},   // 10
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}};  // 11

  std::vector<ThetaStar::coord_t> optimal_coords{{10, 8}, {10, 6},
                                                  {9, 6},  {9, 5},
                                                  {8, 5},  {8, 4},
                                                  {3, 4},  {3, 7},
                                                  {7, 7},  {7, 9},
                                                  {8, 9},  {8, 10},
                                                  {9, 10}, {9, 11},
                                                  {10, 11}};

  ThetaStar ts(grid, *moves_ptr, *costs_ptr);

  std::vector<Node> path = ts.Plan(std::make_pair(10, 8),
                                   std::make_pair(10, 11));

  std::vector<ThetaStar::coord_t> path_coords = ts.Nodes2Pairs(path);

  EXPECT_EQ(path_coords, optimal_coords);
}
}   //  namespace theta_star

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
