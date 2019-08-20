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
 * @file node_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>


#include <sstream>
#include <string>
#include <limits>
#include <utility>

#include "theta_star/node.h"

namespace theta_star {
class ThetaStarTest : public ::testing::Test {
 protected:
  Node::node_ptr parent, child, goal;

  virtual void SetUp() {
    std::pair<int, int> parent_state(1, 2),
                        child_state(3, 4),
                        goal_state(6, 7);
    /*
     * Distances:
     *  parent --> child: 2.82842712475
     *  parent --> goal:  7.07106781187
     *  child  --> goal:  4.24264068712
     */

    parent = std::make_shared<Node>(nullptr,
      parent_state, 0.0, 7.07106781187);

    child = std::make_shared<Node>(parent,
      child_state, 1.0, 4.24264068712);

    goal = std::make_shared<Node>(nullptr,
      goal_state, std::numeric_limits<double>::max(), 0.0);
  }

  virtual void TearDown() {
  }
};

TEST_F(ThetaStarTest, NotGoal) {
  EXPECT_TRUE(*child != *goal);
}

TEST_F(ThetaStarTest, ConstructGoal) {
  Node tmp(*goal);

  EXPECT_TRUE(tmp == *goal);
}

TEST_F(ThetaStarTest, AssignGoal) {
  Node tmp = *goal;

  EXPECT_TRUE(tmp == *goal);
}

TEST_F(ThetaStarTest, AddCost) {
  Node tmp(*child);

  tmp.AddCost(1.2);

  EXPECT_NEAR(tmp.GetCost(), child->GetCost() + 1.2, 1e-8);
}

TEST_F(ThetaStarTest, LowerCost) {
  EXPECT_TRUE(child->GetCost() < parent->GetCost());
}

TEST_F(ThetaStarTest, HigherCost) {
  EXPECT_TRUE(parent->GetCost() > child->GetCost());
}

TEST_F(ThetaStarTest, EqualCostLE) {
  Node tmp(*child);
  EXPECT_TRUE(tmp.GetCost() <= child->GetCost());
}

TEST_F(ThetaStarTest, EqualCostGE) {
  Node tmp(*child);
  EXPECT_TRUE(tmp.GetCost() >= child->GetCost());
}

TEST_F(ThetaStarTest, Parent2GoalDist) {
  double distance = parent->Distance(*goal);

  EXPECT_NEAR(distance, 7.07106781187, 1e-8);
}

TEST_F(ThetaStarTest, Child2GoalDist) {
  double distance = child->Distance(*goal);

  EXPECT_NEAR(distance, 4.24264068712, 1e-8);
}

TEST_F(ThetaStarTest, Parent2ChildDist) {
  double distance = parent->Distance(*child);

  EXPECT_NEAR(distance, 2.82842712475, 1e-8);
}

TEST_F(ThetaStarTest, ParentOutputOperator) {
  std::stringstream output;
  output << *parent;
  std::string expected = "Node = [Coord: 1 2  Parent Coord: -1 -1"
  "  f: 7.07107  g: 0  h: 7.07107]";

  EXPECT_EQ(output.str(), expected);
}

TEST_F(ThetaStarTest, ChildOutputOperator) {
  std::stringstream output;
  output << *child;
  std::string expected = "Node = [Coord: 3 4  Parent Coord: 1 2"
  "  f: 5.24264  g: 1  h: 4.24264]";

  EXPECT_EQ(output.str(), expected);
}
}   //  namespace theta_star

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
