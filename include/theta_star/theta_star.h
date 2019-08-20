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
 * @file theta_star.h
 * @brief Header file for theta_star.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef THETA_STAR_H_
#define THETA_STAR_H_

/// System
#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

/// Library

/// Project
#include <theta_star/node.h>

namespace theta_star {
/**
 * @brief      Class for theta star.
 */
class ThetaStar {
 public:
  const int FREE = 0;
  const int OCCUPIED = 1;

  typedef std::pair<int, int> coord_t;

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _moves  The moves
   * @param[in]  _costs  The costs
   */
  ThetaStar(const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs);

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _grid   The grid
   * @param[in]  _moves  The moves
   * @param[in]  _costs  The costs
   */
  ThetaStar(const std::vector<std::vector<int>>& _grid,
    const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs);

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _grid   The grid
   * @param[in]  _rows   The rows
   * @param[in]  _cols   The cols
   * @param[in]  _moves  The moves
   * @param[in]  _costs  The costs
   */
  ThetaStar(const std::vector<int>& _grid,
    const int _rows,
    const int _cols,
    const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs);

  /**
   * @brief      Sets the grid.
   *
   * @param[in]  _grid  The grid
   */
  void SetGrid(const std::vector<std::vector<int>>& _grid);

  /**
   * @brief      Sets the grid.
   *
   * @param[in]  _grid  The grid
   * @param[in]  _rows  The rows
   * @param[in]  _cols  The cols
   */
  void SetGrid(const std::vector<int>& _grid,
               const int _rows,
               const int _cols);

  /**
   * @brief      Plans optimal path.
   *
   * @param[in]  _start  The start
   * @param[in]  _goal   The goal
   *
   * @return     Path.
   */
  std::vector<Node> Plan(const coord_t& _start,
    const coord_t& _goal);

  /**
   * @brief      Utility to convert nodes to coordinate pairs
   *
   * @param[in]  _nodes  The nodes
   *
   * @return     The coordinate pairs.
   */
  std::vector<coord_t> Nodes2Pairs(const std::vector<Node>& _nodes);

 private:
  /**
   * @brief      Generates the neighbors.
   *
   * @param[in]  _node  The node
   * @param[in]  _goal  The goal
   *
   * @return     The neighbors.
   */
  std::vector<Node> GenerateNeighbors(const Node& _node, const Node& _goal);

  /**
   * @brief      Updates nodes.
   *
   * @param      _neighbor  The neighbor
   * @param      _open      The open
   */
  void Update(Node* _neighbor, std::vector<Node>* _open);

  /**
   * @brief      Checks line of sight.
   *
   * @param[in]  _node      The node
   * @param[in]  _neighbor  The neighbor
   *
   * @return     True if line of sight, False otherwise.
   */
  bool LineOfSight(const Node& _node, const Node& _neighbor);

  /**
   * @brief      Determines if valid move.
   *
   * @param[in]  _coord  The coordinate
   *
   * @return     True if valid move, False otherwise.
   */
  bool IsValidMove(const coord_t& _coord);

  /**
   * @brief      Determines if valid segment.
   *
   * @param[in]  _a    The start coordinate
   * @param[in]  _b    The end coordinate
   *
   * @return     True if valid segment, False otherwise.
   */
  bool IsValidSegment(const coord_t& _a,
                      const coord_t& _b);

  /**
   * @brief      Reconstructs path from goal node to start node.
   *
   * @param[in]  _node  The node
   *
   * @return     Path.
   */
  std::vector<Node> ReconstructPath(const Node& _node);

  /**
   * @brief      Calculates the distance.
   *
   * @param[in]  _lhs  The left hand side
   * @param[in]  _rhs  The right hand side
   *
   * @return     The distance.
   */
  double Distance(const coord_t& _lhs,
                  const coord_t& _rhs);

  /**
   * @brief      Generates index.
   *
   * @param[in]  _coord  The coordinate
   *
   * @return     The index.
   */
  const int GenerateIndex(const coord_t& _coord);

  /**
   * @brief      Hash for std::pair<T1,T2>
   */
  struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& _p) const {
      auto a = std::hash<T1>{}(_p.first);
      auto b = std::hash<T2>{}(_p.second);
      return a ^ b;
    }
  };

  /// Occupancy Grid
  std::vector<int> grid_;
  int rows_;
  int cols_;
  /// Valid moves
  std::vector<coord_t> moves_;
  /// Costs for valid moves
  std::vector<double> costs_;
};
}   // namespace theta_star

#endif  // THETA_STAR_H_