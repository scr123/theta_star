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
 * @file theta_star.cpp
 * @brief Source file for theta_star.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "theta_star/theta_star.h"

namespace theta_star {

  ThetaStar::ThetaStar(const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs)
    : moves_(_moves),
      costs_(_costs) {
        /// Each move has respective cost
        assert((moves_.size() == costs_.size()));
  }

  ThetaStar::ThetaStar(const std::vector<std::vector<int>>& _grid,
    const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs)
    : moves_(_moves),
      costs_(_costs) {
        /// Each move has respective cost
        assert((moves_.size() == costs_.size()));
        /// Grid is populated
        assert(!_grid.empty());

        // TODO(Sean): Check that each vector slice is the same size
        //             generating a m x n matrix
        SetGrid(_grid);
  }

  ThetaStar::ThetaStar(const std::vector<int>& _grid,
    const int _rows,
    const int _cols,
    const std::vector<coord_t>& _moves,
    const std::vector<double>& _costs)
    : moves_(_moves),
      costs_(_costs),
      rows_(_rows),
      cols_(_cols),
      grid_(_grid) {
        /// Each move has respective cost
        assert((moves_.size() == costs_.size()));
        /// Grid is populated
        assert(!_grid.empty());
  }

  void ThetaStar::SetGrid(const std::vector<std::vector<int>>& _grid) {
    rows_ = _grid.size();
    cols_ = _grid[0].size();

    std::vector<int> tmp;
    for (auto it : _grid)
      /// Concatenate grid rows forming 1D vector
      tmp.insert(tmp.end(), it.begin(), it.end());

    /// Assignment operator updating class level grid
    grid_ = tmp;
  }

  void ThetaStar::SetGrid(const std::vector<int>& _grid,
                          const int _rows,
                          const int _cols) {
    rows_ = _rows;
    cols_ = _cols;
    grid_ = _grid;
  }

  std::vector<Node> ThetaStar::Plan(const coord_t& _start,
                                    const coord_t& _goal) {
    /// Check if grid is empty
    assert(!grid_.empty() && "Plan used with empty grid");
    /// Path to be returned
    std::vector<Node> path;
    /// Check start & goal cells to see if valid
    if (IsValidMove(_start) == false
        || IsValidMove(_goal) == false)
      return path;

    /// Open set of nodes to explore
    std::vector<Node> open;
    /// Closed set of nodes already explored
    std::unordered_set<coord_t, hash_pair> closed;

    /// Start and goal nodes
    Node start_node(nullptr, _start, 0.0, Distance(_start, _goal));
    Node goal_node(_goal);

    /// Start node is first on open set
    open.push_back(start_node);

    /// Continue exploring as long as open set is not empty
    while (!open.empty()) {
      /// Find next node in open set with lowest cost
      auto min_it = std::min_element(open.begin(), open.end());

      /// Copy next node to explore, and erase from open
      Node curr_node(*min_it);
      open.erase(min_it);

      /// Base case: goal reached
      if (curr_node == goal_node) {
        path = ReconstructPath(curr_node);
        break;
      }
      /// Insert current node into closed set
      closed.insert(curr_node.GetCoord());
      /// Generate neighbors for current node
      std::vector<Node> neighbors = GenerateNeighbors(curr_node, goal_node);

      /// Iterate over neighbors
      for (auto it : neighbors) {
        /// Check if neighbor has already been visited or is already in open set
        if (closed.find(it.GetCoord()) == closed.end()
          && std::find(open.begin(), open.end(), it) == open.end()) {
            /// Update neighbor node
            Update(&it, &open);
        }
      }
    }
    return path;
  }

  std::vector<ThetaStar::coord_t> ThetaStar::Nodes2Pairs(
    const std::vector<Node>& _nodes) {
    std::vector<coord_t> coords;

    /// Extract coordinates from all nodes
    for (auto it : _nodes)
      coords.push_back(it.GetCoord());

    return coords;
  }

  std::vector<Node> ThetaStar::GenerateNeighbors(const Node& _node,
                                                 const Node& _goal) {
    /// Valid neighbors
    std::vector<Node> neighbors;

    /// Loop to generate combinations of neighbors
    /// and respective costs
    for (int i = 0; i < moves_.size(); i++) {
      /// Generate new coordinates
      coord_t coord_prime(
        (moves_[i].first + _node.GetCoord().first),
        (moves_[i].second + _node.GetCoord().second));

      /// Construct node
      Node neighbor(std::make_shared<Node>(_node),
        coord_prime,
        costs_[i],
        Distance(_node.GetCoord(), _goal.GetCoord()));

      /// Check if coordinates are valid
      if (IsValidMove(neighbor.GetCoord()) == true) {
        /// Add neighbor to valid neighbors
        neighbors.push_back(neighbor);
      }
    }
    return neighbors;
  }

  void ThetaStar::Update(Node* _neighbor, std::vector<Node>* _open) {
    /// Current and previous node_ptr to keep track of last
    /// valid node for line of sight
    Node::node_ptr curr = _neighbor->GetParent();
    Node::node_ptr valid = curr;

    /// Theta* Variant
    while (curr != nullptr) {
      /// Check if there is line of sight between
      /// current and neighbor nodes
      if (LineOfSight(*curr, *_neighbor) == true) {
        /// Store previous parent
        valid = curr;
      }
      curr = curr->GetParent();
    }

    if (curr != valid) {
      _neighbor->SetParent(valid);
      _neighbor->SetCost(_neighbor->Distance(*valid));
    }
    _open->push_back(*_neighbor);
  }

  bool ThetaStar::LineOfSight(const Node& _start, const Node& _end) {
    /// Extract coordinates from nodes (need this so they become mutable)
    coord_t start = _start.GetCoord();
    coord_t end = _end.GetCoord();

    /// Calculate deltas
    int dx = end.first - start.first;
    int dy = end.second - start.second;

    /// Check direction of slope
    bool is_steep = std::abs(dy) > std::abs(dx);

    /// Apply line rotation swapping components
    if (is_steep == true) {
      std::swap(start.first, start.second);
      std::swap(end.first, end.second);
    }

    /// Swap start and end points if slope is negative
    if (start.first > end.first) {
      std::swap(start.first, end.first);
      std::swap(start.second, end.second);
    }

    /// Re-calculate deltas
    dx = end.first - start.first;
    dy = end.second - start.second;

    /// Calculate error and step size required for generating
    /// a line from cell to cell
    int error = (dy / 2.0);
    int y_step = (start.second < end.second) ? 1 : -1;

    /// Calculate y-value w.r.t x-value
    int y = start.second;
    coord_t prev_coord = (is_steep == true)
      ? std::make_pair(y, start.first) : std::make_pair(start.first, y);

    for (int x = start.first; x < (end.first + 1); x++) {
      /// Generate new coordinate following the line segment
      coord_t coord = (is_steep == true)
        ? std::make_pair(y, x) : std::make_pair(x, y);

      /// Check if next coordinate following line is valid
      if (IsValidMove(coord) == false
          || IsValidSegment(prev_coord, coord) == false) {
        return false;
      }
      // coords.push_back(coord);
      prev_coord = coord;

      /// Update error and step-size
      error -= std::abs(dy);
      if (error < 0) {
        y += y_step;
        error += dx;
      }
    }
    return true;
  }

  std::vector<Node> ThetaStar::ReconstructPath(const Node& _node) {
    /// Optimal path from start to goal
    std::vector<Node> path;

    /// Construct node_ptr for traversal to check if nullptr.
    /// Memory for shared_ptr is allocated due to base case
    /// of start and goal coordinates being the same.
    Node::node_ptr curr = std::make_shared<Node>(_node);

    /// Traversal from goal node to start node
    while (curr != nullptr) {
      path.push_back(*curr);
      curr = curr->GetParent();
    }

    /// Reverse path to be in-order
    std::reverse(path.begin(), path.end());

    return path;
  }

  bool ThetaStar::IsValidMove(const coord_t& _coord) {
    /// Check if coordinate is within grid
    if ((_coord.first >= 0 && _coord.first < rows_)
      && (_coord.second >= 0 && _coord.second < cols_)) {
      /// Check that cell is not occupied
      if (grid_[GenerateIndex(_coord)] != OCCUPIED) {
        return true;
      }
    }
    return false;
  }

  bool ThetaStar::IsValidSegment(const coord_t& _a,
                                 const coord_t& _b) {
    /// Calculate deltas for segment
    int dx = _b.first - _a.first;
    int dy = _b.second - _a.second;

    /// Check if segment is diagonal
    if (((_a.first == 0 && _a.second == 0)
          || (_b.first == 0 && _b.second == 0))
          || (dx != 0 && dy != 0)) {
      coord_t neighbor_left(_a.first + dx, _a.second);
      coord_t neighbor_right(_a.first, _a.second + dy);

      if (IsValidMove(neighbor_left) == true
          && IsValidMove(neighbor_right) == true) {
        return true;
      }
      return false;
    }
    /// dx or dy equals 0
    return true;
  }

  double ThetaStar::Distance(const coord_t& _lhs,
    const coord_t& _rhs) {
    /// Get distance between two coordinates
    return std::sqrt(
      std::pow((_lhs.first - _rhs.first), 2.0) +
      std::pow((_lhs.second - _rhs.second), 2.0));
  }

  const int ThetaStar::GenerateIndex(const coord_t& _coord) {
    return static_cast<int>(((_coord.first * cols_) + _coord.second));
  }
}   // namespace theta_star
