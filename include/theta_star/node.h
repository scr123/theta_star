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
 * @file node.h
 * @brief Node class.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef NODE_H_
#define NODE_H_

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

namespace theta_star {
/**
 * @brief      Class for node.
 */
class Node {
 public:
    typedef const std::shared_ptr<Node> const_node_ptr;
    typedef std::shared_ptr<Node> node_ptr;

    /**
     * @brief      Constructs the object.
     */
    Node() {
      this->parent_ = nullptr;
      this->coord_ = std::make_pair(0, 0);
      this->g_ = 0.0;
      this->h_ = std::numeric_limits<double>::max();
      this->f_ = this->g_ + this->h_;
    }

    /**
     * @brief      Constructs the object.
     *
     * @param[in]  _coord  The coordinate
     */
    explicit Node(const std::pair<int, int>& _coord) {
      this->parent_ = nullptr;
      this->coord_ = _coord;
      this->g_ = 0.0;
      this->h_ = std::numeric_limits<double>::max();
      this->f_ = this->g_ + this->h_;
    }

    /**
     * @brief      Constructs the object.
     *
     * @param[in]  _parent  The parent
     * @param[in]  _coord   The coordinate
     * @param[in]  _g       Cost of start node to current node
     * @param[in]  _h       Heuristic cost of current node to goal node
     */
    Node(node_ptr _parent, const std::pair<int, int>& _coord,
         double _g, double _h) {
      this->parent_ = _parent;
      this->coord_ = _coord;
      this->g_ = _g;
      this->h_ = _h;
      this->f_ = this->g_ + this->h_;
    }

    /**
     * @brief      Constructs the object.
     *
     * @param[in]  _other  The other
     */
    Node(const Node& _other) {
      *this = _other;
    }

    /**
     * @brief      Direct assignment operator.
     *
     * @param[in]  _other  The other
     *
     * @return     Node for assignment.
     */
    Node& operator=(const Node& _other) {
      if (_other.parent_ != nullptr) {
        this->parent_ = std::make_shared<Node>(*_other.GetParent());
      } else {
        this->parent_ = nullptr;
      }
      this->coord_ = _other.GetCoord();
      this->f_ = _other.f_;
      this->g_ = _other.g_;
      this->h_ = _other.h_;

      return *this;
    }

    /**
     * @brief      Equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if equal, False otherwise.
     */
    bool operator==(Node const& _other) const {
      return this->coord_ == _other.GetCoord();
    }

    /**
     * @brief      Not equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if not equal, False otherwise.
     */
    bool operator!=(Node const& _other) const {
      return !(*this == _other);
    }

    /**
     * @brief      Less than operator.
     *
     * @param      _other  The other
     *
     * @return     True if less than, False otherwise.
     */
    bool operator<(Node const& _other) const {
      return this->f_ < _other.GetCost();
    }

    /**
     * @brief      Greater than operator.
     *
     * @param      _other  The other
     *
     * @return     True if greater than, False otherwise.
     */
    bool operator>(Node const& _other) const {
      return this->f_ > _other.GetCost();
    }

    /**
     * @brief      Less than or equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if less than or equal to, False otherwise.
     */
    bool operator<=(Node const& _other) const {
      return this->f_ <= _other.GetCost();
    }

    /**
     * @brief      Greater than or equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if greater than or equal to, False otherwise.
     */
    bool operator>=(Node const& _other) const {
      return this->f_ >= _other.GetCost();
    }

    /**
     * @brief      Gets the parent.
     *
     * @return     The parent.
     */
    node_ptr const& GetParent() const {
      return this->parent_;
    }

    /**
     * @brief      Sets the parent.
     *
     * @param[in]  _other  The other
     */
    void SetParent(node_ptr _other) {
      this->parent_ = _other;
    }

    /**
     * @brief      Gets the coordinate.
     *
     * @return     The coordinate.
     */
    std::pair<int, int> const& GetCoord() const {
      return this->coord_;
    }

    /**
     * @brief      Adds cost.
     *
     * @param[in]  _cost  The cost
     */
    void AddCost(const double _cost) {
      this->f_ += _cost;
    }

    /**
     * @brief      Sets the cost.
     *
     * @param[in]  _g     Cost of start node to current node
     */
    void SetCost(const double _g) {
      this->g_ = _g;
      this->f_ = _g + this->h_;
    }

    /**
     * @brief      Sets the cost.
     *
     * @param[in]  _g     Cost of start node to current node
     * @param[in]  _h     Heuristic cost of current node to goal node
     */
    void SetCost(const double _g, const double _h) {
      this->g_ = _g;
      this->h_ = _h;
      this->f_ = _g + _h;
    }

    /**
     * @brief      Gets the cost.
     *
     * @return     The cost.
     */
    double const& GetCost() const {
      return this->f_;
    }

    /**
     * @brief      Calculates the distance.
     *
     * @param      _other  The other
     *
     * @return     The distance.
     */
    double Distance(Node const& _other) {
      return std::sqrt(
        std::pow((this->coord_.first - _other.GetCoord().first), 2.0) +
        std::pow((this->coord_.second - _other.GetCoord().second), 2.0));
    }

    /**
     * @brief      Output operator.
     *
     * @param      _stream  The stream
     * @param[in]  _node    The node
     *
     * @return     Output stream.
     */
    friend std::ostream& operator<<(std::ostream& _stream, const Node& _node) {
      /// Parent coordinates
      int parent_row = -1;
      int parent_col = -1;

      /// Check if parent node is a nullptr
      if (_node.GetParent() != nullptr) {
        parent_row = _node.GetParent()->GetCoord().first;
        parent_col = _node.GetParent()->GetCoord().second;
      }

      /// Populate output stream
      _stream << "Node = [Coord: "
        << _node.GetCoord().first
        << " " << _node.GetCoord().second
        << "  Parent Coord: "
        << parent_row << " " << parent_col
        << "  f: " << _node.GetCost()
        << "  g: " << _node.g_ << "  h: " << _node.h_ << "]";

      return _stream;
    }

 private:
    /// Smart pointer to parent
    node_ptr parent_;
    /// 2D Node Coordinate
    std::pair<int, int> coord_;
    /// Costs: total, movement, heuristic
    double f_, g_, h_;
};
}   // namespace theta_star

#endif  // NODE_H_
