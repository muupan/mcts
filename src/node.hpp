#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>
#include <memory>
#include <unordered_map>
#include <array>
#include <random>
#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>

namespace mcts {

// Whether a given is terminal or not
template<typename STATE>
using IsTerminalFunc = bool(*)(const STATE&);

// Get the value of a given terminal state
template<typename STATE, const int ROLE_COUNT>
using EvaluateTerminalStateFunc = std::array<double, ROLE_COUNT>(*)(const STATE&);

// Get the active role
template<typename STATE>
using GetActiveRoleFunc = int(*)(const STATE&);

// Get legal actions of a given state
template<typename STATE, typename ACTION>
using GetActionFunc = std::vector<ACTION>(*)(const STATE&);

// Get the next state
template<typename STATE, typename ACTION>
using GetNextStateFunc = STATE(*)(const STATE&, const ACTION&);

template<
    typename STATE,
    typename ACTION,
    const int ROLE_COUNT,
    IsTerminalFunc<STATE> IS_TERMINAL,
    EvaluateTerminalStateFunc<STATE, ROLE_COUNT> EVALUATE_TERMINAL_STATE,
    GetActiveRoleFunc<STATE> GET_ACTIVE_ROLE,
    GetActionFunc<STATE, ACTION> GET_ACTION,
    GetNextStateFunc<STATE, ACTION> GET_NEXT_STATE>
class Node {
  using NodeType = Node<STATE, ACTION, ROLE_COUNT, IS_TERMINAL, EVALUATE_TERMINAL_STATE, GET_ACTIVE_ROLE, GET_ACTION, GET_NEXT_STATE>;
public:
  Node(const STATE& state) :
    state_(state),
    visit_count_(0),
    is_terminal_(IS_TERMINAL(state)),
    is_expanded_(false),
    role_index_(GET_ACTIVE_ROLE(state)) {
  }
  std::array<double, ROLE_COUNT> EvaluateTerminalState() const {
    return EVALUATE_TERMINAL_STATE(state_);
  }
  bool IsTerminal() const {
    return is_terminal_;
  }
//  void Update(double value) {
//    ++visit_count_;
//    total_value_ += value;
//  }
  std::array<double, ROLE_COUNT> Update(const std::array<double, ROLE_COUNT>& values) {
    ++visit_count_;
    for (auto i = 0; i < ROLE_COUNT; ++i) {
      total_value_[i] += values[i];
    }
    return values;
  }
  void Expand() {
    const auto actions = GET_ACTION(state_);
    for (const auto action : actions) {
      const auto& next_state = GET_NEXT_STATE(state_, action);
      children_.emplace(action, std::make_shared<NodeType>(next_state));
    }
    is_expanded_ = true;
  }
  bool IsExpanded() const {
    return is_expanded_;
  }
  double EvaluateChild(const ACTION& action) const {
    const auto& child = children_.at(action);
    const auto child_visit_count = child->GetVisitCount();
    if (child_visit_count == 0) {
      return std::numeric_limits<double>::max();
    }
    const auto child_total_value = child->GetTotalValue()[role_index_];
    const auto exploitation_term = child_total_value / child_visit_count;
    const auto exploration_term = 2 * kUCTConstant * std::sqrt((2 * std::log(visit_count_)) / child_visit_count);
    return exploitation_term + exploration_term;
  }
  ACTION GetBestAction() const {
    auto max_value = std::numeric_limits<double>::lowest();
    ACTION action_of_max_value;
    for (const auto child : children_) {
      const auto value = EvaluateChild(child.first);
      if (value > max_value) {
        max_value = value;
        action_of_max_value = child.first;
      }
    }
    return action_of_max_value;
  }
  std::shared_ptr<NodeType> GetBestChild() const {
    return children_.at(GetBestAction());
  }
  std::string ToString() const {
    std::ostringstream oss;
    for (const auto& child : children_) {
      //TODO Needs ToString for ACTION
      oss << "(" << child.first.first << ", " << child.first.second << ")" << " -> visit:" << child.second->GetVisitCount() << " value:" << EvaluateChild(child.first) << std::endl;
//      oss << child.first << " -> visit:" << child.second->GetVisitCount() << " value:" << EvaluateChild(child.first) << std::endl;
    }
    return oss.str();
  }
  int GetVisitCount() const {
    return visit_count_;
  }
  STATE GetState() const {
    return state_;
  }
  std::array<double, ROLE_COUNT> GetTotalValue() const {
    return total_value_;
  }
private:
  STATE state_;
  std::unordered_map<ACTION, std::shared_ptr<NodeType>> children_;
  int visit_count_;
  std::array<double, ROLE_COUNT> total_value_;
  const bool is_terminal_;
  bool is_expanded_;
  const int role_index_;
  static constexpr double kUCTConstant = 1.0;
};

template<
    typename STATE,
    typename ACTION,
    const int ROLE_COUNT,
    IsTerminalFunc<STATE> IS_TERMINAL,
    EvaluateTerminalStateFunc<STATE, ROLE_COUNT> EVALUATE_TERMINAL_STATE,
    GetActiveRoleFunc<STATE> GET_ACTIVE_ROLE,
    GetActionFunc<STATE, ACTION> GET_ACTION,
    GetNextStateFunc<STATE, ACTION> GET_NEXT_STATE>
class Searcher {
  using NodeType = Node<STATE, ACTION, ROLE_COUNT, IS_TERMINAL, EVALUATE_TERMINAL_STATE, GET_ACTIVE_ROLE, GET_ACTION, GET_NEXT_STATE>;
public:
  Searcher(const STATE& root_state) : root_(std::make_shared<NodeType>(root_state)) {
  }
  void SearchOnce() {
    SearchOnePryRecursively(root_);
  }
  std::array<double, ROLE_COUNT> SearchOnePryRecursively(const std::shared_ptr<NodeType>& node) {
    if (node->IsTerminal()) {
      return node->Update(node->EvaluateTerminalState());
    } else if (node->IsExpanded()) {
      return node->Update(SearchOnePryRecursively(node->GetBestChild()));
    } else {
      node->Expand();
      const auto new_node = node->GetBestChild();
      return node->Update(new_node->Update(Simulate(new_node->GetState())));
    }
  }

  std::array<double, ROLE_COUNT> Simulate(const STATE& state) {
    auto temp_state = state;
    while (!IS_TERMINAL(temp_state)) {
      const auto actions = GET_ACTION(temp_state);
      std::uniform_int_distribution<int> dist(0, actions.size() - 1);
      const auto action = actions[dist(random_engine_)];
      temp_state = GET_NEXT_STATE(temp_state, action);
    }
    return EVALUATE_TERMINAL_STATE(temp_state);
  }

  std::string ToString() const {
    return root_->ToString();
  }

private:
  std::shared_ptr<NodeType> root_;
  std::mt19937 random_engine_;
};

}

#endif /* NODE_HPP_ */
