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

template<typename STATE>
using StateToStringFunc = std::string(*)(const STATE&);

template<typename ACTION>
using ActionToStringFunc = std::string(*)(const ACTION&);

template<typename STATE, typename ACTION, int ROLE_COUNT>
class StateMachine {
public:
  constexpr StateMachine(
    IsTerminalFunc<STATE> is_terminal,
    EvaluateTerminalStateFunc<STATE, ROLE_COUNT> evaluate_terminal_state,
    GetActiveRoleFunc<STATE> get_active_role,
    GetActionFunc<STATE, ACTION> get_action,
    GetNextStateFunc<STATE, ACTION> get_next_state,
    StateToStringFunc<STATE> state_to_string,
    ActionToStringFunc<ACTION> action_to_string) :
      is_terminal_(is_terminal),
      evaluate_terminal_state_(evaluate_terminal_state),
      get_active_role_(get_active_role),
      get_action_(get_action),
      get_next_state_(get_next_state),
      state_to_string_(state_to_string),
      action_to_string_(action_to_string) {}
  constexpr bool IsTerminal(const STATE& state) {
    return is_terminal_(state);
  }
  constexpr std::array<double, ROLE_COUNT> EvaluateTerminalState(const STATE& state) {
    return evaluate_terminal_state_(state);
  }
  constexpr int GetActiveRole(const STATE& state) {
    return get_active_role_(state);
  }
  constexpr std::vector<ACTION> GetAction(const STATE& state) {
    return get_action_(state);
  }
  constexpr STATE GetNextState(const STATE& state, const ACTION& action) {
    return get_next_state_(state, action);
  }
  constexpr std::string StateToString(const STATE& state) {
    return state_to_string_(state);
  }
  constexpr std::string ActionToString(const ACTION& action) {
    return action_to_string_(action);
  }
private:
  const IsTerminalFunc<STATE> is_terminal_;
  const EvaluateTerminalStateFunc<STATE, ROLE_COUNT> evaluate_terminal_state_;
  const GetActiveRoleFunc<STATE> get_active_role_;
  const GetActionFunc<STATE, ACTION> get_action_;
  const GetNextStateFunc<STATE, ACTION> get_next_state_;
  const StateToStringFunc<STATE> state_to_string_;
  const ActionToStringFunc<ACTION> action_to_string_;
};

template<
    typename STATE,
    typename ACTION,
    int ROLE_COUNT,
    const StateMachine<STATE, ACTION, ROLE_COUNT>& SM>
class Node {
  using NodeType = Node<STATE, ACTION, ROLE_COUNT, SM>;
public:
  Node(const STATE& state) :
    state_(state),
    visit_count_(0),
    is_terminal_(SM.IsTerminal(state)),
    is_expanded_(false),
    role_index_(GetActiveRole(state)) {
  }
  std::array<double, ROLE_COUNT> EvaluateTerminalState() const {
    return SM.EvaluateTerminalState(state_);
  }
  bool IsTerminal() const {
    return is_terminal_;
  }
  std::array<double, ROLE_COUNT> Update(const std::array<double, ROLE_COUNT>& values) {
    ++visit_count_;
    for (auto i = 0; i < ROLE_COUNT; ++i) {
      total_value_[i] += values[i];
    }
    return values;
  }
  void Expand() {
    const auto actions = SM.GetAction(state_);
    for (const auto action : actions) {
      const auto& next_state = SM.GetNextState(state_, action);
      children_.emplace(action, std::make_shared<NodeType>(next_state));
    }
    is_expanded_ = true;
  }
  bool IsExpanded() const {
    return is_expanded_;
  }
  double EvaluateChild(const ACTION& action) const {
    const auto& child = children_.at(action);
    if (child->IsTerminal()) {
      return child->EvaluateTerminalState()[role_index_];
    }
    const auto child_visit_count = child->GetVisitCount();
    if (child_visit_count == 0) {
      return std::numeric_limits<double>::max();
    }
    const auto child_total_value = child->GetTotalValue()[role_index_];
    const auto exploitation_term = child_total_value / child_visit_count;
    const auto exploration_term = 2 * kUCTConstant * std::sqrt((2 * std::log(visit_count_)) / child_visit_count);
    return exploitation_term + exploration_term;
  }
  double EvaluateChildWithRealValue(const ACTION& action) const {
    const auto& child = children_.at(action);
    if (child->IsTerminal()) {
      return child->EvaluateTerminalState()[role_index_];
    }
    const auto child_visit_count = child->GetVisitCount();
    if (child_visit_count == 0) {
      return std::numeric_limits<double>::max();
    }
    const auto child_total_value = child->GetTotalValue()[role_index_];
    const auto exploitation_term = child_total_value / child_visit_count;
    return exploitation_term;
  }
  ACTION GetBestAction() const {
    std::cout << "GetBestAction at:" << std::endl;
    std::cout << SM.StateToString(state_);
    auto max_value = std::numeric_limits<double>::lowest();
    ACTION action_of_max_value;
    for (const auto child : children_) {
      const auto value = EvaluateChild(child.first);
      std::cout << "  " << SM.ActionToString(child.first) << " (" << child.second->GetVisitCount() << ") -> " << value << "(" << EvaluateChildWithRealValue(child.first) << ")" << std::endl;
      if (value > max_value) {
        max_value = value;
        action_of_max_value = child.first;
      }
    }
    std::cout << "  best:" << SM.ActionToString(action_of_max_value) << std::endl;
    return action_of_max_value;
  }
  std::shared_ptr<NodeType> GetBestChild() const {
    return children_.at(GetBestAction());
  }
  std::string ToString() const {
    std::ostringstream oss;
    oss << "State:" << std::endl;
    oss << SM.StateToString(state_) << std::endl;
    for (const auto& child : children_) {
      oss << SM.ActionToString(child.first) << " -> visit:" << child.second->GetVisitCount() << " value:" << EvaluateChild(child.first) << std::endl;
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
  static constexpr double kUCTConstant = 0.5;
};

template<
  class STATE,
  class ACTION,
  int ROLE_COUNT,
  const StateMachine<STATE, ACTION, ROLE_COUNT>& SM>
class Searcher {
  using NodeType = Node<STATE, ACTION, ROLE_COUNT, SM>;
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
    while (!SM.IsTerminal(temp_state)) {
//      std::cout << SM.StateToString(temp_state);
      const auto actions = SM.GetAction(temp_state);
      std::uniform_int_distribution<int> dist(0, actions.size() - 1);
      const auto action = actions[dist(random_engine_)];
//      std::cout << SM.ActionToString(action) << std::endl;
      temp_state = SM.GetNextState(temp_state, action);
    }
//    std::cout << "terminal:" << std::endl;
//    std::cout << SM.StateToString(temp_state);
    return SM.EvaluateTerminalState(temp_state);
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
