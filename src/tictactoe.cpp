#include <iostream>
#include <array>
#include <cassert>
#include "node.hpp"

constexpr int kBoardSize = 3;

enum class TTTCell {
  empty, x, o
};

// STATE for TicTacToe
struct TTTState {
  std::array<std::array<TTTCell, kBoardSize>, kBoardSize> board;
  int active_role;
};

// ACTION for TicTacToe
using TTTAction = std::pair<int, int>;

// Hasher for action
namespace std
{
template<typename T, typename U>
struct hash<pair<T, U> > {
  typedef pair<T, U> argument_type;
  typedef size_t result_type;
  result_type operator()(const argument_type& a) const {
    hash<T> hasher_first;
    hash<U> hasher_second;
    return hasher_first(a.first) * 31 + hasher_second(a.second);
  }
};
}

bool HasColumn(const TTTState& state, TTTCell cell) {
  for (auto row = 0; row < kBoardSize; ++row) {
    bool line = true;
    for (auto col = 0; col < kBoardSize; ++col) {
      if (state.board[col][row] != cell) {
        line = false;
        break;
      }
    }
    if (line) {
      return true;
    }
  }
  return false;
}

bool HasRow(const TTTState& state, TTTCell cell) {
  for (auto col = 0; col < kBoardSize; ++col) {
    bool line = true;
    for (auto row = 0; row < kBoardSize; ++row) {
      if (state.board[col][row] != cell) {
        line = false;
        break;
      }
    }
    if (line) {
      return true;
    }
  }
  return false;
}


bool HasDiagonalTopLeft(const TTTState& state, TTTCell cell) {
  for (auto i = 0; i < kBoardSize; ++i) {
    if (state.board[i][i] != cell) {
      return false;
    }
  }
  return true;
}

bool HasDiagonalTopRight(const TTTState& state, TTTCell cell) {
  for (auto i = 0; i < kBoardSize; ++i) {
    if (state.board[i][kBoardSize - i - 1] != cell) {
      return false;
    }
  }
  return true;
}

int CountCell(const TTTState& state, TTTCell cell) {
  auto count = 0;
  for (auto col = 0; col < kBoardSize; ++col) {
    for (auto row = 0; row < kBoardSize; ++row) {
      if (state.board[col][row] == cell) {
        ++count;
      }
    }
  }
  return count;
}

bool HasCell(const TTTState& state, TTTCell cell) {
  for (auto col = 0; col < kBoardSize; ++col) {
    for (auto row = 0; row < kBoardSize; ++row) {
      if (state.board[col][row] == cell) {
        return true;
      }
    }
  }
  return false;
}

bool IsTerminal(const TTTState& state) {
  return
      HasColumn(state, TTTCell::x) ||
      HasColumn(state, TTTCell::o) ||
      HasRow(state, TTTCell::x) ||
      HasRow(state, TTTCell::o) ||
      HasDiagonalTopLeft(state, TTTCell::x) ||
      HasDiagonalTopRight(state, TTTCell::o) ||
      !HasCell(state, TTTCell::empty);
}

std::array<double, 2> EvaluateTerminalState(const TTTState& state) {
  if (HasColumn(state, TTTCell::x) || HasRow(state, TTTCell::x) || HasDiagonalTopLeft(state, TTTCell::x)) {
    return std::array<double, 2>{{1.0, -1.0}};
  } else if (HasColumn(state, TTTCell::o) || HasRow(state, TTTCell::o) || HasDiagonalTopLeft(state, TTTCell::o)) {
    return std::array<double, 2>{{-1.0, 1.0}};
  } else {
    return std::array<double, 2>{{0.0, 0.0}};
  }
}

int GetActiveRole(const TTTState& state) {
  return state.active_role;
}

std::vector<TTTAction> GetAction(const TTTState& state) {
  std::vector<TTTAction> actions;
  for (auto col = 0; col < kBoardSize; ++col) {
    for (auto row = 0; row < kBoardSize; ++row) {
      if (state.board[col][row] == TTTCell::empty) {
        actions.emplace_back(col, row);
      }
    }
  }
  return actions;
}

TTTState GetNextState(const TTTState& state, const TTTAction& action) {
  auto next_state = state;
  next_state.active_role = state.active_role == 0 ? 1 : 0;
  next_state.board[action.first][action.second] = state.active_role == 0 ? TTTCell::x : TTTCell::o;
  return next_state;
}

std::string StateToString(const TTTState& state) {
  std::ostringstream oss;
  for (auto row = 0; row < kBoardSize; ++row) {
    for (auto col = 0; col < kBoardSize; ++col) {
      switch (state.board[col][row]) {
      case TTTCell::x:
        oss << 'x';
        break;
      case TTTCell::o:
        oss << 'o';
        break;
      case TTTCell::empty:
        oss << 'e';
        break;
      default:
        assert(false);
        break;
      }
    }
    oss << std::endl;
  }
  return oss.str();
}

std::string ActionToString(const TTTAction& action) {
  std::ostringstream oss;
  oss << "(" << action.first << ", " << action.second << ")";
  return oss.str();
}


extern constexpr mcts::StateMachine<TTTState, TTTAction, 2> kTTTSM(IsTerminal, EvaluateTerminalState, GetActiveRole, GetAction, GetNextState, StateToString, ActionToString);
using TTTSearcher = mcts::Searcher<TTTState, TTTAction, 2, kTTTSM>;

int main() {
  TTTState root_state;
  root_state.active_role = 0;
  for (auto col = 0; col < kBoardSize; ++col) {
    for (auto row = 0; row < kBoardSize; ++row) {
      root_state.board[col][row] = TTTCell::empty;
    }
  }
  TTTSearcher searcher(root_state);
  for (auto i = 0; i < 10000; ++i) {
    searcher.SearchOnce();
  }
  std::cout << searcher.ToString() << std::endl;
}
