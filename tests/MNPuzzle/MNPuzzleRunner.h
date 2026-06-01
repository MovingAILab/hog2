#pragma once

#include "../Runner.h"
#include "MNPuzzle.h"


// Test runner for the MNPuzzle domain using a 3x3 size
class MNPuzzleRunner : public Runner {
 public:
  MNPuzzleRunner();

  virtual void addAllTests();
  virtual void addTest(std::string);
  void addStates(uint64_t, uint64_t);
  void runTests();

 private:
  void load(uint64_t, uint64_t);
  void bfs(std::shared_ptr<Json::Node>);
  void dfs(std::shared_ptr<Json::Node>);

  MNPuzzle<3, 3> mnPuzzle;
  MNPuzzleState<3, 3> start;
  MNPuzzleState<3, 3> goal;

  std::vector<MNPuzzleState<3, 3>> history;
  std::vector<MNPuzzleState<3, 3>> future;

  std::vector<void (MNPuzzleRunner::*)(std::shared_ptr<Json::Node>)> tests;
  std::vector<std::pair<uint64_t, uint64_t>> states;
};
