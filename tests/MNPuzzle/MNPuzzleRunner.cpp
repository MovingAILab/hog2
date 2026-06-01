#include "MNPuzzleRunner.h"
#include "Timer.h"
#include "BFS.h"
#include "DFS.h"


MNPuzzleRunner::MNPuzzleRunner()  { }

// Adds all of the test alogrithms and a few states to traverse
void MNPuzzleRunner::addAllTests() {
  addTest("bfs");
  addStates(0, 1);
  addStates(124, 14131);
}

// Adds an algorithm to test
void MNPuzzleRunner::addTest(std::string testName) {
  std::transform(testName.begin(), testName.end(), testName.begin(),
               [](unsigned char c){ return std::tolower(c); });

  if (testName == "bfs") {
    tests.push_back(&MNPuzzleRunner::bfs);
  } else if (testName == "dfs") {
    tests.push_back(&MNPuzzleRunner::dfs);
  }
}

// Sets the puzzle start and end states using 64 bit hash values
void MNPuzzleRunner::load(uint64_t s, uint64_t g) {
  mnPuzzle.GetStateFromHash(start, s);
  mnPuzzle.GetStateFromHash(goal, g);
}

// Addes a start and end state to traverse in the tests
void MNPuzzleRunner::addStates(uint64_t s, uint64_t g) {
  states.push_back({s, g});
}

// Run all added tests on all added paths
void MNPuzzleRunner::runTests() {
  for (auto &pair : states) {
    load(pair.first, pair.second);
    auto statesLog = log.addChild(std::to_string(pair.first) + "->" + std::to_string(pair.second));
    for (auto &test : tests) {
      (this->*test)(statesLog);
    }
  }

  openLogFile();
  logFile << log;
  closeLogFile();
}

// Runs BFS on the mn puzzle domain
void MNPuzzleRunner::bfs(std::shared_ptr<Json::Node> statesLog) {
  auto bfsLog = statesLog->addChild("bfs");

  try {
    Timer t;
    BFS<MNPuzzleState<3, 3>, MNPuzzle<3, 3>, MNPuzzle<3, 3>> bfs;
    bfs.SetNodeLimit(20000000);
    t.StartTimer();
    bfs.GetPath(&mnPuzzle, start, goal, future);
    t.EndTimer();
    printf("BFS to goal complete in %1.2fs: %llu states reached; path length %lu.\n",
           t.GetElapsedTime(), bfs.GetNodesExpanded(), future.size());

    bfsLog->addChild("Passed", true);
    bfsLog->addChild("PathLength", static_cast<int>(future.size()));
    bfsLog->addChild("Time", t.GetElapsedTime());
  } catch(...) {
    bfsLog->addChild("Passed", false);
  }

  if (future.size() > 0)
    {
      std::reverse(future.begin(), future.end());
      future.pop_back();
    }
}

// Runs DFS on the mn puzzle domain. TODO: does not work
void MNPuzzleRunner::dfs(std::shared_ptr<Json::Node> statesLog) {
  auto dfsLog = statesLog->addChild("dfs");

  try {
    Timer t;
    DFS<MNPuzzleState<3, 3>, MNPuzzle<3, 3>> dfs;
    t.StartTimer();
    //dfs.GetPath(&mnPuzzle, start, goal, future);
    t.EndTimer();
    printf("DFS to goal complete in %1.2fs: %llu states reached; path length %lu.\n",
           t.GetElapsedTime(), dfs.GetNodesExpanded(), future.size());
    dfsLog->addChild("Passed", false);
    dfsLog->addChild("PathLength", static_cast<int>(future.size()));
    dfsLog->addChild("Time", t.GetElapsedTime());
  } catch(...) {
    dfsLog->addChild("Passed", false);
  }

}
