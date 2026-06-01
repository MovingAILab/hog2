/// ABC for test runners

#pragma once

#include <vector>
#include <fstream>
#include "Json.h"

class Runner
{
 public:
  virtual void addTest(std::string) = 0;
  virtual void addAllTests() = 0;

  void setLogFile(std::string s) { logPath = s; }

  protected:
  void openLogFile() { logFile.open(logPath); }
  void closeLogFile() { logFile.close(); }

 std::vector<int> tests;
  std::string logPath;
  std::ofstream logFile;
  Json log;
};
