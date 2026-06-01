/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include <deque>
#include "Witness.h"
#include "Combinations.h"
#include "FileUtil.h"
#include "SVGUtil.h"
#include <thread>
#include <mutex>
std::mutex lock;

bool recording = false;
bool parallel = false;
// 2x2 + 5 (interesting)
// 2x5 + 5 [201/200][149][137/139][82][64][50]
// 3x4 + 5 [481/471][463!][399!][374! - 373][260][150][146][126]
const int puzzleWidth = 4;
const int puzzleHeight = 4;
const int minSolutions = 5000;
const int numRequiredPieces = 4; // 5
const int numSeparationPieces = 4; // 3x4 + 6 [4]
int currBoard = 0;

Witness<puzzleWidth, puzzleHeight> w;
InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
std::vector<Witness<puzzleWidth, puzzleHeight>> best;
//std::vector<uint64_t> otherbest;

void GetAllSolutions();
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions, int &len, int limit);
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   std::vector<int> &solutions,
				   const std::vector<int> &forbidden,
				   int &len, int limit);
void ExamineMustCross(int count);
void Load(uint64_t rank);
void ExamineMustCrossAndRegions(int crossCount, int regionCount);
void ExamineMustCrossAnd3Regions(int crossCount, int regionCount);
void ExamineTetris(int count);
void ExamineTriangles(int count);
void ExamineRegionsAndStars(int count);
void ParallelExamine(int count);
void GenerateUnique44();

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(const Witness<puzzleWidth, puzzleHeight> &w, std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles);

int main(int argc, char* argv[])
{
//	GetAllSolutions();
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Solve current board", kAnyModifier, 'v');
	InstallKeyboardHandler(MyDisplayHandler, "Test", "Test constraints", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current puzzle as svg", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Prev Board", "Jump to next found board.", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Next Board", "Jump to prev found board", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Prev 100 Board", "Jump to next 100 found board.", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Next 100 Board", "Jump to prev 100 found board", kAnyModifier, '}');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");

	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDrag));
}

void DrawPaperLevel(int which)
{
	std::string fname = "/Users/nathanst/Pictures/SVG/witness/";
	rgbColor blueColor(0.354, 0.666, 0.824);
	switch (which)
	{
		case 0:
		{
			Graphics::Display d;
			Witness<1, 2> p;
			InteractiveWitnessState<1, 2> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(0, 1, Colors::black);
			p.SetGoal(1, 1);
			p.SetStart(0, 1);
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.currState = InteractiveWitnessState<1, 2>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 1:
		{
			Graphics::Display d;
			Witness<1, 2> p;
			InteractiveWitnessState<1, 2> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(0, 1, Colors::black);
			p.SetGoal(1, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.currState = InteractiveWitnessState<1, 2>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 2:
		{
			Graphics::Display d;
			Witness<1, 3> p;
			InteractiveWitnessState<1, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.SetGoal(1, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});
			p1.currState = InteractiveWitnessState<1, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 3:
		{
			Graphics::Display d;
			Witness<2, 2> p;
			InteractiveWitnessState<2, 2> p1;
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);
			p.SetGoal(2, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 3});
			p1.currState = InteractiveWitnessState<2, 2>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 4:
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.SetGoal(3, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 4});
			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 5:
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.SetGoal(0, 1);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({-1, 1});
			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 6:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, blueColor);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(3, 1, blueColor);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.AddSeparationConstraint(3, 2, blueColor);
			p.AddSeparationConstraint(0, 3, blueColor);
			p.AddSeparationConstraint(2, 3, Colors::black);
			p.AddSeparationConstraint(3, 3, blueColor);
			p.SetGoal(1, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({4, 0});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({1, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 7:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, blueColor);
			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.AddSeparationConstraint(3, 2, blueColor);

			p.AddSeparationConstraint(0, 3, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 3, Colors::black);
			p.AddSeparationConstraint(3, 3, blueColor);
			p.SetGoal(4, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({5, 2});


			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 8:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, blueColor);

			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(3, 2, blueColor);

			p.AddSeparationConstraint(1, 3, blueColor);
			p.AddSeparationConstraint(2, 3, Colors::black);
			p.AddSeparationConstraint(3, 3, blueColor);
			p.SetGoal(3, 0);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, -1});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 9:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, blueColor);
			p.AddSeparationConstraint(3, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.AddSeparationConstraint(3, 2, blueColor);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(1, 3, blueColor);
			p.AddSeparationConstraint(2, 3, blueColor);
			p.AddSeparationConstraint(3, 3, Colors::black);

			p.SetGoal(4, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({4, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 10:
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(2, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.SetGoal(1, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({1, 5});
			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 11:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.AddSeparationConstraint(3, 2, Colors::black);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(1, 3, Colors::black);
			p.AddSeparationConstraint(2, 3, blueColor);

			p.SetGoal(4, 0);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({4, 0});
			p1.ws.path.push_back({5, 0});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}


		// Spread curriculum
		case 12:
		{
			Graphics::Display d;
			Witness<1, 2> p;
			InteractiveWitnessState<1, 2> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(0, 1, Colors::black);
			p.SetGoal(1, 1);
			p.SetStart(0, 1);
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.currState = InteractiveWitnessState<1, 2>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 13:
		{
			Graphics::Display d;
			Witness<1, 3> p;
			InteractiveWitnessState<1, 3> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.SetGoal(1, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});
			p1.currState = InteractiveWitnessState<1, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 14:
		{
			Graphics::Display d;
			Witness<2, 2> p;
			InteractiveWitnessState<2, 2> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.SetGoal(0, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.currState = InteractiveWitnessState<2, 2>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 15:
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.SetGoal(1, 0);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({1, -1});
			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 16:
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, blueColor);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.SetGoal(2, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({2, 4});
			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 17:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, Colors::black);

			p.AddSeparationConstraint(2, 1, Colors::black);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(0, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.AddSeparationConstraint(3, 2, blueColor);

			p.AddSeparationConstraint(2, 3, blueColor);
			p.AddSeparationConstraint(3, 3, blueColor);

			p.SetGoal(4, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({4, 0});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({5, 2});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 18:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);
			p.AddSeparationConstraint(3, 1, Colors::black);

			p.AddSeparationConstraint(3, 2, Colors::black);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(1, 3, Colors::black);
			p.AddSeparationConstraint(3, 3, Colors::black);

			p.SetGoal(0, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({0, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 19:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, blueColor);
			p.AddSeparationConstraint(3, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);
			p.AddSeparationConstraint(2, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(3, 2, blueColor);

			p.AddSeparationConstraint(1, 3, Colors::black);
			p.AddSeparationConstraint(2, 3, Colors::black);

			p.SetGoal(0, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({3, 4});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({0, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 20:
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);
			p.AddSeparationConstraint(3, 1, Colors::black);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.AddSeparationConstraint(3, 2, Colors::black);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(2, 3, blueColor);
			p.AddSeparationConstraint(3, 3, blueColor);

			p.SetGoal(4, 0);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({1, 4});
			p1.ws.path.push_back({2, 4});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 0});
			p1.ws.path.push_back({5, 0});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		// TODO: finish these
		case 21: // 0 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, blueColor);
			p.AddSeparationConstraint(2, 0, blueColor);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);

			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);

			p.SetGoal(3, 2);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({4, 2});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 22: // 1 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);

			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, Colors::black);

			p.SetGoal(2, 0);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({2, -1});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 23: // 2 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(1, 0, blueColor);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(1, 2, blueColor);

			p.SetGoal(1, 3);
			p.SetStart(0, 0);

			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 24: // 3 of 8
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);
			p.AddSeparationConstraint(3, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.AddSeparationConstraint(3, 2, Colors::black);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(1, 3, Colors::black);

			p.SetGoal(0, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({4, 1});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({0, 4});
			p1.ws.path.push_back({0, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 25: // 4 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, blueColor);
			p.AddSeparationConstraint(2, 2, blueColor);
			p.SetGoal(3, 3);
			p.SetStart(0, 0);

			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({3, 0});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 4});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 26: // 5 of 8
		{
			Graphics::Display d;
			Witness<1, 3> p;
			InteractiveWitnessState<1, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(0, 2, blueColor);
			p.SetGoal(1, 3);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({1, 4});

			p1.currState = InteractiveWitnessState<1, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 27: // 6 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(1, 0, Colors::black);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(1, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);
			p.SetGoal(3, 1);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({0, 3});
			p1.ws.path.push_back({1, 3});
			p1.ws.path.push_back({2, 3});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({4, 1});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 28: // 7 of 8
		{
			Graphics::Display d;
			Witness<3, 3> p;
			InteractiveWitnessState<3, 3> p1;
			p.AddSeparationConstraint(0, 0, Colors::black);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, blueColor);

			p.AddSeparationConstraint(0, 1, blueColor);
			p.AddSeparationConstraint(1, 1, blueColor);

			p.AddSeparationConstraint(0, 2, Colors::black);
			p.AddSeparationConstraint(2, 2, Colors::black);

			p.SetGoal(3, 3);
			p.SetStart(0, 0);

			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({0, 2});
			p1.ws.path.push_back({1, 2});
			p1.ws.path.push_back({2, 2});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({3, 3});
			p1.ws.path.push_back({3, 4});

			p1.currState = InteractiveWitnessState<3, 3>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		case 29: // 8 of 8
		{
			Graphics::Display d;
			Witness<4, 4> p;
			InteractiveWitnessState<4, 4> p1;
			p.AddSeparationConstraint(0, 0, blueColor);
			p.AddSeparationConstraint(1, 0, Colors::black);
			p.AddSeparationConstraint(2, 0, blueColor);

			p.AddSeparationConstraint(0, 1, Colors::black);
			p.AddSeparationConstraint(1, 1, Colors::black);
			p.AddSeparationConstraint(2, 1, Colors::black);
			p.AddSeparationConstraint(3, 1, blueColor);

			p.AddSeparationConstraint(3, 2, Colors::black);

			p.AddSeparationConstraint(0, 3, Colors::black);
			p.AddSeparationConstraint(1, 3, Colors::black);
			p.AddSeparationConstraint(2, 3, Colors::black);
			p.AddSeparationConstraint(3, 3, Colors::black);

			p.SetGoal(4, 4);
			p.SetStart(0, 0);
			p1.ws.path.push_back({0, 0});
			p1.ws.path.push_back({0, 1});
			p1.ws.path.push_back({1, 1});
			p1.ws.path.push_back({1, 0});
			p1.ws.path.push_back({2, 0});
			p1.ws.path.push_back({2, 1});
			p1.ws.path.push_back({3, 1});
			p1.ws.path.push_back({3, 2});
			p1.ws.path.push_back({4, 2});
			p1.ws.path.push_back({4, 3});
			p1.ws.path.push_back({4, 4});
			p1.ws.path.push_back({4, 5});

			p1.currState = InteractiveWitnessState<4, 4>::kInPoint;
			p.Draw(d);
			p.Draw(d, p1);
			printf("Save to '%s'\n", (fname+std::to_string(which)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(which)+".svg").c_str(), 600, 600, 0);
			break;
		}
		default: break;
	}

}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
//		for (int x = 0; x < 50; x++)
//			DrawPaperLevel(x);

		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);

		w.SetStart(0, 0);
		w.SetGoal(3, 5);
//		w.AddCannotCrossConstraint(true, 0, 0);
		w.AddSeparationConstraint(3, 3, Colors::blue);
		w.AddSeparationConstraint(0, 3, Colors::blue);
		w.AddSeparationConstraint(1, 3, Colors::black);
		w.AddStarConstraint(3, 1, Colors::black);
//		w.AddTriangleConstraint(0, 0, 3);
//		w.AddGoal(1, -1);
//		w.AddGoal(-1, 1);
//		w.AddGoal(puzzleWidth, 0);
//		w.AddGoal(0, puzzleHeight);
//		w.AddGoal(puzzleWidth, puzzleHeight+1);
//		w.AddTriangleConstraint(0, 1, 2);
//		w.AddTriangleConstraint(1, 0, 1);
//		ExamineMustCross(numRequiredPieces);
//		w.AddStarConstraint(0, 0, Colors::pink);
//		w.AddSeparationConstraint(0, 0, Colors::green);
//		w.AddSeparationConstraint(0, 4, Colors::white);
//		w.AddSeparationConstraint(4, 0, Colors::white);
//		w.AddSeparationConstraint(2, 2, Colors::pink);
//		w.AddSeparationConstraint(4, 4, Colors::black);
//		w.AddSeparationConstraint(2, 0, Colors::black);
//		w.AddSeparationConstraint(2, 4, Colors::black);
//		w.AddSeparationConstraint(0, 2, Colors::orange);
//		w.AddSeparationConstraint(4, 2, Colors::orange);

//		std::string s = "{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"4;3;#385CDE\",\"3;10;#DCA700\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000000000000000000000000000000000\"}\"";
//		w.LoadFromHashString(s);

//		std::string s2 = "{\"dim\":\"4x4\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"10000000000000000000000000001100000000000000000000000000000000000\"}";
//		std::string s = "{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;1077084160;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"00000000010000010000000000000001000000000000000000000000000000000\"}";
//		s = 	"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#FFFFFF\"},\"mc\":\"00000010000000000000000000000000000000000000000000000000000000000\"}";
//		s = "{\"dim\":\"4x4\",\"cc\":{\"1;0;#0000FF\",\"0;0;#000026D\",\"0;0;#0029D00\",\"2;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"2;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\"},\"mc\":\"10000000000010000000000000000000100000000000000000000000000000000\"}";

//		w.LoadFromHashString(s);
//		w.AddTriangleConstraint(0, 0, 1);
//		w.AddTriangleConstraint(1, 1, 2);
//		w.AddTriangleConstraint(2, 2, 3);
//		w.AddStarConstraint(3, 3, Colors::orange);
//		w.AddCannotCrossConstraint(1, 4);
//		w.AddCannotCrossConstraint(3, 1);
//		w.AddCannotCrossConstraint(false /*vert*/, 2, 1);
//		w.AddCannotCrossConstraint(true /*horiz*/, 0, 2);
		//		w.AddTetrisConstraint(1, 1, 10);
//		w.AddNegativeTetrisConstraint(1, 0, 3);
//		w.AddTetrisConstraint(2, 1, 2);
//		w.AddTetrisConstraint(1, 2, 2);
//		w.AddTetrisConstraint(2, 2, 2);
//		w.AddTetrisConstraint(1, 3, 2);
//		w.AddTetrisConstraint(2, 3, 2);
//		w.AddMustCrossConstraint(false, 2, 0);
//		w.AddMustCrossConstraint(false, 2, 4);


//		w.AddTetrisConstraint(0, 2, 2);
//		w.AddTetrisConstraint(0, 1, 1);
//		w.AddTetrisConstraint(3, 3, );
//		w.AddTetrisConstraint(0, 0, 10);
//		w.AddTetrisConstraint(3, 0, 12);
//		w.AddTetrisConstraint(3, 1, 14);
//		w.AddTetrisConstraint(2, 3, -8);
//		w.AddTetrisConstraint(2, 2, -12);
//		w.AddTetrisConstraint(1, 1, -13);
//		w.AddTetrisConstraint(1, 0, 4);
//		w.AddTetrisConstraint(1, 1, 5);
//		w.AddTetrisConstraint(1, 2, 6);
//		w.AddTetrisConstraint(1, 3, 7);
//		w.AddTetrisConstraint(2, 0, 8);
//		w.AddTetrisConstraint(2, 1, 9);
//		w.AddTetrisConstraint(2, 2, 10);
//		w.AddTetrisConstraint(2, 3, 11);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &d = GetContext(windowID)->display;
	iws.IncrementTime();
	w.Draw(d);
	w.Draw(d, iws);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
//	if (strcmp(argument[0], "-test") == 0)
//	{
//		BaselineTest();
//		exit(0);
//	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't':
//			ParallelExamine(5);
			GenerateUnique44();
//			ExamineMustCross(numRequiredPieces);
//			w.ClearTetrisConstraints();
//			ExamineTetris(4);
//			ExamineMustCrossAndRegions(numRequiredPieces, numSeparationPieces);
//			ExamineMustCrossAnd3Regions(numRequiredPieces, numSeparationPieces);
//			ExamineTriangles(6);
			//ExamineTriangles(puzzleHeight*puzzleWidth);
//			ExamineRegionsAndStars(0);
			break;
		case 'v':
		{
			std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
			GetAllSolutions(w, allSolutions);
			if (allSolutions.size() > 0)
			{
				iws.ws = allSolutions[0];
				iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
			}
		}
			break;
		case 's':
		{
			Graphics::Display d;
			//d.FillRect({-1, -1, 1, 1}, Colors::darkgray);
			w.Draw(d);
			w.Draw(d, iws);
			std::string fname = "/Users/nathanst/Desktop/SVG/witness_";
			int count = 0;
			while (FileExists(fname + std::to_string(count) + ".svg"))
			{
				count++;
			}
			printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0, w.SaveToHashString().c_str());

			{
				int wide, high;
				w.GetDimensionsFromHashString(w.SaveToHashString(), wide, high);
			}
		}
			break;
		case 'r': recording = !recording; break;
		case '\t':
//			if (mod != kShiftDown)
//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//			else
//			{
//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//			}
//			break;
		case '[':
			if (best.size() > 0)
			{
				currBoard = (currBoard+(int)(best.size())-1)%best.size();
				Load(currBoard);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case ']':
			if (best.size() > 0)
			{
				currBoard = (currBoard+1)%best.size();
				Load(currBoard);//, numRequiredPieces, numSeparationPieces);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case '{':
			if (best.size() > 0)
			{
				currBoard = (currBoard+(int)(100*best.size())-100)%best.size();
				Load(currBoard);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case '}':
			if (best.size() > 0)
			{
				currBoard = (currBoard+100)%best.size();
				Load(currBoard);//, numRequiredPieces, numSeparationPieces);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;

		case 'o':
			if (iws.ws.path.size() == 0)
			{
				iws.ws.path.push_back({0, 0});
				iws.ws.path.push_back({0, 1});
				iws.ws.path.push_back({1, 1});
			}
			else {
				iws.Reset();
			}
			break;
		default:
			break;
	}
}

bool MyClickHandler(unsigned long, int, int, point3d p, tButtonType , tMouseEventType e)
{
	if (e == kMouseDrag) // ignore movement with mouse button down
		return true;

	if (e == kMouseUp)
	{
		if (w.Click(p, iws)) // found goal
		{
			if (w.GoalTest(iws.ws))
			{
				printf("Solved!\n");
			}
			else {
				printf("Invalid solution\n");
				//iws.Reset();
			}
		}
	}
	if (e == kMouseMove)
	{
		w.Move(p, iws);
//		if (iws.ws.path.size() > 0)
//			std::cout << iws.ws.path.back().first << ", " << iws.ws.path.back().second << "\n";
	}

	// Don't need any other mouse support
	return true;
}


template <int puzzleWidth, int puzzleHeight>
void DFS(const Witness<puzzleWidth, puzzleHeight> &w,
		 WitnessState<puzzleWidth, puzzleHeight> &s,
		 std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	std::vector<WitnessAction> acts;

	if (w.GoalTest(s))
	{
		puzzles.push_back(s);
		return;
	}

	w.GetActions(s, acts);
	for (auto &a : acts)
	{
		w.ApplyAction(s, a);
		DFS(w, s, puzzles);
		w.UndoAction(s, a);
	}
}


template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(const Witness<puzzleWidth, puzzleHeight> &w, std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	WitnessState<puzzleWidth, puzzleHeight> s;
	s.Reset();
	Timer t;
	t.StartTimer();
	DFS(w, s, puzzles);
	t.EndTimer();
	static bool printed = false;
	if (!printed)
		printf("%lu solutions found in %1.2fs\n", puzzles.size(), t.GetElapsedTime());
	printed = true;
}

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	Witness<puzzleWidth, puzzleHeight> w;
	GetAllSolutions(w, puzzles);
}

void GetAllSolutions()
{
//	std::vector<WitnessState<puzzleWidth, puzzleHeight>> puzzles;
//	GetAllSolutions(puzzles);

	std::vector<WitnessState<2, 2>> p1;
	GetAllSolutions(p1);
	std::vector<WitnessState<2, 3>> p2;
	GetAllSolutions(p2);
	std::vector<WitnessState<3, 3>> p3;
	GetAllSolutions(p3);
	std::vector<WitnessState<3, 4>> p4;
	GetAllSolutions(p4);
	std::vector<WitnessState<4, 4>> p5;
	GetAllSolutions(p5);
	std::vector<WitnessState<4, 5>> p6;
	GetAllSolutions(p6);
	std::vector<WitnessState<5, 5>> p7;
	GetAllSolutions(p7);

}

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &wp,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   int &len, int limit)
{
	int count = 0;
	for (const auto &i : allSolutions)
	{
		if (wp.GoalTest(i))
		{
			len = (int)i.path.size();
			count++;
		}
		if (count > limit)
			break;
	}
	return count;
}

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   std::vector<int> &solutions,
				   const std::vector<int> &forbidden,
				   int &len, int limit)
{
	solutions.resize(0);
	int count = 0;
	for (int x = 0; x < forbidden.size(); x++)
	{
		if (w.GoalTest(allSolutions[forbidden[x]]))
			return 0;
	}
	for (int x = 0; x < allSolutions.size(); x++)
	{
		if (w.GoalTest(allSolutions[x]))
		{
			len = (int)allSolutions[x].path.size();
			count++;
			solutions.push_back(x);
		}
		if (count > limit)
			break;
	}
	return count;
}

void Load(uint64_t which)//, int req, int sep)
{
	lock.lock();
	if (best.size() > which)
	{
		w = best[which];
		iws.Reset();
	}
	lock.unlock();
	//	iws.Reset();
//
//		w.ClearMustCrossConstraints();
//		int *items = new int[req];
//		int *items2 = new int[sep];
//		Combinations<w.GetNumMustCrossConstraints()> c;
//		Combinations<w.GetNumSeparationConstraints()> regionCombs;
//
//	if (best.size() > 0)
//	{
//		c.Unrank(best[which], items, req);
//		for (int x = 0; x < req; x++)
//		{
//			w.SetMustCrossConstraint(items[x]);
//		}
//	}
//
//	if (otherbest.size() > 0)
//	{
//		w.ClearSeparationConstraints();
//		uint64_t bits = otherbest[which]; // will only use bottom bits
//		uint64_t hash = otherbest[which]/(1<<sep);
//		regionCombs.Unrank(hash, items2, sep);
//		for (int x = 0; x < sep; x++)
//		{
//			w.AddSeparationConstraint(items2[x], ((bits>>x)&1)?Colors::white:Colors::black);
//		}
//	}
//
//	delete [] items;
//	delete [] items2;
}

void ExamineMustCross(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<w.GetNumPathConstraints()> c;


	Witness<puzzleWidth, puzzleHeight> w;
	WitnessState<puzzleWidth, puzzleHeight> s;

	uint64_t maxRank = c.MaxRank(count);
	for (uint64_t n = 0; n < maxRank; n++)
	{
		if (0 == n%50000)
			printf("%llu of %llu\n", n, maxRank);
		c.Unrank(n, items, count);
		for (int x = 0; x < count; x++)
		{
			w.SetMustCrossConstraint(items[x]);
		}

		int pathLen = 0;
		int result = CountSolutions(w, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(w);
		}
		else if (result == minCount)
		{
			best.push_back(w);
		}

		for (int x = 0; x < count; x++)
		{
			w.ClearMustCrossConstraint(items[x]);
		}
	}

	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}

	delete [] items;
}

void ExamineMustCrossAndRegions(int crossCount, int regionCount)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	int *crossItems = new int[crossCount];
	int *regionItems = new int[regionCount];
	Combinations<w.GetNumPathConstraints()> c;
	Combinations<w.GetNumSeparationConstraints()> regionCombs;

	{

		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;

		uint64_t maxRank = c.MaxRank(crossCount);
		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000 && regionCount == 0)
				printf("%llu of %llu\n", n, maxRank);
			c.Unrank(n, crossItems, crossCount);
			for (int x = 0; x < crossCount; x++)
			{
				w.SetMustCrossConstraint(crossItems[x]);
			}
			uint64_t colorComb = pow(2, regionCount);
			for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*colorComb; t++)
			{
				uint64_t globalPuzzle = n*regionCombs.MaxRank(regionCount)*colorComb+t;
				if ((0 == globalPuzzle%50000))
					printf("-->%llu of %llu\n", globalPuzzle, maxRank*regionCombs.MaxRank(regionCount)*colorComb);
				uint64_t bits = t; // will only use bottom bits
				uint64_t hash = t/colorComb;

				// easy way to reduce symmetry
				if (t&1) continue;

//				if (0 != bits%3)
//					continue;
//				if (1 != (bits/3)%3)
//					continue;
//				if (2 != (bits/9)%3)
//					continue;

				w.ClearSeparationConstraints();
				regionCombs.Unrank(hash, regionItems, regionCount);
				for (int x = 0; x < regionCount; x++)
				{
					int colour = bits%2;
					bits = bits/2;
//					w.AddSeparationConstraint(regionItems[x], ((bits>>x)&1)?Colors::white:Colors::black);
					w.AddSeparationConstraint(regionItems[x], (colour==0)?Colors::white:((colour==1)?Colors::black:Colors::blue));
				}

				int pathSize = 0;
				int result = CountSolutions(w, allSolutions, pathSize, minCount+1);

				if (result < minCount && result > 0)
				{
					minCount = result;
					bestPathSize = pathSize;
					best.clear();
					best.push_back(w);
				}
				else if (result == minCount && pathSize == bestPathSize)
//				else if (result == minCount)
				{
					best.push_back(w);
				}
			}

			w.ClearPathConstraints();
		}
	}

	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}

	delete [] crossItems;
	delete [] regionItems;
}

void ExamineMustCrossAnd3Regions(int crossCount, int regionCount)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	int *crossItems = new int[crossCount];
	int *regionItems = new int[regionCount];
	Combinations<w.GetNumPathConstraints()> c;
//	Combinations<w.GetNumMustCrossConstraints()> c;
	Combinations<w.GetNumSeparationConstraints()> regionCombs;

	{

		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;

		uint64_t maxRank = c.MaxRank(crossCount);
		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000 && regionCount == 0)
				printf("%llu of %llu\n", n, maxRank);
			c.Unrank(n, crossItems, crossCount);
			for (int x = 0; x < crossCount; x++)
			{
				w.SetMustCrossConstraint(crossItems[x]);
			}
			uint64_t colorComb = pow(3, regionCount);
			for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*colorComb; t++)
			{
				uint64_t globalPuzzle = n*regionCombs.MaxRank(regionCount)*colorComb+t;
				if ((0 == globalPuzzle%50000))
					printf("-->%llu of %llu\n", globalPuzzle, maxRank*regionCombs.MaxRank(regionCount)*colorComb);
				uint64_t bits = t; // will only use bottom bits
				uint64_t hash = t/colorComb;

//				// easy way to reduce symmetry
//				if (t&1) continue;
//
				if (0 != bits%3)
					continue;
				if (1 != (bits/3)%3)
					continue;
				if (2 != (bits/9)%3)
					continue;

				w.ClearSeparationConstraints();
				regionCombs.Unrank(hash, regionItems, regionCount);
				for (int x = 0; x < regionCount; x++)
				{
					int colour = bits%3;
					bits = bits/3;
					w.AddSeparationConstraint(regionItems[x], (colour==0)?Colors::white:((colour==1)?Colors::black:Colors::blue));
				}

				int pathSize = 0;
				int result = CountSolutions(w, allSolutions, pathSize, minCount+1);

				if (result < minCount && result > 0)
				{
					minCount = result;
					bestPathSize = pathSize;
					best.clear();
					best.push_back(w);
				}
				else if (result == minCount && pathSize == bestPathSize)
					//				else if (result == minCount)
				{
					best.push_back(w);
				}
			}

			w.ClearPathConstraints();
		}
	}

	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}

	delete [] crossItems;
	delete [] regionItems;
}

void ExamineRegionsAndStars(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	std::vector<int> items(count);
//	Combinations<w.GetNumStarConstraints()> c;
	Combinations<w.GetNumPathConstraints()> mc;
	std::vector<int> forbidden;
	std::vector<int> currSolutions;

	{
		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;

		if (puzzleWidth!=4 || puzzleHeight!= 4)
		{
			printf("This code only works for 4x4");
			return;
		}
		//static_assert((4==puzzleHeight)&&(puzzleWidth==4), "This code only works for 4x4");
//		uint64_t variations = pow(4, count-1)*2;
//		uint64_t maxRank = c.MaxRank(count)*variations;
		uint64_t variations = pow(4, 8);
		uint64_t mcRank = mc.MaxRank(count);
		uint64_t maxRank = mcRank*variations;//c.MaxRank(count)*variations;

		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000)
				printf("%llu of %llu\n", n, maxRank);
			w.ClearInnerConstraints();
			w.ClearPathConstraints();
//			uint64_t rankPart = n/variations;
//			uint64_t varPart = n%variations;
//			c.Unrank(rankPart, &items[0], count);
//			for (int i : items)
//			{
//				bool colorPart = (varPart/2)%2;
//				bool shapePart = varPart%2;
//				varPart /= 4;
//				if (shapePart)
//					w.AddStarConstraint(i, colorPart?Colors::black:Colors::blue);
//				else
//					w.AddSeparationConstraint(i, colorPart?Colors::black:Colors::blue);
//			}

			uint64_t rank = n%variations;
			for (int x = 0; x < 4; x++)
			{
				rgbColor color;
				color = (rank&1)?Colors::white:Colors::orange;
				if (rank&2)
					w.AddStarConstraint(x, 1, color);
				else {
					w.AddTriangleConstraint(x, 1, 1+(rank&1));
					//w.AddSeparationConstraint(x, 0, color);
				}
				rank>>=2;

				color = (rank&1)?Colors::white:Colors::orange;
				if (rank&2)
					w.AddStarConstraint(x, 2, color);
				else {
//					w.AddSeparationConstraint(x, 3, color);
					w.AddTriangleConstraint(x, 2, 1+(rank&1));
				}
				rank>>=2;
			}
//			for (int y = 0; y < 4; y+=3)
//			{
//				rgbColor color;
//				color = (rank&2)?Colors::white:Colors::red;
//
//				if (rank&1)
//					w.AddStarConstraint(0, y, color);
//				else
//					w.AddSeparationConstraint(0, y, color);
//				rank>>=2;
//				if (rank&1)
//					w.AddStarConstraint(3, y, color);
//				else
//					w.AddSeparationConstraint(3, y, color);
//				rank>>=2;
//			}
			mc.Unrank(n/variations, &items[0], count);
			for (int x = 0; x < items.size(); x++)
			{
				w.SetMustCrossConstraint(items[x]);
			}

			int pathSize = 0;
			//int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
			int result = CountSolutions(w, allSolutions, currSolutions, forbidden, pathSize, minCount+1);

			// don't return two puzzles with the same solution
			if (currSolutions.size() == 1)
				forbidden.push_back(currSolutions[0]);

			if (result < minCount && result > 0)
			{
				minCount = result;
				bestPathSize = pathSize;
				best.clear();
				best.push_back(w);
			}
			else if (result == minCount && pathSize == bestPathSize)
				//				else if (result == minCount)
			{
				best.push_back(w);
			}
		}
	}

	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}

}
FILE *f = 0;
int totalCountHere = 0;
void GenerateUnique44Helper(int count, int threadID, int numThreads)
{
	if (f == 0)
		return;
	best.clear();
	currBoard = 0;
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;

	uint64_t total = 0;
//	for (int count = 0; count <= 16; count++)
	{
		uint64_t minCount = allSolutions.size();
		int bestPathSize = 0;
		std::vector<int> items(count);
		Combinations<wp.GetNumStarConstraints()> c;
		std::vector<int> forbidden;
		std::vector<int> currSolutions;

		//int *items = new int[count];
		
		const int pieceTypes = 2;//24+2;
		
		uint64_t pCount = pow(pieceTypes, count);
		uint64_t maxRank = c.MaxRank(count)*pCount;
		for (uint64_t rank = threadID; rank < maxRank; rank+=numThreads)
		{
			wp.ClearInnerConstraints();
			if (0 == rank%50000)
				printf("%llu of %llu\n", rank, maxRank);
			uint64_t n = rank/pCount; // arrangement on board
			uint64_t pieces = rank%pCount; // pieces in locations
			c.Unrank(n, &items[0], count);
			for (int x = 0; x < count; x++)
			{
				switch (pieces%pieceTypes)
				{
					case 0:
						wp.AddSeparationConstraint(items[x], Colors::black);
						break;
					case 1:
						wp.AddSeparationConstraint(items[x], Colors::blue);
						break;
				}
				pieces/=pieceTypes;
			}
			
			int pathSize = 0;
			
			// find single solution puzzles
			int result = CountSolutions(wp, allSolutions, pathSize, 1);
			
			//lock.lock();
			if (result == 1)
			{
//				Graphics::Display d;
//				d.FillRect({-1, -1, 1, 1}, Colors::darkgray);
//				wp.Draw(d);
//				std::string fname = "/Users/nathanst/Pictures/hog2/witness_"+std::to_string(puzzleWidth)+"x"+std::to_string(puzzleHeight)+"_"+std::to_string(count)+"_";
				lock.lock();
//				while (FileExists(fname + std::to_string(total) + ".svg"))
//				{
//					total++;
//				}
//				printf("Save to '%s'\n", (fname+std::to_string(total)+".svg").c_str());
//				MakeSVG(d, (fname+std::to_string(total)+".svg").c_str(), 400, 400, 0, wp.SaveToHashString().c_str());
				fprintf(f, "%s\n", wp.SaveToHashString().c_str());
				totalCountHere++;
				lock.unlock();
			}
//			best.push_back(wp);
			//lock.unlock();
		}
//		printf("\n%lu boards with %d pieces; %1.2fs elapsed\n", total, count, t.EndTimer());
	}

//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard);
//	}
//	parallel = false;
}

void GenerateUnique44()
{
	Timer timer;
	const int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> t(numThreads);
	f = fopen("/Users/nathanst/Pictures/hog2/witness_4x4.txt", "a+");
	if (f == 0)
	{
		printf("Failure to open file\n");
		exit(0);
	}
	for (int count = 1; count <= puzzleWidth*puzzleHeight; count++)
	{
		printf("Starting %d\n", count);
		timer.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			delete t[x];
			t[x] = new std::thread(GenerateUnique44Helper, count, x, numThreads);
		}
		for (int x = 0; x < numThreads; x++)
		{
			t[x]->join();
			delete t[x];
			t[x] = 0;
		}
		printf("Total: %d\n", totalCountHere);
		printf("%1.2fs elapsed\n", timer.EndTimer());
	}
	printf("Total: %d\n", totalCountHere);
	fclose(f);
	exit(0);
}


void ParallelExamineHelper(int count, int threadID, int numThreads)
{
	best.clear();
	currBoard = 0;
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;
	//wp.SetGoal(puzzleWidth+1, puzzleHeight);

	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	std::vector<int> items(count);
	Combinations<wp.GetNumStarConstraints()> c;
	//	Combinations<wp.GetNumMustCrossConstraints()> mc;
	std::vector<int> forbidden;
	std::vector<int> currSolutions;

	//int *items = new int[count];

	const int pieceTypes = 3;//24+2;

	uint64_t pCount = pow(pieceTypes, count);
	uint64_t maxRank = c.MaxRank(count)*pCount;
	for (uint64_t rank = threadID; rank < maxRank; rank+=numThreads)
	{
//		wp.ClearTetrisConstraints();
//		wp.ClearStarConstraints();
		wp.ClearInnerConstraints();
		if (0 == rank%50000)
			printf("%llu of %llu\n", rank, maxRank);
		uint64_t n = rank/pCount; // arrangement on board
		uint64_t pieces = rank%pCount; // pieces in locations
		c.Unrank(n, &items[0], count);
//		bool t1 = false, t2 = false, t3 = false, t4 = false, t5 = false, t6 = false;
		for (int x = 0; x < count; x++)
		{
			switch (pieces%pieceTypes)
			{
				case 0:
					wp.AddSeparationConstraint(items[x], Colors::orange);
					break;
				case 1:
					wp.AddStarConstraint(items[x], Colors::orange);
					break;
				case 2:
					wp.AddTriangleConstraint(items[x], 2);
//					wp.AddSeparationConstraint(items[x], Colors::blue);
					break;
				case 3:
					wp.AddStarConstraint(items[x], Colors::blue);
					break;
				case 4:
					wp.AddSeparationConstraint(items[x], Colors::green);//
					break;
				case 5:
					wp.AddStarConstraint(items[x], Colors::green);
					break;
			}
			pieces/=pieceTypes;
		}
//		if (0)
//		{
//			int x=0;
//			if ((pieces%pieceTypes) < 24)
//			{
//				wp.AddTetrisConstraint(items[x], 1+(pieces%pieceTypes));
//				t1 = true;
//			}
//			else {
////				wp.AddStarConstraint(items[x], wp.tetrisYellow);
////				wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
//				if ((pieces%pieceTypes)-24)
//				{
//					wp.AddSeparationConstraint(items[x], Colors::black);
//					t2 = true;
//				}
//				else {
//					wp.AddSeparationConstraint(items[x], Colors::white);
//					t3 = true;
//				}
//			}
//			pieces/=pieceTypes;
//		}
//		if (!(t1 && t2 && t3))// && t4 && t5 && t6))
//			continue;


		int pathSize = 0;
		int result = CountSolutions(wp, allSolutions, pathSize, minCount+1);
		//int result = CountSolutions(wp, allSolutions, currSolutions, forbidden, pathSize, minCount+1);

			// don't return two puzzles with the same solution
//			if (currSolutions.size() == 1)
//				forbidden.push_back(currSolutions[0]);

		if ((result < minCount && result > 0) || (result == minCount && pathSize > bestPathSize))
		{
			lock.lock();
			printf("Decreased number of solutions to %d / best path up to %d\n", result, bestPathSize);
			minCount = result;
			bestPathSize = pathSize;
			currBoard = 0;
			best.clear();
			best.push_back(wp);
			lock.unlock();
		}
		else if (result == minCount && pathSize == bestPathSize)
			//				else if (result == minCount)
		{
			lock.lock();
			best.push_back(wp);
			lock.unlock();
		}
	}

	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard);
//	}
	parallel = false;
}

const int numThreads = std::thread::hardware_concurrency();
void ParallelExamine(int count)
{
	std::vector<std::thread *> t(numThreads);
	if (!parallel)
	{
		parallel = true;
		for (int x = 0; x < numThreads; x++)
		{
			delete t[x];
			t[x] = new std::thread(ParallelExamineHelper, count, x, numThreads);
		}
	}
}


//void ExamineRegions(int regionCount)
//{
//	Timer t;
//	t.StartTimer();
//	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
//	GetAllSolutions(allSolutions);
//
//	uint64_t minCount = allSolutions.size();
//	int bestPathSize = 0;
//	//	std::vector<uint64_t> best;
//	int *regionItems = new int[regionCount];
//	Combinations<w.GetNumMustCrossConstraints()> c;
//	Combinations<w.GetNumSeparationConstraints()> regionCombs;
//
//	{
//
//		Witness<puzzleWidth, puzzleHeight> w;
//		WitnessState<puzzleWidth, puzzleHeight> s;
//
//		for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*(1<<regionCount); t++)
//		{
//			if (0 == t%1000)
//				printf("-->%llu of %llu\n", t, regionCombs.MaxRank(regionCount)*(1<<regionCount));
//			uint64_t bits = t; // will only use bottom bits
//			uint64_t hash = t/(1<<regionCount);
//
//			// easy way to reduce symmetry
//			if (t&1) continue;
//
//			w.ClearSeparationConstraints();
//			regionCombs.Unrank(hash, regionItems, regionCount);
//			for (int x = 0; x < regionCount; x++)
//			{
//				w.AddSeparationConstraint(regionItems[x], ((bits>>x)&1)?Colors::white:Colors::black);
//			}
//
//			int pathSize = 0;
//			int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
//			if (result > minSolutions)
//			{
//				// ignore
//			}
//			else if (result < minCount && result > 0)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize > bestPathSize)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize == bestPathSize)
//			{
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//		}
//
//		w.ClearMustCrossConstraints();
//	}
//
//	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard, 0, regionCount);
//	}
//
//	delete [] regionItems;
//}


void ExamineTetris(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;

//	for (int y = 0; y < puzzleHeight+1; y++)
//		for (int x = 0; x < puzzleWidth+1; x++)
//			wp.AddMustCrossConstraint(x, y);

	GetAllSolutions(wp, allSolutions);

	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<wp.GetNumTetrisConstraints()> c;

	const int pieceTypes = 24+2;

	uint64_t pCount = pow(pieceTypes, count);
	uint64_t maxRank = c.MaxRank(count)*pCount;
	for (uint64_t rank = 0; rank < maxRank; rank++)
	{
		wp.ClearTetrisConstraints();
		// wp.ClearTriangleConstraints();
		wp.ClearStarConstraints();
		wp.ClearSeparationConstraints();
		if (0 == rank%50000)
			printf("%llu of %llu\n", rank, maxRank);
		uint64_t n = rank/pCount; // arrangement on board
		uint64_t pieces = rank%pCount; // pieces in locations
		c.Unrank(n, items, count);
		bool t1 = false, t2 = false;
		for (int x = 0; x < count; x++)
		{
//			if (x == 0 && count > 1)
//			{
//				wp.AddNegativeTetrisConstraint(items[x], (1+(pieces%24)));
//			}
//			else {
			if ((pieces%pieceTypes) < 24)
			{
				wp.AddTetrisConstraint(items[x], 1+(pieces%pieceTypes));
				t1 = true;
			}
			else {
				//wp.AddStarConstraint(items[x], wp.tetrisYellow);
				//wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
				if ((pieces%pieceTypes)-23)
					wp.AddSeparationConstraint(items[x], Colors::black);
				else
					wp.AddSeparationConstraint(items[x], Colors::white);
				//wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
				t2 = true;
			}
//			}
			pieces/=pieceTypes;
		}
		if (!(t1 && t2))
			continue;
//		w = wp;
//		if (rank == 2)
//			break;
		int pathLen = 0;
		int result = CountSolutions(wp, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(wp);
			w = wp;
		}
		else if (result == minCount)
		{
			best.push_back(wp);
		}

	}

	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
//	return;

//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard, count, 0);
//	}

	delete [] items;
}


void ExamineTriangles(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;

//	for (int y = 0; y < puzzleHeight+1; y++)
//		for (int x = 0; x < puzzleWidth+1; x++)
//			wp.AddMustCrossConstraint(x, y);

	GetAllSolutions(wp, allSolutions);

	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<wp.GetNumTriangleConstraints()> c;

	const int NUM_TRI = 4;
	uint64_t pCount = pow(NUM_TRI, count);
	uint64_t maxRank = c.MaxRank(count)*pCount;
	for (uint64_t rank = maxRank/10; rank < maxRank && best.size() < 5000; rank+=1)
	{
		if (0 == rank%50000)
			printf("%llu of %llu [%d solutions]\n", rank, maxRank, best.size());
		uint64_t n = rank/pCount; // arrangement on board
		uint64_t pieces = rank%pCount; // pieces in locations
		c.Unrank(n, items, count);
		for (int x = 0; x < count; x++)
		{
			if (0 == pieces%NUM_TRI)
			{
				wp.AddStarConstraint(items[x], wp.triangleColor);
			}
			else {
				wp.AddTriangleConstraint(items[x], 0+(pieces%NUM_TRI));
			}
			pieces/=NUM_TRI;
		}
		int pathLen = 0;
		int result = CountSolutions(wp, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(wp);
			w = wp;
		}
		else if (result == minCount)
		{
			best.push_back(wp);
		}
		wp.ClearStarConstraints();
		wp.ClearTriangleConstraints();
	}

	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
	//	return;

	//	if (best.size() > 0)
	//	{
	//		currBoard = 0;
	//		Load(currBoard, count, 0);
	//	}

	delete [] items;
}
