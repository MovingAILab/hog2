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

#include "Common.h"
#include "Driver.h"
#include "GraphEnvironment.h"
#include "TextOverlay.h"
#include <string>
#include "MNPuzzle.h"
//#include "IncrementalIDA.h"
#include "IDAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "SVGUtil.h"

IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
Heuristic<MNPuzzleState<4, 4>> searchHeuristic;
bool recording = false;
bool running = false;
float rate = 1.0f/6.0f;
float tween = 1;
int numActions = 0;
bool foundOptimal = true;

MNPuzzle<4, 4> mnp;
MNPuzzleState<4, 4> last;
MNPuzzleState<4, 4> curr;
MNPuzzleState<4, 4> start;
MNPuzzleState<4, 4> goal;
std::vector<slideDir> acts;
std::vector<MNPuzzleState<4, 4>> path;

std::vector<int> p1 = {0, 1, 2, 3};//, 4, 5, 6, 7};
std::vector<int> p2 = {0, 4, 5, 6, 7};
std::vector<int> p3 = {0, 8, 9, 10, 11};
std::vector<int> p4 = {0, 12, 13, 14, 15};

PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *pdb1 = 0;
PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *pdb2 = 0;
PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *pdb3 = 0;
PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *pdb4 = 0;

#ifndef __EMSCRIPTEN__
void MakeAnimation();
#endif

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 600, 600);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Picture", "Take a picture", kAnyModifier, '&');
	InstallKeyboardHandler(MyDisplayHandler, "Randomize", "Get Random State", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
	srandom(time(0));

}

void BuildPDBs()
{
	goal.Reset();
	mnp.StoreGoal(goal);
	bool built = false;
	Timer t;
	t.StartTimer();
	if (pdb1 == 0)
	{
		pdb1 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&mnp, goal, p1);
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		built = true;
	}
	if (pdb2 == 0)
	{
		pdb2 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&mnp, goal, p2);
		pdb2->BuildPDB(goal);
		built = true;
	}
	if (pdb3 == 0)
	{
		pdb3 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&mnp, goal, p3);
		pdb3->BuildPDB(goal);
		built = true;
	}
	if (pdb4 == 0)
	{
		pdb4 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&mnp, goal, p4);
		pdb4->BuildPDB(goal);
		built = true;
	}
	
	searchHeuristic.lookups.resize(0);
	searchHeuristic.lookups.push_back({kMaxNode, 1, 5});
	searchHeuristic.lookups.push_back({kLeafNode, 0, 0});
	searchHeuristic.lookups.push_back({kLeafNode, 1, 1});
	searchHeuristic.lookups.push_back({kLeafNode, 2, 2});
	searchHeuristic.lookups.push_back({kLeafNode, 3, 3});
	searchHeuristic.lookups.push_back({kLeafNode, 4, 4});

	searchHeuristic.heuristics.resize(0);
	searchHeuristic.heuristics.push_back(&mnp);
	searchHeuristic.heuristics.push_back(pdb1);
	searchHeuristic.heuristics.push_back(pdb2);
	searchHeuristic.heuristics.push_back(pdb3);
	searchHeuristic.heuristics.push_back(pdb4);
	t.EndTimer();
	if (built)
		srandom(t.GetElapsedTime()*1000);
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
		printf("Window %ld created\n", windowID);

		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);

		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	const auto smooth = [](float a, float b, float mix)
	{ float tmp1 = mix*mix*mix; float tmp2 = (1-mix)*(1-mix)*(1-mix);
		mix = (1-mix)*tmp1+mix*(1-tmp2); return (1-mix)*a+mix*b;
	};

	if (!foundOptimal && tween >= 1+rate)
	{
		//BuildPDBs();
		ida.SetHeuristic(&searchHeuristic);
		WeightedHeuristic<MNPuzzleState<4, 4>> w(&mnp, 2.0);
		ida.SetHeuristic(&w);
		ida.GetPath(&mnp, start, goal, acts);
		foundOptimal = true;
		std::string s = "You solved with "+std::to_string(numActions)+" moves; optimal is between ";
		s +=std::to_string(std::max((int)mnp.HCost(start, goal), (int)(2*acts.size()/3)))+" and "+std::to_string(acts.size())+" moves";
		submitTextToBuffer(s.c_str());
		printf("%s\n", s.c_str());
	}

	if (tween >= 1)
	{
		mnp.Draw(display, curr);
	}
	else {
		mnp.Draw(display, curr, last, smooth(0, 1, tween));
//		mnp.Draw(display, curr, last, tween);
	}

	if (tween <= 1+rate)
		tween += rate;

	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

uint64_t random64()
{
	uint64_t r1 = random();
	uint64_t r2 = random();
	return (r1<<32)|r2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
//		case 'h': //whichHeuristic = (whichHeuristic+1)%16; break;
		case '&':
#ifndef __EMSCRIPTEN__
			MakeSVG(GetContext(windowID)->display, "/Users/nathanst/Pictures/SVG/STP.svg", 500, 500);
			break;
#endif
		case 's':
		{
			submitTextToBuffer("");
			foundOptimal = true;
			numActions = 0;
			mnp.StoreGoal(goal);
			mnp.GetStateFromHash(curr, random64()%mnp.GetMaxStateHash());
			start = curr;
			goal.Reset();
			mnp.StoreGoal(goal);

			break;
		}
		case 'r':
		{
			curr = goal;
//			recording = !recording;
//			running = true;
			break;
		}
		case '?':
		{
			mnp.ApplyAction(curr, kRight);
			printf("Parity: %d\n", mnp.GetParity(curr));
		}
			break;
		default:
			break;
	}
	
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	switch (mType)
	{
		case kMouseDown:
		{
			auto a = mnp.GetAction(curr, loc);
			if (a != kNoSlide)
			{
				last = curr;
				tween = 0;
				mnp.ApplyAction(curr, a);
				numActions++;
				if (curr == goal)
				{
//					std::string s = "Solved in "+std::to_string(numActions)+" moves; computing optimal solution now...";
//					submitTextToBuffer(s.c_str());
//					printf("%s\n", s.c_str());
					foundOptimal = false;
				}
			}
			return true;
		}
		default: return true;
	}
	return false;
}

#ifndef __EMSCRIPTEN__

#include "STPInstances.h"
#include "ParallelIDAStar.h"
void MakeAnimation()
{
	const auto smooth = [](float a, float b, float mix)
	{ float tmp1 = mix*mix*mix; float tmp2 = (1-mix)*(1-mix)*(1-mix);
		mix = (1-mix)*tmp1+mix*(1-tmp2); return (1-mix)*a+mix*b;
	};

	//MNPuzzle<4, 4> mnp;
	//MNPuzzleState<4, 4> start, goal;
	goal.Reset();
	start = STP::GetKorfInstance(88);
	ParallelIDAStar<MNPuzzle<4, 4>, MNPuzzleState<4, 4>, slideDir> pida;
	std::vector<slideDir> p;
	pida.GetPath(&mnp, start, goal, p);
	path.resize(0);
	path.push_back(start);
	for (auto a : p)
	{
		mnp.ApplyAction(start, a);
		path.push_back(start);
	}
	Graphics::Display d;
	d.ReinitViewports({-1, -1, 1, 1}, kScaleToSquare);
	int count = 0;
	for (int x = 0; x < path.size()-1; x++)
	{
		d.StartFrame();
		mnp.Draw(d, path[x]);
		d.EndFrame();
		std::string tmp = "/Users/nathanst/Pictures/SVG/STP"+std::to_string(count)+".svg";
		count++;
		MakeSVG(d, tmp.c_str(), 500, 500);

		for (float f = 0; f <= 1; f += 1.0f/30.0f)
		{
			d.StartFrame();
			mnp.Draw(d, path[x], path[x+1], smooth(0, 1, 1-f));
			d.EndFrame();
			tmp = "/Users/nathanst/Pictures/SVG/STP"+std::to_string(count)+".svg";
			count++;
			MakeSVG(d, tmp.c_str(), 500, 500);
		}
	}
	d.StartFrame();
	mnp.Draw(d, path.back());
	d.EndFrame();
	std::string tmp = "/Users/nathanst/Pictures/SVG/STP"+std::to_string(count)+".svg";
	count++;
	MakeSVG(d, tmp.c_str(), 500, 500);
}

#endif
