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
#include "TOH.h"
//#include "IncrementalIDA.h"
#include "IDAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"

IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
Heuristic<MNPuzzleState<4, 4>> h;
bool recording = false;
bool running = false;
float rate = 1.0/8.0;
float tween = 1;
int numActions = 0;
bool foundOptimal = true;

TOH<16> toh;
TOHState<16>
MNPuzzleState<4, 4> last;
MNPuzzleState<4, 4> curr;
MNPuzzleState<4, 4> start;
MNPuzzleState<4, 4> goal;
std::vector<slideDir> acts;
std::vector<MNPuzzleState<4, 4>> path;


int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
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
	
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 5});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.lookups.push_back({kLeafNode, 3, 3});
	h.lookups.push_back({kLeafNode, 4, 4});

	h.heuristics.resize(0);
	h.heuristics.push_back(&mnp);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	h.heuristics.push_back(pdb3);
	h.heuristics.push_back(pdb4);
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

		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToFill);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	
	if (!foundOptimal && tween >= 1+rate)
	{
		//BuildPDBs();
		ida.SetHeuristic(&h);
		WeightedHeuristic<MNPuzzleState<4, 4>> w(&mnp, 2.0);
		ida.SetHeuristic(&w);
		ida.GetPath(&mnp, start, goal, acts);
		foundOptimal = true;
		std::string s = "You solved with "+std::to_string(numActions)+" moves; optimal is between ";
		s +=std::to_string(std::max((int)mnp.HCost(start), (int)(2*acts.size()/3)))+" and "+std::to_string(acts.size())+" moves";
		submitTextToBuffer(s.c_str());
		printf("%s\n", s.c_str());
	}

	if (tween >= 1)
	{
		mnp.Draw(display, curr);
	}
	else {
		mnp.Draw(display, curr, last, tween);
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

