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
#include "PancakePuzzle.h"
#include "IDAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"

IDAStar<PancakePuzzleState<16>, PancakePuzzleAction> ida;
bool recording = false;
bool running = false;
float rate = 1.0/8.0;
float tween = 1;
int numActions = 0;
bool foundOptimal = true;
bool showOptimal = false;

PancakePuzzle<16> pancake;
PancakePuzzleState<16> last;
PancakePuzzleState<16> curr;
PancakePuzzleState<16> start;
PancakePuzzleState<16> goal;
std::vector<PancakePuzzleAction> acts;
//std::vector<PancakePuzzleState<16>> path;
PancakePuzzleAction drawAction = 0;

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
	InstallKeyboardHandler(MyDisplayHandler, "Optimal", "Show optimal solution", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Randomize", "Get Random State", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseDown));
	srandom(time(0));
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
		ida.SetHeuristic(&pancake);
		ida.GetPath(&pancake, start, goal, acts);
		foundOptimal = true;
		pancake.StoreGoal(goal);
		std::string s = "You solved with "+std::to_string(numActions)+" moves; optimal was ";
		s += std::to_string(acts.size())+" moves.";
//		s +=std::to_string(std::max((int)pancake.HCost(start), (int)(2*acts.size()/3)))+" and "+std::to_string(acts.size())+" moves";
		submitTextToBuffer(s.c_str());
		printf("%s\n", s.c_str());
	}

	pancake.Draw(display);
	if (tween >= 1)
	{
		pancake.Draw(display, curr);
		if (drawAction > 1)
		{
			pancake.Draw(display, drawAction);
		}
	}
	else {
		pancake.Draw(display, last, curr, tween);
	}

	if (tween <= 1+rate)
	{
		if (showOptimal)
			tween += 0.25f*rate;
		else
			tween += rate;
	}
	else if (acts.size() > 0 && showOptimal)
	{
		last = curr;
		pancake.GetNextState(last, acts[0], curr);
		tween = 0;
		acts.erase(acts.begin());
		if (acts.size() == 0)
			showOptimal = false;
	}


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
		case 'o':
			ida.SetHeuristic(&pancake);
			ida.GetPath(&pancake, start, goal, acts);
			if (acts.size() > 0)
			{
				tween = 0;
				showOptimal = true;
				last = start;
				pancake.GetNextState(start, acts[0], curr);
				acts.erase(acts.begin());
			}
			break;
		case 's':
		{
//			{
//				const int s = 4;
//				Permutations<s> p;
//				int vals[s];
//				uint64_t r = p.MaxRank();
//				for (int x = 0; x < r; x++)
//				{
//					p.Unrank(x, vals);
//					std::cout << "[" << x << "] ";
//					for (int y = 0; y < s; y++)
//					std::cout << vals[y] << " ";
//					std::cout << "\n";
//				}
//			}
			
			submitTextToBuffer("");
			foundOptimal = true;
			numActions = 0;
			pancake.StoreGoal(goal);
			pancake.GetStateFromHash(random64()%pancake.GetMaxHash(), curr);
//			mnp.GetStateFromHash(curr, random64()%mnp.GetMaxStateHash());
			start = curr;
			goal.Reset();
			pancake.StoreGoal(goal);
			acts.clear();
			
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
//			pancake.ApplyAction(curr, kRight);
//			printf("Parity: %d\n", mnp.GetParity(curr));
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
		case kMouseMove:
		{
			drawAction = pancake.GetAction(curr, loc);
		}
			break;
		case kMouseDown:
		{
			auto a = pancake.GetAction(curr, loc);
			if (a > 1)
			{
				showOptimal = false;
				acts.clear();
				last = curr;
				tween = 0;
				drawAction = 0;
				pancake.ApplyAction(curr, a);
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
	return true;
}

