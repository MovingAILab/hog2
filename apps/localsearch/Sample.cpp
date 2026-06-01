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
#include "Sample.h"
#include "NQueens.h"
#include <iostream>

NQueenState q(8); // 30
bool run = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}

//ContractionHierarchy *ch;
/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Size", "Change the size of the board to 6*(x+2)", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Pause/Run Simulation", "Pause/run simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to random state", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Take one greedy step", kNoModifier, 'o');
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	NQueens env;
	env.OpenGLDraw(q);
	env.OpenGLDrawConflicts(q);
	if (run)
	{
		MyDisplayHandler(windowID, kNoModifier, 'o');
	}

	if (viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		cnt++;
	}

}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0':case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':
			q = NQueenState(6*(key-'0'+1));
			break;
		case 'r':
		{
			for (unsigned int x = 0; x < q.locs.size(); x++)
				q.locs[x] = random()%q.locs.size();
		}
			break;
		case 'p': run = !run; break;
		case 'o':
		{
			std::vector<NQueenAction> acts;
			NQueens env;
			NQueenState s2;
			env.GetActions(q, acts);
			int best = -1;
			int ties = 1;
			int bestCnt = env.NumCollisions(q);
			if (bestCnt == 0)
				break;
			for (unsigned int x = 0; x < acts.size(); x++)
			{
				env.GetNextState(q, acts[x], s2);
				if (env.NumCollisions(s2) < bestCnt)
				{
					bestCnt = env.NumCollisions(s2);
					best = x;
					ties = 2;
				}
				else if (env.NumCollisions(s2) == bestCnt)
				{
					if (0 == random()%ties)
						best = x;
					ties++;
				}
			}
			if (best != -1)
			{
				env.ApplyAction(q, acts[best]);
				//std::cout << acts[best] << std::endl;
			}
			else {
				printf("No move\n");
			}
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				break;
			case kMouseDrag:
				break;
			case kMouseUp:
				break;
		}
		return true;
	}
	return false;
}
