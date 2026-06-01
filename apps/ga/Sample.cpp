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

NQueenState q(8); // 30

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
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to random state", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
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
//	MyDisplayHandler(windowID, kNoModifier, 'o');

}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-map") == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);

		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': break;
		case ']':  break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'r':
		{
			for (unsigned int x = 0; x < q.locs.size(); x++)
				q.locs[x] = random()%q.locs.size();
		}
			break;
		case 'p': break;
		case 'o':
		{
			std::vector<NQueenAction> acts;
			NQueens env;
			NQueenState s2;
			env.GetActions(q, acts);
			int best = -1;
			int ties = 2;
			int bestCnt = env.NumCollisions(q);
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
