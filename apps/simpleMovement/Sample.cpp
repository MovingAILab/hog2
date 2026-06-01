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
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
//#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "SteeringEnvironment.h"
#include "SeekUnit.h"
#include "FleeUnit.h"
#include "AlignUnit.h"
#include "SteeringUnit.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 100;
int gStepsPerFrame = 4;
double searchWeight = 0;
bool gRecording = false;

SeekUnit *theSeekUnit = 0;
FleeUnit *theFleeUnit = 0;
AlignUnit *theAlignUnit = 0;

std::vector<UnitSteeringSimulation *> unitSims;

static Timer t;
static float elapsed = 0;
int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<steeringState, steeringAction, SteeringEnvironment>(new SteeringEnvironment());
	unitSims[id]->SetStepType(kMinTime);
	t.StartTimer();
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	//InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyRecordingHandler, "Record", "Toggle screen capture", kNoModifier, 'r');
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add Align Unit", "Deploys an aligning unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add Seek Unit", "Deploys a seeking unit", kNoModifier, 's');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add Flee Unit", "Deploys a fleeing unit", kNoModifier, 'f');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add avoid Unit", "Deploys an avoiding unit", kNoModifier, 'v');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add wander Unit", "Deploys a wandering unit", kNoModifier, 'w');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add chase Unit", "Deploys a chasing unit", kNoModifier, 'c');
	
	InstallCommandLineHandler(MyCLHandler, "-makeMaze", "-makeMaze x-dim y-dim corridorsize filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-resize", "-resize filename x-dim y-dim filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-batch integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	
	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
		delete unitSims[windowID];
		unitSims[windowID] = 0;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
//		float currTime = t.EndTimer();
//		while (elapsed < currTime)
//		{
			unitSims[windowID]->StepTime(1.0/30.0);
//			elapsed += 1.0/30.0;
//		}
	}
	unitSims[windowID]->OpenGLDraw();
	
	if (gRecording)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Desktop/Movie%d%d%d", (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	return 0; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'w':
			break;
		case '[': if (gStepsPerFrame > 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			unitSims[windowID]->SetPaused(false);
			unitSims[windowID]->StepTime(1.0/30.0);
			unitSims[windowID]->SetPaused(true);
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char c)
{
	steeringState theState;
	theState.x = random()%(int)worldRadius-worldRadius/2;
	theState.y = random()%(int)worldRadius-worldRadius/2;
	theState.heading = 0;
	theState.v = 0;

	if (c == 'a') // align
	{
		theAlignUnit = new AlignUnit(theState, 0, 0);
		unitSims[windowID]->AddUnit(theAlignUnit);
		worldRadius = 50.0f;
	}
	if (c == 's') // seek
	{
		theSeekUnit = new SeekUnit(theState, 0, 0);
		unitSims[windowID]->AddUnit(theSeekUnit);
		worldRadius = 50.0f;
	}
	if (c == 'f') // flee
	{
		theFleeUnit = new FleeUnit(theState, 0, 0);
		unitSims[windowID]->AddUnit(theFleeUnit);
		worldRadius = 50.0f;
	}
	if (c == 'v') // avoid
	{
		SteeringUnit *u;
		unitSims[windowID]->AddUnit(u = new SteeringUnit(theState, 0, 0));
		u->SetMode(kSeparation);
		worldRadius = 50.0f;
	}
	if (c == 'w') // wander
	{
		SteeringUnit *u;
		unitSims[windowID]->AddUnit(u = new SteeringUnit(theState, 0, 0));
		u->SetMode(kWander);
		worldRadius = 50.0f;
	}
	if (c == 'c') // chase
	{
		steeringState theState;
		theState.x = random()%(int)worldRadius-worldRadius/2;
		theState.y = random()%(int)worldRadius-worldRadius/2;
		theState.heading = random()%360;
		theState.v = 0;
		SteeringUnit *u, *v;
		unitSims[windowID]->AddUnit(u = new SteeringUnit(theState, 0, 0));
		u->SetMode(kSeparation);
		theState.x = random()%(int)worldRadius-worldRadius/2;
		theState.y = random()%(int)worldRadius-worldRadius/2;
		theState.heading = random()%360;
		v = new SteeringUnit(theState, 0, 0);
		v->SetTarget(u);
		v->SetMode(kChase);
		unitSims[windowID]->AddUnit(v);
		worldRadius = 75.0f;
		
	}
}

void MyRecordingHandler(unsigned long windowID, tKeyboardModifier , char)
{
	gRecording = !gRecording;
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	steeringState theState;
	theState.x = random()%(int)worldRadius-worldRadius/2;
	theState.y = random()%(int)worldRadius-worldRadius/2;
	theState.heading = random()%360;
	theState.v = 0;
	SteeringUnit *u, *v;
	unitSims[windowID]->AddUnit(u = new SteeringUnit(theState, 0, 0));
	u->SetMode(kFlock);
	theState.x = random()%(int)worldRadius-worldRadius/2;
	theState.y = random()%(int)worldRadius-worldRadius/2;
	theState.heading = random()%360;
	v = new SteeringUnit(theState, 0, 0);
	v->SetTarget(u);
	v->SetMode(kFlock);
	unitSims[windowID]->AddUnit(v);
	worldRadius = 200.0f;
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	mouseTracking = false;
	if (button == kLeftButton)
	{
		if (theSeekUnit != 0)
		{
			theSeekUnit->AddTarget(loc.x, loc.y);
		}
	}
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
			case kMouseDrag:
				//printf("Mouse tracking at (%f, %f)\n", loc.x, loc.y);
				if (theSeekUnit)
					theSeekUnit->SetTarget(loc.x, loc.y);
				if (theFleeUnit)
					theFleeUnit->SetTarget(loc.x, loc.y);
				if (theAlignUnit)
					theAlignUnit->SetTarget(loc.x, loc.y);
				break;
			case kMouseUp:
			{
				
			}
				break;
		}
		return true;
	}
	return true;
}
