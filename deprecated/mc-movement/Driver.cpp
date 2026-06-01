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
#include "UnitSimulation.h"
#include "AStar.h"
#include "GenericSearchUnit.h"
#include "GenericPatrolUnit.h"
#include "TemplateAStar.h"
#include "MapSectorAbstraction.h"
#include "ScenarioLoader.h"
#include "MotionCaptureMovement.h"

class FixedMCActionUnit : public Unit<mcMovementState, mcMovementAction, MCEnvironment> {
public:
	FixedMCActionUnit(mcMovementState &s)
	:loc(s) { SetColor(0.0, 1.0, 0.0); }
	const char *GetName() { return "fixed-action"; }
	bool MakeMove(MCEnvironment *, OccupancyInterface<mcMovementState,mcMovementAction> *, SimulationInfo<mcMovementState,mcMovementAction,MCEnvironment> *, mcMovementAction& a)
	{
		a = 14;
		return true;
	}
	virtual void UpdateLocation(MCEnvironment *, mcMovementState &s, bool success, SimulationInfo<mcMovementState,mcMovementAction,MCEnvironment> *)
	{ loc = s; }
	virtual void GetLocation(mcMovementState &s) { s = loc; }
	virtual void OpenGLDraw(const MCEnvironment *e, const SimulationInfo<mcMovementState,mcMovementAction,MCEnvironment> * si) const
	{
//		e->SetColor(0.0, 1.0, 0.0, 0.1);
//		e->OpenGLDraw(loc);

	
		PublicUnitInfo<mcMovementState, mcMovementAction, MCEnvironment> i;
		si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
		//		printf("(%f-%f)/(%f-%f)\n",
		//			   si->GetSimulationTime(), i.lastTime, i.nextTime, i.lastTime);
		e->SetColor(1.0, 0.25, 0.25, 1.0);
		//		std::cout << si->GetCurrentUnit() << " is at " << i.currentState << std::endl;
		if (fgreater(si->GetSimulationTime(), i.nextTime))
			e->OpenGLDraw(i.currentState);
		else
			e->OpenGLDraw(i.lastState, i.currentState,
						  (si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));

	}
	virtual void GetGoal(mcMovementState &s) { s = loc; }
	mcMovementState loc;
};


bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
//int mapSize = 30;
bool recording = false;

std::vector<UnitSimulation<mcMovementState, mcMovementAction, MCEnvironment> *> unitSims;
typedef GenericSearchUnit<mcMovementState, mcMovementAction, MCEnvironment> DirSearchUnit;
typedef GenericPatrolUnit<mcMovementState, mcMovementAction, MCEnvironment> DirPatrolUnit;

//void RunBatchTest(char *file, model mm, heuristicType heuristic, int length);

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
	SetNumPorts(id, 1);
//	Map *map;
//	if (gDefaultMap[0] == 0)
//	{
//		map = new Map(mapSize, mapSize);
//	}
//	else {
//		map = new Map(gDefaultMap);
//		map->Scale(2*map->GetMapWidth(), 2*map->GetMapHeight());
//	}

	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<mcMovementState, mcMovementAction, MCEnvironment>(new MCEnvironment());
	unitSims[id]->SetStepType(kRealTime);
	unitSims[id]->GetStats()->EnablePrintOutput(true);
	unitSims[id]->GetStats()->AddIncludeFilter("gCost");
	unitSims[id]->GetStats()->AddIncludeFilter("nodesExpanded");
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
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	//InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-heuristic", "-heuristic <octile, perimeter, extendedPerimeter>", "Selects how many steps of the abstract path will be refined.");
	InstallCommandLineHandler(MyCLHandler, "-planlen", "-planlen <int>", "Selects how many steps of the abstract path will be refined.");
	InstallCommandLineHandler(MyCLHandler, "-scenario", "-scenario filename", "Selects the scenario to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-model", "-model <human, tank, vehicle>", "Selects the motion model.");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-seed", "-seed integer", "Sets the randomized number generator to use specified key.");
	InstallCommandLineHandler(MyCLHandler, "-batch", "-batch numScenarios", "Runs a bunch of test scenarios.");
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
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;
	
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/10.0);
		// If batch mode is on, automatically start the test
		if (unitSims[windowID]->Done() && unitSims[windowID]->GetNumUnits() < 100)
		{
			//MyRandomUnitKeyHandler(windowID, kNoModifier, 'a');
		}
	}
	unitSims[windowID]->OpenGLDraw();	

	if (recording)
	{
		static int index = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/anim-%d%d%d", index/100, (index/10)%10, index%10);
		SaveScreenshot(windowID, fname);
		index++;
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
		case '[': recording = true; break;
		case ']': recording = false; break;
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
			if (unitSims[windowID]->GetPaused())
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

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
//	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();

	mcMovementState l1;
	l1.x = 0;
	l1.y = 0;
	l1.heading = 0;
	
	FixedMCActionUnit *f = new FixedMCActionUnit(l1);
	f->SetSpeed(1);
	unitSims[windowID]->AddUnit(f);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	static TemplateAStar<mcMovementState, mcMovementAction, MCEnvironment> astar;
	astar.SetWeight(2.0);
	mcMovementState l1;
	l1.x = 0;
	l1.y = 0;
	l1.heading = 0;

	mcMovementState l2;
	l2.x = 5;
	l2.y = 0;
	l2.heading = 180;

	DirPatrolUnit *ru1 = new DirPatrolUnit(l1, &astar);
	ru1->SetNumPatrols(5);
	ru1->AddPatrolLocation(l2);
	ru1->AddPatrolLocation(l1);
	ru1->SetSpeed(2);
	unitSims[windowID]->AddUnit(ru1);
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
			}
			break;
		}
		return true;
	}
	return false;
}
