#include "Common.h"
#include "Driver.h"
//#include "PRAStar.h"
//#include "SearchUnit.h"
#include "UnitSimulation.h"
//#include "Directional2DEnvironment.h"
//#include "RandomUnit.h"
//#include "AStar.h"
//#include "GenericSearchUnit.h"
//#include "GenericPatrolUnit.h"
//#include "TemplateAStar.h"
//#include "MapSectorAbstraction.h"
//#include "DirectionalPlanner.h"
#include "ScenarioLoader.h"
#include "Airplane.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
bool recording = false;
double simTime = 0;

//DirectionalPlanner *dp = 0;
//MapSectorAbstraction *quad = 0;
//std::vector<DirectionSimulation *> unitSims;
//typedef GenericSearchUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> DirSearchUnit;
//typedef RandomUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> RandomDirUnit;
//typedef GenericPatrolUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> DirPatrolUnit;

//void RunBatchTest(char *file, model mm, heuristicType heuristic, int length);

AirplaneEnvironment ae;
airplaneState s;
airplaneState s2;
airplaneState s3;
airplaneState s4;
airplaneState tmp;

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
	
//	unitSims.resize(id+1);
//	unitSims[id] = new DirectionSimulation(new Directional2DEnvironment(map, kVehicle));
//	unitSims[id]->SetStepType(kRealTime);
//	unitSims[id]->GetStats()->EnablePrintOutput(true);
//	unitSims[id]->GetStats()->AddIncludeFilter("gCost");
//	unitSims[id]->GetStats()->AddIncludeFilter("nodesExpanded");
//	dp = new DirectionalPlanner(quad);
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
	InstallKeyboardHandler(MyDisplayHandler, "Recird", "Toggle recording.", kNoModifier, 'r');
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

std::vector<airplaneAction> acts;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	simTime += 0.01;
	//recVec v = ae.GetCoordinate(s2.x, s2.y, s2.height);
	//cameraLookAt(v.x, v.y, v.z, 0.01);
	if (simTime > 1)
	{
		simTime = 0;
		s.heading = 2;
		s.x++;
		s.x = s.x%80;

		s2.heading = 3;
		s2.x++;
		s2.y++;
		s2.x = s2.x%80;
		s2.y = s2.y%80;
		s2.height = 4;
	}
	
	//ae.OpenGLDraw();

	//s.heading = (s.heading+1)%8;

	tmp = s2;
	tmp.heading = 3;
	tmp.x++;
	tmp.y++;
	tmp.x = tmp.x%80;
	tmp.y = tmp.y%80;
	tmp.height = 4;
	ae.SetColor(0.0, 1.0, 0.0);
	//ae.OpenGLDraw(s2, tmp, simTime);
	
	tmp = s;
	tmp.heading = 2;
	tmp.x++;
	tmp.x = tmp.x%80;
	ae.SetColor(1.0, 0.0, 0.0);
	//ae.OpenGLDraw(s, tmp, simTime);
	
	s3.x = 40;
	s3.y = 40;
	s3.height = 10;
	s3.heading = 0;
	ae.GetActions(s3, acts);
	ae.SetColor(0, 1, 1);
	//ae.OpenGLDraw(s3);
	for (const auto &a : acts)
	{
		ae.GetNextState(s3, a, s4);
		ae.SetColor(0, 0, 1);
		//	ae.OpenGLDraw(s3, s4, simTime);
	}
	
	if (recording) // deprecated
	{
		static int index = 0;
		char fname[255];
		//sprintf(fname, "/Users/nathanst/anim-%d%d%d", index/100, (index/10)%10, index%10);
		//SaveScreenshot(windowID, fname);
		//printf("Saving '%s'\n", fname);
		//index++;
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
//	static model motionModel = kVehicle;
//	static int length = 3;
//	static heuristicType hType = kExtendedPerimeterHeuristic;
//	
//	if (strcmp(argument[0], "-heuristic") == 0)
//	{
//		if (strcmp(argument[1], "octile") == 0)
//		{
//			hType = kOctileHeuristic;
//			printf("Heuristic: octile\n");
//		}
//		else if (strcmp(argument[1], "perimeter") == 0)
//		{
//			hType = kPerimeterHeuristic;
//			printf("Heuristic: perimeter\n");
//		}
//		else if (strcmp(argument[1], "extendedPerimeter") == 0)
//		{
//			hType = kExtendedPerimeterHeuristic;
//			printf("Heuristic: extended perimeter\n");
//		}
//			
//	}
//	if (strcmp( argument[0], "-planlen" ) == 0 )
//	{
//		length = atoi(argument[1]);
//		printf("Plan Length: %d\n", length);
//		return 2;
//	}
//	if (strcmp( argument[0], "-model" ) == 0 )
//	{
//		if (strcmp(argument[1], "tank") == 0)
//		{
//			motionModel = kTank;
//			printf("Motion Model: Tank\n");
//		}
//		else if (strcmp(argument[1], "vehicle") == 0)
//		{
//			motionModel = kVehicle;
//			printf("Motion Model: Vehicle\n");
//		}
//		else if (strcmp(argument[1], "human") == 0)
//		{
//			motionModel = kHumanoid;
//			printf("Motion Model: Human\n");
//		}
//		return 2;
//	}
//	if (strcmp( argument[0], "-map" ) == 0 )
//	{
//		if (maxNumArgs <= 1)
//			return 0;
//		strncpy(gDefaultMap, argument[1], 1024);
//		return 2;
//	}
//	else if (strcmp( argument[0], "-seed" ) == 0 )
//	{
//		if (maxNumArgs <= 1)
//			return 0;
//		srand(atoi(argument[1]));
//		return 2;
//	}
//	else if (strcmp( argument[0], "-size" ) == 0 )
//	{
//		if (maxNumArgs <= 1)
//			return 0;
//		mapSize = atoi(argument[1]);
//		assert( mapSize > 0 );
//		printf("mapSize = %d\n", mapSize);
//		return 2;
//	}
//	else if (strcmp(argument[0], "-scenario") == 0)
//	{
//		printf("Scenario: %s\n", argument[1]);
//		RunBatchTest(argument[1], motionModel, hType, length);
//		exit(0);
//	}
	return 2; //ignore typos
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r': recording = !recording; break;
		case '[': recording = true; break;
		case ']': recording = false; break;
		case '\t':
			break;
		case 'p': break;//unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
//			if (unitSims[windowID]->GetPaused())
//			{
//				unitSims[windowID]->SetPaused(false);
//				unitSims[windowID]->StepTime(1.0/30.0);
//				unitSims[windowID]->SetPaused(true);
//			}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
	
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
//	// attmpt to learn!
//	Map m(100, 100);
//	Directional2DEnvironment d(&m);
//	//Directional2DEnvironment(Map *m, model envType = kVehicle, heuristicType heuristic = kExtendedPerimeterHeuristic);
//	xySpeedHeading l1(50, 50), l2(50, 50);
//	std::unordered_map<uint64_t, xySpeedHeading, Hash64 > stateTable;
//	
//	std::vector<xySpeedHeading> path;
//	TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> alg;
//	alg.SetStopAfterGoal(false);
//	alg.InitializeSearch(&d, l1, l1, path);
//	for (int x = 0; x < 2000; x++)
//		alg.DoSingleSearchStep(path);
//	int count = alg.GetNumItems();
//	LinearRegression lr(37, 1, 1/37.0); // 10 x, 10 y, dist, heading offset [16]
//	std::vector<double> inputs;
//	std::vector<double> output(1);
//	for (unsigned int x = 0; x < count; x++)
//	{
//		// note that the start state is always at rest;
//		// we actually want the goal state at rest?
//		// or generate everything by backtracking through the parents of each state
//		const AStarOpenClosedData<xySpeedHeading> val = GetItem(x);
//		inputs[0] = sqrt((val.data.x-l1.x)*(val.data.x-l1.x)+(val.data.y-l1.)*(val.data.y-l1.y));
//		// fill in values
//		if (fabs(val.data.x-l1.x) >= 10)
//			inputs[10] = 1;
//		else inputs[1+fabs(val.data.x-l1.x)] = 1;
//		if (fabs(val.data.y-l1.y) >= 10)
//			inputs[20] = 1;
//		else inputs[11+fabs(val.data.y-l1.y)] = 1;
//		// this is wrong -- I need the possibility of flipping 15/1 is only 2 apart
//		intputs[30+((int)(fabs(l1.rotation-val.data.rotation)))%16] = 1;
//		output[0] = val.g;
//		lr.train(inputs, output);
//		// get data and learn to predict the g-cost
//		//val.data.
//		//val.g;
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
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
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				xySpeedHeading l1, l2;
//				l1.x = px1;
//				l1.y = py1;
//				l2.x = px2;
//				l2.y = py2;
//				DirPatrolUnit *ru1 = new DirPatrolUnit(l1, dp);
//				ru1->SetNumPatrols(1);
//				ru1->AddPatrolLocation(l2);
//				ru1->AddPatrolLocation(l1);
//				ru1->SetSpeed(2);
//				unitSims[windowID]->AddUnit(ru1);
			}
			break;
		}
		return true;
	}
	return false;
}
