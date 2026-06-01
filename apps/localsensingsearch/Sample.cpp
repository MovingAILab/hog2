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
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "LocalSensingUnit.h"
#include "RIBS.h"
#include "LRTAStar.h"
#include "LSSLRTAStar.h"
#include "gLSSLRTAStar.h"
#include "ScenarioLoader.h"
#include "StatUtil.h"
#include "HeuristicLearningMeasure.h"
#include "LearningUnit.h"
#include "FLRTAStar2.h"
#include "FLRTAStar.h"
#include "Directional2DEnvironment.h"
#include "LearningUnit.h"
#include "MPLRTAStar.h"
#include "GridLSSLRTAStar.h"
#include "daLRTAStar.h"
//#include "CanonicalRTAStar.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 100;

std::vector<EpisodicSimulation<xyLoc, tDirection, MapEnvironment> *> unitSims;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;
std::vector<xyLoc> path;
double stepsPerFrame = 1.0/100.0;

HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;

void RunScenario(char *name, int which);
void RunWeightedHeuristicScenario(char *name, int lookahead, double heuristicWeight);
void RunTankScenario(char *name, int which);
void RunScalingTest(int size, int which, float weight);
void RunTankScalingTest(int size, int which, float weight);
void RunWorkMeasureTest();
void RunSTPTest(int which);

void RunBigScenario(char *name, int which);

void CreateSimulation(int id);

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
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(mazeSize, mazeSize);
		//MakeMaze(map, 4);
		//MakeMaze(map, 32);
		//map->SetTileSet(kBitmap);
//		map->SetTerrainType(1, mazeSize-2,
//							mazeSize-2, mazeSize-2, kOutOfBounds);
//		map->SetTerrainType(mazeSize-2, 1,
//							mazeSize-2, mazeSize-2, kOutOfBounds);
//		map->Load("/Users/nathanst/Desktop/100.map");
//		map->Load("/Users/nathanst/hog2/maps/dao/arena2.map");
//		map->Load("/Users/nathanst/hog2/maps/random/random512-40-0.map");
//		map->Load("/Users/nathanst/hog2/maps/wc3maps512/gnollwood.map");
		
//		map->SetRectHeight(0, 0, mazeSize-1, mazeSize-1, 0, kOutOfBounds);
//		for (int y = 0; y < mazeSize; y+=2)
//		{
//			map->SetTerrainType(0, y,
//								mazeSize-1, y, kGround);
//			if (0 == (y/2)%2)
//			{
//				map->SetTerrainType(mazeSize-1, y,
//									mazeSize-1, y+2, kGround);
//			}
//			else {
//				map->SetTerrainType(0, y,
//									0, y+2, kGround);
//			}
//		}

		// ladder map
		map = new Map(mazeSize, mazeSize*2);
		map->SetRectHeight(0, 0, mazeSize-1, 2*mazeSize-1, 0, kOutOfBounds);
		for (int x = 0; x < mazeSize; x++)
		{
			map->SetTerrainType(x, x,
								x, x+3, kGround);
			if (1 == x%2)
				map->SetTerrainType(x, x,
									x, x+20, kGround);
			map->SetTerrainType(x, x+20,
								x, x+21, kGround);
		}
		map->SetRectHeight(mazeSize-5*sqrt(mazeSize), 20+mazeSize, mazeSize-1, 20+mazeSize+5*sqrt(mazeSize), 0, kGround);
		map->SetTerrainType(0, 20+mazeSize,
							mazeSize-1, 20+mazeSize, kGround);
		map->SetTerrainType(0, 20+mazeSize,
							0, 2*mazeSize-1, kGround);
		
		
		//MakeMaze(map, 1);
//		BuildRandomRoomMap(map, 16);
//		map->Scale(mazeSize, mazeSize);
	}
	else
		map = new Map(gDefaultMap);
	//map->SetTileSet(kBitmap);
	map->SetTileSet(kWinter);
	MapEnvironment *me;
	unitSims.resize(id+1);
	unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, MapEnvironment>(me = new MapEnvironment(map, false));
	me->SetDiagonalCost(1.5);
	unitSims[id]->SetStepType(kRealTime);//kUniTime
	unitSims[id]->SetThinkingPenalty(0);
	unitSims[id]->GetStats()->EnablePrintOutput(true);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Sum Distance", "Print total distance travelled by each unit", kNoModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Increase Sim Speed", "Increase Sim Speed", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Decrease Sim Speed", "Decrease Sim Speed", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear units", "Clear all units from sim", kAnyModifier, '|');

	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-size <integer>", "If size is set, we create a square maze with the x and y dimensions specified.");
	InstallCommandLineHandler(MyCLHandler, "-scenario", "-scenario <file> <algorithm>", "Load and run a scenario offline.");
	InstallCommandLineHandler(MyCLHandler, "-weightedHeuristicScenario", "-weightedHeuristicScenario <file> <lookahead> <weight>", "run lss-lrta* with weighted initial heuristic on scenario");
	InstallCommandLineHandler(MyCLHandler, "-scenarioTank", "-scenarioTank <file> <algorithm>", "Load and run a scenario offline.");
	InstallCommandLineHandler(MyCLHandler, "-scaleTest", "-scaleTest <size> <which> <weight>", "Run a scaling test with local minima <size>.");
	InstallCommandLineHandler(MyCLHandler, "-scaleTestTank", "-scaleTestTank <size> <which> <weight>", "Run a scaling test with local minima <size>.");
	InstallCommandLineHandler(MyCLHandler, "-STPTest", "-STPTest", "Run a STP test.");
	InstallCommandLineHandler(MyCLHandler, "-flrta2", "-flrta2", "Tests flrta2");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);

		delete ma1;
		ma1 = 0;
		delete ma2;
		ma2 = 0;
		delete gdh;
		gdh = 0;
		delete unitSims[windowID];
		unitSims[windowID] = 0;
		runningSearch1 = false;
		runningSearch2 = false;
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
	}
}

static bool recording = false;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(stepsPerFrame);
	}
	
	if (unitSims[windowID]->GetNumUnits() > 0)
	{
//		if (!unitSims[windowID]->GetPaused())
//			stepsPerFrame*=1.002;//1.01;//
		if (unitSims[windowID]->Done() && recording)
		{
			recording = false;
		}

	}

	if (GetNumPorts(windowID) == unitSims[windowID]->GetNumUnits())
	{
	  //		unitSims[windowID]->GetEnvironment()->OpenGLDraw();
	  //		unitSims[windowID]->OpenGLDraw(viewport);
		//measure.OpenGLDraw(unitSims[windowID]->GetEnvironment());

//		int tmp = GetActivePort(windowID);
//		SetActivePort(windowID, viewport);
//		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
//		PublicUnitInfo<xyLoc,tDirection,MapEnvironment> pui;
//		unitSims[windowID]->GetPublicUnitInfo(viewport, pui);
//		GLdouble a1, b1, c1, r1;
//		m->GetOpenGLCoord(pui.currentState.x, pui.currentState.y, a1, b1, c1, r1);
//		cameraMoveTo(a1, b1, c1-600*r1, 0.05);
//		SetActivePort(windowID, tmp);
	}
	else {
	  //		unitSims[windowID]->GetEnvironment()->OpenGLDraw();
	  //		unitSims[windowID]->OpenGLDraw();
	}
	
	if (mouseTracking)
	{
	  /*
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		GLdouble x, y, z, r;
		m->GetOpenGLCoord(px1, py1, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		m->GetOpenGLCoord(px2, py2, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		glEnd();
	  */
	}
	if (runningSearch1)
	{
	  /*
		a1.OpenGLDraw();
	  */
		a1.DoSingleSearchStep(path);
	}
	if (runningSearch2)
	{
	  /*
		a2.OpenGLDraw();
	  */
		a2.DoSingleSearchStep(path);
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	  { // deprecated
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		//SaveScreenshot(windowID, fname);
		cnt++;
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	else if (strcmp( argument[0], "-size" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		return 2;
	}
	else if (strcmp(argument[0], "-scenario") == 0)
	{
		RunScenario(argument[1], atoi(argument[2]));
		//RunBigScenario(argument[1], atoi(argument[2]));
	}
	else if (strcmp(argument[0], "-weightedHeuristicScenario") == 0)
	{
		RunWeightedHeuristicScenario(argument[1], atoi(argument[2]), atoi(argument[3]));
		//RunBigScenario(argument[1], atoi(argument[2]));
	}
	else if (strcmp(argument[0], "-scenarioTank") == 0)
	{
		RunTankScenario(argument[1], atoi(argument[2]));
		//RunBigScenario(argument[1], atoi(argument[2]));
	}
	else if (strcmp(argument[0], "-scaleTest") == 0)
	{
		RunScalingTest(atoi(argument[1]), atoi(argument[2]), atof(argument[3]));
	}
	else if (strcmp(argument[0], "-scaleTestTank") == 0)
	{
		RunTankScalingTest(atoi(argument[1]), atoi(argument[2]), atof(argument[3]));
	}
	else if (strcmp(argument[0], "-STPTest") == 0)
	{
		RunSTPTest(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-flrta2") == 0)
	{
		RunScalingTest(30, 7, 1.0);
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': stepsPerFrame/=2.0; break; 
		case ']': stepsPerFrame*=2.0; break; 
		case '|':
			unitSims[windowID]->ClearAllUnits();
			unitSims[windowID]->GetStats()->ClearAllStats();
			break;
		case 'q':
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(.02);
				unitSims[windowID]->SetPaused(true);
			}
			else
				unitSims[windowID]->StepTime(1.0/30.0);
		}
			break;
		case 'd':
		{
			printf("===Stats for this run===\n");
			printf("First travel:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				statValue v;
				int choice = unitSims[windowID]->GetStats()->FindNextStat("trialDistanceMoved", unitSims[windowID]->GetUnit(x)->GetName(), 0);
				unitSims[windowID]->GetStats()->LookupStat(choice, v);
				
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(), v.fval);
			}
			printf("First nodes:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				statValue v;
				int choice = unitSims[windowID]->GetStats()->FindNextStat("nodesExpanded", unitSims[windowID]->GetUnit(x)->GetName(), 0);
				unitSims[windowID]->GetStats()->LookupStat(choice, v);
				
				printf("%s\t%ld\n", unitSims[windowID]->GetUnit(x)->GetName(), v.lval);
			}
			printf("First time:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				statValue v;
				int choice = unitSims[windowID]->GetStats()->FindNextStat("MakeMoveThinkingTime", unitSims[windowID]->GetUnit(x)->GetName(), 0);
				unitSims[windowID]->GetStats()->LookupStat(choice, v);
				
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(), v.fval);
			}
			printf("First Nodes/travel:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				statValue v1;
				int choice = unitSims[windowID]->GetStats()->FindNextStat("nodesExpanded", unitSims[windowID]->GetUnit(x)->GetName(), 0);
				unitSims[windowID]->GetStats()->LookupStat(choice, v1);
				statValue v2;
				choice = unitSims[windowID]->GetStats()->FindNextStat("trialDistanceMoved", unitSims[windowID]->GetUnit(x)->GetName(), 0);
				unitSims[windowID]->GetStats()->LookupStat(choice, v2);
				
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(), v1.lval/v2.fval);			
			}

			printf("Total travel:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(),
					   SumStatEntries(unitSims[windowID]->GetStats(),
									  "trialDistanceMoved",
									  unitSims[windowID]->GetUnit(x)->GetName()));
			}
			printf("Total nodes:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(),
					   SumStatEntries(unitSims[windowID]->GetStats(),
									  "nodesExpanded",
									  unitSims[windowID]->GetUnit(x)->GetName()));
			}
			printf("Total time:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(),
					   SumStatEntries(unitSims[windowID]->GetStats(),
									  "MakeMoveThinkingTime",
									  unitSims[windowID]->GetUnit(x)->GetName()));
			}
			printf("Nodes/travel:\n");
			for (unsigned int x = 0; x < unitSims[windowID]->GetNumUnits(); x++)
			{
				printf("%s\t%f\n", unitSims[windowID]->GetUnit(x)->GetName(),
					   SumStatEntries(unitSims[windowID]->GetStats(),
									  "nodesExpanded",
									  unitSims[windowID]->GetUnit(x)->GetName())/
					   SumStatEntries(unitSims[windowID]->GetStats(),
									  "trialDistanceMoved",
									  unitSims[windowID]->GetUnit(x)->GetName())
					   );
			}
			
		}
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
//	if (mod == kShiftDown)
//	{
//		RunWorkMeasureTest();
//		return;
//	}
	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
	unitSims[windowID]->ClearAllUnits();
	unitSims[windowID]->GetStats()->ClearAllStats();
	m->SetTileSet(kFast);
//	recording = true;
//	startRecording();

	int x1, y1, x2, y2;
	if (mod == kShiftDown)
	{
		x1 = 0;
		y1 = 0;//m->GetMapWidth()-3; // 0;
//		x2 = m->GetMapWidth()-3;
		x2 = m->GetMapWidth()-1;
		y2 = m->GetMapHeight()-1;//-3
	}
	else {
		do {
			x2 = random()%m->GetMapWidth();
			y2 = random()%m->GetMapHeight();
			x1 = random()%m->GetMapWidth();
			y1 = random()%m->GetMapHeight();
		} while ((m->GetTerrainType(x1, y1) != kGround) || (m->GetTerrainType(x2, y2) != kGround));

		do {
			x2 = random()%m->GetMapWidth();
			y2 = random()%m->GetMapHeight();
			x1 = random()%m->GetMapWidth();
			y1 = random()%m->GetMapHeight();
		} while ((m->GetTerrainType(x1, y1) != kGround) || (m->GetTerrainType(x2, y2) != kGround));
	}

	xyLoc a(x1, y1), b(x2, y2);
	//xyLoc a(0, 0), b(mazeSize-1, mazeSize-1);
	//	xyLoc a(0, 0), b(mazeSize-1, 0);
	double a1, b1, c1, r1;
	m->GetCoord((x1+x2)/2, (y1+y2)/2, a1, b1, c1, r1);
	
	for (int x = 0; x < 4; x++)
	{ //break;
	  //	cameraMoveTo(a1, b1, c1-600*r1, 1.0, x);
	  //	cameraLookAt(a1, b1, c1, 1.0, x);
	}
	stepsPerFrame = 1.0/30.0;//1.0/120.0;
	int lookAheadSize = 10;

//	measure.MeasureDifficultly(unitSims[windowID]->GetEnvironment(), a, b);
//	measure.ShowHistogram();
	
//	LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment>(a, b);
//	u1->SetWeight(1.0);
//	u1->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u1);
//
	LearningUnit<xyLoc, tDirection, MapEnvironment> *u2 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(lookAheadSize));
	u2->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u2);

	
	
//	for (int x = 0; x < 2; x++)
//	{
//		LSSLRTAStar<xyLoc, tDirection, MapEnvironment> *alg;
//		LearningUnit<xyLoc, tDirection, MapEnvironment> *u3 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, alg = new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(lookAheadSize));
//		if (x) alg->SetAvoidLearning(true);
//		u3->SetSpeed(0.02);
//		unitSims[windowID]->AddUnit(u3);
//	}

//	LearningUnit<xyLoc, tDirection, MapEnvironment> *u4 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(100));
//	u4->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u4);

//	LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(1, 100));
//	u5->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u5);

	FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
	LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(lookAheadSize, 20.5));
	f->SetUseLocalGCost(false);
	u6->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u6);

//	u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(lookAheadSize, 1.5));
//	//f->SetOrderRedundant(true);
//	f->SetUseLocalGCost(true);
//	u6->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u6);
//
//	u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(lookAheadSize, 1.0));
//	f->SetOrderRedundant(true);
//	u6->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u6);

//	u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(10, 100.5));
//	f->SetOrderRedundant(true);
//	u6->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u6);
	
	unitSims[windowID]->GetStats()->AddFilter("trialDistanceMoved");
	unitSims[windowID]->GetStats()->AddFilter("TotalLearning");
	unitSims[windowID]->GetStats()->AddFilter("nodesExpanded");
	unitSims[windowID]->GetStats()->AddFilter("MakeMoveThinkingTime");
	unitSims[windowID]->GetStats()->EnablePrintOutput(true);
	unitSims[windowID]->SetTrialLimit(50000);
	
	//SetNumPorts(windowID, 1+(unitSims[windowID]->GetNumUnits()-1)%MAXPORTS);

}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<graphState> thePath;
	MapEnvironment *env = unitSims[windowID]->GetEnvironment();
	Map *m = env->GetMap();
	Graph *g = GraphSearchConstants::GetGraph(m);

//	GraphDistanceHeuristic diffHeuristic(g);
//	diffHeuristic.SetPlacement(kAvoidPlacement);
//	for (int x = 0; x < 20; x++)
//		diffHeuristic.AddHeuristic();

	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	for (int x = 0; x < 20; x++)
		diffHeuristic.AddHeuristic();
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);

	diffHeuristic.SetMode(kRandom);
	diffHeuristic.SetNumUsedHeuristics(2);

	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;
	TemplateAStar<graphState, graphMove, GraphEnvironment> taOld;
	Timer t;
	
	for (int x = 0; x < 500; x++)
	{
		graphState s1, g1;
		do {
			s1 = g->GetRandomNode()->GetNum();
			g1 = g->GetRandomNode()->GetNum();
		} while (gEnv.HCost(s1, g1) < 100);
		double firstLength;

		taNew.SetUseBPMX(false);
		t.StartTimer();
		taNew.GetPath(&gEnv, s1, g1, thePath);
		t.EndTimer();
		printf("old: %lld nodes expanded. Path length %d / %f. Time: %f\nrate0: %lld %f\n", 
			   taNew.GetNodesExpanded(), (int)thePath.size(), gEnv.GetPathLength(thePath), t.GetElapsedTime(),
			   taNew.GetNodesExpanded(), taNew.GetNodesExpanded()/t.GetElapsedTime());
		firstLength = gEnv.GetPathLength(thePath);

		for (int y = 1; y < 5; y++)
		{
			taNew.SetUseBPMX(y);
			t.StartTimer();
			taNew.GetPath(&gEnv, s1, g1, thePath);
			t.EndTimer();
			printf("new: %lld nodes expanded. Path length %d / %f. Time: %f\nrate%d: %lld %f\n", 
				   taNew.GetNodesExpanded(), (int)thePath.size(), gEnv.GetPathLength(thePath), t.GetElapsedTime(),
				   y, taNew.GetNodesExpanded(), taNew.GetNodesExpanded()/t.GetElapsedTime());

			if (gEnv.GetPathLength(thePath) != firstLength)
				printf("\n\n\n!!!!!!!!!!!!!!!!!! IT FAILED!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
		}
		printf("--\n");
	}	
	delete g;
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	mouseTracking = false;
//	if (mType != kNoModifier)
//		return false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				
				recording = true;
				
				xyLoc a(px1, py1);
				xyLoc b(px2, py2);
				
//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new CanonicalRTAStar());

//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
				LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1));
				u6->SetSpeed(0.02);
//
//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
//				u7->SetSpeed(0.02);

//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new GridLRTA::GridLRTAStar(10));
//				u7->SetSpeed(0.02);
//				LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new MPLRTA::MPLRTAStar());
//				u7->SetSpeed(0.02);
				unitSims[windowID]->ClearAllUnits();
				unitSims[windowID]->GetStats()->ClearAllStats();
				unitSims[windowID]->GetStats()->AddFilter("trialDistanceMoved");
				unitSims[windowID]->GetStats()->AddFilter("nodesExpanded");
				unitSims[windowID]->GetStats()->AddFilter("TotalLearning");
				unitSims[windowID]->GetStats()->AddFilter("Trial End");

				unitSims[windowID]->AddUnit(u6);
//				unitSims[windowID]->AddUnit(u7);
				SetNumPorts(windowID, 1);
			}
			break;
		}
		return true;
	}
	return false;
}


typedef EpisodicSimulation<xyLoc, tDirection, MapEnvironment> EpSim;
typedef EpisodicSimulation<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> EpSimTank;
void RunSingleTest(EpSim *es, const Experiment &e, int which);
void RunSingleTankTest(EpSimTank *es, const Experiment &e, int which);

void RunSingleWeightedTest(EpSim *es, const Experiment &e, int lookahead, double weight)
{
	es->ClearAllUnits();
	// add units
	es->GetStats()->ClearAllStats();
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->AddFilter("MinInitial");
	es->GetStats()->AddFilter("MaxLater");
	es->GetStats()->AddFilter("MaxStateLearning");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(e.GetStartX(), e.GetStartY()), b(e.GetGoalX(), e.GetGoalY());
	
	printf("Running LSSLRTA*\n");
	LSSLRTAStar<xyLoc, tDirection, MapEnvironment> *alg;
	LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, alg = new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(lookahead));
	alg->SetInititialHeuristicWeight(weight);
	u1->SetSpeed(1.0);
	es->AddUnit(u1);
	
	
	es->SetTrialLimit(0);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
	printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
	es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v);
	//	printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	
	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MinInitial", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("MinInitial\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MinInitial\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MinInitial", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MaxLater", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MaxLater\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MaxLater\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MaxLater", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MaxStateLearning", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MaxStateLearning", es->GetUnit(0)->GetName()));
	}
	
	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval+1);
}

void RunWeightedHeuristicScenario(char *name, int lookahead, double heuristicWeight)
{
	ScenarioLoader *sl = new ScenarioLoader(name);
	printf("Loading map: %s\n", sl->GetNthExperiment(0).GetMapName());
	
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->Scale(sl->GetNthExperiment(0).GetXScale(),
			   sl->GetNthExperiment(0).GetYScale());
	MapEnvironment *me;
	EpSim *es = new EpSim(me = new MapEnvironment(map, false));
	me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		if (sl->GetNthExperiment(x).GetBucket() >= 100 &&
			sl->GetNthExperiment(x).GetBucket() <  101)
		{
			printf("Experiment %d of %d\n", x+1, sl->GetNumExperiments());
			RunSingleWeightedTest(es, sl->GetNthExperiment(x), lookahead, heuristicWeight);
		}
	}
	exit(0);

}

void RunScenario(char *name, int which)
{
	ScenarioLoader *sl = new ScenarioLoader(name);
	printf("Loading map: %s\n", sl->GetNthExperiment(0).GetMapName());
	
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->Scale(sl->GetNthExperiment(0).GetXScale(),
			   sl->GetNthExperiment(0).GetYScale());
	MapEnvironment *me;
	EpSim *es = new EpSim(me = new MapEnvironment(map, false));
	me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);

	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		if (sl->GetNthExperiment(x).GetBucket() >= 100 &&
			sl->GetNthExperiment(x).GetBucket() <  101)
//		if (sl->GetNthExperiment(x).GetBucket() >= 100 &&
//			sl->GetNthExperiment(x).GetBucket() <  111)
		{
			printf("Experiment %d of %d\n", x+1, sl->GetNumExperiments());
			RunSingleTest(es, sl->GetNthExperiment(x), which);
		}
	}
	exit(0);
}

void RunSingleTest(EpSim *es, const Experiment &e, int which)
{
	es->ClearAllUnits();
	// add units
	es->GetStats()->ClearAllStats();
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->AddFilter("MinInitial");
	es->GetStats()->AddFilter("MaxLater");
	es->GetStats()->AddFilter("MaxStateLearning");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(e.GetStartX(), e.GetStartY()), b(e.GetGoalX(), e.GetGoalY());

//	HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;
//	double requiredLearning = measure.MeasureDifficultly(es->GetEnvironment(), a, b);
	double weight = 20;

	if (which == -2)
	{
		double maxError = 0;
		std::vector<xyLoc> thePath;
		// Run A* and measure maximum heuristic error
		a1.GetPath(es->GetEnvironment(), a, b, thePath);
		double pathCost = 0;
		for (int x = 0; x < thePath.size()-2; x++)
		{
			//pathCost += es->GetEnvironment()->GCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1-x]);
			double diff = es->GetEnvironment()->HCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1]) -
			es->GetEnvironment()->HCost(thePath[0], thePath[thePath.size()-1]);
			//pathCost - es->GetEnvironment()->HCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1]);
			if (diff > maxError)
				maxError = diff;
		}
		printf("h_Delta: %1.2f\n", maxError);
		return;
	}
	else if (which == -1)
	{
		double maxError = 0;
		std::vector<xyLoc> thePath;
		// Run A* and measure maximum heuristic error
		a1.GetPath(es->GetEnvironment(), a, b, thePath);
		double pathCost = 0;
		for (int x = 0; x < thePath.size()-2; x++)
		{
			pathCost += es->GetEnvironment()->GCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1-x]);
			double diff = pathCost - es->GetEnvironment()->HCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1]);
			if (diff > maxError)
				maxError = diff;
		}
		printf("Maximum error on optimal path: %1.2f\n", maxError);
		return;
	}
	else if (which == 0)
	{
		printf("Running RIBS\n");
		LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment>(a, b);
		u1->SetSpeed(1.0);
		es->AddUnit(u1); // go to goal and stop
	}
	else if (which == 1)
	{
		printf("Running LSSLRTA*\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 2)
	{
		printf("Running LSS-LRTA*(10)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 3)
	{
		printf("Running LSS-LRTA*(100)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 4)
	{
		printf("Running FLRTA*(1)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(1, 1.5));
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 5)
	{
		printf("Running FLRTA*(10)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(10, 1.5));
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 6)
	{
		printf("Running FLRTA*(100)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(100, 1.5));
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 8)
	{
		printf("Running MPLRTAStar*(1)\n");
		MPLRTA::MPLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new MPLRTA::MPLRTAStar());
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 9)
	{
		printf("Running MPLRTAStar*(1)\n");
		MPLRTA::MPLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new MPLRTA::MPLRTAStar(false));
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 10)
	{
		printf("Running GridLSSLRTAStar*(10)\n");
		GridLRTA::GridLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new GridLRTA::GridLRTAStar(10));
		//		LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new GridLRTA::GridLRTAStar(10));
		
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 11)
	{
		printf("Running daLRTA*(10)\n");
		DALRTA::daLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 =
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b,
															f = new DALRTA::daLRTAStar<xyLoc, tDirection, MapEnvironment>(10));		
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 12)
	{
		printf("Running LSSLRTA*\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LRTAStar<xyLoc, tDirection, MapEnvironment>());
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 13)
	{
		printf("Running gLSS-LRTA*(1)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 14)
	{
		printf("Running gLSS-LRTA*(10)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 15)
	{
		printf("Running gLSS-LRTA*(100)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}

	
	es->SetTrialLimit(0);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
	printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
	es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v);
//	printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	
	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MinInitial", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("MinInitial\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MinInitial\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MinInitial", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MaxLater", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MaxLater\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MaxLater\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MaxLater", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MaxStateLearning", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MaxStateLearning", es->GetUnit(0)->GetName()));
	}

	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval+1);
}

void RunTankScenario(char *name, int which)
{
	ScenarioLoader *sl = new ScenarioLoader(name);
	printf("Loading map: %s\n", sl->GetNthExperiment(0).GetMapName());
	
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->Scale(sl->GetNthExperiment(0).GetXScale(),
			   sl->GetNthExperiment(0).GetYScale());
	Directional2DEnvironment *me;
	EpSimTank *es = new EpSimTank(me = new Directional2DEnvironment(map, kBetterTank, kOctileHeuristic));
//	me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		printf("Experiment %d of %d\n", x+1, sl->GetNumExperiments());
		if (sl->GetNthExperiment(x).GetBucket() == 25)
			RunSingleTankTest(es, sl->GetNthExperiment(x), which);
	}
	exit(0);
}

void RunSingleTankTest(EpSimTank *es, const Experiment &e, int which)
{
	es->ClearAllUnits();
	// add units
	es->GetStats()->ClearAllStats();
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesTouched");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->EnablePrintOutput(false);
	xySpeedHeading a(e.GetStartX(), e.GetStartY()), b(e.GetGoalX(), e.GetGoalY());
	printf("Solving from (%d, %d) to (%d, %d)\n", e.GetStartX(), e.GetStartY(), e.GetGoalX(), e.GetGoalY());
	HeuristicLearningMeasure<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> measured;
	double requiredLearning = measured.MeasureDifficultly(es->GetEnvironment(), a, b);
	
	double weight = 1.5;

	if (which == 0)
	{
		printf("Running RIBS\n");
		LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b);
		u1->SetSpeed(1.0);
		es->AddUnit(u1); // go to goal and stop
	}
	else if (which == 1)
	{
		printf("Running LSSLRTA*\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(1));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 2)
	{
		printf("Running LSS-LRTA*(10)\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 3)
	{
		printf("Running LSS-LRTA*(100)\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(100));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 4)
	{
		printf("Running FLRTA*(1)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(1, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 5)
	{
		printf("Running FLRTA*(10)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 6)
	{
		printf("Running FLRTA*(100)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(100, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	
	es->SetTrialLimit(500000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
	printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
	es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v);
	printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	
	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("nodesTouched", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesTouched\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesTouched\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesTouched", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval+1);
}

void RunBigTest(EpSim *es, const Experiment &e, int which);

void RunBigScenario(char *name, int which)
{
	ScenarioLoader *sl = new ScenarioLoader(name);
	printf("Loading map: %s\n", sl->GetNthExperiment(0).GetMapName());
	
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->Scale(sl->GetNthExperiment(0).GetXScale()*2,
			   sl->GetNthExperiment(0).GetYScale()*2);
	MapEnvironment *me;
	EpSim *es = new EpSim(me = new MapEnvironment(map, false));
	me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		printf("Experiment %d of %d\n", x+1, sl->GetNumExperiments());
		RunBigTest(es, sl->GetNthExperiment(x), which);
	}
	exit(0);
}

void RunBigTest(EpSim *es, const Experiment &e, int which)
{
	if (e.GetBucket() != 127)
		return;
	
	es->ClearAllUnits();
	// add units
	es->GetStats()->ClearAllStats();
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesTouched");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(2*e.GetStartX(), 2*e.GetStartY()), b(2*e.GetGoalX(), 2*e.GetGoalY());
	
	//HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;
	double requiredLearning = 1;//measure.MeasureDifficultly(es->GetEnvironment(), a, b);
	
	if (which == 0)
	{
		printf("Running RIBS\n");
		LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment>(a, b);
		u1->SetSpeed(1.0);
		es->AddUnit(u1); // go to goal and stop
	}
	else if (which == 1)
	{
		printf("Running lSSLRTA*\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 2)
	{
		printf("Running LSS-LRTA*(10)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 3)
	{
		printf("Running LSS-LRTA*(100)\n");
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	
	es->SetTrialLimit(500000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
	printf("sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
	es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v);
	printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	
	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("nodesTouched", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesTouched\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval);
		printf("sum-nodesTouched\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "nodesTouched", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), e.GetBucket(), v.lval+1);
}

void RunScalingTest(int size, int which, float weight)
{
	Map *map = new Map(size, size);
	map->SetTerrainType(1, size-2,
						size-2, size-2, kOutOfBounds);
	map->SetTerrainType(size-2, 1,
						size-2, size-2, kOutOfBounds);

	MapEnvironment *me;
	EpSim *es = new EpSim(me = new MapEnvironment(map, false));
	me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);

	es->ClearAllUnits();
	// add units
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesTouched");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->AddFilter("MaxStateLearning");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(0, 0), b(size-1, size-1);
	xyLoc c(size-3, size-3);
	
	HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;
	double requiredLearning = measure.MeasureDifficultly(es->GetEnvironment(), a, b);
	LearningAlgorithm<xyLoc, tDirection, MapEnvironment> *agent2 = 0;
	Heuristic<xyLoc> *agent = 0;

	if (which == -2)
	{
		double maxError = 0;
		std::vector<xyLoc> thePath;
		// Run A* and measure maximum heuristic error
		
		a1.GetPath(es->GetEnvironment(), c, b, thePath);
		double pathCost = 0;
		for (int x = 0; x < thePath.size()-2; x++)
		{
			//pathCost += es->GetEnvironment()->GCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1-x]);
			double diff = es->GetEnvironment()->HCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1]) -
			es->GetEnvironment()->HCost(thePath[0], thePath[thePath.size()-1]);
			//pathCost - es->GetEnvironment()->HCost(thePath[thePath.size()-2-x], thePath[thePath.size()-1]);
			if (diff > maxError)
				maxError = diff;
		}
		printf("h_Delta: %1.2f\n", maxError);
		exit(0);
	}
	else if (which == 0)
	{
		printf("Running RIBS\n");
		LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::RIBS<xyLoc, tDirection, MapEnvironment>(a, b);
		u1->SetSpeed(1.0);
		es->AddUnit(u1); // go to goal and stop
	}
	else if (which == 1)
	{
		printf("Running LSSLRTA*\n");
		auto tmp = new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b,  tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 2)
	{
		printf("Running LSS-LRTA*(10)\n");
		auto tmp = new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 3)
	{
		printf("Running LSS-LRTA*(100)\n");
		auto tmp = new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 4)
	{
		printf("Running FLRTA*(1)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(1, weight));
		//f->SetOrderRedundant(true);
		f->SetUseLocalGCost(false);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 5)
	{
		printf("Running FLRTA*(10)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(10, weight));
		//f->SetOrderRedundant(true);
		f->SetUseLocalGCost(false);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 6)
	{
		printf("Running FLRTA*(100)\n");
		FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, f = new FLRTA::FLRTAStar<xyLoc, tDirection, MapEnvironment>(100, weight));
		//f->SetOrderRedundant(true);
		f->SetUseLocalGCost(false);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 7)
	{
		printf("Running FLRTA2*(1)\n");
		FLRTA2::FLRTAStar2<xyLoc, tDirection, MapEnvironment> *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new FLRTA2::FLRTAStar2<xyLoc, tDirection, MapEnvironment>(1));
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 8)
	{
		printf("Running MPLRTAStar*(1)\n");
		MPLRTA::MPLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new MPLRTA::MPLRTAStar());
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 9)
	{
		printf("Running MPLRTAStar*(1)\n");
		MPLRTA::MPLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u5 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new MPLRTA::MPLRTAStar(false));
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 10)
	{
		printf("Running GridLSSLRTAStar*(10)\n");
		GridLRTA::GridLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new GridLRTA::GridLRTAStar(10));
		//		LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new GridLRTA::GridLRTAStar(10));
		
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 11)
	{
		printf("Running GridLSSLRTAStar*(100)\n");
		GridLRTA::GridLRTAStar *f;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u6 = 
		new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, 
															f = new GridLRTA::GridLRTAStar(100));
		//		LearningUnit<xyLoc, tDirection, MapEnvironment> *u7 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, new GridLRTA::GridLRTAStar(10));
		
		u6->SetSpeed(1.0);
		es->AddUnit(u6);
	}
	else if (which == 13)
	{
		printf("Running gLSS-LRTA*(1)\n");
		auto tmp = new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(1);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 14)
	{
		printf("Running gLSS-LRTA*(10)\n");
		auto tmp = new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(10);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 15)
	{
		printf("Running gLSS-LRTA*(100)\n");
		auto tmp = new gLSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100);
		agent = tmp;
		LearningUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LearningUnit<xyLoc, tDirection, MapEnvironment>(a, b, tmp);
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	fflush(stdout);
	es->SetTrialLimit(0);
	//es->SetTrialLimit(500000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	if (agent)
	{
		printf("Final heuristic value: %1.5f on map size %d\n", agent->HCost(c, b), size);
	}
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, v.fval);
	printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));

	if (es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v))
		printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);

	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("nodesTouched", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesTouched\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval);
		printf("sum-nodesTouched\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "nodesTouched", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MaxStateLearning", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, v.fval);
		printf("sum-MaxLearn\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "MaxStateLearning", es->GetUnit(0)->GetName()));
	}
	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval+1);
	exit(0);
}

void RunTankScalingTest(int size, int which, float weight)
{
	Map *map = new Map(size, size);
	map->SetTerrainType(1, size-2,
						size-2, size-2, kOutOfBounds);
	map->SetTerrainType(size-2, 1,
						size-2, size-2, kOutOfBounds);
	
	Directional2DEnvironment *me;
	EpSimTank *es = new EpSimTank(me = new Directional2DEnvironment(map, kBetterTank, kOctileHeuristic));
	//me->SetDiagonalCost(1.5);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	es->ClearAllUnits();
	// add units
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesTouched");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->EnablePrintOutput(false);
	xySpeedHeading a(0, 0), b(size-1, size-1);
	
	HeuristicLearningMeasure<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> measure;
	double requiredLearning = measure.MeasureDifficultly(es->GetEnvironment(), a, b);
	
	if (which == 0)
	{
		printf("Running RIBS\n");
		LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b);
		u1->SetSpeed(1.0);
		es->AddUnit(u1); // go to goal and stop
	}
	else if (which == 1)
	{
		printf("Running LSSLRTA*\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(1));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 2)
	{
		printf("Running LSS-LRTA*(10)\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 3)
	{
		printf("Running LSS-LRTA*(100)\n");
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(100));
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
	}
	else if (which == 4)
	{
		printf("Running FLRTA*(1)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(1, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 5)
	{
		printf("Running FLRTA*(10)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	else if (which == 6)
	{
		printf("Running FLRTA*(100)\n");
		FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
		LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new LearningUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(100, weight));
		//f->SetOrderRedundant(false);
		f->SetUseLocalGCost(true);
		u5->SetSpeed(1.0);
		es->AddUnit(u5);
	}
	fflush(stdout);
	es->SetTrialLimit(500000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	printf("Done\n");
	int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
	es->GetStats()->LookupStat(choice, v);
	printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, v.fval);
	printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
	
	if (es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v))
		printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	
	choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval);
		printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("nodesTouched", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-nodesTouched\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval);
		printf("sum-nodesTouched\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "nodesTouched", es->GetUnit(0)->GetName()));
	}
	choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
	if (choice != -1)
	{
		es->GetStats()->LookupStat(choice, v);
		printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, v.fval);
		printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), size, SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
	}
	
	es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
	printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), size, v.lval+1);
	exit(0);
}

#include "MNPuzzle.h"

void RunWorkMeasureTest()
{
	MNPuzzle<4, 4> *mnp = new MNPuzzle<4, 4>;
	srandom(81);
	for (int x = 0; x < 50; x++)
	{
		HeuristicLearningMeasure<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> measure;
		MNPuzzleState<4, 4> s;
		MNPuzzleState<4, 4> g;
		std::vector<slideDir> acts;
		for (unsigned int y = 0; y < 100; y++)
		{
			mnp->GetActions(s, acts);
			mnp->ApplyAction(s, acts[random()%acts.size()]);
		}
		std::cout << "Start: " << s << std::endl << "Goal: " << g << std::endl;
		std::cout << "Heuristic: " << mnp->HCost(s, g) << std::endl;

		double requiredLearning = measure.MeasureDifficultly(mnp, s, g);
		std::cout << "Required learning: " << requiredLearning << std::endl;
		measure.ShowHistogram();
	}
}


void RunSTPTest(int which)
{
	typedef EpisodicSimulation<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> STPSim;
	
	MNPuzzle<4, 4> *mnp = new MNPuzzle<4, 4>;

	srandom(81);

	STPSim *es = new STPSim(mnp);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	// add units
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("nodesTouched");
	es->GetStats()->AddFilter("nodesExpanded");
	es->GetStats()->AddFilter("stepNodesExpanded");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->AddFilter("MakeMoveThinkingTime");
	es->GetStats()->AddFilter("Trial End");
	es->GetStats()->EnablePrintOutput(false);
	
	//for (int x = 0; x < 500; x++)
	for (int x = 0; x < 100; x++)
	{
		es->ClearAllUnits();
		es->GetStats()->ClearAllStats();

		MNPuzzleState<4, 4> s;
		MNPuzzleState<4, 4> g;
		std::vector<slideDir> acts;
		for (unsigned int y = 0; y < 100; y++)
		//for (unsigned int y = 0; y < 75; y++)
		{
			mnp->GetActions(s, acts);
			mnp->ApplyAction(s, acts[random()%acts.size()]);
		}
		std::cout << "Start: " << s << std::endl << "Goal: " << g << std::endl;
		std::cout << "Heuristic: " << mnp->HCost(s, g) << std::endl;
		
		double requiredLearning;
		{
			HeuristicLearningMeasure<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> measure;
			requiredLearning = measure.MeasureDifficultly(mnp, s, g);
			std::cout << "Required learning: " << requiredLearning << std::endl;
		}
		
		if (which == 0)
		{
			LocalSensing::RIBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u1 = new LocalSensing::RIBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g);
			u1->SetSpeed(1.0);
			es->AddUnit(u1); // go to goal and stop
		}
		else if (which == 1)
		{
			LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u1 = new LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g, new LSSLRTAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(1));
			u1->SetSpeed(1.0);
			es->AddUnit(u1);
		}
		else if (which == 2)
		{
			LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u1 = new LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g, new LSSLRTAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(10));
			u1->SetSpeed(1.0);
			es->AddUnit(u1);
		}
		else if (which == 3)
		{
			LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u1 = new LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g, new LSSLRTAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(100));
			u1->SetSpeed(1.0);
			es->AddUnit(u1);
		}
		else if (which == 4)
		{
			LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u4 = new LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g, new FLRTA::FLRTAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(100, 1.5));
			u4->SetSpeed(1.0);
			es->AddUnit(u4);
		}
		else if (which == 5)
		{
			LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> *u4 = new LearningUnit<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(s, g, new FLRTA::FLRTAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(100, 10.0));
			u4->SetSpeed(1.0);
			es->AddUnit(u4);
		}
		
		es->SetTrialLimit(100000);
		while (!es->Done())
		{
			es->StepTime(10.0);
		}

		statValue v;
		printf("Done\n");
		int choice = es->GetStats()->FindNextStat("trialDistanceMoved", es->GetUnit(0)->GetName(), 0);
		es->GetStats()->LookupStat(choice, v);
		printf("dist-first\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, v.fval);
		printf("dist-sum\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, SumStatEntries(es->GetStats(), "trialDistanceMoved", es->GetUnit(0)->GetName()));
		es->GetStats()->LookupStat("TotalLearning", es->GetUnit(0)->GetName(), v);
		printf("learning\t%s\t%f\t%f\t%f\n", es->GetUnit(0)->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
		
		choice = es->GetStats()->FindNextStat("nodesExpanded", es->GetUnit(0)->GetName(), 0);
		if (choice != -1)
		{
			es->GetStats()->LookupStat(choice, v);
			printf("first-nodesExpanded\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), x, v.lval);
			printf("sum-nodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, SumStatEntries(es->GetStats(), "nodesExpanded", es->GetUnit(0)->GetName()));
		}
		choice = es->GetStats()->FindNextStat("stepNodesExpanded", es->GetUnit(0)->GetName(), 0);
		if (choice != -1)
		{
			es->GetStats()->LookupStat(choice, v);
			printf("stepNodesExpanded\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, averageStatEntries(es->GetStats(), "stepNodesExpanded", es->GetUnit(0)->GetName()));
		}
		choice = es->GetStats()->FindNextStat("nodesTouched", es->GetUnit(0)->GetName(), 0);
		if (choice != -1)
		{
			es->GetStats()->LookupStat(choice, v);
			printf("first-nodesTouched\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), x, v.lval);
			printf("sum-nodesTouched\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, SumStatEntries(es->GetStats(), "nodesTouched", es->GetUnit(0)->GetName()));
		}
		choice = es->GetStats()->FindNextStat("MakeMoveThinkingTime", es->GetUnit(0)->GetName(), 0);
		if (choice != -1)
		{
			es->GetStats()->LookupStat(choice, v);
			printf("first-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, v.fval);
			printf("sum-MakeMoveThinkingTime\t%s\t%d\t%f\n", es->GetUnit(0)->GetName(), x, SumStatEntries(es->GetStats(), "MakeMoveThinkingTime", es->GetUnit(0)->GetName()));
		}
		
		es->GetStats()->LookupStat("Trial End", "Race Simulation", v);
		printf("trials\t%s\t%d\t%ld\n", es->GetUnit(0)->GetName(), x, v.lval+1);
	}
	exit(0);
}
