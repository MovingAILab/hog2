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
#include "LocalSensingUnit.h"
#include "RIBS.h"
#include "LRTAStarUnit.h"
#include "LSSLRTAStarUnit.h"
#include "ScenarioLoader.h"
#include "StatUtil.h"
#include "HeuristicLearningMeasure.h"
#include "FLRTAStarUnit.h"
#include "Directional2DEnvironment.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 20;

std::vector<EpisodicSimulation<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *> unitSims;
TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> a1;
TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> a2;
Directional2DEnvironment *ma1 = 0;
Directional2DEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;
std::vector<xySpeedHeading> path;
double stepsPerFrame = 1.0/30.0;

HeuristicLearningMeasure<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> measure;

void RunScenario(char *name, int which);
void RunScalingTest(int size, int which);
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
		map->SetTerrainType(1, mazeSize-2,
							mazeSize-2, mazeSize-2, kOutOfBounds);
		map->SetTerrainType(mazeSize-2, 1,
							mazeSize-2, mazeSize-2, kOutOfBounds);

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
		
//		map = new Map(mazeSize, mazeSize*2);
//		map->SetRectHeight(0, 0, mazeSize-1, 2*mazeSize-1, 0, kOutOfBounds);
//		for (int x = 0; x < mazeSize; x++)
//		{
//			map->SetTerrainType(x, x,
//								x, x+3, kGround);
//			if (1 == x%2)
//				map->SetTerrainType(x, x,
//									x, x+20, kGround);
//			map->SetTerrainType(x, x+20,
//								x, x+21, kGround);
//		}
//		map->SetRectHeight(mazeSize-5*sqrt(mazeSize), 20+mazeSize, mazeSize-1, 20+mazeSize+5*sqrt(mazeSize), 0, kGround);
//		map->SetTerrainType(0, 20+mazeSize,
//							mazeSize-1, 20+mazeSize, kGround);
//		map->SetTerrainType(0, 20+mazeSize,
//							0, 2*mazeSize-1, kGround);
		
		
		//MakeMaze(map, 1);
//		BuildRandomRoomMap(map, 16);
//		map->Scale(mazeSize, mazeSize);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);
	Directional2DEnvironment *me;
	unitSims.resize(id+1);
	unitSims[id] = new EpisodicSimulation<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(me = new Directional2DEnvironment(map, kBetterTank, kOctileHeuristic));
	//	me->SetDiagonalCost(1.5);
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
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
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
	InstallCommandLineHandler(MyCLHandler, "-scenario", "-scenario <file>", "Load and run a scenario offline.");
	InstallCommandLineHandler(MyCLHandler, "-scaleTest", "-scaleTest <size>", "Run a scaling test with local minima <size>.");
	InstallCommandLineHandler(MyCLHandler, "-STPTest", "-STPTest", "Run a STP test.");
	
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

//void startRecording();
//void stopRecording();
static bool recording = false;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(stepsPerFrame);
	}
	
	if (unitSims[windowID]->GetNumUnits() > 0)
	{
		if (!unitSims[windowID]->GetPaused())
			stepsPerFrame*=1.002;//1.01;//
		if (unitSims[windowID]->Done() && recording)
		{
			recording = false;
//			stopRecording();
		}

	}

	if (GetNumPorts(windowID) == unitSims[windowID]->GetNumUnits())
	{
		unitSims[windowID]->GetEnvironment()->OpenGLDraw();
		unitSims[windowID]->OpenGLDraw(viewport);
		//measure.OpenGLDraw(unitSims[windowID]->GetEnvironment());
	}
	else {
		unitSims[windowID]->GetEnvironment()->OpenGLDraw();
		unitSims[windowID]->OpenGLDraw();
	}
	
	if (mouseTracking)
	{
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		GLdouble x, y, z, r;
		m->GetOpenGLCoord(px1, py1, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		m->GetOpenGLCoord(px2, py2, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		glEnd();
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
		//RunScenario(argument[1], atoi(argument[2]));
		//RunBigScenario(argument[1], atoi(argument[2]));
	}
	else if (strcmp(argument[0], "-scaleTest") == 0)
	{
		//RunScalingTest(atoi(argument[1]), atoi(argument[2]));
	}
	else if (strcmp(argument[0], "-STPTest") == 0)
	{
		//RunSTPTest(atoi(argument[1]));
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
			break;
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
	m->SetTileSet(kFast);
//	recording = true;
//	startRecording();

	int x1, y1, x2, y2;
	if (mod == kShiftDown)
	{
		x1 = 0; y1 = 0;
		x2 = m->GetMapWidth()-1;
//		x2 = 0;//m->GetMapWidth()-1;
		y2 = m->GetMapHeight()-1;
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
	//x1 = 6; y1 = 3; x2 = 15; y2 = 72;
	xySpeedHeading a(x1, y1), b(x2, y2);
	//xySpeedHeading a(0, 0), b(mazeSize-1, mazeSize-1);
	//	xySpeedHeading a(0, 0), b(mazeSize-1, 0);
	a1.GetPath(unitSims[windowID]->GetEnvironment(), a, b, path);
	std::cout << "Found path -- " << a1.GetNodesExpanded() << " nodes expanded; length: " << unitSims[windowID]->GetEnvironment()->GetPathLength(path) << std::endl;
	std::cout << "Solving " << a << " to " << b << std::endl;
	stepsPerFrame = 1.0/120.0;
//	GLdouble a1, b1, c1, r1;
	
//	m->GetOpenGLCoord((x1+x2)/2, (y1+y2)/2, a1, b1, c1, r1);
//	cameraMoveTo(a1, b1, c1-600*r1, 1.0);
//	cameraLookAt(a1, b1, c1, 1.0);

//	measure.MeasureDifficultly(unitSims[windowID]->GetEnvironment(), a, b);
//	measure.ShowHistogram();
	
//	LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u1 = new LocalSensing::RIBS<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b);
//	u1->SetWeight(1.0);
//	u1->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u1);

//	LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u2 = new LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(1));
//	u2->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u2);

	LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u2 = new LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10));
	u2->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u2);
	
//	LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u3 = new LSSLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new LSSLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10));
//	u3->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u3);

//	FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u4 = new FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10, 1.5));
//	u4->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u4);
//
//	FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u5 = new FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10, 5.5));
//	u5->SetSpeed(0.02);
//	unitSims[windowID]->AddUnit(u5);
//
	FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *f;
	FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u6 = new FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, f = new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(10, 1.5));
	f->SetOrderRedundant(false);
	f->SetUseLocalGCost(true);
	u6->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u6);
	
	unitSims[windowID]->GetStats()->AddFilter("trialDistanceMoved");
	unitSims[windowID]->GetStats()->AddFilter("TotalLearning");
	unitSims[windowID]->GetStats()->AddFilter("nodesExpanded");
	unitSims[windowID]->GetStats()->EnablePrintOutput(true);
	unitSims[windowID]->SetTrialLimit(50000);

	SetNumPorts(windowID, 1+(unitSims[windowID]->GetNumUnits()-1)%MAXPORTS);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<graphState> thePath;
	Directional2DEnvironment *env = unitSims[windowID]->GetEnvironment();
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

				xySpeedHeading a(px1, py1);
				xySpeedHeading b(px2, py2);
				FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> *u4 = new FLRTAStarUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(a, b, new FLRTA::FLRTAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>(100));
				u4->SetSpeed(0.02);
				unitSims[windowID]->AddUnit(u4);
			}
			break;
		}
		return true;
	}
	return false;
}
