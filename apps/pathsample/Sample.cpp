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
//#include "MapSectorAbstraction.h"
//#include "ContractionHierarchy.h"
#include "MapGenerators.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
bool bSaveAndQuit = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 512;
int gStepsPerFrame = 4;

std::vector<UnitMapSimulation *> unitSims;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;
GraphDistanceHeuristic *gdh2 = 0;

//MapSectorAbstraction *msa;

std::vector<xyLoc> path;

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
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(mazeSize/2, mazeSize/2);
		MakeMaze(map, 1);
		map->Scale(mazeSize, mazeSize);
	}
	else {
		map = new Map(gDefaultMap);
//		map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);
//	msa = new MapSectorAbstraction(map, 2);
//	msa->ToggleDrawAbstraction(0);
//	msa->ToggleDrawAbstraction(1);
//	msa->ToggleDrawAbstraction(3);
//	msa->ToggleDrawAbstraction(4);
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	
	//ch = new ContractionHierarchy(map, GraphSearchConstants::GetGraph(map));
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
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-convert", "-map file1 file2", "Converts a map and saves as file2, then exits");
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

		delete ma1;
		ma1 = 0;
		delete ma2;
		ma2 = 0;
		delete gdh;
		gdh = 0;
		delete gdh2;
		gdh2 = 0;
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

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
		
		//		ch->OpenGLDraw();
//		ch->Contract();
//		return;
	}
	//	unitSims[windowID]->GetEnvironment()->GetMap()->OpenGLDraw();
	//if (msa) msa->OpenGLDraw();
	
	if (mouseTracking)
	{
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		double x, y, z, r;
		m->GetCoord(px1, py1, x, y, z, r);
		m->GetCoord(px2, py2, x, y, z, r);
	}

	if ((gdh) && (gdh2))
	{
	  /*
		if (viewport == 0)
			gdh->OpenGLDraw();
		if (viewport == 1)
			gdh2->OpenGLDraw();
	  */
	}

	if ((ma1) && (viewport == 0)) // only do this once...
	{
		ma1->SetColor(0.0, 0.5, 0.0, 0.75);
		if (runningSearch1)
		{
			ma1->SetColor(0.0, 0.0, 1.0, 0.75);
			for (int x = 0; x < gStepsPerFrame; x++)
			{
				if (a1.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded());
					runningSearch1 = false;
					break;
				}
			}
		}
		//		a1.OpenGLDraw();
	}
	if ((ma2) && (GetNumPorts(windowID) == 1 || (viewport == 1)))
	{
		ma2->SetColor(1.0, 0.0, 0.0, 0.5);
		if (runningSearch2)
		{
			ma2->SetColor(1.0, 0.0, 1.0, 0.5);
			for (int x = 0; x < gStepsPerFrame; x++)
			{
				if (a2.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma2->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
					break;
				}
			}
		}
		//		a2.OpenGLDraw();
	}

	if (bSaveAndQuit)
	{
	  //SaveScreenshot(windowID, gDefaultMap);
	  exit(0);
	}
}

void doExport()
{
	exit(0);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-map") == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);

		doExport();
		return 2;
	}
	else if (strcmp(argument[0], "-convert") == 0)
	  {
		  if (maxNumArgs <= 2)
			  return 0;
		  Map m(argument[1]);
		  m.Save(argument[2]);
		  strncpy(gDefaultMap, argument[1], 1024);
		  bSaveAndQuit = true;
		  return 3;
	  }
	else if (strcmp(argument[0], "-size" ) == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		return 2;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0': case '1': case '2': case '3': case '4': case '5':
			case '6': case '7': case '8': case '9':
				//msa->ToggleDrawAbstraction(key); 
				break;
		case '[': if (gStepsPerFrame > 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case '\t':
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			//ch->Contract();
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	if (gdh == 0)
	{
		gdh = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(unitSims[windowID]->GetEnvironment()->GetMap()));
		gdh->SetPlacement(kFarPlacement);
	}
	if (gdh2 == 0)
	{
		gdh2 = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(unitSims[windowID]->GetEnvironment()->GetMap()));
		gdh2->SetPlacement(kAvoidPlacement);
	}
	if (gdh != 0)
	{
		gdh->AddHeuristic();
	}
	if (gdh2 != 0)
	{
		gdh2->AddHeuristic();
	}
	
	if ((gdh != 0) && (gdh2 != 0))
	{
		int larger = 0;
		int smaller = 0;
		int same = 0;
		double sum1 = 0, sum2 = 0;
		for (int x = 0; x < 10000; x++)
		{
			Graph *g = gdh->GetGraph();
//			xyLoc a(random()%m->GetMapWidth(), random()%m->GetMapHeight());
//			xyLoc b(random()%m->GetMapWidth(), random()%m->GetMapHeight());
//			while ((m->GetTerrainType(a.x, a.y) != kGround) &&
//				   (m->GetTerrainType(b.x, b.y) != kGround))
//			{
//				a.x = random()%m->GetMapWidth();
//				b.x = random()%m->GetMapWidth();
//				a.y = random()%m->GetMapHeight();
//				b.y = random()%m->GetMapHeight();
//			}

			graphState a, b;
			a = g->GetRandomNode()->GetNum();
			b = g->GetRandomNode()->GetNum();
			double h1 = gdh->HCost(a, b);
			sum1+= h1;
			double h2 = gdh2->HCost(a, b);
			sum2+= h2;
			if (fequal(h1, h2))
				same++;
			if (fgreater(h1, h2))
				larger++;
			if (fless(h1, h2))
				smaller++;
		}
		printf("Far larger: %d, Avoid larger: %d, equal: %d\n", larger, smaller, same);
		printf("Far: %1.3f; Avoid: %1.3f\n", sum1, sum2);
	}
//	if (ma == 0)
//		return;
//	if (gdh == 0)
//	{
//		gdh = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(ma->GetMap()));
//		gdh->SetPlacement(kAvoidPlacement);
//		ma->SetGraphHeuristic(gdh);
//		for (int x = 0; x < 10; x++)
//			gdh->AddHeuristic();
//	}
//	if (mod == kShiftDown)
//		ma->SetGraphHeuristic(0);
//	else if (ma->GetGraphHeuristic() == 0)
//	{
//		ma->SetGraphHeuristic(gdh);
//	}
//	else {
//		gdh->AddHeuristic();
//	}
	
//	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
//	
//	int x1, y1, x2, y2;
//	x2 = random()%m->GetMapWidth();
//	y2 = random()%m->GetMapHeight();
//	x1 = random()%m->GetMapWidth();
//	y1 = random()%m->GetMapHeight();
//	SearchUnit *su1 = new SearchUnit(x1, y1, 0, 0);
//	SearchUnit *su2 = new SearchUnit(x2, y2, su1, new TemplateAStar<xyLoc, tDirection, MapEnvironment>());
//	//unitSim->AddUnit(su1);
//	unitSims[windowID]->AddUnit(su2);
//	
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
				
				if (ma1 == 0)
				{
					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
				}
//					gdh = new GraphMapInconsistentHeuristic(ma1->GetMap(), GraphSearchConstants::GetGraph(ma1->GetMap()));
//					gdh->SetPlacement(kAvoidPlacement);
//					ma1->SetGraphHeuristic(gdh);
//					for (int x = 0; x < 10; x++)
//						gdh->AddHeuristic();
//					((GraphMapInconsistentHeuristic*)gdh)->SetNumUsedHeuristics(10);
//					((GraphMapInconsistentHeuristic*)gdh)->SetMode(kMax);
				if (ma2 == 0)
					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());

				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
//				a2.SetWeight(2.0);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
					   
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
//				a2.SetUseBPMX(false);
				//runningSearch1 = true;
				runningSearch2 = true;
				
			}
			break;
		}
		return true;
	}
	return false;
}
