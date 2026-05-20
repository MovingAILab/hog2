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
//#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "Map3DGrid.h"
#include "MinimalSectorAbstraction.h"
#include "MapGenerators.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
bool gEdit = true;
int px1, py1, pz1;//, px2, py2;
int absType = 0;
int mazeSize = 40;
int gStepsPerFrame = 4;
double searchWeight = 0;
bool screenShot = false;
bool addPoints = true;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;

//MapSectorAbstraction *msa;

std::vector<xyLoc> path;

Map3DGrid *m3d = 0;
#include "BitMap.h"

int main(int argc, char* argv[])
{
//	BitMapPic p("/Users/nathanst/Desktop/Untitled.bmp");
//	for (int x = 0; x < p.GetWidth(); x++)
//	{
//		for (int y = 0; y < p.GetHeight(); y++)
//		{
//			uint8_t r, g, b, a;
//			p.GetPixel(x, y, r, g, b, a);
//			if ((2*g > 3*r) && (2*g > 3*b) && (abs(r-b) < 75))
//				p.SetPixel(x, y, 255, 255, 255, a);
//			else
//				p.SetPixel(x, y, 0, 0, 0, a);
//		}
//	}
//	p.Save("/Users/nathanst/Desktop/New.bmp");
//	exit(0);
	InstallHandlers();
//	int h = 0;
//	for (int x = 0; x < 20; x++)
//	{
//		if (0 == x%2)
//		{
//			for (int y = 0; y < 20; y++)
//			{
//				m3d.AddPoint(x, y, h++);
//			}
//		}
//		else {
//			for (int y = 20-1; y >= 0; y--)
//			{
//				m3d.AddPoint(x, y, h);
//			}
//		}
//	}
//	m3d.Rebuild();

//	for (int x = 19; x >= 0; x--)
//	{
//		for (int y = 0; y < 20; y++)
//		{
//			bool added = false;
//			for (int z = 0; z < 10; z++)
//			{
//				if (abs(x-y) < z)
//				{
//					m3d.AddPoint(19-x, y, 10-z);
//					added = true;
//					break;
//				}
//			}
//			if (!added)
//				m3d.AddPoint(19-x, y, 0);
//		}
//	}

	//	for (int x = 0; x < 10; x++)
//	{
//		for (int y = 0; y < 10; y++)
//		{
//			m3d.AddPoint(x, y, 5);
//		}
//	}
	
	RunHOGGUI(argc, argv);
}


void MakeStaircase()
{
	delete m3d;
	m3d = new Map3DGrid(10, 10, 8);
	int height = 0;
	int dir = 1;
	for (int x = 0; x < 10; x++)
		for (int y = 2; y < 10; y++)
			m3d->AddPoint(x, y, height);
	
	for (int t = 0; t < 5; t++)
	{
		for (int x = 0; x < 10; x++)
		{
			m3d->AddPoint(x, 0, height);
			m3d->AddPoint(x, 1, height++);
		}
		for (int x = 9; x >= 0; x--)
		{
			m3d->AddPoint(x, 2, height);
			m3d->AddPoint(x, 3, height++);
		}
	}

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
		MakeMaze(map, 10);
//		map->Scale(mazeSize, mazeSize);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	
	if (0 && gDefaultMap[0] == 0)
	{
		MakeStaircase();
//		m3d = new Map3DGrid(map, 16);
		//m3d->AddMap(map, 0);
		m3d->SetDrawGrid(true);
		m3d->PrintStats();
		//m3d->AddMap(map, 50);
	}
	else {
		m3d = new Map3DGrid(map, 16);
	}
	map->SetTileSet(kWinter);
//	msa = new MapSectorAbstraction(map, 8);
//	msa->ToggleDrawAbstraction(0);
	//msa->ToggleDrawAbstraction(2);
	//msa->ToggleDrawAbstraction(3);
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	m3d->SetDrawGrid(true);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Add", "Toggle adding/removing agents", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Edit", "Toggle editing", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-memory", "-memory <map> <sectors>", "Measures the memory used by a particular map.");
	InstallCommandLineHandler(MyCLHandler, "-cut", "-cut <map> <sectors>", "put a 100 cell gash across the middle of the map");
	InstallCommandLineHandler(MyCLHandler, "-speed", "-speed <map> <sectors>", "Measures the speed of successor generation on a particular map.");

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

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  //	glColor3f(0, 0, 0);
  //	DrawSquare(0, 0, 0.01, 1);
	//	if (m3d)
	//	m3d->OpenGLDraw();
}

void buildProblemSet()
{
	ScenarioLoader s;
	printf(gDefaultMap); printf("\n");
	Map map(gDefaultMap);
	Graph *g = GraphSearchConstants::GetGraph(&map);
	GraphDistanceHeuristic gdh(g);
	gdh.SetPlacement(kFarPlacement);
	// make things go fast; we're doing tons of searches, so use a good heuristic
	for (unsigned int x = 0; x < 30; x++)
		gdh.AddHeuristic();
	GraphEnvironment *ge = new GraphEnvironment(&map, g, &gdh);
	ge->SetDirected(false);
	
	std::vector<std::vector<Experiment> > experiments;
	for (unsigned int x = 0; x < 100000; x++)
	{
		if (0==x%100)
		{ printf("\r%d", x); fflush(stdout); }
		node *s1 = g->GetRandomNode();
		node *g1 = g->GetRandomNode();
//		uint64_t nodesExpanded = 0;
//		printf("%d\t%d\t",  s1->GetNum(), g1->GetNum());
		graphState gs1, gs2;
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		std::vector<graphState> thePath;
		astar.GetPath(ge, gs1, gs2, thePath);
//		printf("%d\n", (int)ge->GetPathLength(thePath));

		if (experiments.size() <= ge->GetPathLength(thePath)/4)
			experiments.resize(ge->GetPathLength(thePath)/4+1);
		if (experiments[ge->GetPathLength(thePath)/4].size() < 10)
		{
			Experiment e(s1->GetLabelL(GraphSearchConstants::kMapX), s1->GetLabelL(GraphSearchConstants::kMapY),
						 g1->GetLabelL(GraphSearchConstants::kMapX), g1->GetLabelL(GraphSearchConstants::kMapY),
						 map.GetMapWidth(), map.GetMapHeight(), ge->GetPathLength(thePath)/4, ge->GetPathLength(thePath), gDefaultMap);
			experiments[ge->GetPathLength(thePath)/4].push_back(e);
		}
		bool done = true;
		for (unsigned int y = 0; y < experiments.size(); y++)
		{
			if (experiments[y].size() != 10)
			{ done = false; break; }
		}
		if (done) break;
	}
	for (unsigned int x = 1; x < experiments.size(); x++)
	{
		if (experiments[x].size() != 10)
			break;
		for (unsigned int y = 0; y < experiments[x].size(); y++)
			s.AddExperiment(experiments[x][y]);
	}
	printf("\n");
	char name[255];
	sprintf(name, "%s.scen", gDefaultMap);
	s.Save(name);
	exit(0);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		Map map(argument[1]);
//		map.Scale(512, 512);
		map.Save(argument[2]);
//		//buildProblemSet();
//		//doExport();
		exit(0);
		return 2;
	}
	else if (strcmp(argument[0], "-memory" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		Map map(argument[1]);
		map.Trim();
		MinimalSectorAbstraction msabs(&map, atoi(argument[2]));
		Map3DGrid m3g(&map, atoi(argument[2]));
		printf("Previously: abstraction: %d map: %d\n", msabs.GetAbstractionBytesUsed(), map.GetMapWidth()*map.GetMapHeight());
		printf("New: abstraction: %d map: %d\n", m3g.GetAbstractionBytesUsed(), m3g.GetGridBytesUsed());
		exit(0);
		return 3;
	}
	else if (strcmp(argument[0], "-cut" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		Map map(argument[1]);
		Map3DGrid m3g(&map, atoi(argument[2]));
		Timer t;
		t.StartTimer();
		for (int x = 1; x < 100; x++)
		{
			bool doRepair = ((map.GetMapWidth()/2-50+x)%gSectorSize)==(gSectorSize-1);
			m3g.RemovePoint(map.GetMapWidth()/2-50+x, map.GetMapHeight()/2, 0, doRepair);
		}
		t.EndTimer();
		printf("Cut time: %1.6f\n", t.GetElapsedTime());
		t.StartTimer();
		m3g.AddPoint(map.GetMapWidth()/2, map.GetMapHeight()/2, 0);
		for (int x = 1; x < 50; x++)
		{
			if ((map.GetMapWidth()/2-x >= 0) && (map.GetMapWidth()/2+x < map.GetMapWidth()))
			{
				m3g.AddPoint(map.GetMapWidth()/2-x, map.GetMapHeight()/2, 0);
				m3g.AddPoint(map.GetMapWidth()/2+x, map.GetMapHeight()/2, 0);
			}
		}
		t.EndTimer();
		printf("Add time: %1.6f\n", t.GetElapsedTime());
		exit(0);
		return 3;
	}
	else if (strcmp(argument[0], "-speed" ) == 0 )
	{
		Map map(argument[1]);
		MinimalSectorAbstraction msabs(&map, atoi(argument[2]));
		Map3DGrid m3g(&map, atoi(argument[2]));

		state3d s;
		Timer t;
		double oldTime = 0, newTime = 0;
		int cnt = 0;
		std::vector<state3d> n1;
		srandom(34);
		for (int x = 0; x < map.GetMapWidth(); x++)
		{
			for (int y = 0; y < map.GetMapHeight(); y++)
			{
				if (m3g.FindNearState(x, y, 0, s) != -1)
				{
					t.StartTimer();
					m3g.GetSuccessors(s, n1);
					newTime += t.EndTimer();
					cnt++;
//					if (n1.size() == 0)
//						continue;
//					for (int x = 0; x < 0; x++)
//					{
//						s = n1[random()%n1.size()];
//						m3g.GetSuccessors(s, n1);
//						if (n1.size() == 0)
//						{
//							assert(!"No successors to state");
//							break;
//						}
//					}
//					x = map.GetMapWidth();
//					y = map.GetMapHeight();
				}
			}
		}
		printf("Time: %d points New: %1.7f time; ", cnt, newTime);

		MapEnvironment me(&map);
		std::vector<xyLoc> n2;
		srandom(34);
		t.StartTimer();
		for (int x = 0; x < map.GetMapWidth(); x++)
		{
			for (int y = 0; y < map.GetMapHeight(); y++)
			{
				if (map.GetTerrainType(x, y) != kGround)
					continue;
				xyLoc s(x, y);
				t.StartTimer();
				me.GetSuccessors(s, n2);
				oldTime += t.EndTimer();

//				if (n2.size() == 0)
//					continue;
//				for (int x = 0; x < 0; x++)
//				{
//					me.GetSuccessors(n2[random()%n2.size()], n2);
//				}
//				x = map.GetMapWidth();
//				y = map.GetMapHeight();
			}
		}
		printf("Old: %1.7f time\n", oldTime);
		exit(0);
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'a':
			addPoints = !addPoints;
			break;
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 0;
			break;
		case '[': if (gStepsPerFrame > 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case 'e': gEdit = !gEdit; break;
		case '\t':
			m3d->SetDrawGrid(!m3d->GetDrawGrid());
//			if (mod != kShiftDown)
//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//			else
//			{
//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			MakeStaircase();
		}
			break;
		default:
			break;
	}
}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	static int y = 18;
	static int x = 0;
	static bool add = false;

	if (m3d)
	{
		Timer timer;
		timer.StartTimer();
		//for (int t = 0; t < m3d->GetWidth(); t++)
		{
			if (add)
				m3d->AddPoint(x, 18, 0);
			else
				m3d->RemovePoint(x, y, 0);
			x = (x+1)%m3d->GetWidth();
			if (x == 0)
				add=!add;
		}
		timer.EndTimer();
//		y+=2;
		printf("%1.2fµs elapsed\n", 1000000*timer.GetElapsedTime());
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (!gEdit)
		return false;
	mouseTracking = false;
	state3d s;
	if (button == kLeftButton)
	{
		switch (mType)
		{
			case kMouseDown:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				//unitSims[windowID]->GetEnvironment()->GetMap()
				printf("Mouse down at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}

				break;
			case kMouseDrag:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				//mouseTracking = true;
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				printf("Mouse tracking at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}
				break;
			case kMouseUp:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				printf("Mouse tracking at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}

				break;
		}
		return true;
	}
	return false;
}


