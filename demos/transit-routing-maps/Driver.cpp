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
#include "Map2DEnvironment.h"
#include "CanonicalGrid.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include "Transit.h"
#include <string>
#include "MapGenerators.h"

enum mode {
	kAddDH = 0,
	kShowHeuristicDiff = 1,
	kFindPath = 2
};

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
std::vector<xyLoc> hd;
std::vector<xyLoc> farStates;

void DoHighwayDimension(xyLoc s);
void DoHighwayDimension1(xyLoc s);
void DoHighwayDimension2(CanonicalGrid::xyLoc s);
void RedoHDDisplay();
void LoadMap(Map *m);

Transit *transit = 0;

xyLoc start, goal;
int hdSearchType = 0;

mode m = kAddDH;

bool recording = false;
bool running = false;
bool mapChange = true;
bool startTransit = false;

int radius = 5;
int multiplier = 4;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
//		SetNumPorts(windowID, 1);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		Map *map = new Map(100, 100);//("/Users/nathanst/hog2/maps/dao/hrt201d.map");
//		MakeRandomMap(map, 15);
//		MakePseudoMaze(map, 1);
//		MakeMaze(map);
		LoadMap(map);
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);

		transit = new Transit(map, 22, 11, 22, false);
		
//		if ((0))
//		{
//			// draw basic map with grid and start state
//			std::fstream svgFile;
//			svgFile.open("/Users/nathanst/Desktop/transit.svg", std::fstream::out | std::fstream::trunc);
//			svgFile << me->SVGHeader();
//			svgFile << me->SVGDraw();
////			px1 = 19; py1 = 18;
////			me->SetColor(1, 1, 1);
////			svgFile << me->SVGLabelState({20, 20}, "S", 1);
//			svgFile << "</svg>";
//			svgFile.close();
//		}
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (startTransit)
	{
		if (!transit->DoneComputing())
		{
			if (running)
			{
				float p = transit->IncrementalCompute();
				std::string s = std::to_string(100.0*p)+"% done";
				submitTextToBuffer(s.c_str());
				if (p == 1)
				{
					s = " "+std::to_string(100.0*transit->GetPercentTransit())+"% transit";
					appendTextToBuffer(s.c_str());
				}
			}
		}
	}
	
	if (mapChange == true)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		mapChange = false;
	}

	if (startTransit)
		transit->Draw(display);
	//	me->OpenGLDraw();
	
	if (start.x != static_cast<uint16_t>(-1) && start.y != static_cast<uint16_t>(-1))
	{
		me->SetColor(0.0, 1.0, 0.0);
		me->Draw(display, start);
//		transit->Draw(display, start, goal);
	}

	me->SetColor(1.0, 0.0, 0.0);
	for (const xyLoc &l : hd)
		me->Draw(display, l);

	me->SetColor(0.0, 0.0, 1.0);
	for (const xyLoc &l : farStates)
		me->Draw(display, l);
	

}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '{':
		{
			startTransit = true;
			start = goal = xyLoc();
			hd.clear();
			farStates.clear();
			running = true;
			break;
		}
		case ']':
		{
			m = mode((int(m)+1)%3);
			switch (m)
			{
				//case
				case kAddDH: submitTextToBuffer("Add DH"); break;
				case kShowHeuristicDiff: submitTextToBuffer("Show Heuristic Diff"); break;
				case kFindPath: submitTextToBuffer("Find Path!"); break;
			}

		}
			break;
		case '[':
		{
			switch (m)
			{
//				case kMoveNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); break;
//				case kFindPath: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); break;
//				case kAddNodes: m = kFindPath; te.AddLine("Current mode: find path"); break;
//				case kAddEdges: m = kAddNodes; te.AddLine("Current mode: add nodes"); break;
			}
		}
			break;
		case '|':
		{
			delete transit;
			transit = new Transit(me->GetMap(), 22, 11, 11, false);
//			transit = new Transit(me->GetMap(), 30, 15, 15, false);
			startTransit = false;
		}
			break;
		case 'w':
//			if (weight > 0.5)
//				weight = 0.0;
//			else
//				weight = 1.0;
//			astar.SetWeight(weight);
//			astar.InitializeSearch(ge, astar.start, astar.goal, path);
//			ShowSearchInfo();
//			
//			running = true;
			break;
		case 'r':
			recording = !recording;
			break;
		case '0':
			delete transit;
			transit = new Transit(me->GetMap(), 22, 11, 11, false);
			startTransit = false;
			break;
		case '1':
			delete transit;
			transit = new Transit(me->GetMap(), 10, 5, 5, false);
			startTransit = false;
			break;
		case '2':
			delete transit;
			transit = new Transit(me->GetMap(), 22, 11, 22, false);
			startTransit = false;
			break;
		case '3':
			delete transit;
			transit = new Transit(me->GetMap(), 10, 5, 10, false);
			startTransit = false;
			break;
		case '4':

		case '\t':
			printf("Hit tab!\n");
			hdSearchType = (hdSearchType+1)%3;
			RedoHDDisplay();
			
//			if (mod != kShiftDown)
//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//			else
//			{
//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//			}
			break;
		case 'p':
			running = !running;
			break;
		case 'o':
		{
			if (!running)
			{
				float p = transit->IncrementalCompute();
				std::string s = std::to_string(100.0*p)+"% done";
				submitTextToBuffer(s.c_str());
				if (p == 1)
				{
					s = " "+std::to_string(100.0*transit->GetPercentTransit())+"% transit";
					appendTextToBuffer(s.c_str());
				}
			}
		}
			break;
		case '?':
		{
		}
			break;
		case 's':
			break;
		case 'l':
			break;
		default:
			break;
	}
	
}

bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
//	if (mType == kMouseDown)
//	{
//		switch (button)
//		{
//			case kRightButton: printf("Right button\n"); break;
//			case kLeftButton: printf("Left button\n"); break;
//			case kMiddleButton: printf("Middle button\n"); break;
//		}
//	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			startTransit = false;
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				start.x = x;
				start.y = y;
				goal = start;
//				printf("Hit (%d, %d)\n", x, y);
				RedoHDDisplay();
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
				start = goal;
				RedoHDDisplay();
			}
			break;
		}
		case kMouseUp:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
//				printf("UnHit (%d, %d)\n", x, y);
			}

			if (m == kFindPath)
			{
				astar.InitializeSearch(me, start, goal, path);
				//astar.SetHeuristic(&h);
				running = true;
			}
			return true;
		}
	}
	return false;
}

/*
 * Showing HD states based on individual searches to radius r
 */
void DoHighwayDimension(xyLoc s)
{
	hd.clear();
	farStates.clear();
	
	std::vector<xyLoc> v;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search2;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(me, s, s, v);
	while (search.GetNumOpenItems() > 0)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
	}
	Map *m = me->GetMap();
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				xyLoc t(x, y);
				farStates.push_back(t);
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					//std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					//std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				search2.GetPath(me, s, t, v);
				//search.ExtractPathToStart(t, v);
				for (const xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;

					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
						hd.push_back(l);
				}
			}
		}
	}
}

/*
 * Showing HD states based on one dijkstra search to radius r
 * (Single tree is used to trace back all paths.)
 */
void DoHighwayDimension1(xyLoc s)
{
	hd.clear();
	farStates.clear();
	
	std::vector<xyLoc> v;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(me, s, s, v);
	while (search.GetNumOpenItems() > 0)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
	}
	Map *m = me->GetMap();
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				xyLoc t(x, y);
				farStates.push_back(t);
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					//std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					//std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				
				search.ExtractPathToStart(t, v);
				std::reverse(v.begin(), v.end());
				for (const xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;
					
					// only need the first state on the path
					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
					{
						hd.push_back(l);
						break;
					}
				}
			}
		}
	}
}

/*
 * Showing HD states based on canonical A* search to radius r
 * (Single tree is used to trace back all paths, but this won't impact the results, since there
 *  is a single canonical path for every goal state.)
 */
void DoHighwayDimension2(CanonicalGrid::xyLoc s)
{
	// draw basic map with grid and start state
//	std::fstream svgFile;
//	svgFile.open("/Users/nathanst/Desktop/transit.svg", std::fstream::out | std::fstream::trunc);
//	svgFile << me->SVGHeader();
//	svgFile << me->SVGDraw();
//	svgFile << me->SVGDraw(xyLoc(s.x, s.y));
	
	
	
	hd.clear();
	farStates.clear();
	
	Map *m = me->GetMap();
	CanonicalGrid::CanonicalGrid g(m);
	
	std::vector<CanonicalGrid::xyLoc> v;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> search;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<CanonicalGrid::xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(&g, s, s, v);
	while (true)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
		if (search.GetNumOpenItems() == 0)
			break;
	}
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				CanonicalGrid::xyLoc t(x, y);
				farStates.push_back({t.x, t.y});
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					//std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					//std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				me->SetColor(0, 0, 1);
//				svgFile << me->SVGDraw(xyLoc(x, y));
				search.ExtractPathToStart(t, v);
				for (const CanonicalGrid::xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;
					
					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
						hd.push_back({l.x, l.y});
				}
			}
		}
	}
	
	
//	me->SetColor(0.5, 0.5, 0.5);
//	for (int y = 0; y < m->GetMapHeight(); y++)
//	{
//		for (int x = 0; x < m->GetMapWidth(); x++)
//		{
//			if (y < s.y-radius || y > s.y+radius)
//				continue;
//			if ((x < s.x-radius) || (x > s.x+radius))
//				continue;
//
//			if (m->GetTerrainType(x, y) == kGround &&
//				((x == s.x-radius) ||
//				 (x == s.x+radius) ||
//				 (y == s.y-radius) ||
//				 (y == s.y+radius)))
//			{
//				svgFile << me->SVGDraw(xyLoc(x, y));
//			}
//		}
//	}
//	svgFile << "</svg>";
//	svgFile.close();

}

void RedoHDDisplay()
{
	if (start.x == static_cast<uint16_t>(-1) || start.y == static_cast<uint16_t>(-1))
		return;

	if (hdSearchType == 0)
	{
		submitTextToBuffer("Showing Canonical");
		CanonicalGrid::xyLoc l(start.x, start.y);
		DoHighwayDimension2(l);
	}
	else if (hdSearchType == 1)
	{
		submitTextToBuffer("Showing Regular (Individual searches)");
		DoHighwayDimension(start);
	}
	else {
		submitTextToBuffer("Showing Regular (Single dijkstra search)");
		DoHighwayDimension1(start);
	}
}

void LoadMap(Map *m)
{
	m->Scale(65, 81);
	const char map[] = "TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTTTT.TTTTTTT.........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT........TTTTTT..........TTTTTTTTTTTTTTTTTTTTTTTT....TTTTTTTTTTTTT........TTTTTTT.........TTTTTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTT.........TTTTTTT..........TTTTTTTTTTTTTTTTTTTTT..........TTTTTTTT........TTTTTTTT..........TTTTTTTTTTTTTTTTTTTTT..........TTTTTTT.........TTTTTTTT..........TTTTTTTTTTTTTTTTTTTTT..........TTTTTTT.........TTTTTTT...........TTTTTTT...TTTT...TTTT............TTTTT.........TTTTTTTTT.........TTTTTTT....TTT...TTT.............TTTTT.........TTTTTTTTT..........TTTT............................TTTTTT............TTTTT..........................................TTTTTT............TTTT...........................................TTTTTT............TTTT..........................................TTTTTT.............TTTT...........TTTTT....TTT.....TTT....TTTTTTTTTTTTTTT...........TTTT.........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTTT.....TTTTTT........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTTT....TTTTTTT........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTTT....TTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTTT....................TTTTTTT....TTTTTTTTTTT........TTTTTTTTTT.........................TTTTTTT....TTTTTTTTTTT.....TTTTTTTTTTTTT.........................TTTTT......TTTTTTTTTTT.....TTTTTTTTTTTT..........TTTTTT..........TTTTT......TTTTTTTTTTT.....TTTTTTTTTTTT..........TTTTTT..........TTTTT......TTTTTTTTTTT.....TTTTTTTTTTTT.TTTTTTTTT@TTTTT..........TTTTTT....TTTTTTTTTTTT.....TTTTTTTTTTT..TTTTTTTTT@TTTTT..........TTTTTT....TTTTTTTTTTTT.....TTTTTTTTTTT..TTTTTTTTT@TTTTT...........TTTT.....TTTTTTTTTTTT.....TTTTTTTTTTT..TTTTTTTTT@TTTTT.....................................TTTTTTTTTTT..TTTTTTTTT@TTTTT.....................................TTTTTTTTTTT..TTTTTTTTTTTTTTT...........................................TTTTTTTTTTTTTTTTTTTTTT...........TTTT............................TTTTTTTTTTTTTTTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@T............TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@T............TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@T.................TTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@T.................TTTTTTT....TTTTTTT.TTTTTTTTTTT@@@@@@@@@@@@@@@@@T.....................TTT.....TTT.............TT@@@@@@@@@@@@@@@@@TT............................................TT@@@@@@@@@@@@@@@@@TT...........................................TTT@@@@@@@@@@@@@@@@TTT...........................................TTT@@@@@@@@@@@@@@@@TTT...........................................TTT@@@@@@@@@@@@@@@@TTT..........................................TTTT@@@@@@@@@@@@@@@@TTTT..................TTTTTTTTT..............TTTT@@@@@@@@@@@@@@@@TTTT..................TTTTTTTTT..............TTTT@@@@@@@@@@@@@@@@TTTT..TTTT.....TTT..TTTTTTTTTTT...............TTT@@@@@@@@@@@@@@@@TTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTT@@@@@@@@@@@@@@@@TTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTTTTT..TTTTTTTTTTTTTTTTTTT.TTTTTTT.TTTTTTTTT.....TTTTTTTTT.TTTTTTTTTT......TTT@TTTTTTTTTTTT.TTTTTTT.TTTTTTT..........TTTTTT.TTTTTTTTTT......TTT@TTTTTTTTTTTT..TTTTTT.TTTTTTT..........TTTTTT.TTTTTTTTTT.......TT@TTTTTTTT..TT..TTT.TT...TTT........................TTTTT.........TTTTTTT............................................TTTTT.........TTTTTTT............................................TTTTT........TTTTTTTT............................................TTTTT........TTTTTTTT...........TT.T.............................TTTT.........TTTTTTTTT..TT......TTTT....TT.......................TTTT...........TTTTTTT..TTTTT..TTTTT....TTTT.........TTTTTT.TTT..TTTT..........TTTTTTTT..TTTTT...TTTT....TTTT.........TTTTTT.TTTT.TTTT............TTTTTT..TTT.......TT...TTTT..........TTTTTT.TTTT.TTTT.....TTTTTT.TTTTTT.................TTTTT.....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT....................TT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT..............TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT..............TTTTTTTTTTTTTTTTTTTTTTTTT.......TTTTTTTTTTTTTTTTTTT................TTTT...........................TTTTTTTTTTTTTTTTTT...............................................TTTTTTTTTT.....TTT...............................................TTTTTTTTTT.....TTT.................................................TTTTTTT.....TTTT................TTTT.............................TTTTTTT.....TTTT........TTT....TTTTT.............................TTTTTTT.....TTTT........TTT....TTTTT.............................TTTTTTT.....TTTT........TTTTTT..TTTT.............................TTTTTTT.....TTTT........TTTTTT..TTTTTT...........................TTTTTTT.....TTTT........TTTTTT..TTTTTT.............................TTTTT............TTT..TTTTTT.TTTTTTT.............................TTTTT...........TTTT..TTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTT.......TTTTT...........TTTT....TTTTTTTTTTTT...TTTTTTTTTTTTTTTTTTT.....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT";
	int which = 0;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				m->SetTerrainType(x, y, kGround);
			else if (map[which] == 'T')
				m->SetTerrainType(x, y, kTrees);
			else
				m->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
}
