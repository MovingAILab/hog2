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
#include "BoundingBox.h"
#include <string>
#include "SVGUtil.h"

enum mode {
	kShowBB = 0,
	kFindPath = 1,
	kShowBBFromSearch = 2
};

MapEnvironment *me = 0;
CanonicalGrid::CanonicalGrid *grid;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
std::vector<CanonicalGrid::xyLoc> path2;
void DrawJPSGoalArea(Graphics::Display &display, CanonicalGrid::tDirection dir, bool drawBound);
void DrawRegularGoalArea(Graphics::Display &display, tDirection dir, bool drawBound);
void LoadMap(Map *m);
void LoadSmallMap(Map *m);

xyLoc start, goal;
bool showCanonical = false;
int stepsPerFrame = 1;
mode m = kShowBB;

bool recording = false;
bool running = false;
bool didMapChange = true;
bool useBB = false;

CanonicalGrid::tDirection cDir = CanonicalGrid::kN;
tDirection dir = kN;
BoundingBox *bb;

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
	InstallKeyboardHandler(MyDisplayHandler, "BB", "Show BBs", kAnyModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Plain A*", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "A*+BB", "A* with BB enhancement", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Expand states faster", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Expand states slower", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate dir", "Rotate direction", kAnyModifier, 'w');
//	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');

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
		SetNumPorts(windowID, 1);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		Map *map = new Map(1, 1);//("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		LoadSmallMap(map);
//		Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak103d.map");
//		map->SetTileSet(kWinter);
		
		me = new MapEnvironment(map);
		grid = new CanonicalGrid::CanonicalGrid(map);
		bb = new BoundingBox(me, false);
		start.x = 0;
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (!bb->DoneComputing())
	{
		float p = bb->IncrementalCompute();
		std::string s = std::to_string(100.0*p)+"% done";
		submitTextToBuffer(s.c_str());
	}
	
	if (didMapChange == true)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		didMapChange = false;
	}

	if (m == kShowBB)
	{
		if (showCanonical)
			DrawJPSGoalArea(display, cDir, true);
		else
			DrawRegularGoalArea(display, dir, true);
	}

	if (m == kShowBBFromSearch)
	{
		bb->Draw(display, start, dir);
	}
	
	if (m == kFindPath)
	{
		if (start.x != static_cast<uint16_t>(-1) && start.y != static_cast<uint16_t>(-1) && !running)
		{
			me->SetColor(0.0, 1.0, 0.0);
			me->Draw(display, start);
			me->SetColor(1.0, 0, 0.0);
			me->Draw(display, goal);
			me->DrawLine(display, start, goal);
		}

		if (running)
		{
			astar.Draw(display);
			
			for (int x = 0; x < stepsPerFrame; x++)
				if (path.size() == 0)
					astar.DoSingleSearchStep(path);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 4);
				}
			}
			
		}
	}
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		//recording = false;
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/BB%d%d%d%d.svg", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		MakeSVG(display, fname, 2048, 2048);
		//		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
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
			break;
		}
		case ']':
		{
			stepsPerFrame *= 2;
			break;
		}
			break;
		case '[':
		{
			if (stepsPerFrame > 1)
				stepsPerFrame /= 2;
			break;
		}
			break;
		case '|':
		{
		}
			break;
		case 'w':
			if (dir == kN) { dir = kNE; cDir = CanonicalGrid::kNE; }
			else if (dir == kNE) { dir = kE; cDir = CanonicalGrid::kE; }
			else if (dir == kE) { dir = kSE; cDir = CanonicalGrid::kSE; }
			else if (dir == kSE) { dir = kS; cDir = CanonicalGrid::kS; }
			else if (dir == kS) { dir = kSW; cDir = CanonicalGrid::kSW; }
			else if (dir == kSW) { dir = kW; cDir = CanonicalGrid::kW; }
			else if (dir == kW) { dir = kNW; cDir = CanonicalGrid::kNW; }
			else if (dir == kNW) { dir = kN; cDir = CanonicalGrid::kN; }
			break;
		case 'r':
			recording = !recording;
			break;
		case '1': useBB = false; m = kFindPath; submitTextToBuffer("A* not using bounding boxes"); break;
		case '2': useBB = true; m = kFindPath;  submitTextToBuffer("A* using bounding boxes"); break;
		case '0': m = kShowBB; submitTextToBuffer("Showing bounding boxes"); break;
			break;
		case '\t':
			printf("Hit tab!\n");
			showCanonical = !showCanonical;
			if (showCanonical)
			{
				printf("Showing canonical reach\n");
				submitTextToBuffer("Canonical Reach");
			}
			else {
				printf("Showing regular reach\n");
				submitTextToBuffer("Regular Reach");
			}
			break;
		case 'p':
			//running = !running;
			break;
		case 'o':
		{
			if (running)
			{
				astar.DoSingleSearchStep(path);
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
			running = false;
			path.resize(0);
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				start.x = x;
				start.y = y;
				goal = start;
				printf("Hit (%d, %d)\n", x, y);
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				if (m == kFindPath)
				{
					goal.x = x;
					goal.y = y;
				}
				else {
					start.x = x;
					start.y = y;
					goal = start;
				}
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
				printf("UnHit (%d, %d)\n", x, y);

			
				if (m == kFindPath)
				{
					std::cout << start << " " << goal << "\n";
//					astar.GetPath(me, start, goal, path);
//					printf("Path has %d moves; %" PRId64 " nodes\n", path.size(), astar.GetNodesExpanded());
					if (useBB)
						astar.SetConstraint(bb);
					else
						astar.SetConstraint(0);
					astar.InitializeSearch(me, start, goal, path);
					running = true;
				}

			}

			return true;
		}
	}
	return false;
}

void DrawRegularGoalArea(Graphics::Display &display, tDirection dir, bool drawBound)
{
	if (start.x == 0)
		return;
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> regAstar;
	xyLoc cStart;
	cStart.x = start.x;
	cStart.y = start.y;
	
	regAstar.SetWeight(1.0);
	regAstar.SetStopAfterGoal(false);
	regAstar.GetPath(me, cStart, cStart, path);
	me->SetColor(0.0, 0.0, 1.0);
	me->Draw(display, cStart);
	
	std::deque<xyLoc> queue;
	// draw everything
	me->ApplyAction(cStart, dir);
	if (me->GetMap()->GetTerrainType(cStart.x, cStart.y) != kGround)
		return;
	queue.push_back(cStart);
	std::vector<xyLoc> v;
	std::vector<bool> visited(me->GetMap()->GetMapHeight()*me->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		xyLoc next = queue.front();
		queue.pop_front();
		if (visited[next.x+next.y*me->GetMap()->GetMapWidth()] == true)
			continue;
		
		visited[next.x+next.y*me->GetMap()->GetMapWidth()] = true;
		grid->SetColor(1.0, 0.5, 0.0);
		me->SetColor(1.0, 0.5, 0.0);
		AStarOpenClosedDataWithF<xyLoc> data;
		if (regAstar.GetClosedItem(next, data))
		{
			me->GetSuccessors(data.data, v);
			CanonicalGrid::xyLoc t(data.data.x, data.data.y);
			grid->Draw(display, t);
			
			if (data.data.x > maxx)
				maxx = data.data.x;
			if (data.data.x < minx)
				minx = data.data.x;
			if (data.data.y > maxy)
				maxy = data.data.y;
			if (data.data.y < miny)
				miny = data.data.y;
			
			for (auto &s : v)
			{
				double g;
				regAstar.GetClosedListGCost(s, g);
				if (fequal(g, data.g+me->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
	if (drawBound)
	{
		double t, l, r, b, z, rad;
		//printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		me->GetMap()->GetCoord(minx, miny, l, t, z, rad);
		me->GetMap()->GetCoord(maxx, maxy, r, b, z, rad);
		//glColor3f(1.0, 1.0, 1.0);
		display.FrameRect({static_cast<float>(l-rad),
			static_cast<float>(t-rad),
			static_cast<float>(r+rad),
			static_cast<float>(b+rad)}, Colors::white, rad);
	}
}

void DrawJPSGoalArea(Graphics::Display &display, CanonicalGrid::tDirection dir, bool drawBound)
{
	if (start.x == 0)
		return;
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;
	CanonicalGrid::xyLoc cStart;
	cStart.x = start.x;
	cStart.y = start.y;
	
	canAstar.SetWeight(1.0);
	canAstar.SetStopAfterGoal(false);
	canAstar.GetPath(grid, cStart, cStart, path2);
	me->SetColor(0.0, 0.0, 1.0);
	me->Draw(display, start);

	std::deque<CanonicalGrid::xyLoc> queue;
	// draw everything
	grid->ApplyAction(cStart, dir);
	if (grid->GetMap()->GetTerrainType(cStart.x, cStart.y) != kGround)
		return;
	//cStart.y--;
	queue.push_back(cStart);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<xyLoc> l;
	while (!queue.empty())
	{
		CanonicalGrid::xyLoc next = queue.front();
		queue.pop_front();
		
		grid->SetColor(0.0, 0.5, 1.0);
		me->SetColor(0.0, 0.5, 1.0);
		AStarOpenClosedDataWithF<CanonicalGrid::xyLoc> data;
		if (canAstar.GetClosedItem(next, data))
		{
			grid->GetSuccessors(data.data, v);
			//grid->OpenGLDraw(data.data);
			grid->Draw(display, data.data);
			
			if (data.data.x > maxx)
				maxx = data.data.x;
			if (data.data.x < minx)
				minx = data.data.x;
			if (data.data.y > maxy)
				maxy = data.data.y;
			if (data.data.y < miny)
				miny = data.data.y;
			
			for (auto &s : v)
			{
				double g;
				canAstar.GetClosedListGCost(s, g);
				//printf("Cost in closed: %f; cost through parent %f (%f+%f)\n", g, data.g+grid->GCost(data.data, s), data.g, grid->GCost(data.data, s));
				if (fequal(g, data.g+grid->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
	if (drawBound)
	{
		double t, l, r, b, z, rad;
		//printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		me->GetMap()->GetCoord(minx, miny, l, t, z, rad);
		me->GetMap()->GetCoord(maxx, maxy, r, b, z, rad);
		//glColor3f(1.0, 1.0, 1.0);
		display.FrameRect({static_cast<float>(l-rad),
			static_cast<float>(t-rad),
			static_cast<float>(r+rad),
			static_cast<float>(b+rad)}, Colors::white, rad);
	}
}


void LoadMap(Map *m)
{
	m->Scale(162, 141);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.TTT@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@T.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....TTTTTTTT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@TTT.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@T.............TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@T...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...T@@@@@@@@@@@@@@@@T...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...T@@@@@@@@@@@@@TTTT...............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..T@@@@@@@@@@@@T...................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@TT..TTT@@@@@@@@@@T...................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.......TTTTTT@@@@TTTTTTT.......TT@@@@@@@T....................TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..........T@@T................TTTTT@TT....................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........T@@T.....................T......................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..........T@@T............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..........T@@T............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT...........T@@T............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.............T@@T............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..........T@T..............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...........TT..............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..........TT..............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT........................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T........................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T........TT..............TT......TT......................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@TTTTTTT.......TT......TT.....TTTTTTTTTTTTTTTTTTTTTTTTTT.......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.....T@T.....T@T....T@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@T.....T@T.....T@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.......TTTTTTTTTTTTTTTTTT.....TTT.....TTT....TTTTTTTTTTTTTTTTTTTTTTTTTTT......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@@TTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.........................TTTTTTTTTTTTTTTTTTTT@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@T.....TTTT@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T....................................................T@@@@@@@@@@@@@@@@@@T.........T..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T....................................................T@@@@@@@@@@@@@@@@@@T.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T....................................................T@@@@@@@@@@@@@@@@@@T.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T....................................................T@@@@@@@@@@@@@@@@@@T.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....TTTT............T@@T....TTTTTTTTTTTTTTTTTTTT.TT.TTTT....TTTT............T@@@@@@@@@@@@@@@@@@T.........TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@TTTTTTTTTTTT@@@T....T@@@@@@@@@@@@@@@@@@@T@T.T@@T....T@@T............T@@@@@@@@@@@@@@@@@@@TTT......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@@@@T....T@@@@TTT@@@@T@@@@@@@@T..T@@T....T@@T............T@@@@@@@@@@@@@@@@@@@@@T......TTTTT@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@TTTTT@@T....T@@@T...TTTT.T@@@@@@T...T@@T....T@@T............TTTTTT@@@@@@@@TTTTTTTTT...........TTTT@@@@@@@@@TTTT......T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@T.....T@T....T@@T.........T@@@@@@T....TT......TT...................T@@@@@@T........................T@@@@@@@T..........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@T.....T@T....T@T..........TT@@@@@T.................................T@@@@@@T........................T@@@@@@@T..........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@T......T@T....T@T............TT@@@T..................................T@@TTT..........................T@@@TTT...........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@T......T@T....T@T..............T@@T...................................TT..............................TTT...........TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@T.....TT......TT..............T@@T.................................................................................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@TTT..TT......TT..............T@@T...................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@T..TT......TT..............T@@T...................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@T..........................T@@T...................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@T..........................T@T....................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@T..........................T@T....................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@@T.........................T@T....................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@TT.........................T@T....................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@T...........................T@@TT..................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@T...TT......TT...........TTT@@@@T..................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@T...TT......TT...........T@@@@@T...................................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@T...TT......TT............TT@@@@T................................................................................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@TT...T@T....T@T..............T@@@T.................................TT..T............................TTTT..........TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@T.....T@T....T@T..........TTTT@@@@.................................T@@TT@T..........................T@@@@TT...........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@T....T@T....T@T..........T@@@@@@@T................................T@@@@@T..........................T@@@@@T...........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@T....T@T....T@@T.........T@@@@@@@@T..TT......TT...................T@@@@@@T.........................T@@@@@@T..........T@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@TTTT@@T....T@@@T...TTTT.T@@@@@@@@T.T@@T....T@@T............TTTTTT@@@@@@@@TTTT...........TTTTTTTTTT@@@@@@@@TTT.......T@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@@@@T....T@@@@TTT@@@@T@@@@@@@@@T.T@@T....T@@T............T@@@@@@@@@@@@@@@@@TTTTT......T@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@TTTTTTTTTTTT@@@T....T@@@@@@@@@@@@@@@@@@@@T..T@@T....T@@T............T@@@@@@@@@@@@@@@@@@@@@T......TTTTTTT@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....TTTT............T@@T....TTTTTTTTTTTTTTTTTTTTT...TTTT....TTTT............T@@@@@@@@@@@@@@@@@@@TTT.............TTTT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T....................................................T@@@@@@@@@@@@@@@@@@T.....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T....................................................T@@@@@@@@@@@@@@@@@@T.....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T....................................................T@@@@@@@@@@@@@@@@@@T.....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T....................................................T@@@@@@@@@@@@@@@@@@T.TT..................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@@@@T....TTT...TT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@@@@@TTTT@@@TTT@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........TTTTTTTTTT..T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.............................................TT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@T.........................TTTTTTTTTTTTTTTTTTTT@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@@TTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T......................TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T......TTTTTTTTTTTTTTTTT......................TTTTTTTTTTTTTTTTTTTTTTTTTT.......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@TT.....TTT.....TTT....T@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....T@T.....T@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT@@TTTTTTTTT.....T@T.....T@T....TTTTTTTTTTTTTTTTTTTTTTTTTTT......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............TT..............TT......TT......................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............................................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........................................................................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............TT...........................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.............TT...................................................T.......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.............T@T.............................................TTTTT@TT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............T@@T.............................................T@@@@@@T.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............T@@T.............................................TT@@TTTT.....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...........T@@T...............................................TT.........T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...........T@@T..........................................................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...........T@@T.............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.........T@@T.............................................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.TTTTTTT@@@@TTTTTT.......TTTTTTTTTTT....................TTTTTTTTTT.......T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@TTT..TT@@@@@@@@@@T....................T@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..T@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...T@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT............TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.........T..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....TTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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


void LoadSmallMap(Map *m)
{
	m->Scale(49, 49);
	const char map[] = "TTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT.TT....TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT..............TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT................TTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT..................TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.................TTT@@@@@@@@@@@@@@@@@@@@@@@@TTTT...................TT@@@@@@@@@@@@@@@@@@@@@@@@T.............TT.......TT@@@@@@@@@@@@@@@@@@@@@@@@T........TTT..TT.......TT@@@@@@@@@@@@@@@@@@@@@@@@T........TTTTTTT.......TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTT..TTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTT..TTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTT...TTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT..............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT..............TT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT.......TTT....TT@@@@@@@@@@@@@@@@@@@@@@@@TTT...TT........TTT....TT@@@@@@@@@@@@@@@@@@@@@@@@TTT.............TTT....TT@@@@@@@@@@@@@@@@@@@@@@@@TTT.............TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...........TTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@TTTTTT..........TTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@TTTTTT..........TTTTTTTTTTTTT...TTTT@@@@@@@@@@@@@TTTTTTTTT................TTTT....TTTT@@@@@@@@@@@@TTTTTTTTTT.......................TTTTTTTT@@@@@@@@TTTTTTTTTT.......................TTTTTTTT@@@@@@@@TTTTTT.TTT.........................TTTTTT@@@@@@@@T......TTT.........TTT.TT..........TTTTTT@@@@@@@@...................TTTTTTT.........TTTTTTT..........................TTTTTT..........TTTTTTT.......T......TT..........................TTTTTTT.......T..TTTTTT..........................TTTTTTT.......TTTTTTTTT..........................TTTTTTTTTT....TTTTTTTTT..........................TTTTTTTTTT....@@@@@@@@T.......TT..TT..TTTTTT....TTTTTTTTTT.....@@@@@@@@T..TTTTTTTTTTTTTTTTTTTTTT..........T.....@@@@@@@@T..TTTTTTTTTTTTTTTTTTTTTT................@@@@@@@@T..TTTTTTTTTTTTTTTTTTTTTT................@@@@@@@@T..TTTTTTTTTTTTTTTTTTTTTT.TTTTTTTTTT.....@@@@@@@@T..TTTTTTTTTTTTTTTTTTTTTT.TTTTTTTTTT.....@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT....@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT....@@@@@@@@.........................................@@@@@@@@.........................................@@@@@@@@.........................................";
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


