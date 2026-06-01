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
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "MapOverlay.h"
#include "JPS.h"
#include "CanonicalDijkstra.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
bool runningSearch3 = false;
bool runningSearch4 = false;
int px1 = 30, py1 = 79, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 20;
int gStepsPerFrame = 1;
double searchWeight = 1;
bool reopenNodes = false;
bool screenShot = false;
bool recording = false;
int paused = 0;

std::string DrawLimitedJumpPoints(MapEnvironment *me);
std::string DrawBasicCanonicalOrdering(bool gray = false);
std::string DrawCanonicalOrdering(bool gray = false);
void WeightedCanAStarExperiments(char *scenario, double weight);
void WeightedAStarExperiments(char *scenario, double weight);
void DijkstraExperiments(char *scenario);
void JPSExperiments(char *scenario, double weight, uint32_t jump);
void OpenGridExperiments(int width);
void ComputeReach();

std::vector<UnitMapSimulation *> unitSims;
std::vector<double> reach;
MapOverlay *mo = 0;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

std::vector<TemplateAStar<graphState, graphMove, GraphEnvironment> > astars;
CanonicalGrid::CanonicalGrid *grid;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> a2;
JPS *jps;
JPS *bjps;
//CanonicalDijkstra *bjps;

MapEnvironment *ma1 = 0;
CanonicalGrid::CanonicalGrid *ma2 = 0;

//std::string mapName = "/Users/nathanst/hog2/maps/dao/den001d.map";
//std::string mapName = "/Users/nathanst/hog2/maps/da2/lt_backalley_g.map";
//std::string mapName = "/Users/nathanst/hog2/maps/dao/orz107d.map";
//std::string mapName = "/Users/nathanst/hog2/maps/dao/den407d.map";
std::string mapName = "/Users/nathanst/hog2/maps/local/den201d-small.map";


std::vector<xyLoc> path;
std::vector<CanonicalGrid::xyLoc> path2;
std::vector<xyLoc> path3;
std::vector<xyLoc> path4;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
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
		//ht_chantry.arl.map // den012d
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz100d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz101d.map");
		
		// good -
		//map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		map = new Map(mapName.c_str());
		
		//map = new Map("/Users/nathanst/hog2/maps/dao/brc502d.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");

		//map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/local/weight.map");
		//map = new Map("weight.map");
//		map = new Map(16, 16);
		//map = new Map(4, 4);
		//map->SetTerrainType(2, 2, kOutOfBounds);
//		map = new Map(128, 128);
//		MakeMaze(map, 2);

//		for (int y = 8; y < 16; y++)
//			map->SetTerrainType(0, y, 8, y, kTrees);
//		map->SetTerrainType(25, 25, kTrees);
//		map->SetTerrainType(75, 75, kTrees);
//		map->SetTerrainType(75, 25, kTrees);
//		map->SetTerrainType(25, 75, kTrees);
//		map = new Map(mazeSize, mazeSize);
		Map *t = new Map(128, 128);
//		MakeMaze(t, 5);
//		t->Save("/Users/nathanst/maze128.map");
//		MakeRandomMap(t, 30);
//		t->Save("/Users/nathanst/random128-30.map");
//		MakeMaze(map, 2);
//		MakePseudoMaze(map, 3);
//		map->Scale(512, 512);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);

	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
	grid = new CanonicalGrid::CanonicalGrid(map);
	if (ma1 == 0)
	{
		ma1 = new MapEnvironment(unitSims[id]->GetEnvironment()->GetMap());

		
		if ((0))
		{
			{
				// draw basic map with grid and start state
				std::fstream svgFile;
				svgFile.open("/Users/nathanst/Desktop/den201d-0.svg", std::fstream::out | std::fstream::trunc);
				svgFile << ma1->SVGHeader();
				svgFile << ma1->SVGDraw();
				px1 = 19; py1 = 18;
				ma1->SetColor(1, 1, 1);
				svgFile << ma1->SVGLabelState({20, 20}, "S", 1);
				svgFile << "</svg>";
				svgFile.close();
			}
			{
				// draw map with basic canonical ordering
				std::fstream svgFile;
				svgFile.open("/Users/nathanst/Desktop/den201d-1.svg", std::fstream::out | std::fstream::trunc);
				svgFile << ma1->SVGHeader();
				svgFile << ma1->SVGDraw();
				px1 = 19; py1 = 18;
				svgFile << DrawBasicCanonicalOrdering();
				ma1->SetColor(1, 1, 1);
				svgFile << ma1->SVGLabelState({20, 20}, "S", 1);
				svgFile << "</svg>";
				svgFile.close();
			}
			{
				// draw map with canonical ordering and jump points
				std::fstream svgFile;
				svgFile.open("/Users/nathanst/Desktop/den201d-2.svg", std::fstream::out | std::fstream::trunc);
				svgFile << ma1->SVGHeader();
				svgFile << ma1->SVGDraw();
				px1 = 19; py1 = 18;
				svgFile << DrawCanonicalOrdering();
				ma1->SetColor(1, 1, 1);
				svgFile << ma1->SVGLabelState({20, 20}, "S", 1);
				
				svgFile << DrawLimitedJumpPoints(ma1);
				svgFile << "</svg>";
				svgFile.close();
			}
		}
	}
	if (ma2 == 0)
	{
		ma2 = new CanonicalGrid::CanonicalGrid(unitSims[id]->GetEnvironment()->GetMap());
		ma2->SetEightConnected();
	}
	jps = new JPS(map);
	jps->SetJumpLimit(-1);
	bjps = new JPS(map);
	bjps->SetJumpLimit(4);
	//bjps->SetWeight(4.0);
//	bjps = new CanonicalDijkstra();
	//ComputeReach();
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record the screen.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Reopen", "Toggle re-opening policy.", kNoModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Compression", "Rotate Compression being shown in heuristic", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Displayed Heuristic", "Rotate which heuristic is shown", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Rotations", "Reset the current rotation/translation of the map.", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Screenshot", "Save SVG Screenshot", kNoModifier, 's');

	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-wastar", "-wastar <scenario> <weight>", "Run weighted A* experiments on scenario with given weight.");
	InstallCommandLineHandler(MyCLHandler, "-wcanastar", "-wcanastar <scenario> <weight>", "Run weighted A* experiments on scenario with given weight.");
	InstallCommandLineHandler(MyCLHandler, "-dijkstra", "-dijkstra <scenario> <weight>", "Run Dijkstra experiments on scenario.");
	InstallCommandLineHandler(MyCLHandler, "-jps", "-jps <scenario> <weight> <jump distance>", "Run JPS experiments on scenario.");
	InstallCommandLineHandler(MyCLHandler, "-open", "-open <size>", "Run JPS on open grid of given size.");

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
		delete unitSims[windowID];
		unitSims[windowID] = 0;
		runningSearch1 = false;
		runningSearch2 = false;
		runningSearch3 = false;
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
		//SetZoom(windowID, 10);
	}

}

std::string DrawAllJumpPoints(MapEnvironment *me)
{
	me->SetColor(0.0, 0.0, 1.0);
	std::string s;
	Map *map = me->GetMap();
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			// x y is a corner and x-1, y-1 is a subgoal
			if (!map->CanStep(x, y, x-1, y) && !map->CanStep(x, y, x-1, y-1) && !map->CanStep(x, y, x, y-1) &&
				map->CanStep(x-1, y, x-1, y-1) && map->CanStep(x-1, y-1, x, y-1) &&
				map->GetTerrainType(x-1, y-1) == kGround)
			{
				xyLoc next(x-1, y-1);
				s += me->SVGDraw(next);
			}
			
			if (!map->CanStep(x, y, x+1, y) && !map->CanStep(x, y, x+1, y+1) && !map->CanStep(x, y, x, y+1) &&
				map->CanStep(x+1, y, x+1, y+1) && map->CanStep(x+1, y+1, x, y+1) &&
				map->GetTerrainType(x+1, y+1) == kGround)
			{
				xyLoc next(x+1, y+1);
				s += me->SVGDraw(next);
			}
			
			if (!map->CanStep(x, y, x-1, y) && !map->CanStep(x, y, x-1, y+1) && !map->CanStep(x, y, x, y+1) &&
				map->CanStep(x-1, y, x-1, y+1) && map->CanStep(x-1, y+1, x, y+1) &&
				map->GetTerrainType(x-1, y+1) == kGround)
			{
				xyLoc next(x-1, y+1);
				s += me->SVGDraw(next);
			}
			
			if (!map->CanStep(x, y, x+1, y) && !map->CanStep(x, y, x+1, y-1) && !map->CanStep(x, y, x, y-1) &&
				map->CanStep(x+1, y, x+1, y-1) && map->CanStep(x+1, y-1, x, y-1) &&
				map->GetTerrainType(x+1, y-1) == kGround)
				
			{
				xyLoc next(x+1, y-1);
				s += me->SVGDraw(next);
			}
		}
	}
	return s;
}

std::string DrawLimitedJumpPoints(MapEnvironment *me)
{
	std::string str;
	CanonicalGrid::xyLoc gLoc(px1, py1);
	std::deque<CanonicalGrid::xyLoc> queue;
	queue.push_back(gLoc);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<bool> visited(grid->GetMap()->GetMapHeight()*grid->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		grid->GetSuccessors(queue.front(), v);
		for (auto &s : v)
		{
			if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
			{
				ma1->SetColor(0.0, 0.0, 0.0);
				queue.push_back(s);
			}
			else {
				ma1->SetColor(1.0, 0.0, 0.0);
			}
			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
			if (((queue.front().parent|s.parent) != (queue.front().parent)) ||
				(abs(s.x-queue.front().x)+abs(s.y-queue.front().y) <= 1 && (s.parent&(s.parent-1)) != 0))
				str += ma1->SVGDraw({s.x, s.y});
		}
		queue.pop_front();
	}
	return str;
}

std::string DrawInitialJumpPoints(MapEnvironment *me)
{
	std::string str;
	xyLoc from(px1, py1);
	xyLoc to(-1, -1);
	jps->InitializeSearch(me, from, to, path);
	jps->DoSingleSearchStep(path);
	me->SetColor(0.0, 0.0, 0.0);
	for (int x = 0; x < jps->GetNumOpenItems(); x++)
	{
		str += me->SVGDraw(jps->GetOpenItem(x));
	}
	return str;
}


std::string DrawCanonicalOrdering(bool gray)
{
	std::string str;
	CanonicalGrid::xyLoc gLoc(px1, py1);
	std::deque<CanonicalGrid::xyLoc> queue;
	queue.push_back(gLoc);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<bool> visited(grid->GetMap()->GetMapHeight()*grid->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		grid->GetSuccessors(queue.front(), v);
		for (auto &s : v)
		{
			if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
			{
				if (gray)
					ma1->SetColor(0.75, 0.75, 0.75);
				else
					ma1->SetColor(0.0, 0.0, 0.0);
				queue.push_back(s);
			}
			else {
				if (gray)
					ma1->SetColor(1.0, 0.5, 0.5);
				else
					ma1->SetColor(1.0, 0.0, 0.0);
			}
			//grid->GLDrawLine(queue.front(), s);
			str += ma1->SVGDrawLine({queue.front().x, queue.front().y}, {s.x, s.y});
			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
		}
		queue.pop_front();
	}
	return str;
	
}

void AnimateBasicCanonicalOrdering(const std::string file, bool gray, xyLoc l)
{
	std::string str;
	xyLoc gLoc(px1, py1);
	std::deque<xyLoc> up, down, left, right, upleft, upright, downleft, downright;
	up.push_back(gLoc);
	down.push_back(gLoc);
	left.push_back(gLoc);
	right.push_back(gLoc);
	upleft.push_back(gLoc);
	upright.push_back(gLoc);
	downleft.push_back(gLoc);
	downright.push_back(gLoc);
	
	int frame = 0;
	bool stillDrawing = true;
	while (stillDrawing)
	{
		std::string fileName = file;
		fileName += std::to_string((frame/100)%10);
		fileName += std::to_string((frame/10)%10);
		fileName += std::to_string((frame/1)%10);
		fileName += ".svg";
		std::fstream svgFile;
		svgFile.open(fileName.c_str(), std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << str;
		ma1->SetColor(1, 1, 1);
		svgFile << ma1->SVGLabelState(l, "S", 1);
		svgFile << "</svg>";
		svgFile.close();
		frame++;
		
		if (gray)
			ma1->SetColor(0.75, 0.75, 0.75);
		else
			ma1->SetColor(0.0, 0.0, 0.0);

		stillDrawing = false;
		if (upright.size() > 0)
		{
			xyLoc curr = upright.front();
			xyLoc next = curr;
			next.x += 1;
			next.y -= 1;
			upright.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				upright.push_front(next);
				right.push_front(next);
				up.push_front(next);
			}
			if (upright.size() > 0)
				stillDrawing = true;
		}
		if (upleft.size() > 0)
		{
			xyLoc curr = upleft.front();
			xyLoc next = curr;
			next.x -= 1;
			next.y -= 1;
			upleft.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				upleft.push_front(next);
				left.push_front(next);
				up.push_front(next);
			}
			if (upleft.size() > 0)
				stillDrawing = true;
		}
		if (downright.size() > 0)
		{
			xyLoc curr = downright.front();
			xyLoc next = curr;
			next.x += 1;
			next.y += 1;
			downright.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				downright.push_front(next);
				down.push_front(next);
				right.push_front(next);
			}
			if (downright.size() > 0)
				stillDrawing = true;
		}
		if (downleft.size() > 0)
		{
			xyLoc curr = downleft.front();
			xyLoc next = curr;
			next.x -= 1;
			next.y += 1;
			downleft.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				downleft.push_front(next);
				down.push_front(next);
				left.push_front(next);
			}
			if (downleft.size() > 0)
				stillDrawing = true;
		}
		
		if (stillDrawing) // draws all diagonals before cardinals
			continue;
		xyLoc none(-1, -1);
		if (up.size() > 0)
		{
			up.push_back(none);
			while (up.front() != none)
			{
				xyLoc curr = up.front();
				xyLoc next = curr;
				next.y -= 1;
				up.pop_front();
				if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
				{
					str += ma1->SVGDrawLine(curr, next);
					up.push_back(next);
				}
				if (up.size() > 0)
					stillDrawing = true;
			}
			up.pop_front();
		}
		if (down.size() > 0)
		{
			down.push_back(none);
			while (down.front() != none)
			{
				xyLoc curr = down.front();
				xyLoc next = curr;
				next.y += 1;
				down.pop_front();
				if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
				{
					str += ma1->SVGDrawLine(curr, next);
					down.push_back(next);
				}
				if (down.size() > 0)
					stillDrawing = true;
			}
			down.pop_front();
		}
		if (left.size() > 0)
		{
			left.push_back(none);
			while (left.front() != none)
			{
				xyLoc curr = left.front();
				xyLoc next = curr;
				next.x -= 1;
				left.pop_front();
				if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
				{
					str += ma1->SVGDrawLine(curr, next);
					left.push_back(next);
				}
				if (left.size() > 0)
					stillDrawing = true;
			}
			left.pop_front();
		}
		if (right.size() > 0)
		{
			right.push_back(none);
			while (right.front() != none)
			{
				xyLoc curr = right.front();
				xyLoc next = curr;
				next.x += 1;
				right.pop_front();
				if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
				{
					str += ma1->SVGDrawLine(curr, next);
					right.push_back(next);
				}
				if (right.size() > 0)
					stillDrawing = true;
			}
			right.pop_front();
		}
	}
}


std::string DrawBasicCanonicalOrdering(bool gray)
{
	std::string str;
	xyLoc gLoc(px1, py1);
	std::deque<xyLoc> up, down, left, right, upleft, upright, downleft, downright;
	up.push_back(gLoc);
	down.push_back(gLoc);
	left.push_back(gLoc);
	right.push_back(gLoc);
	upleft.push_back(gLoc);
	upright.push_back(gLoc);
	downleft.push_back(gLoc);
	downright.push_back(gLoc);
	
	if (gray)
		ma1->SetColor(0.75, 0.75, 0.75);
	else
		ma1->SetColor(0.0, 0.0, 0.0);
	bool stillDrawing = true;
	while (stillDrawing)
	{
		stillDrawing = false;
		if (up.size() > 0)
		{
			xyLoc curr = up.front();
			xyLoc next = curr;
			next.y -= 1;
			up.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				up.push_front(next);
			}
			stillDrawing = true;
		}
		if (down.size() > 0)
		{
			xyLoc curr = down.front();
			xyLoc next = curr;
			next.y += 1;
			down.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				down.push_front(next);
			}
			stillDrawing = true;
		}
		if (left.size() > 0)
		{
			xyLoc curr = left.front();
			xyLoc next = curr;
			next.x -= 1;
			left.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				left.push_front(next);
			}
			stillDrawing = true;
		}
		if (right.size() > 0)
		{
			xyLoc curr = right.front();
			xyLoc next = curr;
			next.x += 1;
			right.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				right.push_front(next);
			}
			stillDrawing = true;
		}
		if (upright.size() > 0)
		{
			xyLoc curr = upright.front();
			xyLoc next = curr;
			next.x += 1;
			next.y -= 1;
			upright.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				upright.push_front(next);
				right.push_front(next);
				up.push_front(next);
			}
			stillDrawing = true;
		}
		if (upleft.size() > 0)
		{
			xyLoc curr = upleft.front();
			xyLoc next = curr;
			next.x -= 1;
			next.y -= 1;
			upleft.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				upleft.push_front(next);
				left.push_front(next);
				up.push_front(next);
			}
			stillDrawing = true;
		}
		if (downright.size() > 0)
		{
			xyLoc curr = downright.front();
			xyLoc next = curr;
			next.x += 1;
			next.y += 1;
			downright.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				downright.push_front(next);
				down.push_front(next);
				right.push_front(next);
			}
			stillDrawing = true;
		}
		if (downleft.size() > 0)
		{
			xyLoc curr = downleft.front();
			xyLoc next = curr;
			next.x -= 1;
			next.y += 1;
			downleft.pop_front();
			if (ma1->GetMap()->CanStep(curr.x, curr.y, next.x, next.y))
			{
				str += ma1->SVGDrawLine(curr, next);
				downleft.push_front(next);
				down.push_front(next);
				left.push_front(next);
			}
			stillDrawing = true;
		}
	}
	return str;

}

std::string DrawRegularGoalArea(tDirection dir, bool drawBound, bool svg = false)
{
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> regAstar;
	xyLoc cStart;
	cStart.x = px1;
	cStart.y = py1;
	
	regAstar.SetWeight(1.0);
	regAstar.SetStopAfterGoal(false);
	regAstar.GetPath(ma1, cStart, cStart, path);
	ma1->SetColor(0.0, 0.0, 1.0);
	//ma1->OpenGLDraw(cStart);
	
	std::deque<xyLoc> queue;
	// draw everything
	ma1->ApplyAction(cStart, dir);
	queue.push_back(cStart);
	std::vector<xyLoc> v;
	std::vector<bool> visited(ma1->GetMap()->GetMapHeight()*ma1->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		xyLoc next = queue.front();
		queue.pop_front();
		if (visited[next.x+next.y*ma1->GetMap()->GetMapWidth()] == true)
			continue;
		
		visited[next.x+next.y*ma1->GetMap()->GetMapWidth()] = true;
		grid->SetColor(1.0, 0.5, 0.0);
		ma1->SetColor(1.0, 0.5, 0.0);
		AStarOpenClosedDataWithF<xyLoc> data;
		if (regAstar.GetClosedItem(next, data))
		{
			ma1->GetSuccessors(data.data, v);
			CanonicalGrid::xyLoc t(data.data.x, data.data.y);
			//grid->OpenGLDraw(t);
			if (svg)
				str += ma1->SVGDraw(data.data);
			//ma1->OpenGLDraw(data.data);
			
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
				if (fequal(g, data.g+ma1->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
	if (drawBound)
	{
		double t, l, r, b, z, rad, tmp;
		printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		ma1->GetMap()->GetCoord(minx, miny, l, t, z, rad);
		ma1->GetMap()->GetCoord(maxx, maxy, r, b, z, rad);
		//glColor3f(1.0, 1.0, 1.0);
		//glLineWidth(4.0);
		//OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
		if (svg)
		{
			ma1->SetColor(0.0, 0.0, 0.0);
			str += ma1->SVGFrameRect(minx, miny, maxx, maxy, 4);
		}
	}
	
	return str;
}

std::string DrawJPSGoalArea(CanonicalGrid::tDirection dir, bool drawBound, bool svg = false)
{
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;
	CanonicalGrid::xyLoc cStart;
	cStart.x = px1;
	cStart.y = py1;
	
	canAstar.SetWeight(1.0);
	canAstar.SetStopAfterGoal(false);
	canAstar.GetPath(grid, cStart, cStart, path2);
	
	std::deque<CanonicalGrid::xyLoc> queue;
	// draw everything
	grid->ApplyAction(cStart, dir);
	//cStart.y--;
	queue.push_back(cStart);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<xyLoc> l;
	while (!queue.empty())
	{
		CanonicalGrid::xyLoc next = queue.front();
		queue.pop_front();
		
		grid->SetColor(0.0, 0.5, 1.0);
		ma1->SetColor(0.0, 0.5, 1.0);
		AStarOpenClosedDataWithF<CanonicalGrid::xyLoc> data;
		if (canAstar.GetClosedItem(next, data))
		{
			grid->GetSuccessors(data.data, v);
			//grid->OpenGLDraw(data.data);
			if (svg)
				str += ma1->SVGDraw({data.data.x, data.data.y});

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
		double t, l, r, b, z, rad, tmp;
		printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		ma1->GetMap()->GetCoord(minx, miny, l, t, z, rad);
		ma1->GetMap()->GetCoord(maxx, maxy, r, b, z, rad);
		//glColor3f(1.0, 1.0, 1.0);
		//glLineWidth(4.0);
		//OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
		if (svg)
		{
			ma1->SetColor(0.0, 0.0, 0.0);
			str += ma1->SVGFrameRect(minx, miny, maxx, maxy, 4);
		}
	}
	return str;
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  //	if (mo)
  //		mo->OpenGLDraw();
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}

	// draw canonical ordering in SVG
	if (screenShot&&0)
	{
		std::fstream svgFile;
		svgFile.open("/Users/nathanst/Desktop/canonical.svg", std::fstream::out | std::fstream::trunc);
		
		svgFile << ma1->SVGHeader();
//		svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*ma1->GetMap()->GetMapWidth()+10)+"\" height = \""+std::to_string(10*ma1->GetMap()->GetMapHeight()+10+100)+"\">";
		
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering();
		svgFile << "</svg>";
		svgFile.close();
	}
	if (screenShot&&0) // canonical ordering with jump points
	{
		std::fstream svgFile;
		svgFile.open("/Users/nathanst/Desktop/canonical+jp.svg", std::fstream::out | std::fstream::trunc);
		
		svgFile << ma1->SVGHeader();
//		svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*ma1->GetMap()->GetMapWidth()+10)+"\" height = \""+std::to_string(10*ma1->GetMap()->GetMapHeight()+10+100)+"\">";
		
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering();
		svgFile << DrawLimitedJumpPoints(ma1);
		//		svgFile << DrawJumpPoints(ma1);
		svgFile << "</svg>";
		svgFile.close();
	}
	if (px1 != -1) // canonical open gl
	{
	  //glLineWidth(1);
		CanonicalGrid::xyLoc gLoc(px1, py1);
		std::deque<CanonicalGrid::xyLoc> queue;
		queue.push_back(gLoc);
		std::vector<CanonicalGrid::xyLoc> v;
		std::vector<bool> visited(grid->GetMap()->GetMapHeight()*grid->GetMap()->GetMapWidth());
		while (!queue.empty())
		{
			grid->GetSuccessors(queue.front(), v);
			for (auto &s : v)
			{
				if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
				{
					grid->SetColor(0.5, 0.5, 0.5);
					queue.push_back(s);
				}
				else {
					grid->SetColor(1.0, 0.5, 0.5);
				}
				//grid->GLDrawLine(queue.front(), s);
				visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
			}
			queue.pop_front();
		}
	}

	if (0)
	{
		static int x = 1;
		x++;
		if (0 == x%2)
		{
			if (px1 > 0)
				DrawJPSGoalArea(CanonicalGrid::kS, true);
		}
		else {
			if (px1 > 0)
				DrawRegularGoalArea(kS, true);
		}
	}
	
	//unitSims[windowID]->OpenGLDraw();
	
	if (screenShot&&0)
	{
		std::fstream svgFile;
		svgFile.open("/Users/nathanst/Desktop/goal-reg.svg", std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawRegularGoalArea(kS, true, true);
		svgFile << "</svg>";
		svgFile.close();
	}

	if (screenShot&&0)
	{
		std::fstream svgFile;
		svgFile.open("/Users/nathanst/Desktop/goal-can.svg", std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawJPSGoalArea(CanonicalGrid::kS, true, true);
		svgFile << "</svg>";
		svgFile.close();
	}

	if (screenShot)
	{
		screenShot = false;
		static int frame = -1;
		frame++;
		std::fstream svgFile;
		char name[1024];
		sprintf(name, "/Users/nathanst/Desktop/jps%d%d.svg", frame/10, frame%10);
		svgFile.open(name, std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering(true);
		svgFile << jps->SVGDraw();
		svgFile << "</svg>";
		svgFile.close();
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
	
	
	if (GetNumPorts(windowID) > 1)
	{
		if ((ma1) && (viewport == 0)) // only do this once...
		{
			ma1->SetColor(0.0, 0.5, 0.0, 0.75);
			if (runningSearch1)// && !unitSims[windowID]->GetPaused())
			{
				ma1->SetColor(0.0, 0.0, 1.0, 0.75);
				for (int x = 0; x < gStepsPerFrame*paused; x++)
				{
					if (a1.DoSingleSearchStep(path))
					{
						printf("A* Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
							   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded(),
							   a1.GetNumOpenItems());
						runningSearch1 = false;
						break;
					}
				}
			}
			//if (path.size() == 0)
			//a1.OpenGLDraw();
			if (path.size() > 0)
			{
			  //glLineWidth(10);
				ma1->SetColor(0.5, 1.0, 1.0);
				//for (int x = 0; x < path.size()-1; x++)
				//	ma1->GLDrawLine(path[x], path[x+1]);
				//	glLineWidth(1);
			}
		}

		if ((ma2) && viewport == 1)
		{
			ma2->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch2)
			{
				ma2->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame*paused; x++)
				{
					if (a2.DoSingleSearchStep(path2))
					{
						printf("CA* Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
							   (int)path2.size(), ma2->GetPathLength(path2), a2.GetNodesExpanded(),
							   a2.GetNumOpenItems());
						runningSearch2 = false;
						break;
					}
				}
			}
			//if (path2.size() == 0)
			//a2.OpenGLDraw();
			if (path2.size() > 0)
			{
			  //glLineWidth(10);
				ma1->SetColor(1.0, 0.5, 1.0);
				//	for (int x = 0; x < path2.size()-1; x++)
				//	ma2->GLDrawLine(path2[x], path2[x+1]);
				//	glLineWidth(1);
			}
		}
		if ((ma1) && viewport == 2)
		{
			ma1->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch3)
			{
				ma1->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame*paused; x++)
				{
					if (jps->DoSingleSearchStep(path3))
					{
						printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
							   (int)path3.size(), ma1->GetPathLength(path3), jps->GetNodesExpanded(),
							   jps->GetNumOpenItems());
						runningSearch3 = false;
						break;
					}
				}
			}
			//			jps->OpenGLDraw();
			if (path3.size() > 0)
			{
			  //glLineWidth(10);
				ma1->SetColor(1.0, 0.0, 1.0);
				//for (int x = 0; x < path3.size()-1; x++)
				  //		ma1->GLDrawLine(path3[x], path3[x+1]);
				//	glLineWidth(1);
			}

		}
		if ((ma1) && viewport == 3)
		{
			ma1->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch4)
			{
				ma1->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame*paused; x++)
				{
					if (bjps->DoSingleSearchStep(path4))
					{
						printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
							   (int)path4.size(), ma1->GetPathLength(path4), bjps->GetNodesExpanded(),
							   bjps->GetNumOpenItems());
						runningSearch4 = false;
						break;
					}
				}
			}
			//			bjps->OpenGLDraw();
			if (path4.size() > 0)
			{
			  //glLineWidth(10);
			  //	ma1->SetColor(1.0, 0.0, 1.0);
				//				for (int x = 0; x < path4.size()-1; x++)
				//		ma1->GLDrawLine(path4[x], path4[x+1]);
				//glLineWidth(1);
			}
			
		}
	}
	else if (GetNumPorts(windowID) == 1)
	{
		if (ma1)
		{
			ma1->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch3)
			{
				ma1->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame*paused; x++)
				{
					if (jps->DoSingleSearchStep(path3))
					{
						runningSearch3 = false;
						break;
					}
				}
			}
			//jps->OpenGLDraw();
		}
		if (path3.size() > 0)
		{
		  //			glLineWidth(10);
		  //	ma1->SetColor(1.0, 0.0, 1.0);
		  //	for (int x = 0; x < path3.size()-1; x++)
		  //		ma1->GLDrawLine(path3[x], path3[x+1]);
		  //	glLineWidth(1);
		}
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		//SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
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
	if (strcmp( argument[0], "-dijkstra" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		DijkstraExperiments(argument[1]);
		return 3;
	}
	if (strcmp( argument[0], "-jps" ) == 0 )
	{
		if (maxNumArgs <= 2)
			return 0;
		JPSExperiments(argument[1], atof(argument[2]), atoi(argument[3]));
		return 3;
	}
	if (strcmp( argument[0], "-open" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		OpenGridExperiments(atoi(argument[1]));
		return 2;
	}
	if (strcmp( argument[0], "-wastar" ) == 0 )
	{
		if (maxNumArgs <= 2)
			return 0;
		WeightedAStarExperiments(argument[1], atof(argument[2]));
		return 3;
	}
	if (strcmp( argument[0], "-wcanastar" ) == 0 )
	{
		if (maxNumArgs <= 2)
			return 0;
		WeightedCanAStarExperiments(argument[1], atof(argument[2]));
		return 3;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
	case '|': break;//resetCamera(); break;
		case 'r': recording = !recording; break;
		case '0': jps->SetJumpLimit(-1); break;
		case '1': jps->SetJumpLimit(1); break;
		case '2': jps->SetJumpLimit(2); break;
		case '3': jps->SetJumpLimit(4); break;
		case '4': jps->SetJumpLimit(8); break;
		case '5': jps->SetJumpLimit(16); break;
		case '6': jps->SetJumpLimit(32); break;
		case '7': jps->SetJumpLimit(64); break;
		case '8': jps->SetJumpLimit(128); break;
		case '9':
			break;
		case 's': screenShot = true; break;
		case 't':
			reopenNodes = !reopenNodes;
			if (reopenNodes) printf("Will re-open nodes\n");
			if (!reopenNodes) printf("Will not re-open nodes\n");
			break;
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 0;
			printf("Search weight is %1.2f\n", searchWeight);
			break;
		case '[': if (gStepsPerFrame >= 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case '{':
		case '}':
		case '\t':
			break;
		case 'p': //unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused());
			paused = 1-paused;
			printf("Paused is : %d\n", paused);
			break;
		case 'o':
		{
			if (runningSearch1)
			{
				if (a1.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded());
					runningSearch1 = false;
				}
			}
			if (runningSearch2)
			{
				if (a2.DoSingleSearchStep(path2))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
				}
			}
			if (runningSearch3)
			{
				if (jps->DoSingleSearchStep(path3))
				{
					printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
						   (int)path3.size(), ma1->GetPathLength(path3), jps->GetNodesExpanded(),
						   jps->GetNumOpenItems());
					runningSearch3 = false;
					break;
				}
			}
			if (runningSearch4)
			{
				if (bjps->DoSingleSearchStep(path4))
				{
					printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
						   (int)path4.size(), ma1->GetPathLength(path4), bjps->GetNodesExpanded(),
						   bjps->GetNumOpenItems());
					runningSearch4 = false;
					break;
				}
			}

		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long w, tKeyboardModifier , char)
{
	printf("Testing Dijkstra vs CanonicalDijkstra\n");
	Map *m = ma1->GetMap();
	CanonicalDijkstra gjps;
	double t1, t2;
	Timer t;
	xyLoc s(px1, py1);
	t.StartTimer();
	gjps.GetPath(ma1, s, s, path);
	t.EndTimer();
	printf("gJPS: %f\n", t1 = t.GetElapsedTime());
	a1.SetStopAfterGoal(false);
	t.StartTimer();
	a1.GetPath(ma1, s, s, path);
	t.EndTimer();
	printf("Dijkstra: %f\n", t2 = t.GetElapsedTime());
	printf("Ratio: %f\n", t2/t1);
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				double g1;
				xyLoc l(x, y);
				a1.GetClosedListGCost(l, g1);
				if (!fequal(gjps.GetClosedGCost(l), g1))
				{
					printf("(%d, %d) Error: dijkstra: %f, gjps: %f\n", l.x, l.y, g1, gjps.GetClosedGCost(l));
					exit(0);
				}
			}
		}
	}
}

xyLoc GetRandomState()
{
	xyLoc l(random()%ma1->GetMap()->GetMapWidth(),
			random()%ma1->GetMap()->GetMapHeight());
	return l;
}

//void TestJPS()
//{
//	std::vector<xyLoc> states;
//	srandom(1234);
//	const int totalStates = 10000;
//	// 1. Generate 10000 states
//	for (int x = 0; x < totalStates; x++)
//	{
//		states.push_back(GetRandomState());
//	}
//	Timer t;
//	// 2. Check validity of each
//	int numLegal = 0;
//	t.StartTimer();
//	for (int x = 0; x < totalStates; x++)
//		numLegal += (ma1->GetMap()->GetTerrainType(states[x].x, states[x].y) == kGround)?1:0;
//	//r->LegalState(states[x])?1:0;
//	t.EndTimer();
//	printf("%f elapsed; %d of %d legal\n", t.GetElapsedTime(), numLegal, totalStates);
//	
//	// 3. Add each to queue
//	AStarOpenClosed<xyLoc, AStarCompare<xyLoc> > openClosedList;
//	int numAdded = 0;
//	t.StartTimer();
//	for (int x = 0; x < totalStates; x++)
//	{
//		uint64_t objid;
//		if (openClosedList.Lookup(ma1->GetStateHash(states[x]), objid) != kOpenList)
//		{
//			numAdded++;
//			openClosedList.AddOpenNode(states[x], ma1->GetStateHash(states[x]), 0, 0, 0);
//		}
//	}
//	t.EndTimer();
//	printf("%f elapsed added %d of %d to open\n", t.GetElapsedTime(), numAdded, totalStates);
//
//	// 3. Add each to queue
//	IndexOpenClosed<xyLoc> openClosedList2;
//	openClosedList2.Reset(ma1->GetMap()->GetMapHeight()*ma1->GetMap()->GetMapWidth());
//	numAdded = 0;
//	t.StartTimer();
//	for (int x = 0; x < totalStates; x++)
//	{
//		uint64_t objid;
//		if (openClosedList2.Lookup(ma1->GetStateHash(states[x]), objid) != kOpenList)
//		{
//			numAdded++;
//			openClosedList2.AddOpenNode(states[x], ma1->GetStateHash(states[x]), 0, 0, 0);
//		}
//	}
//	t.EndTimer();
//	printf("%f elapsed added %d of %d to open (grid)\n", t.GetElapsedTime(), numAdded, totalStates);
//
//	//	t.StartTimer();
//	//	astar.GetPath(r, s, g, ourPath);
//	//	t.EndTimer();
//	//	printf("Astar %f %llu %llu %1.2f\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), r->GetPathLength(ourPath));
//	
//}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	xyLoc s1;
	xyLoc g1;
	s1.x = px1; s1.y = py1;
	g1.x = px2; g1.y = py2;

	CanonicalDijkstra d;
	d.InitializeSearch(ma1, s1, g1, path);
	ma1->SetDiagonalCost(1.5);
	int step = 0;
	bool done = false;
	while (true)
	{
		std::fstream svgFile;
		std::string file = "/Users/nathanst/Movies/tmp/cdijkstra";
		file += std::to_string((step/100)%10);
		file += std::to_string((step/10)%10);
		file += std::to_string(step%10);
		file += ".svg";
		svgFile.open(file.c_str(), std::fstream::out | std::fstream::trunc);
		
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
//		svgFile << DrawCanonicalOrdering(false);
//		svgFile << d.SVGDraw();
		svgFile << d.SVGDrawDetailed();
		
		svgFile << "</svg>";
		svgFile.close();
		if (done)
			break;
		done = d.DoSingleSearchStep(path);
		step++;
	}
	
}

void LogScreenShots(xyLoc l)
{
	std::size_t found = mapName.find_last_of("/\\");
	std::string name = mapName.substr(found+1);
	std::string prefix = "/Users/nathanst/Desktop/";
	std::string m1, m2, m3, m4;
	m1 = prefix+name+"-0.svg";
	m2 = prefix+name+"-1.svg";
	m3 = prefix+name+"-2.svg";
	m4 = prefix+name+"-3.svg";
	{
		// draw basic map with grid and start state
		std::fstream svgFile;
		svgFile.open(m1.c_str(), std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		ma1->SetColor(0, 0, 0);
		svgFile << ma1->SVGDraw(l);
		ma1->SetColor(1, 1, 1);
		svgFile << ma1->SVGLabelState(l, "S", 1);
		svgFile << "</svg>";
		svgFile.close();
	}
	{
		// draw map with basic canonical ordering
		std::fstream svgFile;
		svgFile.open(m2.c_str(), std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawBasicCanonicalOrdering();
		ma1->SetColor(0, 0, 0);
		svgFile << ma1->SVGDraw(l);
		ma1->SetColor(1, 1, 1);
		svgFile << ma1->SVGLabelState(l, "S", 1);
		svgFile << "</svg>";
		svgFile.close();
	}
	{
		// draw map with basic canonical ordering and initial jump points
		std::fstream svgFile;
		svgFile.open(m3.c_str(), std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawBasicCanonicalOrdering();
		ma1->SetColor(0, 0, 0);
		svgFile << ma1->SVGDraw(l);
		ma1->SetColor(1, 1, 1);
		svgFile << ma1->SVGLabelState(l, "S", 1);
		svgFile << DrawInitialJumpPoints(ma1);
		svgFile << "</svg>";
		svgFile.close();
	}
	{
		// draw map with canonical ordering and jump points
		std::fstream svgFile;
		svgFile.open(m4.c_str(), std::fstream::out | std::fstream::trunc);
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering();
		ma1->SetColor(0, 0, 0);
		svgFile << ma1->SVGDraw(l);
		ma1->SetColor(1, 1, 1);
		svgFile << ma1->SVGLabelState(l, "S", 1);
		
		svgFile << DrawLimitedJumpPoints(ma1);
		svgFile << "</svg>";
		svgFile.close();
	}
	AnimateBasicCanonicalOrdering(prefix+std::string("anim/")+name, false, l);
}

void AnimateAlgorithms(const std::string &prefix)
{
//	a1.InitializeSearch(ma1, s1, g1, path);
//	a2.InitializeSearch(ma2, s2, g2, path2);
//	jps->InitializeSearch(ma1, s1, g1, path3);

	int frame = -1;
	while (true)
	{
		frame++;
		{
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/jps/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << jps->SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		if (jps->DoSingleSearchStep(path3))
		{
			frame++;
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/jps/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << jps->SVGDraw();
			ma1->SetColor(0, 0, 1);
			for (int x = 0; x < path3.size()-1; x++)
				svgFile << ma1->SVGDrawLine(path3[x], path3[x+1], 2);
			svgFile << "</svg>";
			svgFile.close();
			break;
		}
	}

	
	frame = -1;
	while (true)
	{
		frame++;
		{
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/bjps/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << bjps->SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		if (bjps->DoSingleSearchStep(path4))
		{
			frame++;
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/bjps/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << bjps->SVGDraw();
			ma1->SetColor(0, 0, 1);
			for (int x = 0; x < path4.size()-1; x++)
				svgFile << ma1->SVGDrawLine(path4[x], path4[x+1], 2);
			svgFile << "</svg>";
			svgFile.close();
			break;
		}
	}

	frame = -1;
	while (true)
	{
		frame++;
		{
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/astar/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << a1.SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		if (a1.DoSingleSearchStep(path))
		{
			frame++;
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/astar/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << a1.SVGDraw();
			ma1->SetColor(0, 0, 1);
			for (int x = 0; x < path.size()-1; x++)
				svgFile << ma1->SVGDrawLine(path[x], path[x+1], 2);
			svgFile << "</svg>";
			svgFile.close();
			break;
		}
	}

	frame = -1;
	while (true)
	{
		frame++;
		{
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/castar/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << a2.SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		if (a2.DoSingleSearchStep(path2))
		{
			frame++;
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/castar/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << a2.SVGDraw();
			ma2->SetColor(0, 0, 1);
			for (int x = 0; x < path2.size()-1; x++)
				svgFile << ma2->SVGDrawLine(path2[x], path2[x+1], 2);
			svgFile << "</svg>";
			svgFile.close();
			break;
		}
	}

}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	mouseTracking = false;

	if (button == kRightButton || button == kLeftButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				startLoc = loc;
				
				LogScreenShots(xyLoc(px1, py1));
				
//				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
//				printf("Mouse down at (%d, %d)\n", px1, py1);
				//MyRandomUnitKeyHandler(0, kShiftDown, 0);
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

				printf("\n------\nSearching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				
				
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				//a2.SetWeight(1.8);
				xyLoc s1;
				xyLoc g1;
				
				//156	135	158	142
//				px1 = 156; px2 = 158;
//				py1 = 135; py2 = 142;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
				CanonicalGrid::xyLoc s2(px1, py1), g2(px2, py2);
				
				if (1)
				{
					a1.SetWeight(1.0);
					a2.SetWeight(1.0);

					Timer t1, t2, t3, t4;
					t1.StartTimer();
					a1.GetPath(ma1, s1, g1, path);
					t1.EndTimer();
					printf("Regular search: %1.4fs, path length %1.6f, %llu nodes %llu touched\n", t1.GetElapsedTime(), ma1->GetPathLength(path), a1.GetNodesExpanded(), a1.GetNodesTouched());
//					for (const auto &s : path)
//						std::cout << s << " ";
//					std::cout << "\n";
					t2.StartTimer();
					a2.GetPath(ma2, s2, g2, path2);
					t2.EndTimer();
					printf("Canonical search: %1.4fs, path length %1.6f, %llu nodes %llu touched\n", t2.GetElapsedTime(), ma2->GetPathLength(path2), a2.GetNodesExpanded(), a2.GetNodesTouched());
					printf("Speedup: %1.5f\n", t1.GetElapsedTime()/t2.GetElapsedTime());
					printf("Nodes ratio: %1.5f\n", a1.GetNodesExpanded()/float(a2.GetNodesExpanded()));
					if (!fequal(ma2->GetPathLength(path2), ma1->GetPathLength(path)))
					{
						printf("ERROR: path lengths do not match\n");
						exit(0);
					}

					a1.SetWeight(10.0);
					a2.SetWeight(10.0);
					t3.StartTimer();
					a1.GetPath(ma1, s1, g1, path);
					t3.EndTimer();
					printf("Weighted Regular search: %1.4fs, path length %1.6f, %llu nodes %llu touched\n", t3.GetElapsedTime(), ma1->GetPathLength(path), a1.GetNodesExpanded(), a2.GetNodesTouched());
					t4.StartTimer();
					a2.GetPath(ma2, s2, g2, path2);
					t4.EndTimer();
					printf("Weighted Canonical search: %1.4fs, path length %1.6f, %llu nodes %llu touched\n", t4.GetElapsedTime(), ma2->GetPathLength(path2), a2.GetNodesExpanded(), a2.GetNodesTouched());
					printf("wSpeedup: %1.5f\n", t3.GetElapsedTime()/t4.GetElapsedTime());
					printf("wNodes ratio: %1.5f\n", a1.GetNodesExpanded()/float(a2.GetNodesExpanded()));
					
					printf("wwSpeedup: %1.5f\n", t1.GetElapsedTime()/t3.GetElapsedTime());
					printf("wcSpeedup: %1.5f\n", t2.GetElapsedTime()/t4.GetElapsedTime());

					t1.StartTimer();
					jps->GetPath(ma1, s1, g1, path);
					t1.EndTimer();
					
					printf("JPS: %1.4fs, %llu nodes expanded %llu touched\n", t1.GetElapsedTime(), jps->GetNodesExpanded(), jps->GetNodesTouched());
					printf("\n\n");
				}

				a1.SetWeight(searchWeight);
				a2.SetWeight(searchWeight);
				ma1->SetEightConnected();

				if (0)
				{
					a1.InitializeSearch(ma1, s1, g1, path);
					a2.InitializeSearch(ma2, s2, g2, path2);
					jps->InitializeSearch(ma1, s1, g1, path3);
					bjps->InitializeSearch(ma1, s1, g1, path3);
					AnimateAlgorithms("/Users/nathanst/Desktop/anim/");
				}
				
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s2, g2, path2);
				jps->InitializeSearch(ma1, s1, g1, path3);
				bjps->InitializeSearch(ma1, s1, g1, path4);
				runningSearch1 = true;
				runningSearch2 = true;
				runningSearch3 = true;
				runningSearch4 = true;
				SetNumPorts(windowID, 4);

//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 0);
//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 1);
			}
				break;
		}
		return true;
	}
	return false;
}

void WeightedAStarExperiments(char *scenario, double weight)
{
	ScenarioLoader s(scenario);
	Map *m = new Map(s.GetNthExperiment(0).GetMapName());
	MapEnvironment *me = new MapEnvironment(m);
	//CanonicalGrid::CanonicalGrid *cge = new CanonicalGrid::CanonicalGrid(m);
	TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc, IndexCompareWithF<xyLoc>, IndexOpenClosedDataWithF<xyLoc>>> astar;
	//TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;

	Timer t;
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		xyLoc start, goal;
		start.x = s.GetNthExperiment(x).GetStartX();
		start.y = s.GetNthExperiment(x).GetStartY();
		goal.x = s.GetNthExperiment(x).GetGoalX();
		goal.y = s.GetNthExperiment(x).GetGoalY();

//		CanonicalGrid::xyLoc cStart, cGoal;
//		cStart.x = s.GetNthExperiment(x).GetStartX();
//		cStart.y = s.GetNthExperiment(x).GetStartY();
//		cGoal.x = s.GetNthExperiment(x).GetGoalX();
//		cGoal.y = s.GetNthExperiment(x).GetGoalY();

		if (s.GetNthExperiment(x).GetBucket() > 0)
		{
			astar.SetWeight(weight);
			t.StartTimer();
			astar.GetPath(me, start, goal, path);
			t.EndTimer();
			printf("%1.4f %f %f %llu %llu %u\n",
				   t.GetElapsedTime(), me->GetPathLength(path), s.GetNthExperiment(x).GetDistance(), astar.GetNodesExpanded(), astar.GetNodesTouched(), astar.GetNumOpenItems());
		}
	}
	exit(0);
}

void WeightedCanAStarExperiments(char *scenario, double weight)
{
	ScenarioLoader s(scenario);
	Map *m = new Map(s.GetNthExperiment(0).GetMapName());
	MapEnvironment *me = new MapEnvironment(m);
	//CanonicalGrid::CanonicalGrid *cge = new CanonicalGrid::CanonicalGrid(m);
	TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc>> astar;
	//TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;
	JPS jps(m);
	
	jps.SetJumpLimit(5);
	Timer t;
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		xyLoc start, goal;
		start.x = s.GetNthExperiment(x).GetStartX();
		start.y = s.GetNthExperiment(x).GetStartY();
		goal.x = s.GetNthExperiment(x).GetGoalX();
		goal.y = s.GetNthExperiment(x).GetGoalY();
		
		//		CanonicalGrid::xyLoc cStart, cGoal;
		//		cStart.x = s.GetNthExperiment(x).GetStartX();
		//		cStart.y = s.GetNthExperiment(x).GetStartY();
		//		cGoal.x = s.GetNthExperiment(x).GetGoalX();
		//		cGoal.y = s.GetNthExperiment(x).GetGoalY();
		
		if (s.GetNthExperiment(x).GetBucket() > 0)
		{
			jps.SetWeight(weight);
			t.StartTimer();
			jps.GetPath(me, start, goal, path);
			t.EndTimer();
			printf("%1.4f %f %f %llu %llu %u\n",
				   t.GetElapsedTime(), me->GetPathLength(path), s.GetNthExperiment(x).GetDistance(), jps.GetNodesExpanded(), jps.GetNodesTouched(), jps.GetNumOpenItems());
			if (path.size() == 0 || me->GetPathLength(path) > s.GetNthExperiment(x).GetDistance()*weight)
			  {
			    fprintf(stderr, "Error: path not properly bounded.\n");
			    printf("Error: path not properly bounded.\n");
			    exit(1);
			  }
		}
	}
	exit(0);
}

//void WeightedCanAStarExperiments(char *scenario, double weight)
//{
//	ScenarioLoader s(scenario);
//	Map *m = new Map(s.GetNthExperiment(0).GetMapName());
////	MapEnvironment *me = new MapEnvironment(m);
//	CanonicalGrid::CanonicalGrid *cge = new CanonicalGrid::CanonicalGrid(m);
////	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
//	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid, IndexOpenClosed<CanonicalGrid::xyLoc>> canAstar;
//	
//	Timer t;
//	for (int x = 0; x < s.GetNumExperiments(); x++)
//	{
//		CanonicalGrid::xyLoc cStart, cGoal;
//		cStart.x = s.GetNthExperiment(x).GetStartX();
//		cStart.y = s.GetNthExperiment(x).GetStartY();
//		cGoal.x = s.GetNthExperiment(x).GetGoalX();
//		cGoal.y = s.GetNthExperiment(x).GetGoalY();
//		
//		if (s.GetNthExperiment(x).GetBucket() > 0)
//		{
//			canAstar.SetWeight(weight);
//			t.StartTimer();
//			canAstar.GetPath(cge, cStart, cGoal, path2);
//			t.EndTimer();
//			printf("%1.4f %f %f %llu %llu %u\n",
//				   t.GetElapsedTime(), cge->GetPathLength(path2), s.GetNthExperiment(x).GetDistance(), canAstar.GetNodesExpanded(), canAstar.GetNodesTouched(), canAstar.GetNumOpenItems());
//
////			printf("%1.4f %f %llu %llu %u\n", t.GetElapsedTime(), cge->GetPathLength(path2), canAstar.GetNodesExpanded(), canAstar.GetNodesTouched(), canAstar.GetNumOpenItems());
//		}
//	}
//	exit(0);
//}


void DijkstraExperiments(char *scenario)
{
	ScenarioLoader s(scenario);
	Map *m = new Map(s.GetNthExperiment(0).GetMapName());
	MapEnvironment *me = new MapEnvironment(m);
	CanonicalGrid::CanonicalGrid *cge = new CanonicalGrid::CanonicalGrid(m);
	TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc>> astar;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid, IndexOpenClosed<CanonicalGrid::xyLoc>> canAstar;
	CanonicalDijkstra gjps;
	astar.SetStopAfterGoal(false);
	canAstar.SetStopAfterGoal(false);
	Timer t;
	for (int x = std::max(s.GetNumExperiments()-10, 0); x < s.GetNumExperiments(); x++)
	{
		xyLoc start, goal;
		goal.x = s.GetNthExperiment(x).GetStartX();
		goal.y = s.GetNthExperiment(x).GetStartY();
		start.x = s.GetNthExperiment(x).GetGoalX();
		start.y = s.GetNthExperiment(x).GetGoalY();
		
		CanonicalGrid::xyLoc cStart, cGoal;
		cGoal.x = s.GetNthExperiment(x).GetStartX();
		cGoal.y = s.GetNthExperiment(x).GetStartY();
		cStart.x = s.GetNthExperiment(x).GetGoalX();
		cStart.y = s.GetNthExperiment(x).GetGoalY();
		
		if (s.GetNthExperiment(x).GetBucket() > 0)
		{
			astar.SetWeight(1.0);
			t.StartTimer();
			astar.GetPath(me, start, goal, path);
			t.EndTimer();
			printf("%1.4f %llu %llu ", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched());
			
			canAstar.SetWeight(1.0);
			t.StartTimer();
			canAstar.GetPath(cge, cStart, cGoal, path2);
			t.EndTimer();
			printf("%1.4f %llu %llu ", t.GetElapsedTime(), canAstar.GetNodesExpanded(), canAstar.GetNodesTouched());

			t.StartTimer();
			gjps.GetPath(me, start, goal, path);
			t.EndTimer();
			printf("%1.4f %llu %llu\n", t.GetElapsedTime(), gjps.GetNodesExpanded(), gjps.GetNodesTouched());
		}
	}
	exit(0);
}

void JPSExperiments(char *scenario, double weight, uint32_t jump)
{
	ScenarioLoader s(scenario);
	Map *m = new Map(s.GetNthExperiment(0).GetMapName());
	MapEnvironment *me = new MapEnvironment(m);
	JPS jps(m);
	jps.SetJumpLimit(jump);
	jps.SetWeight(weight);
	Timer t;
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		xyLoc start, goal;
		start.x = s.GetNthExperiment(x).GetStartX();
		start.y = s.GetNthExperiment(x).GetStartY();
		goal.x = s.GetNthExperiment(x).GetGoalX();
		goal.y = s.GetNthExperiment(x).GetGoalY();
		
		if (s.GetNthExperiment(x).GetBucket() > 0)
		{
			t.StartTimer();
			jps.GetPath(me, start, goal, path);
			t.EndTimer();
			printf("%f %f %f %llu %llu %llu\n",
				   t.GetElapsedTime(), me->GetPathLength(path), s.GetNthExperiment(x).GetDistance(), jps.GetNodesExpanded(), jps.GetNodesTouched(), jps.GetNumOpenItems());
		}
	}
	exit(0);
}

void OpenGridExperiments(int width)
{
	Map *m = new Map(width, width);
	MapEnvironment *me = new MapEnvironment(m);
	JPS jps(m);
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

	Timer t;
	
	xyLoc start, goal;
	goal.x = width-1;
	goal.y = width-1;
	start.x = 0;
	start.y = 0;
	
	t.StartTimer();
	astar.GetPath(me, start, goal, path);
//	jps.GetPath(me, start, goal, path);
	t.EndTimer();
//	printf("%d %1.4f %f %f %llu %llu %llu\n", width,
//		   1000*t.GetElapsedTime(), me->GetPathLength(path), (width-1)*ROOT_TWO, jps.GetNodesExpanded(),
//		   jps.GetNodesTouched(), jps.GetNumOpenItems());
	printf("%d %1.4f %f %f %llu %llu %llu\n", width,
		   1000*t.GetElapsedTime(), me->GetPathLength(path), (width-1)*ROOT_TWO,
		   astar.GetNodesExpanded(),
		   astar.GetNodesTouched(), astar.GetNumOpenItems());
	exit(0);
}

void ComputeReach(xyLoc start, TemplateAStar<xyLoc, tDirection, MapEnvironment> &search)
{
	std::vector<xyLoc> neighbors;
	Map *m = ma1->GetMap();
	
	for (int i = 0; i < search.GetNumItems(); i++)
	{
		const AStarOpenClosedDataWithF<xyLoc> &d = search.GetItem(i);
		ma1->GetSuccessors(d.data, neighbors);
		bool foundParent = false;
		// check if we have a neighbor that has this state as a parent
		for (int x = 0; x < neighbors.size(); x++)
		{
			AStarOpenClosedDataWithF<xyLoc> n;
			if (search.GetClosedItem(neighbors[x], n))
			{
				if (n.parentID == i)
				{
					foundParent = true;
					break;
				}
			}
		}
		if (foundParent)
			continue;
		
		// trace from i back to the start state updating the reach of each state
		double perfectHCost = 0;
		AStarOpenClosedDataWithF<xyLoc> v = search.GetItem(i);
		int currParent = i;
		while (true)
		{
			int64_t nextParent = v.parentID;
			if (nextParent == kTAStarNoNode || nextParent == currParent)
				break;
			xyLoc next = search.GetItem(nextParent).data;
			perfectHCost += ma1->GCost(next, v.data);
			double r = std::min(perfectHCost, search.GetItem(nextParent).g);
			reach[ma1->GetStateHash(next)] = std::max(r, reach[ma1->GetStateHash(next)]);
			mo->SetOverlayValue(next.x, next.y, reach[ma1->GetStateHash(next)]);
			v = search.GetItem(nextParent);
			currParent = nextParent;
		}
	}
}

// Not using canonical ordering
void ComputeReach()
{
	if (ma1 == 0)
		return;
	Map *m = ma1->GetMap();
	if (mo == 0)
	{
		mo = new MapOverlay(m);
		mo->SetColorMap(2);
		mo->SetTransparentValue(0.1);
	}
	reach.clear();
	reach.resize(m->GetMapWidth()*m->GetMapHeight());

	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	std::vector<xyLoc> result;
	search.SetStopAfterGoal(false);
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				xyLoc l(x, y);
				std::cout << l << "\n";
				search.GetPath(ma1, l, l, result);
				ComputeReach(l, search);
			}
			else {
				mo->SetOverlayValue(x, y, 0.1);
			}
		}
	}
}
