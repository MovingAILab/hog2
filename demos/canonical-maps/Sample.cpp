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
#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "JPS.h"
#include "gJPS.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
bool runningSearch3 = false;
int px1 = 30, py1 = 79, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 20;
int gStepsPerFrame = 1;
double searchWeight = 1;
bool reopenNodes = false;
bool screenShot = false;
bool recording = false;

void WeightedCanAStarExperiments(char *scenario, double weight);
void WeightedAStarExperiments(char *scenario, double weight);
void DijkstraExperiments(char *scenario);
void JPSExperiments(char *scenario, double weight, uint32_t jump);
void OpenGridExperiments(int width);

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

std::vector<TemplateAStar<graphState, graphMove, GraphEnvironment> > astars;
CanonicalGrid::CanonicalGrid *grid;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> a2;
JPS *jps;

MapEnvironment *ma1 = 0;
CanonicalGrid::CanonicalGrid *ma2 = 0;

std::vector<xyLoc> path;
std::vector<CanonicalGrid::xyLoc> path2;
std::vector<xyLoc> path3;

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
		map = new Map("/Users/nathanst/hog2/maps/dao/orz101d.map");
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
	}
	if (ma2 == 0)
	{
		ma2 = new CanonicalGrid::CanonicalGrid(unitSims[id]->GetEnvironment()->GetMap());
		ma2->SetEightConnected();
	}
	jps = new JPS(map);
	jps->SetJumpLimit(0);
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
			// also draw lines
			//str += ma1->SVGDrawLine({queue.front().x, queue.front().y}, {s.x, s.y});
			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
			if (((queue.front().parent|s.parent) != (queue.front().parent)) ||
				(abs(s.x-queue.front().x)+abs(s.y-queue.front().y) <= 1 && (s.parent&(s.parent-1)) != 0))
				str += ma1->SVGDraw({s.x, s.y});
		}
		queue.pop_front();
	}
	return str;
}

std::string DrawCanonicalOrdering(bool gray = false)
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
	ma1->OpenGLDraw(cStart);
	
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
		AStarOpenClosedData<xyLoc> data;
		if (regAstar.GetClosedItem(next, data))
		{
			ma1->GetSuccessors(data.data, v);
			CanonicalGrid::xyLoc t(data.data.x, data.data.y);
			grid->OpenGLDraw(t);
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
		GLdouble t, l, r, b, z, rad, tmp;
		printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		ma1->GetMap()->GetOpenGLCoord(minx, miny, l, t, z, rad);
		ma1->GetMap()->GetOpenGLCoord(maxx, maxy, r, b, z, rad);
		glColor3f(1.0, 1.0, 1.0);
		glLineWidth(4.0);
		OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
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
		AStarOpenClosedData<CanonicalGrid::xyLoc> data;
		if (canAstar.GetClosedItem(next, data))
		{
			grid->GetSuccessors(data.data, v);
			grid->OpenGLDraw(data.data);
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
		GLdouble t, l, r, b, z, rad, tmp;
		printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		ma1->GetMap()->GetOpenGLCoord(minx, miny, l, t, z, rad);
		ma1->GetMap()->GetOpenGLCoord(maxx, maxy, r, b, z, rad);
		glColor3f(1.0, 1.0, 1.0);
		glLineWidth(4.0);
		OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
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
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}

	// draw canonical ordering in SVG
	if (screenShot)
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
	if (screenShot) // canonical ordering with jump points
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
	if (0) // canonical open gl
	{
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
					grid->SetColor(0.0, 0.0, 0.0);
					queue.push_back(s);
				}
				else {
					grid->SetColor(1.0, 0.0, 0.0);
				}
				grid->GLDrawLine(queue.front(), s);
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
	
	unitSims[windowID]->OpenGLDraw();
	
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

		std::fstream svgFile;
		svgFile.open("/Users/nathanst/Desktop/jps.svg", std::fstream::out | std::fstream::trunc);

		svgFile << ma1->SVGHeader();
//		svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*ma1->GetMap()->GetMapWidth()+10)+"\" height = \""+std::to_string(10*ma1->GetMap()->GetMapHeight()+10+100)+"\">";
		
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering(true);

		svgFile << jps->SVGDraw();

		svgFile << "</svg>";
		svgFile.close();
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
	
	
	if (GetNumPorts(windowID) == 3)
	{
		if ((ma1) && (viewport == 0)) // only do this once...
		{
			ma1->SetColor(0.0, 0.5, 0.0, 0.75);
			if (runningSearch1 && !unitSims[windowID]->GetPaused())
			{
				ma1->SetColor(0.0, 0.0, 1.0, 0.75);
				for (int x = 0; x < gStepsPerFrame; x++)
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
			if (path.size() == 0)
				a1.OpenGLDraw();
		}

		if ((ma2) && viewport == 1)
		{
			ma2->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch2)
			{
				ma2->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame; x++)
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
			if (path2.size() == 0)
				a2.OpenGLDraw();
		}
		if ((ma1) && viewport == 2)
		{
			ma1->SetColor(1.0, 0.0, 0.0, 0.5);
			if (runningSearch3)
			{
				ma1->SetColor(1.0, 0.0, 1.0, 0.5);
				for (int x = 0; x < gStepsPerFrame; x++)
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
			jps->OpenGLDraw();
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
				for (int x = 0; x < gStepsPerFrame; x++)
				{
					if (jps->DoSingleSearchStep(path3))
					{
						runningSearch3 = false;
						break;
					}
				}
			}
			jps->OpenGLDraw();
		}
	}
	if (path.size() > 0)
	{
		ma1->SetColor(0.0, 1.0, 1.0);
		for (int x = 0; x < path.size()-1; x++)
			ma1->GLDrawLine(path[x], path[x+1]);
	}
	if (path3.size() > 0)
	{
		glLineWidth(10);
		ma1->SetColor(1.0, 0.0, 1.0);
		for (int x = 0; x < path3.size()-1; x++)
			ma1->GLDrawLine(path3[x], path3[x+1]);
		glLineWidth(1);
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
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
		case '|': resetCamera(); break;
		case 'r': recording = !recording; break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
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
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long w, tKeyboardModifier , char)
{
	printf("Testing Dijkstra vs gJPS\n");
	Map *m = ma1->GetMap();
	gJPS gjps;
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

void TestJPS()
{
	std::vector<xyLoc> states;
	srandom(1234);
	const int totalStates = 10000;
	// 1. Generate 10000 states
	for (int x = 0; x < totalStates; x++)
	{
		states.push_back(GetRandomState());
	}
	Timer t;
	// 2. Check validity of each
	int numLegal = 0;
	t.StartTimer();
	for (int x = 0; x < totalStates; x++)
		numLegal += (ma1->GetMap()->GetTerrainType(states[x].x, states[x].y) == kGround)?1:0;
	//r->LegalState(states[x])?1:0;
	t.EndTimer();
	printf("%f elapsed; %d of %d legal\n", t.GetElapsedTime(), numLegal, totalStates);
	
	// 3. Add each to queue
	AStarOpenClosed<xyLoc, AStarCompare<xyLoc> > openClosedList;
	int numAdded = 0;
	t.StartTimer();
	for (int x = 0; x < totalStates; x++)
	{
		uint64_t objid;
		if (openClosedList.Lookup(ma1->GetStateHash(states[x]), objid) != kOpenList)
		{
			numAdded++;
			openClosedList.AddOpenNode(states[x], ma1->GetStateHash(states[x]), 0, 0);
		}
	}
	t.EndTimer();
	printf("%f elapsed added %d of %d to open\n", t.GetElapsedTime(), numAdded, totalStates);

	// 3. Add each to queue
	IndexOpenClosed<xyLoc> openClosedList2;
	openClosedList2.Reset(ma1->GetMap()->GetMapHeight()*ma1->GetMap()->GetMapWidth());
	numAdded = 0;
	t.StartTimer();
	for (int x = 0; x < totalStates; x++)
	{
		uint64_t objid;
		if (openClosedList2.Lookup(ma1->GetStateHash(states[x]), objid) != kOpenList)
		{
			numAdded++;
			openClosedList2.AddOpenNode(states[x], ma1->GetStateHash(states[x]), 0, 0);
		}
	}
	t.EndTimer();
	printf("%f elapsed added %d of %d to open (grid)\n", t.GetElapsedTime(), numAdded, totalStates);

	//	t.StartTimer();
	//	astar.GetPath(r, s, g, ourPath);
	//	t.EndTimer();
	//	printf("Astar %f %llu %llu %1.2f\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), r->GetPathLength(ourPath));
	
}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	TestJPS();
	return;
	
	xyLoc s1;
	xyLoc g1;
	s1.x = px1; s1.y = py1;
	g1.x = px2; g1.y = py2;

	jps->InitializeSearch(ma1, s1, g1, path3);

	int step = 0;
	bool done = false;
	while (true)
	{
		std::fstream svgFile;
		std::string file = "/Users/nathanst/Movies/tmp/jps";
		file += std::to_string((step/100)%10);
		file += std::to_string((step/10)%10);
		file += std::to_string(step%10);
		file += ".svg";
		svgFile.open(file.c_str(), std::fstream::out | std::fstream::trunc);
		
		svgFile << ma1->SVGHeader();
		svgFile << ma1->SVGDraw();
		svgFile << DrawCanonicalOrdering(true);
		
		svgFile << jps->SVGDraw();
		
		svgFile << "</svg>";
		svgFile.close();
		if (done)
			break;
		done = jps->DoSingleSearchStep(path3);
		step++;
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

	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				startLoc = loc;
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

				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s2, g2, path2);
				jps->InitializeSearch(ma1, s1, g1, path3);
				runningSearch1 = true;
				runningSearch2 = true;
				runningSearch3 = true;
				SetNumPorts(windowID, 3);

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
	TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc>> astar;
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
	
	jps.SetJumpLimit(0);
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
	gJPS gjps;
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

