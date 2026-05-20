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

#include <random>
#include <algorithm>
#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
//#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
//#include "MapSectorAbstraction.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "Graphics.h"
#include "SVGUtil.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 20;
int gStepsPerFrame = 1;
double searchWeight = 0;
bool reopenNodes = false;
bool screenShot = false;
bool recording = false;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

std::vector<TemplateAStar<graphState, graphMove, GraphEnvironment> > astars;
CanonicalGrid::CanonicalGrid *grid;

//EPEAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;

GraphEnvironment *ge = 0;

//MapSectorAbstraction *msa;

std::vector<xyLoc> path;

std::vector<xyLoc> subgoals;
std::vector<std::pair<xyLoc, xyLoc>> subgoalEdges;

Map *ReduceMap(Map *inputMap);
void MeasureHighwayDimension(Map *m, int depth);
void EstimateDimension(Map *m);
double EstimateLongPath(Map *m);

void testHeuristic(char *problems);

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
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
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz101d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		map = new Map("/Users/nathanst/hog2/maps/local/den407-crop.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/wc3maps/battleground.map");
		//map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/local/weight.map");
		//map = new Map("weight.map");
//		map = new Map(100, 100);
//		map->SetTerrainType(25, 25, kTrees);
//		map->SetTerrainType(75, 75, kTrees);
//		map->SetTerrainType(75, 25, kTrees);
//		map->SetTerrainType(25, 75, kTrees);
//		map = new Map(mazeSize, mazeSize);
//		MakeMaze(map, 5);
//		map->Scale(512, 512);
//		map = MakeWarehouseMap(10, 10, 1, 10, 5, 25, 25);
//		map = MakeWarehouseMap(20, 20, 1, 10, 5, 50, 50);
//		map = MakeWarehouseMap(10, 10, 2, 10, 5, 25, 25);
//		map = MakeWarehouseMap(20, 20, 2, 10, 5, 50, 50);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);
//	msa = new MapSectorAbstraction(map, 2);
	//msa->ToggleDrawAbstraction(1);
	//msa->ToggleDrawAbstraction(2);
	// ->ToggleDrawAbstraction(3);
	unitSims.resize(id+1);
	MapEnvironment *me;
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(me = new MapEnvironment(map));
	me->SetDiagonalCost(1.5);
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
	grid = new CanonicalGrid::CanonicalGrid(map);
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
	InstallKeyboardHandler(MyDisplayHandler, "Detail", "Export detailed SVG from A*", kAnyModifier, 'q');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	InstallKeyboardHandler(MySubgoalHandler, "Setup Subgoals", "Set up and display subgoal information", kNoModifier, 's');
	
	
	InstallCommandLineHandler(MyCLHandler, "-makeMazeSVG", "-makeMazeSVG x-dim y-dim corridorsize x-output y-output filename", "Makes map and exports as svg");
	InstallCommandLineHandler(MyCLHandler, "-makeMaze", "-makeMaze x-dim y-dim corridorsize filename", "Makes maze");
	InstallCommandLineHandler(MyCLHandler, "-makeRoom", "-makeRoom x-dim y-dim roomSie filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-makeRandom", "-makeRandom x-dim y-dim %%obstacles [0-100] filename", "makes a randomly filled with obstacles");
	InstallCommandLineHandler(MyCLHandler, "-makeWarehouse", "-makeWarehouse <columns> <rows> <corridor> <shelf-width> <shelf-height> <left buffer> <right buffer> <filename>", "Build a warehouse with the given parameters");
	InstallCommandLineHandler(MyCLHandler, "-resize", "-resize filename x-dim y-dim filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-buildProblemSet", "-buildProblemSet <map> <scen>", "Build problem set with the given map.");
	InstallCommandLineHandler(MyCLHandler, "-buildRandomSet", "-buildRandomSet <map> <scen>", "Build random problem set with all locations in the map.");
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems3", "-problems3 filename", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems4", "-problems4 filename", "Selects the problem set to run comparing weighted A* with and without re-openings.");
	InstallCommandLineHandler(MyCLHandler, "-problems5", "-problems5 filename additive_bound", "Selects the problem set to run comparing different methods of additive bounds.");
	InstallCommandLineHandler(MyCLHandler, "-problems6", "-problems6 filename", "Tests alpha.");
	InstallCommandLineHandler(MyCLHandler, "-screen", "-screen <map>", "take a screenshot of the screen and then exit");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <map> <output> <optional-x> <optional-y>", "Save the map as SVG format");
	InstallCommandLineHandler(MyCLHandler, "-size", "-batch integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	InstallCommandLineHandler(MyCLHandler, "-reduceMap", "-reduceMap input output", "Find the largest connected component in map and reduce.");
	InstallCommandLineHandler(MyCLHandler, "-highwayDimension", "-highwayDimension map radius", "Measure the highway dimension of a map.");
	InstallCommandLineHandler(MyCLHandler, "-estimateDimension", "-estimateDimension map", "Estimate the dimension.");
	InstallCommandLineHandler(MyCLHandler, "-estimateLongPath", "-estimateLongPath map", "Estimate the longest path in the map.");
	InstallCommandLineHandler(MyCLHandler, "-testHeuristic", "-testHeuristic scenario", "measure the ratio of the heuristic to the optimal dist");

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
		//SetZoom(windowID, 10);
	}

}

// 1. resize window to 1024x768 

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  Graphics::Display &display = getCurrentContext()->display;
	if (ge)
	{
	  ge->Draw(display);
	  return;
	}
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}

//	CanonicalGrid::xyLoc gLoc(px2, py2, CanonicalGrid::kAll);
//	//CanonicalGrid::xyLoc gLoc(10, 23, CanonicalGrid::kAll);
//	std::deque<CanonicalGrid::xyLoc> queue;
//	queue.push_back(gLoc);
//	std::vector<CanonicalGrid::xyLoc> v;
//	std::vector<bool> visited(grid->GetMap()->GetMapHeight()*grid->GetMap()->GetMapWidth());
//	while (!queue.empty())
//	{
//		grid->GetSuccessors(queue.front(), v);
//		for (auto &s : v)
//		{
//			if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
//			{
//				grid->SetColor(0.0, 0.0, 0.0);
//				queue.push_back(s);
//			}
//			else {
//				grid->SetColor(1.0, 0.0, 0.0);
//			}
//			grid->GLDrawLine(queue.front(), s);
//			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
//		}
//		queue.pop_front();
//	}
	
	//	for (unsigned int x = 0; x < astars.size(); x++)
//		astars[x].OpenGLDraw();
	//	astar.OpenGLDraw();	
	//	unitSims[windowID]->OpenGLDraw();
	//unitSims[windowID]->GetEnvironment()->OpenGLDraw();
	
	if (screenShot)
	{
	  //SaveScreenshot(windowID, gDefaultMap);
	  exit(0);
	}
//	glTranslatef(0, 0, -0.1);
//	glLineWidth(6.0);
//	if (ge == 0)
//		ge = new GraphEnvironment(GraphSearchConstants::GetGraph(unitSims[windowID]->GetEnvironment()->GetMap()));
//	ge->OpenGLDraw();

	//if (astars.size() > 0)
//	msa->OpenGLDraw();
	
	if (mouseTracking)
	{
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		double x, y, z, r;
		m->GetCoord(px1, py1, x, y, z, r);
		m->GetCoord(px2, py2, x, y, z, r);
	}
	
	//	if ((gdh) && (!ma1))
	//	gdh->OpenGLDraw();

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
		      printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
			     (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded(),
			     a1.GetNumOpenItems());
		      runningSearch1 = false;
		      break;
		    }
		}
	    }
	  a1.Draw(display);
	}
	if ((ma2) && viewport == 1)
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
						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
					break;
				}
			}
		}
		//a2.OpenGLDraw();
		a2.Draw(display);
	}

	
	if (subgoals.size() > 0)
	{
		printf("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>");

		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		printf("<svg xmlns:svg=\"http://www.w3.org/2000/svg\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" version=\"1.0\" id=\"svg2\" height=\"%ld\" width=\"%ld\">\n",
			   m->GetMapHeight()*10,
			   m->GetMapWidth()*10);
	
//		for (int x = 0; x < m->GetMapWidth(); x++)
//		{
//			for (int y = 0; y < m->GetMapHeight(); y++)
//			{
//				printf("<rect x = \"%d\" y=\"%d\" width=\"10\" height=\"10\" style=\"fill:rgb(255,255,255);stroke-width:3;stroke:rgb(255,255,255)\" />\n", x*10, y*10);
//			}
//		}

	}
	unitSims[windowID]->GetEnvironment()->SetColor(0.0, 0.0, 1.0);
	//glLineWidth(2.0);
	for (auto &x : subgoalEdges)
	{
	  //unitSims[windowID]->GetEnvironment()->GLDrawLine(x.first, x.second);
		printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />\n",
			   10*x.first.x,
			   10*x.first.y,
			   10*x.second.x,
			   10*x.second.y);
	}

	unitSims[windowID]->GetEnvironment()->SetColor(1.0, 0.0, 0.0);
	for (const xyLoc &x : subgoals)
	{
	  //unitSims[windowID]->GetEnvironment()->OpenGLDraw(x);
		
		printf("<circle cx=\"%d\" cy=\"%d\" r=\"5\" stroke=\"black\" stroke-width=\"0.25\" fill=\"red\" />\n", 10*x.x, 10*x.y);
	}

	if (subgoals.size() > 0)
	{
		printf("</svg>\n");
		exit(0);
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

void doExport()
{
	assert(!"Abstraction export deprecated");
/*
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8);
	msa->ToggleDrawAbstraction(1);
	Graph *g = msa->GetAbstractGraph(1);
	printf("g\n%d %d\n", g->GetNumNodes(), g->GetNumEdges());
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		int x1, y1;
		msa->GetTileFromNode(n, x1, y1);
		printf("%d %d %d\n", x, x1, y1);
	}
	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		printf("%d %d\n", e->getFrom(), e->getTo());//, (int)(100.0*e->GetWeight())); // %d 0
	}
 */
	exit(0);
}

#include <thread>
#include "SharedQueue.h"
SharedQueue<std::pair<graphState, graphState>> workQueue;
SharedQueue<std::pair<std::pair<graphState, graphState>, double>> resultQueue;

void GetPathLengthInRange(GraphEnvironment *ge, double minLen, double maxLen, int numNeeded)
{
	while (resultQueue.size() < numNeeded)
	{
		graphState start;
		graphState goal;
		double finalCost;
		std::vector<graphState> endPath;
		std::vector<graphState> eligible;
		
		start = ge->GetGraph()->GetRandomNode()->GetNum();
		
		// 2. search to depth d (all g-costs >= d)
		TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
		theSearch.SetStopAfterGoal(false);
		theSearch.InitializeSearch(ge, start, start, endPath);
		while (1)
		{
			if (theSearch.GetNumOpenItems() == 0)
				break;
			graphState next = theSearch.CheckNextNode();
			if (theSearch.DoSingleSearchStep(endPath))
				break;
			double tmpCost;
			theSearch.GetClosedListGCost(next, tmpCost);
			if (tmpCost >= minLen && tmpCost < maxLen)
				eligible.push_back(next);
			else if (tmpCost >= maxLen)
				break;
		}
		if (eligible.size() == 0)
		{
			finalCost = 0;
			continue;
		}
		goal = eligible[random()%eligible.size()];
		double cost;
		theSearch.GetClosedListGCost(goal, cost);
		assert(cost >= minLen && cost < maxLen);
		
		if (random()%2) // randomize start and goal
		{
			graphState tmp = start;
			start = goal;
			goal = tmp;
		}
		finalCost = cost;

		resultQueue.Add({{start, goal}, finalCost});
	}
}

void ThreadedGetPathLengthInRange(GraphEnvironment *ge, std::vector<graphState> &start, std::vector<graphState> &goal,
								  std::vector<double> &finalCost, double minLen, double maxLen, int numNeeded)
{
	std::vector<std::thread *> threads;
	for (size_t x = 0; x < std::thread::hardware_concurrency(); x++)
	{
		threads.push_back(new std::thread(GetPathLengthInRange, ge, minLen, maxLen, numNeeded));
	}
	for (size_t x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
	}
	while (resultQueue.size() > 0)
	{
		std::pair<std::pair<graphState, graphState>, double> result;
		resultQueue.WaitRemove(result);
		start.push_back(result.first.first);
		goal.push_back(result.first.second);
		finalCost.push_back(result.second);
	}
}



void PathfindingThread(GraphEnvironment *ge)
{
	TemplateAStar<graphState, graphMove, GraphEnvironment> searcher;

	std::pair<graphState, graphState> p;
	std::vector<graphState> thePath;
	while (true)
	{
		workQueue.WaitRemove(p);
		if (p.first == p.second)
			return;
		searcher.GetPath(ge, p.first, p.second, thePath);
		resultQueue.Add({{p.first, p.second}, ge->GetPathLength(thePath)});
	}
}

void buildProblemSet(const char *output = 0)
{
	srandom(time(0));
	ScenarioLoader s;
	printf("Generating scenarios for map: %s\n", gDefaultMap);
	Map map(gDefaultMap);
	Graph *g = GraphSearchConstants::GetGraph(&map);
	GraphDistanceHeuristic gdh(g);
	gdh.SetPlacement(kFarPlacement);
	// make things go fast; we're doing tons of searches, so use a good heuristic
	for (unsigned int x = 0; x < 10; x++)
		gdh.AddHeuristic();
	GraphEnvironment *ge = new GraphEnvironment(&map, g, &gdh);
	ge->SetDirected(true);
	
	const auto numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (size_t x = 0; x < numThreads; x++)
	{
		threads.push_back(new std::thread(PathfindingThread, ge));
	}
	std::vector<std::vector<Experiment> > experiments;

	std::vector<bool> startpoints;
	std::vector<bool> endpoints;
	endpoints.resize(g->GetNumNodes());
	startpoints.resize(g->GetNumNodes());
	//double len = EstimateLongPath(&map);
	printf("First pass: 100%% random\n");
	int totalTries = g->GetNumNodes()/10;//9*g->GetNumNodes()/10;
	for (int x = 0; x < totalTries; x++)
	{
		if (0==x%100)
		{ printf("\r%d of %d", x, totalTries); fflush(stdout); }
		node *s1 = g->GetRandomNode();
		node *g1 = g->GetRandomNode();
		if (startpoints[s1->GetNum()]) // no duplicate start locations
		{
			x--;
			continue;
		}
		if (endpoints[g1->GetNum()]) // no duplicate goal locations
		{
			x--;
			continue;
		}
		if (g1->GetNum() == s1->GetNum())
		{
			x--;
			continue;
		}
		startpoints[s1->GetNum()] = true;
		endpoints[g1->GetNum()] = true;
		graphState gs1, gs2;
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		workQueue.WaitAdd({gs1, gs2});
	}
	for (size_t x = 0; x < numThreads; x++)
	{
		workQueue.WaitAdd({0, 0});
	}
	printf("\nDone adding to threads\n");
	for (unsigned int x = 0; x < totalTries; x++)
	{
		if (0==x%100)
		{ printf("\r%d of %d", x, totalTries); fflush(stdout); }
		std::pair<std::pair<graphState, graphState>, double> res;
		resultQueue.WaitRemove(res);

		if (res.second == 0)
		{
			startpoints[res.first.first] = false;
			endpoints[res.first.second] = false;
			continue;
		}
		node *s1 = g->GetNode(res.first.first);
		node *g1 = g->GetNode(res.first.second);
		if (experiments.size() <= res.second/4)
			experiments.resize(res.second/4+1);
		if (experiments[res.second/4].size() < 10)
		{
			Experiment e(s1->GetLabelL(GraphSearchConstants::kMapX), s1->GetLabelL(GraphSearchConstants::kMapY),
						 g1->GetLabelL(GraphSearchConstants::kMapX), g1->GetLabelL(GraphSearchConstants::kMapY),
						 map.GetMapWidth(), map.GetMapHeight(), res.second/4, res.second, gDefaultMap);
			experiments[res.second/4].push_back(e);
		}
		else {
			startpoints[s1->GetNum()] = false;
			endpoints[g1->GetNum()] = false;
		}
	}
	printf("\nDone receiving from threads\n");
	
	for (size_t x = 0; x < numThreads; x++)
	{
		threads[x]->join();
		delete threads[x];
	}
	threads.resize(0);
	
	for (size_t x = 0; x < experiments.size(); x++)
	{
		while (experiments[x].size() != 10)
		{
			printf("Filling bucket %d (cost %d to %d) Have %d so far\n", x, x*4, x*4+4, (int)experiments[x].size());
			std::vector<graphState> start, goal;
			std::vector<double> length;

			ThreadedGetPathLengthInRange(ge, start, goal, length, x*4, x*4+4, 10-int(experiments[x].size()));
			int success = 0;
			for (int y = 0; y < start.size(); y++)
			{
				if (startpoints[start[y]]) // no duplicate start locations
					continue;
				if (endpoints[goal[y]]) // no duplicate goal locations
					continue;
				Experiment e(g->GetNode(start[y])->GetLabelL(GraphSearchConstants::kMapX), g->GetNode(start[y])->GetLabelL(GraphSearchConstants::kMapY),
							 g->GetNode(goal[y])->GetLabelL(GraphSearchConstants::kMapX), g->GetNode(goal[y])->GetLabelL(GraphSearchConstants::kMapY),
							 map.GetMapWidth(), map.GetMapHeight(), length[y]/4, length[y], gDefaultMap);
				experiments[x].push_back(e);
				startpoints[start[y]] = true;
				endpoints[goal[y]] = true;
				success++;
				if (experiments[x].size() >= 10)
					break;
			}
			if (success == 0)
				break;
		}
		if (experiments[x].size() != 10)
		{
			printf("Can't fill bucket; aborting fill procedure and cutting off scenarios at %d buckets\n", x-1);
			break;
		}
	}
	
	
	for (unsigned int x = 0; x < experiments.size(); x++)
	{
		if (x > experiments.size()/10 && experiments[x].size() != 10)
			break;
		for (unsigned int y = 0; y < experiments[x].size(); y++)
			s.AddExperiment(experiments[x][y]);
	}
	printf("\n");
	if (output == 0)
	{
		char name[255];
		sprintf(name, "%s.scen", gDefaultMap);
		s.Save(name);
	}
	else
		s.Save(output);
	exit(0);
}

void buildRandomSet(const char *output = 0)
{
	srandom(time(0));
	ScenarioLoader s;
	printf("Generating scenarios for map: %s\n", gDefaultMap);
	Map map(gDefaultMap);
	xyLoc start;
	std::vector<xyLoc> path;
	std::vector<xyLoc> points;
	MapEnvironment me(&map);
	while (true)
	{
		start.x = random()%map.GetMapWidth();
		start.y = random()%map.GetMapWidth();
		if (map.GetTerrainType(start.x, start.y) == kGround)
			break;
	}
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	astar.SetStopAfterGoal(false);
	astar.GetPath(&me, start, start, path);

	points.clear();
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		points.push_back(astar.GetItem(x).data);
	}
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle( points.begin(), points.end(), g);
	if (1 == points.size()%2)
		points.pop_back();
	if (points.size() > 2000)
		points.resize(2000);
	astar.SetStopAfterGoal(true);
	for (size_t x = 0; x < points.size(); x+= 2)
	{
		astar.GetPath(&me, points[x], points[x+1], path);
		//	Experiment(int sx,int sy,int gx,int gy,int b, double d, string m)
		double len = me.GetPathLength(path);
		Experiment e(points[x].x, points[x].y, points[x+1].x, points[x+1].y, map.GetMapWidth(), map.GetMapHeight(), len/4, len, gDefaultMap);
		s.AddExperiment(e);
	}
	if (output == 0)
	{
		char name[255];
		sprintf(name, "%s.scen", gDefaultMap);
		s.Save(name);
	}
	else
		s.Save(output);
	exit(0);
}
void runProblemSet3(char *scenario)
{
	printf("Loading scenario %s\n", scenario);
	ScenarioLoader sl(scenario);
	
	printf("Loading map %s\n", sl.GetNthExperiment(0).GetMapName());
	Map *map = new Map(sl.GetNthExperiment(0).GetMapName());
	map->Scale(sl.GetNthExperiment(0).GetXScale(), 
			   sl.GetNthExperiment(0).GetYScale());
	
	PEAStar<xyLoc, tDirection, MapEnvironment> pea;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	std::vector<xyLoc> thePath;
	MapEnvironment ma(map);
	ma.SetFourConnected();
	
	for (int x = 0; x < sl.GetNumExperiments(); x++)
	{
		if (sl.GetNthExperiment(x).GetBucket() != 127)
			continue;
		xyLoc from, to;
		printf("%d\t", sl.GetNthExperiment(x).GetBucket());
		from.x = sl.GetNthExperiment(x).GetStartX();
		from.y = sl.GetNthExperiment(x).GetStartY();
		to.x = sl.GetNthExperiment(x).GetGoalX();
		to.y = sl.GetNthExperiment(x).GetGoalY();
		Timer t;
		t.StartTimer();
		pea.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		printf("pea\t%ld\t%1.6f\t%llu\t%u", thePath.size(), t.GetElapsedTime(), pea.GetNodesExpanded(), pea.GetNumOpenItems());
		t.StartTimer();
		astar.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		printf("\tastar\t%ld\t%1.6f\t%llu\t%u\n", thePath.size(), t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNumOpenItems());
	}

	exit(0);
}

void runProblemSet4(char *scenario)
{
	printf("Loading scenario %s\n", scenario);
	ScenarioLoader sl(scenario);
	
	printf("Loading map %s\n", sl.GetNthExperiment(0).GetMapName());
	Map *map = new Map(sl.GetNthExperiment(0).GetMapName());
	map->Scale(sl.GetNthExperiment(0).GetXScale(),
			   sl.GetNthExperiment(0).GetYScale());
	
	std::vector<xyLoc> thePath;
	MapEnvironment ma(map);
	ma.SetEightConnected();
	//ma.SetFourConnected();
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	astar.SetWeight(10.0);
	astar.SetReopenNodes(false);
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar2;
	astar2.SetWeight(10.0);
	astar2.SetReopenNodes(true);
	
	for (int x = 0; x < sl.GetNumExperiments(); x++)
	{
		if (sl.GetNthExperiment(x).GetBucket() != 32)
			continue;
		xyLoc from, to;
		printf("%d\t", sl.GetNthExperiment(x).GetBucket());
		from.x = sl.GetNthExperiment(x).GetStartX();
		from.y = sl.GetNthExperiment(x).GetStartY();
		to.x = sl.GetNthExperiment(x).GetGoalX();
		to.y = sl.GetNthExperiment(x).GetGoalY();
		printf("(%d, %d) (%d, %d)\t", from.x, from.y, to.x, to.y);
		Timer t;
		
		t.StartTimer();
		astar.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		
		printf("astar-no\t%1.2f\t%1.6f\t%llu\t%u", ma.GetPathLength(thePath), t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNumOpenItems());

		t.StartTimer();
		astar2.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		
		printf("\tastar-yes\t%1.2f\t%1.6f\t%llu\t%u\n", ma.GetPathLength(thePath), t.GetElapsedTime(), astar2.GetNodesExpanded(), astar2.GetNumOpenItems());
	}
	
	exit(0);
}

void runProblemSet5(char *scenario, double bound)
{
	printf("Loading scenario %s\n", scenario);
	ScenarioLoader sl(scenario);
	
	printf("Loading map %s\n", sl.GetNthExperiment(0).GetMapName());
	Map *map = new Map(sl.GetNthExperiment(0).GetMapName());
	map->Scale(sl.GetNthExperiment(0).GetXScale(),
			   sl.GetNthExperiment(0).GetYScale());
	
	std::vector<xyLoc> thePath;
	MapEnvironment ma(map);
	ma.SetEightConnected();
	//ma.SetFourConnected();
	
	// K = bound+1
	// K = 2*bound
	// y < K

	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar2;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar3;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar4;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar5;

	astar2.SetPhi([=](double x,double y){return (y<(bound+1))?(x+y/(bound+1)):(x+y-bound);});
	astar3.SetPhi([=](double x,double y){return (y<(2*bound))?(x+y/2):(x+y-bound);});
	astar4.SetReopenNodes(true);
	
	for (int x = 0; x < sl.GetNumExperiments(); x++)
	{
		if (sl.GetNthExperiment(x).GetBucket() != 32)
			continue;
		xyLoc from, to;
		printf("%d\t", sl.GetNthExperiment(x).GetBucket());
		from.x = sl.GetNthExperiment(x).GetStartX();
		from.y = sl.GetNthExperiment(x).GetStartY();
		to.x = sl.GetNthExperiment(x).GetGoalX();
		to.y = sl.GetNthExperiment(x).GetGoalY();
		printf("(%d, %d) (%d, %d)\t", from.x, from.y, to.x, to.y);
		Timer t;
		
		t.StartTimer();
		astar.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		printf("astar-opt\t%1.2f\t%1.6f\t%llu\t%u\t", ma.GetPathLength(thePath), t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNumOpenItems());

		t.StartTimer();
		astar2.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		printf("astar-K=b+1\t%1.2f\t%1.6f\t%llu\t%u\t", ma.GetPathLength(thePath), t.GetElapsedTime(), astar2.GetNodesExpanded(), astar2.GetNumOpenItems());

		t.StartTimer();
		astar3.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		printf("astar-K=b*2\t%1.2f\t%1.6f\t%llu\t%u\t", ma.GetPathLength(thePath), t.GetElapsedTime(), astar3.GetNodesExpanded(), astar3.GetNumOpenItems());

		
		double hi = ma.HCost(from, to);
		astar4.SetPhi([=](double h,double g){return g+h+bound*std::min(h, hi)/hi;});
		t.StartTimer();
		astar4.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		
		printf("\tastar-yes\t%1.2f\t%1.6f\t%llu\t%u\t%u\t", ma.GetPathLength(thePath), t.GetElapsedTime(), astar4.GetNodesExpanded(), astar4.GetNumOpenItems(), astar4.GetNodesExpanded()-astar4.GetUniqueNodesExpanded());


		if (hi < bound)
			hi = bound+1;
		astar5.SetPhi([=](double x,double y){return (y<hi)?(x+(hi-bound)*y/(hi)):(x+y-bound);});
		t.StartTimer();
		astar5.GetPath(&ma, from, to, thePath);
		t.EndTimer();
		
		printf("\tastar\t%1.2f\t%1.6f\t%llu\t%u\n", ma.GetPathLength(thePath), t.GetElapsedTime(), astar5.GetNodesExpanded(), astar5.GetNumOpenItems());

	}
	
	exit(0);
}

#include "GridHeuristics.h"
void runProblemSet6(char *scenario, bool heuristic)
{
	printf("Loading scenario %s\n", scenario);
	ScenarioLoader sl(scenario);
	
	printf("Loading map %s\n", sl.GetNthExperiment(0).GetMapName());
	Map *map = new Map(sl.GetNthExperiment(0).GetMapName());
	map->Scale(sl.GetNthExperiment(0).GetXScale(),
			   sl.GetNthExperiment(0).GetYScale());
	
	std::vector<xyLoc> thePath;
	MapEnvironment ma(map);
	ma.SetEightConnected();
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

	GridEmbedding *ge = new GridEmbedding(&ma, 10, kLINF);
	if (heuristic)
	{
		for (int x = 0; x < 10; x++)
			ge->AddDimension(kDifferential, kFurthest);
		astar.SetHeuristic(ge);
	}
	
	
	Timer t;
	for (float w = 9; w >= 1.05; w=(w-1)/2.0f+1) // weight
		//		for (float w = 9; w >= 1.2; w=(w-1)/2.0f+1) // weight
		//for (float w = 1.25; w <= 10; w=(w-1)*2+1) // weight
	{
		for (float alpha = 0; flesseq(alpha, 1); alpha += 0.25f*0.5f) // alpha
		{
			for (int z = 0; z < 4; z++) // priority functions
			{
				switch (z)
				{
					case 0:
					{
						// 0: XDP/XUP            (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(false);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (g <= w1*h) return (g+w1*h)/w1;
							else return (g+w2*h)/w;
						});
					}
						break;
					case 1:
					{
						// 1: XDP/XUP-0.5 offset (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(false);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (3*g <= w1*h) return (g+w1*h)/w1;
							else if (g <= h*(4*w-w1)) return 4*(g+w2*h)/(w1+3*w2);
							else return (g+w1*h)/w;
						});
					}
						break;
					case 2:
					{
						// 0: XDP/XUP            (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(true);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (g <= w1*h) return (g+w1*h)/w1;
							else return (g+w2*h)/w;
						});
					}
						break;
					case 3:
					{
						// 3: XDP/XUP-0.5 offset (   reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(true);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (3*g <= w1*h) return (g+w1*h)/w1;
							else if (g <= h*(4*w-w1)) return 4*(g+w2*h)/(w1+3*w2);
							else return (g+w1*h)/w;
						});
					}
						break;
				}
				printf("Starting w=%1.2f, alpha=%1.2f, alg = %d\n", w, alpha, z);
				for (int x = 0; x < sl.GetNumExperiments(); x++)
				{
					if (sl.GetNthExperiment(x).GetBucket() < 25 || sl.GetNthExperiment(x).GetBucket() >= 50)
						continue;
					xyLoc from, to;
//					printf("%d\t", sl.GetNthExperiment(x).GetBucket());
					from.x = sl.GetNthExperiment(x).GetStartX();
					from.y = sl.GetNthExperiment(x).GetStartY();
					to.x = sl.GetNthExperiment(x).GetGoalX();
					to.y = sl.GetNthExperiment(x).GetGoalY();
//					printf("(%d, %d) (%d, %d)\t", from.x, from.y, to.x, to.y);
					Timer t;
					
					t.StartTimer();
					astar.GetPath(&ma, from, to, path);
					t.EndTimer();
					printf("[Re-open: %s. w = %1.2f. alpha=%1.2f. Alg: %d.] Problem %d solved in %f seconds. %llu expanded. Solution length %f; h = %s\n",
						   astar.GetReopenNodes() ? "yes" : "no", w, alpha, z,
						   x+1,
						   t.GetElapsedTime(), astar.GetNodesExpanded(), ma.GetPathLength(path), heuristic?"yes":"no");
				}
//				for (int x = 0; x < 100; x++)
//				{
//					start = STP::GetKorfInstance(x);
//					goal.Reset();
//					t.StartTimer();
//					astar.GetPath(&mnp44, start, goal, path);
//					t.EndTimer();
//				}
			}
		}
	}
}
void runProblemSet2(char *problems, int multiplier)
{
	/*
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8, multiplier);
	
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah1(msa, 1);
	GraphDistanceHeuristic localGDH(g);
	localGDH.SetPlacement(kAvoidPlacement);
	for (unsigned int x = 0; x < 10; x++)
		localGDH.AddHeuristic();

	GraphHeuristicContainer ghc(g);
	
	GraphRefinementEnvironment env1(msa, 1, &localGDH);
	GraphRefinementEnvironment env2(msa, 1, 0);
//	ghc.AddHeuristic(&localGDH);
//	ghc.AddHeuristic(&gah1);
	env1.SetDirected(false);
	
	FILE *f = fopen(problems, "r");
	if (f == 0)
	{
		printf("Cannot open file: '%s'\n", problems);
		exit(0);
	}
	Timer t;
	printf("len\tnodes\ttoucht\tlen\ttime\tdiff_n\tdiff_t\tdiff_l\ttime\n");
	while (!feof(f))
	{
		int from, to, cost;
		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
			break;
		node *s1 = g->GetNode(from);
		node *g1 = g->GetNode(to);
		graphState gs, gg;
		gs = s1->GetNum();
		gg = g1->GetNum();
		std::vector<graphState> thePath;
		t.StartTimer();
		astar.GetPath(&env2, gs, gg, thePath);
		t.EndTimer();
		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t%e\t",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   env1.GetPathLength(thePath), t.GetElapsedTime());
		t.StartTimer();
		astar.GetPath(&env1, gs, gg, thePath);
		t.EndTimer();
		printf("%llu\t%llu\t%1.2f\t%e",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   env1.GetPathLength(thePath), t.GetElapsedTime());
		printf("\n");
	}
	fclose(f);
	exit(0);
	 */
}

void runProblemSet(char *problems, int multiplier)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	ScenarioLoader s(problems);
	Map *map = new Map(s.GetNthExperiment(0).GetMapName());
	map->Scale(s.GetNthExperiment(0).GetXScale(), s.GetNthExperiment(0).GetYScale());
	MapEnvironment e(map);
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());
		searcher.GetPath(&e, from, to, path);
		if (fgreater(fabs(e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance()), 0.01))
		{
			std::cout << "From: " << from << " to " << to << "\n";
			printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), e.GetPathLength(path), s.GetNthExperiment(x).GetDistance(),
				   e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance());
			exit(1);
		}
		printf("%f\t%llu\n", e.GetPathLength(path), searcher.GetNodesExpanded());
	}
	
	exit(0);
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-makeRoom" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		Map map(atoi(argument[1]), atoi(argument[2]));
		BuildRandomRoomMap(&map, atoi(argument[3]));
		map.Save(argument[4]);
		exit(0);
		return 5;
	}
	if (strcmp( argument[0], "-makeMazeSVG" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		Map map(atoi(argument[1]), atoi(argument[2]));
		MakeMaze(&map, atoi(argument[3]));
		Graphics::Display disp;
		for (int y = 0; y < map.GetMapHeight(); y++)
		{
			for (int x = 0; x < map.GetMapWidth(); x++)
			{
				if (x+1 < map.GetMapWidth() && map.GetTerrainType(x, y) != kGround && map.GetTerrainType(x+1, y) != kGround)
				{
					double a, b, c, d, e, f, r;
					map.GetCoord(x, y, a, b, c, r);
					map.GetCoord(x+1, y, d, e, f, r);
					disp.DrawLine(Graphics::point(a, b), Graphics::point(d, e), r/4.0, Colors::black);
				}
				if (y+1 < map.GetMapHeight() && map.GetTerrainType(x, y) != kGround && map.GetTerrainType(x, y+1) != kGround)
				{
					double a, b, c, d, e, f, r;
					map.GetCoord(x, y, a, b, c, r);
					map.GetCoord(x, y+1, d, e, f, r);
					disp.DrawLine(Graphics::point(a, b), Graphics::point(d, e), r/4.0, Colors::black);
				}
			}
		}
		{
			double a, b, c, r;
			map.GetCoord(1, 1, a, b, c, r);
			disp.DrawText("S", Graphics::point(a, b), Colors::blue, r*2);
			map.GetCoord((int)map.GetMapWidth()-2, (int)map.GetMapHeight()-2, a, b, c, r);
			disp.DrawText("F", Graphics::point(a, b), Colors::blue, r*2);
		}
		//map.Save(argument[4]);
		MakeSVG(disp, argument[6], atoi(argument[4]), atoi(argument[5]));
		exit(0);
		return 5;
	}//
	if (strcmp( argument[0], "-makeMaze" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		Map map(atoi(argument[1]), atoi(argument[2]));
		MakeMaze(&map, atoi(argument[3]));
		map.Save(argument[4]);
		exit(0);
		return 5;
	}//	
	else if (strcmp( argument[0], "-testHeuristic" ) == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		testHeuristic(argument[1]);
		exit(0);
	}
	else if (strcmp( argument[0], "-estimateLongPath" ) == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		Map map(argument[1]);
		EstimateLongPath(&map);
		exit(0);
		return 2;
	}
	else if (strcmp( argument[0], "-estimateDimension" ) == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		Map map(argument[1]);
		EstimateDimension(&map);
		exit(0);
		return 2;
	}
	else if (strcmp( argument[0], "-highwayDimension" ) == 0)
	{
		if (maxNumArgs <= 2)
			return 0;
		Map map(argument[1]);
		MeasureHighwayDimension(&map, atoi(argument[2]));
		exit(0);
		return 3;
	}
	else if (strcmp( argument[0], "-makeRandom" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		Map map(atoi(argument[1]), atoi(argument[2]));
		MakeRandomMap(&map, atoi(argument[3]));
		Map *f = ReduceMap(&map);
		f->Save(argument[4]);
		exit(0);
		return 5;
	}
	else if (strcmp( argument[0], "-makeWarehouse" ) == 0 )
	{
		if (maxNumArgs <= 8)
			return 0;
		// 0: -makeWarehouse
		// 1:<columns>
		// 2:<rows>
		// 3:<corridor>
		// 4:<shelf-width>
		// 5:<shelf-height>
		// 6:<left buffer>
		// 7:<right buffer>
		// 8:<filename>
		Map *map = MakeWarehouseMap(atoi(argument[1]), atoi(argument[2]), atoi(argument[3]), atoi(argument[4]), atoi(argument[5]),
									atoi(argument[6]), atoi(argument[7]));
		map->Save(argument[8]);
		delete map;
		return 9;
	}
	else if (strcmp( argument[0], "-reduceMap" ) == 0 )
	{
		if (maxNumArgs <= 2)
			return 0;
		Map map(argument[1]);
		map.Scale(512, 512);
		Map *m = ReduceMap(&map);
		m->Save(argument[2]);
		exit(0);
		return 3;
	}
	else if (strcmp( argument[0], "-resize" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		Map map(argument[1]);
		map.Scale(atoi(argument[2]), atoi(argument[3]));
		map.Save(argument[4]);
		exit(0);
		return 5;
	}
	if (strcmp( argument[0], "-screen" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		screenShot = true;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	if (strcmp( argument[0], "-svg" ) == 0 )
	{
		if (maxNumArgs <= 2)
			return 0;
		Graphics::Display d;
		
		std::fstream svgFile;
		MapEnvironment *me = new MapEnvironment(new Map(argument[1]));
		me->SetDrawOptions(kEfficientCells);
		me->Draw(d);
		int width = 1024;
		int height = 1024;
		if (maxNumArgs > 3)
		{
			height = width = atoi(argument[3]);
		}
		if (maxNumArgs > 4)
		{
			height = atoi(argument[4]);
		}
		printf("Making image %dx%d\n", width, height);

		MakeSVG(d, argument[2], width, height);
//		svgFile.open(argument[2], std::fstream::out | std::fstream::trunc);
//		svgFile << me->SVGHeader();
//		svgFile << me->SVGDraw();
//		svgFile << "</svg>\n";
//		svgFile.close();
		exit(0);
		return 2;
	}
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
//		Map map(argument[1]);
//		map.Scale(512, 512);
//		map.Save(argument[2]);
		//buildProblemSet();
		//doExport();
//		exit(0);
		return 2;
	}
	else if (strcmp( argument[0], "-buildProblemSet" ) == 0 )
	{
		strncpy(gDefaultMap, argument[1], 1024);
		if (maxNumArgs >= 2)
			buildProblemSet(argument[2]);
		else
			buildProblemSet();
	}
	else if (strcmp( argument[0], "-buildRandomSet" ) == 0 )
	{
		strncpy(gDefaultMap, argument[1], 1024);
		if (maxNumArgs >= 2)
			buildRandomSet(argument[2]);
		else
			buildRandomSet();

	}
	else if (strcmp( argument[0], "-size" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		return 2;
	}
	else if (strcmp(argument[0], "-problems" ) == 0 )
	{
		int val; 
		if (maxNumArgs <= 1)
			exit(0);
		else if (maxNumArgs <= 2)
			val = 1;
		else val = atoi(argument[2]);
		runProblemSet(argument[1], val);
		return 3;
	}
	else if (strcmp(argument[0], "-problems2" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		runProblemSet2(argument[1], atoi(argument[2]));
		return 3;
	}
	else if (strcmp(argument[0], "-problems3" ) == 0 )
	{
		if (maxNumArgs <= 1) exit(0);
		runProblemSet3(argument[1]);
		return 2;
	}
	else if (strcmp(argument[0], "-problems4" ) == 0 )
	{
		if (maxNumArgs <= 1) exit(0);
		runProblemSet4(argument[1]);
		return 2;
	}
	else if (strcmp(argument[0], "-problems5" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		runProblemSet5(argument[1], atof(argument[2]));
		return 3;
	}
	else if (strcmp(argument[0], "-problems6" ) == 0 )
	{
		if (maxNumArgs <= 1) exit(0);
		runProblemSet6(argument[1], false);
		runProblemSet6(argument[1], true);
		exit(0);
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
	case '|': //resetCamera(); break;
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
//			msa->ToggleDrawAbstraction(key-'0');
			break;
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
			if (gdh && ((GraphMapInconsistentHeuristic*)gdh)->GetMode() == kCompressed)
				((GraphMapInconsistentHeuristic*)gdh)->SetMode(kMax);
			else
				((GraphMapInconsistentHeuristic*)gdh)->SetMode(kCompressed);
			break;
		case '}':
			if (gdh)
				((GraphMapInconsistentHeuristic*)gdh)->IncreaseDisplayHeuristic();
			break;
		case '\t':
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
				if (a2.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
				}
			}
		}
			break;
		case 'q':
		{
			std::fstream svgFile;
			svgFile.open("/Users/nathanst/Desktop/AStarSample.svg", std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			svgFile << a1.SVGDrawDetailed();
			svgFile << "</svg>\n";
			svgFile.close();
			svgFile.open("/Users/nathanst/Desktop/AStarSample2.svg", std::fstream::out | std::fstream::trunc);
			svgFile << ma1->SVGHeader();
			svgFile << ma1->SVGDraw();
			ma1->SetColor(0, 0, 0);
			svgFile << ma1->SVGLabelState(xyLoc(px1, py1), "S", 1.0, -0.5, -0.5);
			svgFile << ma1->SVGLabelState(xyLoc(px2, py2), "G", 1.0, -0.5, -0.5);
			svgFile << "</svg>\n";
			svgFile.close();
		}
		default:
			break;
	}
}

bool IsDirectHReachable(xyLoc a, xyLoc b, std::vector<std::vector<uint8_t> > &world)
{
	MapEnvironment *env = unitSims[0]->GetEnvironment();
	double h = env->HCost(a, b);
	int minx = std::min(a.x, b.x);
	int maxx = std::max(a.x, b.x);
	int miny = std::min(a.y, b.y);
	int maxy = std::max(a.y, b.y);
	for (int x = minx; x <= maxx; x++)
	{
		for (int y = miny; y <= maxy; y++)
		{
			xyLoc tmp(x, y);
			// Are we inside the shortest-path region
			if (fequal(env->HCost(a, tmp)+env->HCost(tmp, b), h))
			{
				if (tmp == a || tmp == b)
					continue;
				if (world[x][y] != 0) // == 1 (obstacles)
					return false;
			}
		}
	}
	return true;
}

void MySubgoalHandler(unsigned long windowID, tKeyboardModifier, char key)
{
	std::vector<std::vector<uint8_t> > world;
	//	std::vector<xyLoc> subgoals;
//	std::vector<std::pair<xyLoc, xyLoc>> subgoalEdges;
	Map *map = unitSims[windowID]->GetEnvironment()->GetMap();
	world.resize(map->GetMapWidth());
	for (size_t x = 0; x < world.size(); x++)
	{
		world[x].resize(map->GetMapHeight());
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			if (map->GetTerrainType(x, y) != kGround)
				world[x][y] = 1;
		}
	}
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
				if (std::find(subgoals.begin(), subgoals.end(), next) == subgoals.end())
				{
					subgoals.push_back(next);
					world[next.x][next.y] = 2;
				}
			}

			if (!map->CanStep(x, y, x+1, y) && !map->CanStep(x, y, x+1, y+1) && !map->CanStep(x, y, x, y+1) &&
				map->CanStep(x+1, y, x+1, y+1) && map->CanStep(x+1, y+1, x, y+1) &&
				map->GetTerrainType(x+1, y+1) == kGround)
			{
				xyLoc next(x+1, y+1);
				if (std::find(subgoals.begin(), subgoals.end(), next) == subgoals.end())
				{
					subgoals.push_back(next);
					world[next.x][next.y] = 2;
				}
			}

			if (!map->CanStep(x, y, x-1, y) && !map->CanStep(x, y, x-1, y+1) && !map->CanStep(x, y, x, y+1) &&
				map->CanStep(x-1, y, x-1, y+1) && map->CanStep(x-1, y+1, x, y+1) &&
				map->GetTerrainType(x-1, y+1) == kGround)
			{
				xyLoc next(x-1, y+1);
				if (std::find(subgoals.begin(), subgoals.end(), next) == subgoals.end())
				{
					subgoals.push_back(next);
					world[next.x][next.y] = 2;
				}
			}

			if (!map->CanStep(x, y, x+1, y) && !map->CanStep(x, y, x+1, y-1) && !map->CanStep(x, y, x, y-1) &&
				map->CanStep(x+1, y, x+1, y-1) && map->CanStep(x+1, y-1, x, y-1) &&
				map->GetTerrainType(x+1, y-1) == kGround)

			{
				xyLoc next(x+1, y-1);
				if (std::find(subgoals.begin(), subgoals.end(), next) == subgoals.end())
				{
					subgoals.push_back(next);
					world[next.x][next.y] = 2;
				}
			}
		}
	}
	
	for (int x = 0; x < subgoals.size(); x++)
	{
		for (int y = x+1; y < subgoals.size(); y++)
		{
			if (IsDirectHReachable(subgoals[x], subgoals[y], world))
			{
				subgoalEdges.push_back(std::pair<xyLoc, xyLoc>(subgoals[x], subgoals[y]));
			}
		}
	}

}

void MyRandomUnitKeyHandler(unsigned long w, tKeyboardModifier , char)
{
	/*
	astars.resize(0);
	static uint64_t average1=0, average2 = 0;
	static int count = 0;
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic *gah2 = new GraphAbstractionHeuristic(msa, 2);
	GraphAbstractionHeuristic *gah1 = new GraphAbstractionHeuristic(msa, 1);

	GraphRefinementEnvironment *env2 = new GraphRefinementEnvironment(msa, 2, gah2, unitSims[w]->GetEnvironment()->GetMap());
	GraphRefinementEnvironment *env1 = new GraphRefinementEnvironment(msa, 1, gah1, unitSims[w]->GetEnvironment()->GetMap());
	env1->SetDirected(false);
	env2->SetDirected(false);

	for (unsigned int x = 0; x < 1; x++)
	{
//		node *s1 = g->GetRandomNode();
//		node *g1 = g->GetRandomNode();
//		node *s2 = msa->GetParent(s1);
//		node *g2 = msa->GetParent(g1);
//		int from, to, cost;
//		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
//			break;
		node *s1 = g->GetRandomNode();//g->GetNode(from);
		node *g1 = g->GetRandomNode();//g->GetNode(to);
		node *s2 = msa->GetParent(s1);
		node *g2 = msa->GetParent(g1);
		uint64_t nodesExpanded = 0;
		uint64_t nodesTouched = 0;
		printf("Searching from %d to %d in level 1; %d to %d in level 2\n",
			   s1->GetNum(), g1->GetNum(), s2->GetNum(), g2->GetNum());
		graphState gs1, gs2;
		gs1 = s2->GetNum();
		gs2 = g2->GetNum();
		std::vector<graphState> thePath;
		std::vector<graphState> abstractPath;
		astar.GetPath(env2, gs1, gs2, abstractPath);
		//printf("Abstract length %d\n", abstractPath.size());
		//		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env2->GetPathLength(abstractPath));
		if (abstractPath.size() == 0)
			break;

		nodesExpanded += astar.GetNodesExpanded();
		nodesTouched += astar.GetNodesTouched();
		
		env1->SetPlanningCorridor(abstractPath, 2);
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		astar.GetPath(env1, gs1, gs2, thePath);
		printf("Full\t%llu\t%1.2f\t", astar.GetNodesExpanded(), env1->GetPathLength(thePath));
//		for (unsigned int x = 0; x < thePath.size(); x++)
//			printf("{%d}", thePath[x]);
		
		int abstractStart = 0;
		gs1 = s1->GetNum();
		double totalLength = 0;
		int refineAmt = 5;
		do { // not working yet -- fully check!
			astars.resize(astars.size()+1);
			env1->SetPlanningCorridor(abstractPath, 2, abstractStart);
			gs2 = g1->GetNum();
			if (abstractPath.size()-abstractStart > refineAmt)
			{
				env1->SetUseAbstractGoal(true, 2);
				gs2 = abstractPath[abstractStart+refineAmt];
				//printf("Using abstract goal of %d\n", abstractStart+refineAmt);
			}
			else {
				env1->SetUseAbstractGoal(false, 0);
			}
			astars.back().GetPath(env1, gs1, gs2, thePath);
			abstractStart += refineAmt;
			gs1 = thePath.back();
			
//			for (unsigned int x = 0; x < thePath.size(); x++)
//				printf("(%d)", thePath[x]);
			
			nodesExpanded += astar.GetNodesExpanded();
			nodesTouched += astar.GetNodesTouched();
			totalLength += env1->GetPathLength(thePath);
			if (thePath.back() == gs2)
				break;
		} while (thePath.back() != g1->GetNum());
		
		printf("Part\t%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), totalLength);
		thePath.resize(0);
		printf("Tot\t%llu\t%llu\n", nodesExpanded, nodesTouched);
	}
	 */
}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	Graph *g = new Graph();
	node *n = new node("");
	g->AddNode(n);
	FILE *f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.co", "r");
	std::vector<double> xloc, yloc;
	double minx = DBL_MAX, maxx=-DBL_MAX, miny=DBL_MAX, maxy=-DBL_MAX;
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'v')
		{
			float x1, y1;
			int id;
			if (3 != sscanf(line, "v %d %f %f", &id, &x1, &y1))
				continue;
			//assert(id == xloc.size()+1);
			xloc.push_back(x1);
			yloc.push_back(y1);
			//printf("%d: (%f, %f) [%f, %f]\n", xloc.size(), x1, y1, minx, maxx);
			if (x1 > maxx) maxx = x1;
			if (x1 < minx) minx = x1;
			if (y1 > maxy) maxy = y1;
			if (y1 < miny) miny = y1;
			//if (maxx > -1)
		}
	}
	fclose(f); 
	printf("x between (%f, %f), y between (%f, %f)\n",
		   minx, maxx, miny, maxy);
	for (unsigned int x = 0; x < xloc.size(); x++)
	{
		//printf("(%f, %f) -> ", xloc[x], yloc[x]);
		xloc[x] -= (minx);
		xloc[x] /= (maxx-minx);
		xloc[x] = xloc[x]*2-1;

		yloc[x] -= (miny);
		yloc[x] /= (maxy-miny);
		yloc[x] = yloc[x]*2-1;
		
		node *n = new node("");
		g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, xloc[x]);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, yloc[x]);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		//printf("(%f, %f)\n", xloc[x], yloc[x]);
	}
	//a 1 2 1988
	f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.gr", "r");
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'a')
		{
			int x1, y1;
			sscanf(line, "a %d %d %*d", &x1, &y1);
			g->AddEdge(new edge(x1, y1, 1.0));
			//printf("%d to %d\n", x1, y1);
		}
	}
	fclose(f); 
	ge = new GraphEnvironment(g);
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
	if ((button == kMiddleButton) && (mType == kMouseDown))
	{
		unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
//		delete gdh;
		if (gdh == 0)
		{
			gdh = new GraphMapInconsistentHeuristic(unitSims[windowID]->GetEnvironment()->GetMap(),
													GraphSearchConstants::GetGraph(unitSims[windowID]->GetEnvironment()->GetMap()));
		}
		gdh->SetPlacement(kFarPlacement);
		node *n = gdh->GetGraph()->GetNode(unitSims[windowID]->GetEnvironment()->GetMap()->GetNodeNum(px1, py1));
		
		//for (int x = 0; x < 10; x++)
		gdh->AddHeuristic(n);
		if (ma1)
			ma1->SetGraphHeuristic(gdh);

//		((GraphMapInconsistentHeuristic*)gdh)->SetNumUsedHeuristics(10);
//		((GraphMapInconsistentHeuristic*)gdh)->SetMode(kMax);
		printf("Added heuristic at %d, %d\n", px1, px2);
		return true;
	}
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				startLoc = loc;
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
				//px1 = 128; py1 = 181; px2 = 430; py2 = 364; //128 181 430 364
				//(31, 3) (30, 113)
				//(63, 395) (129, 460)
				//px1 = 63; py1 = 395; px2 = 129; py2 = 460; //128 181 430 364
				//px1 = 129-22; py1 = 460-25; px2 = 129; py2 = 460; //128 181 430 364

				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				//331 355 67 318
//				px1 = 331; py1 = 355;
//				px2 = 67; py2 = 318;
				if (ma1 == 0)
				{
					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
					ma1->SetDiagonalCost(1.5);
//					if (gdh == 0)
//					{
//						gdh = new GraphMapInconsistentHeuristic(ma1->GetMap(), GraphSearchConstants::GetGraph(ma1->GetMap()));
//						gdh->SetPlacement(kAvoidPlacement);
					if (gdh != 0)
					{
						ma1->SetGraphHeuristic(gdh);
//						for (int x = 0; x < 10; x++)
						//							gdh->AddHeuristic();
//						((GraphMapInconsistentHeuristic*)gdh)->SetNumUsedHeuristics(4);
//						((GraphMapInconsistentHeuristic*)gdh)->SetMode(kCompressed);
					}
//					}
				}
				if (ma2 == 0)
				{
					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
					ma2->SetDiagonalCost(1.5);
					ma2->SetEightConnected();
				}
				
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				//a2.SetWeight(1.8);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
								
				a1.SetWeight(searchWeight);
				ma1->SetEightConnected();
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
				//a1.SetUseBPMX(1);
				a1.SetReopenNodes(reopenNodes);
				runningSearch1 = true;
				runningSearch2 = false;
				SetNumPorts(windowID, 1);
//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 0);
//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 1);
			}
				break;
		}
		return true;
	}
	return false;
}

int LabelConnectedComponents(Graph *g);

Map *ReduceMap(Map *inputMap)
{
	Graph *g = GraphSearchConstants::GetGraph(inputMap);

	int biggest = LabelConnectedComponents(g);
	
	Map *m = new Map(inputMap->GetMapWidth(), inputMap->GetMapHeight());
	for (int x = 0; x < inputMap->GetMapWidth(); x++)
	{
		for (int y = 0; y < inputMap->GetMapHeight(); y++)
		{
			if (inputMap->GetTerrainType(x, y) == kTrees)
				m->SetTerrainType(x, y, kTrees);
			else if (inputMap->GetTerrainType(x, y) == kWater)
				m->SetTerrainType(x, y, kWater);			
			else m->SetTerrainType(x, y, kOutOfBounds);
		}
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest)
		{
			int theX, theY;
			theX = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX);
			theY = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY);
			if (g->GetNode(inputMap->GetNodeNum(theX+1, theY)) &&
				(g->GetNode(inputMap->GetNodeNum(theX+1, theY))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
				(!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
			else if (g->GetNode(inputMap->GetNodeNum(theX, theY+1)) &&
					 (g->GetNode(inputMap->GetNodeNum(theX, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
					 (!g->FindEdge(x, inputMap->GetNodeNum(theX, theY+1))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
//			else if (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1)) &&
//					 (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
//					 (!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY+1))))
//			{
//				m->SetTerrainType(theX, theY, kOutOfBounds);
//			}
			else {
				if (inputMap->GetTerrainType(theX, theY) == kSwamp)
					m->SetTerrainType(theX, theY, kSwamp);
				else
					m->SetTerrainType(theX, theY, kGround);
			}
		}
		else if (inputMap->GetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
										  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY)) == kGround)
			m->SetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
							  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY), kTrees);
	}
	return m;
}

int LabelConnectedComponents(Graph *g)
{
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	int group = 0;
	std::vector<int> groupSizes;
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
		{
			group++;
			groupSizes.resize(group+1);

			std::vector<unsigned int> ids;
			ids.push_back(x);
			while (ids.size() > 0)
			{
				unsigned int next = ids.back();
				ids.pop_back();
				if (g->GetNode(next)->GetLabelL(GraphSearchConstants::kTemporaryLabel) != 0)
					continue;
				groupSizes[group]++;
				g->GetNode(next)->SetLabelL(GraphSearchConstants::kTemporaryLabel, group);
				for (int y = 0; y < g->GetNode(next)->GetNumEdges(); y++)
				{
					edge *e = g->GetNode(next)->getEdge(y);
					if (g->GetNode(e->getFrom())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getFrom());
					if (g->GetNode(e->getTo())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getTo());
				}
			}
		}
	}
	int best = 0;
	for (unsigned int x = 1; x < groupSizes.size(); x++)
	{
		printf("%d states in group %d\n", groupSizes[x], x);
		if (groupSizes[x] > groupSizes[best])
			best = x;
	}
	printf("Keeping group %d\n", best);
	return best;
	//	kMapX;
}

void MeasureHighwayDimension(Map *m, int depth)
{
	Timer t;
	t.StartTimer();
	srandom(10);
	Graph *g = GraphSearchConstants::GetGraph(m);
	GraphEnvironment ge(g);
	ge.SetDirected(true);
	std::vector<graphState> endPath;
	ge.SetDirected(true);
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	srandom(t.EndTimer()*1000000 + time(0));
	// 1. choose a random point
	node *n = g->GetRandomNode();
	
	// 2. search to depth d (all g-costs >= d)
	TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
	theSearch.SetStopAfterGoal(false);
	theSearch.InitializeSearch(&ge, n->GetNum(), n->GetNum(), endPath);
	while (theSearch.GetNumOpenItems() > 0)
	{
		double gCost;
		graphState s = theSearch.CheckNextNode();
		if (theSearch.DoSingleSearchStep(endPath))
			break;
		assert(theSearch.GetClosedListGCost(s, gCost));
		if (gCost > depth)
			break;
	}
	
	// 3. mark all nodes on OPEN
	unsigned int radiusCount = theSearch.GetNumOpenItems();
	for (unsigned int x = 0; x < radiusCount; x++)
	{
		g->GetNode(theSearch.GetOpenItem(x).data)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 1);
	}

	// 4. continue search to depth 4d (all g-costs >= 4d)
	while (theSearch.GetNumOpenItems() > 0)
	{
		double gCost;
		graphState s = theSearch.CheckNextNode();
		if (theSearch.DoSingleSearchStep(endPath))
			break;
		assert(theSearch.GetClosedListGCost(s, gCost));
		if (gCost > 4*depth)
			break;
	}
	
	// 5. for every state on open, trace back to marked node
	//    (marking?)
	radiusCount = theSearch.GetNumOpenItems();
	for (unsigned int x = 0; x < radiusCount; x++)
	{
		graphState theNode = theSearch.GetOpenItem(x).data;
		theSearch.ExtractPathToStart(theNode, endPath);
		for (unsigned int y = 0; y < endPath.size(); y++)
		{
			if (g->GetNode(endPath[y])->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 1)
				g->GetNode(endPath[y])->SetLabelL(GraphSearchConstants::kTemporaryLabel, 2);
		}
	}

	int dimension = 0;
	// 6. count marked nodes to see how many were found
	for (int x = 0; x < g->GetNumNodes(); x++)
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 2)
			dimension++;

	printf("%d states at radius %d; %d states [highway dimension] at radius %d\n", radiusCount, depth, dimension, 4*depth);
}

void EstimateDimension(Map *m)
{
//	Graph *g = GraphSearchConstants::GetGraph(m);
//	GraphEnvironment ge(g);
//	std::vector<graphState> endPath;
//	node *n = g->GetRandomNode();
	Timer t;
	t.StartTimer();
	Graph *g = GraphSearchConstants::GetGraph(m);
	GraphEnvironment ge(g);
	ge.SetDirected(true);
	std::vector<graphState> endPath;
	srandom(t.EndTimer()*10000000 + time(0));
	
	// 1. choose a random point
	node *n = g->GetRandomNode();
	
	// 2. search to depth d (all g-costs >= d)
	double limit = 0;
	TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
	theSearch.SetStopAfterGoal(false);
	theSearch.InitializeSearch(&ge, n->GetNum(), n->GetNum(), endPath);
	while (theSearch.GetNumOpenItems() > 0)
	{
		double gCost = -1;

		graphState s = theSearch.CheckNextNode();
		if (theSearch.DoSingleSearchStep(endPath))
			break;
		theSearch.GetClosedListGCost(s, gCost);
		//printf("Expanding g-cost %f next\n", gCost);
		if (gCost >= limit)
		{
			printf("%d\t%d\n", (int)limit, theSearch.GetNodesExpanded());
			limit++;
		}

		//printf("%d items on open [%1.1f/%1.1f]\n", theSearch.GetNumOpenItems(), gCost, limit);
	}	
	graphState start = n->GetNum();
	BFS<graphState, graphMove, GraphEnvironment> b;
	b.GetPath(&ge, start, start, endPath);
}

double FindFarDist(Graph *g, node *n, graphState &from, graphState &to);

double EstimateLongPath(Map *m)
{
	Graph *g = GraphSearchConstants::GetGraph(m);
	GraphMapHeuristic gh(m, g);
	
	double heur = 0;
	double dist = 0;
	graphState from, to;
	for (int x = 0; x < 20; x++)
	{
		node *n = g->GetRandomNode();
		double newDist = FindFarDist(g, n, from, to);
		if (newDist > dist)
		{
			dist = newDist;
			heur = gh.HCost(from, to);
		}
		printf("%f\t%f\t%f\t%f\n", dist, dist/g->GetNumNodes(), heur, dist/heur);
	}
	return dist;
}

double FindFarDist(Graph *g, node *n, graphState &from, graphState &to)
{
	std::vector<graphState> endPath;
	GraphEnvironment ge(g);
	ge.SetDirected(true);
	// 2. search to depth d (all g-costs >= d)
	TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
	theSearch.SetStopAfterGoal(false);
	theSearch.InitializeSearch(&ge, n->GetNum(), n->GetNum(), endPath);
	double gCost;
	graphState s;
	while (1)
	{
		if (theSearch.GetNumOpenItems() == 0)
			break;
		s = theSearch.CheckNextNode();
		if (theSearch.DoSingleSearchStep(endPath))
			break;
		theSearch.GetClosedListGCost(s, gCost);
	}	
	theSearch.InitializeSearch(&ge, s, s, endPath);
	from = s;
	while (1)
	{
		if (theSearch.GetNumOpenItems() == 0)
			break;
		s = theSearch.CheckNextNode();
		if (theSearch.DoSingleSearchStep(endPath))
			break;
		to = s;
		double tmpCost;
		theSearch.GetClosedListGCost(s, tmpCost);
		if (tmpCost > gCost)
			gCost = tmpCost;
	}	
	return gCost;
}

void testHeuristic(char *problems)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	ScenarioLoader s(problems);
	Map *map = new Map(s.GetNthExperiment(0).GetMapName());
	map->Scale(s.GetNthExperiment(0).GetXScale(), s.GetNthExperiment(0).GetYScale());
	MapEnvironment e(map);
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetBucket() == 127)
		{
			xyLoc a, b;
			a.x = s.GetNthExperiment(x).GetStartX();
			a.y = s.GetNthExperiment(x).GetStartY();
			b.x = s.GetNthExperiment(x).GetGoalX();
			b.y = s.GetNthExperiment(x).GetGoalY();
			searcher.GetPath(&e, a, b, path);
			double len = e.GetPathLength(path);
			printf("%s Opt: %f (%f) heur: %f ratio: %f nodes: %d\n", s.GetNthExperiment(0).GetMapName(),
				   s.GetNthExperiment(x).GetDistance(), len, e.HCost(a, b),
				   e.HCost(a, b)/s.GetNthExperiment(x).GetDistance(), searcher.GetNodesExpanded());
		}
	}
	
	exit(0);
}
