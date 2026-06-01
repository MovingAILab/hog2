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
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapOverlay.h"
#include <tr1/unordered_map>
#include "MapGenerators.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 200;
int gStepsPerFrame = 2;
double searchWeight = 0;
bool screenShot = false;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

std::vector<TemplateAStar<graphState, graphMove, GraphEnvironment> > astars;

EPEAStar<xyLoc, tDirection, MapEnvironment> a1;
//TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;

GraphEnvironment *ge = 0;

MapSectorAbstraction *msa;
MapOverlay *mo = 0;
MapOverlay *mo_g = 0;
MapOverlay *mo_h = 0;

std::vector<xyLoc> path;

Map *ReduceMap(Map *inputMap);
void MeasureHighwayDimension(Map *m, int depth);
void EstimateDimension(Map *m);
void EstimateLongPath(Map *m);

void testHeuristic(char *problems);

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
		//ht_chantry.arl.map
		//		map = new Map("/Users/nathanst/hog2/maps/dao/lak503d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.arl.map");
		
//		map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/local/weight.map");
		//map = new Map("weight.map");
		map = new Map(mazeSize, mazeSize);
		MakeMaze(map, 5);
//		map->Scale(512, 512);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);
	msa = new MapSectorAbstraction(map, 8);
	msa->ToggleDrawAbstraction(1);
	//msa->ToggleDrawAbstraction(2);
	// ->ToggleDrawAbstraction(3);
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Viewports", "Cycle number/which viewport (via shift)", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Compression", "Rotate Compression being shown in heuristic", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Displayed Heuristic", "Rotate which heuristic is shown", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
//	InstallCommandLineHandler(MyCLHandler, "-testHeuristic", "-testHeuristic scenario", "measure the ratio of the heuristic to the optimal dist");

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
	if (ge)
	{
		ge->OpenGLDraw();
		return;
	}
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
//	for (unsigned int x = 0; x < astars.size(); x++)
//		astars[x].OpenGLDraw();
	//	astar.OpenGLDraw();	
	unitSims[windowID]->OpenGLDraw();
	//unitSims[windowID]->GetEnvironment()->OpenGLDraw();
	
	if (screenShot)
	{
		SaveScreenshot(windowID, gDefaultMap);
		exit(0);
	}
//	glTranslatef(0, 0, -0.1);
//	glLineWidth(6.0);
//	if (ge == 0)
//		ge = new GraphEnvironment(GraphSearchConstants::GetGraph(unitSims[windowID]->GetEnvironment()->GetMap()));
//	ge->OpenGLDraw();

	if (astars.size() > 0)
		msa->OpenGLDraw();
	
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
	
	if ((gdh) && (!ma1))
		gdh->OpenGLDraw();

	if (viewport == 1)
		mo->OpenGLDraw();
	if (viewport == 2)
		mo_g->OpenGLDraw();
	if (viewport == 3)
		mo_h->OpenGLDraw();
	
	if ((ma1) && (viewport == 0)) // only do this once...
	{
		ma1->SetColor(0.0, 0.5, 0.0, 0.75);
		if (runningSearch1 && !unitSims[windowID]->GetPaused())
		{
			ma1->SetColor(0.0, 0.0, 1.0, 0.75);
			for (int x = 0; x < gStepsPerFrame*4; x++)
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
		a1.OpenGLDraw();
	}
	if (ma2)
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
		a2.OpenGLDraw();
	}

	if (0&&(runningSearch1 || runningSearch2 || mouseTracking) && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/epea/EPEA%d%d%d", (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}

void doExport()
{
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
	exit(0);
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
	ge->SetDirected(true);
	
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
		if (thePath.size() == 0)
			continue;
		
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


void runProblemSet2(char *problems, int multiplier)
{
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
}

void runProblemSet(char *problems, int multiplier)
{
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8, multiplier);

	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah2(msa, 2);
	GraphAbstractionHeuristic gah1(msa, 1);
	
	GraphRefinementEnvironment env2(msa, 2, &gah2);
	GraphRefinementEnvironment env1(msa, 1, &gah1);
	env1.SetDirected(false);
	env2.SetDirected(false);
	
	FILE *f = fopen(problems, "r");
	if (f == 0)
	{
		printf("Cannot open file: '%s'\n", problems);
		exit(0);
	}
	printf("len\tlvl2n\tlvl2nt\tlvl2len\tlvl2tim\tlvl1nf\tlvl1ntf\tlvl1tn\tlvl1tt\tlvl1len_f\ttot\ttott\ttot_len\n");
	Timer t;
	while (!feof(f))
	{
		int from, to, cost;
		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
			break;
		node *s1 = g->GetNode(from);
		node *g1 = g->GetNode(to);
		node *s2 = msa->GetParent(s1);
		node *g2 = msa->GetParent(g1);
		uint64_t nodesExpanded = 0;
		uint64_t nodesTouched = 0;
		double totalTime = 0;
		//		printf("Searching from %d to %d in level 1; %d to %d in level 2\n",
		//			   s1->GetNum(), g1->GetNum(), s2->GetNum(), g2->GetNum());
		graphState gs1, gs2;
		gs1 = s2->GetNum();
		gs2 = g2->GetNum();
		std::vector<graphState> thePath;
		std::vector<graphState> abstractPath;
		t.StartTimer();
		astar.GetPath(&env2, gs1, gs2, abstractPath);
		totalTime = t.EndTimer();
		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t%f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env2.GetPathLength(abstractPath), totalTime);
		if (abstractPath.size() == 0)
		{
			printf("%llu\t%llu\t%llu\t%llu\t%1.2f\t%f\t", (uint64_t)0, (uint64_t)0, astar.GetNodesExpanded(), astar.GetNodesTouched(), 0.0, 0.0);
			printf("%llu\t%llu\t%1.2f\t%f\t%d\t%d\n", astar.GetNodesExpanded(), astar.GetNodesTouched(), 0.0, 0.0, 0, 0);
//			printf("\n");
			continue;
		}

		nodesExpanded += astar.GetNodesExpanded();
		nodesTouched += astar.GetNodesTouched();

		env1.SetPlanningCorridor(abstractPath, 2);
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		t.StartTimer();
		astar.GetPath(&env1, gs1, gs2, thePath);
		t.EndTimer();
		printf("%llu\t%llu\t%llu\t%llu\t%1.2f\t%f\t",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   astar.GetNodesExpanded()+nodesExpanded, astar.GetNodesTouched()+nodesTouched,
			   env1.GetPathLength(thePath), totalTime+t.GetElapsedTime());
		
		int abstractStart = 0;
		gs1 = s1->GetNum();
		double totalLength = 0;
		int refineAmt = 2;
		int refinedPathNodes = 0;
		do { // not working yet -- fully check!
			env1.SetPlanningCorridor(abstractPath, 2, abstractStart);
			gs2 = g1->GetNum();
			if (abstractPath.size()-abstractStart > refineAmt)
			{
				env1.SetUseAbstractGoal(true, 2);
				gs2 = abstractPath[abstractStart+refineAmt];
			}
			else {
				env1.SetUseAbstractGoal(false, 0);
			}
			t.StartTimer();
			astar.GetPath(&env1, gs1, gs2, thePath);
			t.EndTimer();
			refinedPathNodes += thePath.size();
			totalTime+=t.GetElapsedTime();
			abstractStart += refineAmt;
			gs1 = thePath.back();
			
			nodesExpanded += astar.GetNodesExpanded();
			nodesTouched += astar.GetNodesTouched();
			totalLength += env1.GetPathLength(thePath);
			if (thePath.back() == gs2)
				break;
		} while (thePath.back() != g1->GetNum());
		
//		printf("%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env1.GetPathLength(thePath));
		thePath.resize(0);
		printf("%llu\t%llu\t%1.2f\t%f\t%d\t%d\n", nodesExpanded, nodesTouched, totalLength, totalTime, abstractPath.size(), refinedPathNodes);
		
//		gs1 = s1->GetNum();
//		gs2 = g1->GetNum();
//		env1.SetPlanningCorridor(abstractPath, 2);
//		astar.GetPath(&env1, gs1, gs2, thePath);
//		printf("%llu\t%1.2f\n", astar.GetNodesExpanded(), env1.GetPathLength(thePath));
	}
	fclose(f);
	exit(0);
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 0;
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
		case 'q':
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
				if (a2.DoSingleSearchStep(path))
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
}

#include "RubiksCubeEdges.h"
void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	RubikEdge e;
	RubikEdgeState es;
	// get a long sample of states/moves
	struct history {
		uint64_t state;
		int cycle;
	};
	std::tr1::unordered_map<uint64_t, int> hist;
	std::vector<history> path;
	std::vector<uint64_t> prob_cycle;
	std::vector<uint64_t> prob_count;
	// start at root (depth 0)
	es.Reset();
	for (int x = 0; x < 1000000; x++)
	{
		history h;
		h.state = e.rankPlayer(es, 0);
//		printf("Next: %llu\n", h.state);
		if (hist.find(h.state) != hist.end())
		{
			h.cycle = hist[h.state];
//			printf("Found at location %d\n", h.cycle);
//			if (h.cycle == x-1)
//				printf("Cycle length 1!\n");
			hist[h.state] = (int)path.size();
		}
		else {
			hist[h.state] = (int)path.size();
			h.cycle = -1;
		}
		path.push_back(h);
		e.ApplyAction(es, random()%18);
	}
	prob_cycle.resize(65);
	prob_count.resize(65);
	// stop at the end to avoid getting bogus data
	// or we can use the data if we don't do anything with paths that hit the end
	for (int x = 0; x < path.size()-65; x++)
	{
		// find longest path without cycle
		int length = 1;
		for (int y = x+1; y < path.size() && (y < x + 65); y++)
		{
			if (path[y].cycle < x)
				length++;
			else
				break;
		}
		if (length > 64)
			length = 64;
		for (int y = 0; y < length; y++)
			prob_count[y]++;
		prob_cycle[length]++;
	}
	for (int x = 0; x < prob_count.size(); x++)
	{
		printf("[%d] %llu / %llu = %f\n", x, prob_cycle[x], prob_count[x], (double)prob_cycle[x]/(double)prob_count[x]);
	}
	uint64_t numStates = 1;
	for (int x = 0; x < 32; x++)
	{
		printf("Depth %d; %llu states. [%llu of %llu are duplicates]\n", x, numStates, prob_cycle[x], prob_count[x]);
		double prob = 1.0;
		for (int y = 0; y <= x*2; y+=2)
		{
			// I think this is correct, since both contribute to the reduction
			prob *= 1-((double)prob_cycle[y]/(double)prob_count[y] + (double)prob_cycle[y+1]/(double)prob_count[y+1]);
//			prob *= 1-((double)prob_cycle[y]/(double)prob_count[y]);
//			prob *= 1-((double)prob_cycle[y+1]/(double)prob_count[y+1]);
		}
		numStates = (numStates*18.0*prob);
	}
	exit(0);
	// how many state are there at depth 1?
	// (measure branching factor; check the fraction of 1-step cycles)
	
	// how many states are tehre at depth 2?
	// (measure the branching factor; remove the fraction of 2-step cycles and the fraction of 1-step cycles not in 2-step cycles)
	
	// how many states are there at depth 3?
	// (depth 2) * (branching factor) * (percentage of unique 1-step, 2-step & 3-step cycles)
	
	// how many states at depth n?
	// (depth n-1) * branching factor * ∑ (percentage of unique i-step paths without cycles)
}

void Predict(Map *m, int x1, int y1, int x2, int y2)
{
	if (mo == 0)
		mo = new MapOverlay(m);
	mo->Clear();
	if (mo_g == 0)
		mo_g = new MapOverlay(m);
	mo_g->Clear();
	if (mo_h == 0)
		mo_h = new MapOverlay(m);
	mo_h->Clear();
	
	MapEnvironment me(m);
	me.SetDiagonalCost(1.5);
	if (ma2 == 0)
		ma2 = new MapEnvironment(m);
	ma2->SetDiagonalCost(1.5);
	TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;

	std::vector<int> fCostDistribution;
	std::vector<int> gCostDistribution;
	std::vector<int> hCostDistribution;
	
	xyLoc start(x1, y1), goal(x2, y2);
	std::vector<xyLoc> path;
	
	double hCost = 2*me.HCost(start, goal);
	double gCost;
	
	a1.SetStopAfterGoal(false);
	a1.GetPath(&me, start, goal, path);

	a2.GetPath(ma2, start, goal, path);
	gCost = ma2->GetPathLength(path);
	printf("Actual nodes expanded: %llu\n", a2.GetNodesExpanded());
	printf("Actual depth         : %1.2f\n", gCost);
	
	for (int x = 0; x < a1.GetNumItems(); x++)
	{
		xyLoc l = a1.GetItem(x).data;
		double f = 2*(a1.GetItem(x).g+a1.GetItem(x).h);
		double g = 2*a1.GetItem(x).g;
		double h = 2*a1.GetItem(x).h;//me.HCost(a1.GetItem(x).data, goal);
		if (g >= gCostDistribution.size())
		{
			gCostDistribution.resize(g+1);
		}
		gCostDistribution[g]++;
		if (h >= hCostDistribution.size())
		{
			hCostDistribution.resize(h+1);
		}
		hCostDistribution[h]++;
		if (f >= fCostDistribution.size())
		{
			fCostDistribution.resize(f+1);
		}
		fCostDistribution[f]++;
		mo->SetOverlayValue(l.x, l.y, f);
		mo_g->SetOverlayValue(l.x, l.y, g);
		mo_h->SetOverlayValue(l.x, l.y, h);
	}
	for (unsigned int x = 1; x < hCostDistribution.size(); x++)
	{
		hCostDistribution[x] += hCostDistribution[x-1];
	}
	for (unsigned int x = 1; x < fCostDistribution.size(); x++)
	{
		fCostDistribution[x] += fCostDistribution[x-1];
	}
	
	printf("Predicted node expansions: (%d states in the map)\n", a1.GetNumItems());
	printf("Depth\tN(i)\tp(...)\ttot\tactual(?)\n");
	for (int x = hCost; x <= gCost*2; x++)
	{
		double a = (x-hCost)/2.0;
		double ab = (x-hCost-1)/2.0;
		printf("%5.1f\t%5d\t", 2.0*hCost*a+a*a, static_cast<int>(2.0*hCost*a+a*a-2.0*hCost*ab-ab*ab));
		printf("%5.1f\t%5d\t%5.2f\t%5.3f\t%5d\t%5d\n", x/2.0, gCostDistribution[x], hCostDistribution[x]/static_cast<double>(a1.GetNumItems()),
			   (gCostDistribution[x]*hCostDistribution[x])/static_cast<double>(a1.GetNumItems()),
			   fCostDistribution[x], fCostDistribution[x]-fCostDistribution[x-1]);
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
	static point3d startLoc;
//	switch (button)
//	{
//		case kRightButton: printf("Right button\n"); break;
//		case kLeftButton: printf("Left button\n"); break;
//		case kMiddleButton: printf("Middle button\n"); break;
//	}
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
				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				Predict(unitSims[windowID]->GetEnvironment()->GetMap(), px1, py1, px2, py2);
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
	srandom(10);
	Graph *g = GraphSearchConstants::GetGraph(m);
	GraphEnvironment ge(g);
	ge.SetDirected(true);
	std::vector<graphState> endPath;
	
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	
	// 1. choose a random point
	node *n = g->GetRandomNode();
	
	// 2. search to depth d (all g-costs >= d)
	TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
	theSearch.SetStopAfterGoal(false);
	theSearch.InitializeSearch(&ge, n->GetNum(), n->GetNum(), endPath);
	while (1)
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
	while (1)
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
	Graph *g = GraphSearchConstants::GetGraph(m);
	GraphEnvironment ge(g);
	std::vector<graphState> endPath;
	
	// 1. choose a random point
	node *n = g->GetRandomNode();
	
	// 2. search to depth d (all g-costs >= d)
	double limit = 0;
	TemplateAStar<graphState, graphMove, GraphEnvironment> theSearch;
	theSearch.SetStopAfterGoal(false);
	theSearch.InitializeSearch(&ge, n->GetNum(), n->GetNum(), endPath);
	while (1)
	{
		double gCost;
		graphState s = theSearch.CheckNextNode();
		theSearch.GetClosedListGCost(s, gCost);
		//printf("Expanding g-cost %f next\n", gCost);
		if (gCost >= limit)
		{
			printf("%d\t%d\n", (int)limit, theSearch.GetNodesExpanded());
			limit++;
		}

		if (theSearch.DoSingleSearchStep(endPath))
			break;
	}	
	graphState start = n->GetNum();
	BFS<graphState, graphMove> b;
	b.GetPath(&ge, start, start, endPath);
}

double FindFarDist(Graph *g, node *n, graphState &from, graphState &to);

void EstimateLongPath(Map *m)
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
			printf("Opt: %f (%f) heur: %f ratio: %f nodes: %d\n", s.GetNthExperiment(x).GetDistance(), len, e.HCost(a, b),
				   e.HCost(a, b)/s.GetNthExperiment(x).GetDistance(), searcher.GetNodesExpanded());
		}
	}
	
	exit(0);
}

