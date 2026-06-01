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
#include "MapGenerators.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 128;
int gStepsPerFrame = 4;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;

MapSectorAbstraction *msa;

std::vector<xyLoc> path;

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
		map = new Map(mazeSize/2, mazeSize/2);
		MakeMaze(map, 1);
		map->Scale(mazeSize, mazeSize);
	}
	else {
		map = new Map(gDefaultMap);
		map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);
	msa = new MapSectorAbstraction(map, 8);
	msa->ToggleDrawAbstraction(1);
	msa->ToggleDrawAbstraction(2);
	//msa->ToggleDrawAbstraction(3);
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
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
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
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
	}
//	astar.OpenGLDraw();
	unitSims[windowID]->OpenGLDraw();
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
	
	if ((ma1) && (viewport == 0)) // only do this once...
	{
		ma1->SetColor(0.0, 0.5, 0.0, 0.75);
		if (gdh)
			gdh->OpenGLDraw();
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
		a1.OpenGLDraw();
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
						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
					break;
				}
			}
		}
		a2.OpenGLDraw();
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
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 4);
	//msa->ToggleDrawAbstraction(1);
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah2(msa, 2);
	GraphAbstractionHeuristic gah1(msa, 1);
	
	GraphRefinementEnvironment env2(msa, 2, &gah2);
	GraphRefinementEnvironment env1(msa, 1, &gah1);
	env1.SetDirected(false);
	env2.SetDirected(false);
	
	for (unsigned int x = 0; x < 10000; x++)
	{
		node *s1 = g->GetRandomNode();
		node *g1 = g->GetRandomNode();
		node *s2 = msa->GetParent(s1);
		node *g2 = msa->GetParent(g1);
		uint64_t nodesExpanded = 0;
		printf("%d\t%d\t",  s1->GetNum(), g1->GetNum());
		graphState gs1, gs2;
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		std::vector<graphState> thePath;
		
		astar.GetPath(&env1, gs1, gs2, thePath);
		printf("%d\n", (int)env1.GetPathLength(thePath));
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
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		//buildProblemSet();
		//doExport();
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
	else if (strcmp(argument[0], "-problems" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		runProblemSet(argument[1], atoi(argument[2]));
		return 3;
	}
	else if (strcmp(argument[0], "-problems2" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		runProblemSet2(argument[1], atoi(argument[2]));
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': if (gStepsPerFrame > 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
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
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long, tKeyboardModifier , char)
{
	static uint64_t average1=0, average2 = 0;
	static int count = 0;
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah2(msa, 2);
	GraphAbstractionHeuristic gah1(msa, 1);

	GraphRefinementEnvironment env2(msa, 2, &gah2);
	GraphRefinementEnvironment env1(msa, 1, &gah1);
	env1.SetDirected(false);
	env2.SetDirected(false);

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
		//		printf("Searching from %d to %d in level 1; %d to %d in level 2\n",
		//			   s1->GetNum(), g1->GetNum(), s2->GetNum(), g2->GetNum());
		graphState gs1, gs2;
		gs1 = s2->GetNum();
		gs2 = g2->GetNum();
		std::vector<graphState> thePath;
		std::vector<graphState> abstractPath;
		astar.GetPath(&env2, gs1, gs2, abstractPath);
		//printf("Abstract length %d\n", abstractPath.size());
		//		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env2.GetPathLength(abstractPath));
		if (abstractPath.size() == 0)
			break;

		nodesExpanded += astar.GetNodesExpanded();
		nodesTouched += astar.GetNodesTouched();
		
		env1.SetPlanningCorridor(abstractPath, 2);
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		astar.GetPath(&env1, gs1, gs2, thePath);
		printf("Full\t%llu\t%1.2f\t", astar.GetNodesExpanded(), env1.GetPathLength(thePath));
//		for (unsigned int x = 0; x < thePath.size(); x++)
//			printf("{%d}", thePath[x]);
		
		int abstractStart = 0;
		gs1 = s1->GetNum();
		double totalLength = 0;
		int refineAmt = 5;
		do { // not working yet -- fully check!
			env1.SetPlanningCorridor(abstractPath, 2, abstractStart);
			gs2 = g1->GetNum();
			if (abstractPath.size()-abstractStart > refineAmt)
			{
				env1.SetUseAbstractGoal(true, 2);
				gs2 = abstractPath[abstractStart+refineAmt];
				//printf("Using abstract goal of %d\n", abstractStart+refineAmt);
			}
			else {
				env1.SetUseAbstractGoal(false, 0);
			}
			astar.GetPath(&env1, gs1, gs2, thePath);
			abstractStart += refineAmt;
			gs1 = thePath.back();
			
//			for (unsigned int x = 0; x < thePath.size(); x++)
//				printf("(%d)", thePath[x]);
			
			nodesExpanded += astar.GetNodesExpanded();
			nodesTouched += astar.GetNodesTouched();
			totalLength += env1.GetPathLength(thePath);
			if (thePath.back() == gs2)
				break;
		} while (thePath.back() != g1->GetNum());
		
		printf("Part\t%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), totalLength);
		thePath.resize(0);
		printf("Tot\t%llu\t%llu\n", nodesExpanded, nodesTouched);
	}
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
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
					gdh = new GraphMapInconsistentHeuristic(ma1->GetMap(), GraphSearchConstants::GetGraph(ma1->GetMap()));
					gdh->SetPlacement(kAvoidPlacement);
					ma1->SetGraphHeuristic(gdh);
					for (int x = 0; x < 10; x++)
						gdh->AddHeuristic();
					((GraphMapInconsistentHeuristic*)gdh)->SetNumUsedHeuristics(10);
					((GraphMapInconsistentHeuristic*)gdh)->SetMode(kMax);
				}
				if (ma2 == 0)
					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
				
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				//a2.SetWeight(1.8);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
				
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
				a2.SetUseBPMX(true);
				runningSearch1 = true;
				runningSearch2 = true;
				
			}
				break;
		}
		return true;
	}
	return false;
}
