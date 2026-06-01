
/*
 *  $Id: Inconsistency.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Inconsistency.h"
#include "SearchUnit.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "Propagation.h"
#include "GraphEnvironment.h"
#include "GraphAlgorithm.h"
#include "AStarDelay.h"
#include "ScenarioLoader.h"
#include "TemplateAStar.h"

Graph *BuildInconsistentGraph(int d, int w, int h, int c1 = 1000, int c2 = 1000);
void RunInconsistencyExperiment();
void RunExperiments(ScenarioLoader *sl, int memory);

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
//int GraphSizeN = 10;

std::vector<GraphSimulation *> unitSims;

//extern bool drawtext;

unsigned int fig = 4;
unsigned int N = 5;
unsigned int vid = PROP_BP;//6;//PROP_A;//

double delta = 0;

graphState gFrom, gTo;
std::vector<graphState> gPath;
Graph* grp=0;
Map* mp=0;
GraphEnvironment* env = 0;

GraphAlgorithm* ALG = 0;

bool done = false;

TemplateAStar<graphState, graphMove, GraphEnvironment> vis1, vis2;
bool v1 = false, v2 = false;
bool v1r = false, v2r = false;

int main(int argc, char* argv[])
{
	preProcessArgs(argc,argv);
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


void preProcessArgs(int argc, char* argv[]) 
{
	// -fig fig -N n -v vid -delta del -map mapfile
	int i = 1;
	gDefaultMap[0] = 0;
	while(i<argc) {
		if(strcmp(argv[i],"-fig")==0) {
			fig = atoi(argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-N")==0) {
			N = atoi(argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-v")==0) {
			vid = atoi(argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-delta")==0) {
			sscanf(argv[i+1],"%lf",&delta);
			i += 2;
		}
		else if(strcmp(argv[i],"-map")==0) {
			strcpy(gDefaultMap,argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-dt")==0) {
		  //	drawtext = true;
			i++;
		}
		else
			i++;
	}

	if(strlen(gDefaultMap)) 
	{
		FILE * mf = fopen(gDefaultMap,"r");
		if(!mf)
		{
			printf("Map file not exists.\n");
			exit(-1);
		}
		else
		{
			fclose(mf);
		}
	}
}


Graph *BuildInconsistentGraph(int d, int w, int h, int c1, int c2)
{
	//int d = 2;
	gFrom = d*w*h-1;
	gTo = 0;
	
	Graph *g = new Graph();
	int cost[d][w][h];
	int index[d][w][h];
	for (int z = 0; z < d; z++)
	{
		for (int x = 0; x < w; x++)
		{
			for (int y = 0; y < h; y++)
			{
				node *n;
				cost[z][x][y] = MAXINT;
				index[z][x][y] = g->AddNode(n = new node(""));
				n->SetLabelF(GraphSearchConstants::kXCoordinate, -1.0+2.0*(double)x/(double)(w-1.0));
				n->SetLabelF(GraphSearchConstants::kYCoordinate, -1.0+2.0*(double)y/(double)(h-1.0));
				if (d > 1)
					n->SetLabelF(GraphSearchConstants::kZCoordinate, -1.0+2.0*(double)z/(double)(d-1.0));
				else
					n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
			}
		}
	}
	for (int z = 0; z < d; z++)
	{
		for (int x = 0; x < w; x++)
		{
			for (int y = 0; y < h; y++)
			{
				if ((x == 0) && (y == 0) && (z == 0))
				{
					if (d > 1)
						g->AddEdge(new edge(index[z+1][x][y], index[z][x][y], c2));
					if (w > 1)
						g->AddEdge(new edge(index[z][x+1][y], index[z][x][y], c2));
					g->AddEdge(new edge(index[z][x][y+1], index[z][x][y], c2));
					continue;
				}
				if (z+1 <d)
					g->AddEdge(new edge(index[z+1][x][y], index[z][x][y], 1+random()%c1));
				if (x+1 <w)
					g->AddEdge(new edge(index[z][x+1][y], index[z][x][y], 1+random()%c1));
				if (y+1 < h)
					g->AddEdge(new edge(index[z][x][y+1], index[z][x][y], 1+random()%c1));
			}
		}
	}
	cost[0][0][0] = 0;
	while (1)
	{
		int changes = 0;

		for (int z = 0; z < d; z++)
		{
			for (int x = 0; x < w; x++)
			{
				for (int y = 0; y < h; y++)
				{
					if (z+1 <d)
					{
						int from1 = index[z][x][y];
						int to1 = index[z+1][x][y];
						edge *e = g->FindEdge(from1, to1);
						if (cost[z+1][x][y] > cost[z][x][y] + e->GetWeight())
						{
							cost[z+1][x][y] = cost[z][x][y] + g->FindEdge(index[z][x][y], index[z+1][x][y])->GetWeight();
							changes++;
						}
					}
					if (x+1 <w)
					{
						if (cost[z][x+1][y] > cost[z][x][y] + g->FindEdge(index[z][x][y], index[z][x+1][y])->GetWeight())
						{
							cost[z][x+1][y] = cost[z][x][y] + g->FindEdge(index[z][x][y], index[z][x+1][y])->GetWeight();
							changes++;
						}
					}
					if (y+1 < h)
					{
						if (cost[z][x][y+1] > cost[z][x][y] + g->FindEdge(index[z][x][y], index[z][x][y+1])->GetWeight())
						{
							cost[z][x][y+1] = cost[z][x][y] + g->FindEdge(index[z][x][y], index[z][x][y+1])->GetWeight();
							changes++;
						}
					}
				}
			}
		}

		if (changes == 0)
			break;
	}
	
	for (int z = 0; z < d; z++)
	{
		for (int x = 0; x < w; x++)
		{
			for (int y = 0; y < h; y++)
			{
				//printf("cost of node %d (%d, %d) is %d\n", x, y, index[x][y], cost[x][y]);
				node *n = g->GetNode(index[z][x][y]);
				if (cost[z][x][y] == 0)
					n->SetLabelF(GraphSearchConstants::kHCost, 0);
				else {
					//int val = ((random()%4) == 0)?(random()%(cost[z][x][y]+1)):0;
					int val = random()%(cost[z][x][y]+1);
					//printf("Assigning h = %d\n", val);
					n->SetLabelF(GraphSearchConstants::kHCost, (double)val);
				}
			}
		}
	}
	return g;
}

/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	//Graph *g;

	if (gDefaultMap[0] != 0)
	{
		mp = new Map(gDefaultMap);
		mp->Scale(512, 512);
		grp = GraphSearchConstants::GetGraph(mp);
		env = new GraphEnvironment(mp, grp, new GraphMapInconsistentHeuristic(mp, grp));
		
		while(gFrom==gTo) {
			gFrom = grp->GetRandomNode()->GetNum();
			gTo = grp->GetRandomNode()->GetNum();
		}
	}
	else if (fig == 0)
	{
		grp = BuildInconsistentGraph(5, N, N);
//		gFrom = 5*N*N-1;
//		to = 0;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, gTo));
		env->SetDirected(true);
	}
	else if (fig == 1) 
	{
		grp = PropUtil::graphGenerator::genFig1(N);
		gFrom = N;
		gTo = 0;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, gTo));
	}
	else if(fig == 2)
	{
		grp = PropUtil::graphGenerator::genFig2(N);
		gFrom = N;
		gTo = 0;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, gTo));
	}	
	else if (fig == 3){
		grp = PropUtil::graphGenerator::genFig3(N);
		gFrom = 0;
		gTo = 2*N - 1;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, gTo));
	}
	else {
		grp = PropUtil::graphGenerator::genFig4(N);
		gFrom = 0;
		gTo = 2*N + 1;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, gTo));
	}
	
	printf("Environment ready.\n");

	unitSims.resize(id+1);
	unitSims[id] = new GraphSimulation(env);
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
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
	InstallKeyboardHandler(MyDisplayHandler, "Non-interactive mode", "Run all 5 algorithms using the same environment.", kNoModifier, 'k');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-scenario", "-scenario filename memory", "Runs a specified scenario file with the memory limit.");
	//InstallCommandLineHandler(MyCLHandler, "-numHeuristics", "-numHeuristics <N>", "Uses the max of N diff heuristics.");
	
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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;

	if (viewport == 0)
	{
//		unitSims[windowID]->StepTime(1.0/30.0);
//		if ((!unitSims[windowID]->GetPaused()) && CFOR)
//			CFOR->DoOneSearchStep();
//
//		if (CFOR)
//		{
//			CFOR->OpenGLDraw();
//		}
		unitSims[windowID]->StepTime(1.0/30.0);
		//if((!unitSims[windowID]->GetPaused()) && ALG)
		//	ALG->DoSingleSearchStep(gPath);

		if(ALG)
		{
			ALG->OpenGLDraw();
		}
	}
	
	static std::vector<graphState> ourPath;
	if (viewport == 0)
	{
		if (v1r)
		{
			printf("Stepping search 1: f:%f g:%f, h:%f\n",
				   vis1.GetOpenItem(0).g + vis1.GetOpenItem(0).h,
				   vis1.GetOpenItem(0).g, vis1.GetOpenItem(0).h);
			for (int x = 0; x < 10 && v1r; x++)
				v1r = !vis1.DoSingleSearchStep(ourPath);
			
		}
		if (v1)
			vis1.OpenGLDraw();
	}
	else {
		if (v2r)
		{
			printf("Stepping search 2: f:%f g:%f, h:%f\n",
				   vis2.GetOpenItem(0).g + vis2.GetOpenItem(0).h,
				   vis2.GetOpenItem(0).g, vis2.GetOpenItem(0).h);
			for (int x = 0; x < 10 && v2r; x++)
				v2r = !vis2.DoSingleSearchStep(ourPath);
		}
		if (v2)
			vis2.OpenGLDraw();
	}

	unitSims[windowID]->OpenGLDraw();
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
//	if (strcmp(argument[0], "-numHeuristics") == 0)
//		numHeuristics = atoi(argument[1]);
	if (strcmp(argument[0], "-scenario") == 0)
		RunExperiments(new ScenarioLoader(argument[1]), atoi(argument[2]));
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
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
			if (ALG == 0) 
			{
				if (vid != 6)
					ALG = new Prop(vid,delta);
				else
					ALG = new AStarDelay();
				ALG->InitializeSearch(env,grp,gFrom,gTo,gPath);
			}

			if(!done && ALG->DoSingleSearchStep(gPath)) 
			{
				printf("\nDone! Nodes expanded=%lld, Nodes touched=%lld, Reopenings=%lld.\n",ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetNodesReopened());
				printf("Algorithm %s, solution cost=%lf, solution edges=%d.\n", ALG->GetName(),ALG->GetSolutionCost(),ALG->GetSolutionEdges());
				
				done = true;
			}



		}
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
			break;
		case 'k':
			RunInconsistencyExperiment();
			break;
			ALG = new Prop(0);
			ALG->GetPath(env,grp,gFrom,gTo,gPath);
			printf("\n");

			ALG = new Prop(1);
			ALG->GetPath(env,grp,gFrom,gTo,gPath);
			printf("\n");

			ALG = new Prop(2,delta);
			ALG->GetPath(env,grp,gFrom,gTo,gPath);
			printf("\n");

//			ALG = new Prop(3);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");

//			ALG = new Prop(4);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");

//			ALG = new Prop(5);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");

			ALG = new AStarDelay();
			ALG->GetPath(env,grp,gFrom,gTo,gPath);
			printf("Delay\t%lld\t%lld\t%lld\n",ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetNodesReopened());

//			ALG = new Prop(7);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");
//
//			ALG = new Prop(8);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");
//
//			ALG = new Prop(9);
//			ALG->GetPath(env,grp,gFrom,gTo,gPath);
//			printf("\n");

			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			// set N to be whatever size I press
			//GraphSizeN = key-'0';
			//if (GraphSizeN == 0)
			//	GraphSizeN = 10;
			break;
	}
}

// 'a'
void MyRandomUnitKeyHandler(unsigned long , tKeyboardModifier , char)
{

	GraphMapInconsistentHeuristic diffHeuristic(mp, grp);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	for (int x = 0; x < 1; x++)
		diffHeuristic.AddHeuristic();
	diffHeuristic.SetNumUsedHeuristics(1);
	double sum = 0;
	uint64_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;
	for (int x = 0; x < 100; x++)
	{
		graphState n1 = grp->GetRandomNode()->GetNum();
		graphState n2 = grp->GetRandomNode()->GetNum();
		
		double a, b;
		
//		diffHeuristic.SetMode(kIgnore);
//		printf("Max: %f\t", a = diffHeuristic.HCost(n1, n2));
//		diffHeuristic.SetMode(kCompressed);
//		printf("Compressed: %f\t", b = diffHeuristic.HCost(n1, n2));
//		printf("Error: %f\n", a-b);
//		assert(a >= b);
//		sum += a-b;
//		printf("Average error: %f\n", sum/(x+1));
		GraphEnvironment ge(mp, grp, &diffHeuristic);
		ge.SetDirected(true);
		diffHeuristic.SetMode(kIgnore);
		vis1.GetPath(&ge, n1, n2, gPath);
		printf("%lld\t%1.1f\t", vis1.GetNodesExpanded(), ge.GetPathLength(gPath));
		s1+=vis1.GetNodesExpanded();
		diffHeuristic.SetMode(kCompressed);
		vis1.SetUseBPMX(1);
		vis1.GetPath(&ge, n1, n2, gPath);
		printf("%lld\t%1.1f\t", vis1.GetNodesExpanded(), ge.GetPathLength(gPath));
		s2+=vis1.GetNodesExpanded();
		vis1.SetUseBPMX(2);
		vis1.GetPath(&ge, n1, n2, gPath);
		printf("%lld\t%1.1f\t", vis1.GetNodesExpanded(), ge.GetPathLength(gPath));
		s3+=vis1.GetNodesExpanded();
		vis1.SetUseBPMX(3);
		vis1.GetPath(&ge, n1, n2, gPath);
		printf("%lld\t%1.1f\n", vis1.GetNodesExpanded(), ge.GetPathLength(gPath));
		s4+=vis1.GetNodesExpanded();
	}
	printf("%10lld %10lld %10lld %10lld\n", s1, s2, s3, s4);
}

// letter d
void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	//115 maps/bgmaps/AR0011SR.map 512 512 190 61 477 321 462.65
	
	graphState s1 = mp->GetNodeNum(190, 61);//grp->GetRandomNode()->GetNum(); //
	graphState g1 = mp->GetNodeNum(477, 321);//grp->GetRandomNode()->GetNum(); //
	
	
	GraphMapInconsistentHeuristic *diffHeuristic1 = new GraphMapInconsistentHeuristic(mp, grp);
	diffHeuristic1->SetPlacement(kAvoidPlacement);
//	diffHeuristic1->SetMode(kRandom);
	GraphEnvironment *gEnv1 = new GraphEnvironment(mp, grp, diffHeuristic1);
	gEnv1->SetDirected(true);
	for (int x = 0; x < 10; x++)
		diffHeuristic1->AddHeuristic();
	diffHeuristic1->SetNumUsedHeuristics(10);
	diffHeuristic1->SetMode(kMax);
	vis1.SetUseBPMX(1);
	
	GraphMapInconsistentHeuristic *diffHeuristic2 = new GraphMapInconsistentHeuristic(mp, grp);
	diffHeuristic2->SetPlacement(kAvoidPlacement);
//	diffHeuristic2->SetMode(kRandom);
	GraphEnvironment *gEnv2 = new GraphEnvironment(mp, grp, diffHeuristic2);
	gEnv2->SetDirected(true);
	for (int x = 0; x < 80; x++)
		diffHeuristic2->AddHeuristic();
	diffHeuristic2->SetNumUsedHeuristics(8);
	diffHeuristic2->SetMode(kCompressed);
	diffHeuristic2->Compress();
	vis2.SetUseBPMX(1);

//	std::vector<double> test;
//	printf("Benchmarking\n", s1, g1);
//	int total = grp->GetNumNodes();
//	if (total > 30000) total = 30000;
//	
//	for (graphState x = 0; x < total; x+=13)
//	{
//		if ((x%10000) == 0)
//			printf("%d ", x);
//		for (graphState y = 0; y < total; y++)
//		{
//			test.push_back(diffHeuristic2->HCost(y, x));
//		}
//		//printf("%f\n", diffHeuristic2->HCost(x, s1));
//	}
//	printf("Compressing\n");
//	diffHeuristic2->Compress();
//	printf("Testing:\n");
//	int cnt = 0;
//	for (graphState x = 0; x < total; x+=13)
//	{
//		if ((x%10000) == 0)
//			printf("%d ", x);
//		for (graphState y = 0; y < total; y++)
//		{
//			assert(test[cnt] == diffHeuristic2->HCost(y, x));
//			cnt++;
//		}
//		//printf("%f\n", diffHeuristic2->HCost(x, s1));
//	}
//	printf("Compression verified on %d problems\n", cnt);
	
	vis1.InitializeSearch(gEnv1, s1, g1, gPath);
	vis2.InitializeSearch(gEnv2, s1, g1, gPath);
	v1=true;v1r = true;
	v2=true;v2r = true;
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
//	mouseTracking = false;
//	if (button == kRightButton)
//	{
//		switch (mType)
//		{
//			case kMouseDown:
//				unitSim->GetMap()->GetPointFromCoordinate(loc, px1, py1);
//				//printf("Mouse down at (%d, %d)\n", px1, py1);
//				break;
//			case kMouseDrag:
//				mouseTracking = true;
//				unitSim->GetMap()->GetPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
//				break;
//			case kMouseUp:
//			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				unitSim->GetMap()->GetPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse up at (%d, %d)\n", px2, py2);
//				unit *u, *u2 = new unit(px2, py2, 0);
//				//praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//				aStar *pra = new aStar();
//				unitSim->addUnit(u2);
//				u = new SearchUnit(px1, py1, u2, pra);
//				unitSim->addUnit(u);
//				u->setSpeed(0.5); // time to go 1 distance						
//			}
//			break;
//		}
//		return true;
//	}
//	return false;
}

void RunInconsistencyExperiment()
{
	printf("Running experiment\n");
	ALG = new Prop(0);
	GraphAlgorithm *asd = new AStarDelay();

	for (int size = 50; size <= 50; size++)
	{
		for (int cost = 50; cost <= 10000; cost *= 2)
		{
			for (int count = 0; count < 100; count++)
			{
				fflush(stdout);
				Graph *g = BuildInconsistentGraph(size, size, size, 50, cost);
				env = new GraphEnvironment(g, new GraphLabelHeuristic(g, gTo));
				env->SetDirected(true);
				
				//((Prop*)ALG)->SetAlgorithm(PROP_A);
				((Prop*)ALG)->verID = PROP_A;
				ALG->GetPath(env,g,gFrom,gTo,gPath);
				printf("A\t%lld\t%lld\t%lld\t%d\t%d\n",
					   ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetNodesReopened(),
					   size, cost);
				
				//((Prop*)ALG)->SetAlgorithm(PROP_B);
				((Prop*)ALG)->verID = PROP_B;

				ALG->GetPath(env,g,gFrom,gTo,gPath);
				printf("B\t%lld\t%lld\t%lld\t%d\t%d\n",
					   ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetNodesReopened(),
					   size, cost);
				
				//((Prop*)ALG)->SetAlgorithm(PROP_BP);
				((Prop*)ALG)->verID = PROP_BP;
				ALG->GetPath(env,g,gFrom,gTo,gPath);
				printf("P\t%lld\t%lld\t%lld\t%d\t%d\n",
					   ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetNodesReopened(),
					   size, cost);
				
				asd->GetPath(env,g,gFrom,gTo,gPath);
				printf("D\t%lld\t%lld\t%lld\t%d\t%d\n",
					   asd->GetNodesExpanded(),asd->GetNodesTouched(),asd->GetNodesReopened(),
					   size, cost);

				delete env;
			}
		}
	}
	ALG = 0;
}

// experiments with 0, 10, ..., 100 differential heuristics
void RunExperiments1(ScenarioLoader *sl)
{
	std::vector<graphState> aPath;

	Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
	m->Scale(sl->GetNthExperiment(0).GetXScale(), 
			 sl->GetNthExperiment(0).GetYScale());
	Graph *g = GraphSearchConstants::GetGraph(m);
	
	GraphDistanceHeuristic diffHeuristic(g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);
	
	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;

	
	for (int z = 0; z <= 10; z++)
	{
		for (int x = 0; x < sl->GetNumExperiments(); x++)
		{
			Experiment e = sl->GetNthExperiment(x);

			graphState start, goal;
			start = m->GetNodeNum(e.GetStartX(), e.GetStartY());
			goal = m->GetNodeNum(e.GetGoalX(), e.GetGoalY());

			Timer t;
			t.StartTimer();
			taNew.GetPath(&gEnv, start, goal, aPath);
			t.EndTimer();

			printf("%d\t%d\t%lld\t%f\n", e.GetBucket(), diffHeuristic.GetNumHeuristics(), taNew.GetNodesExpanded(), t.GetElapsedTime());
		}
		for (int x = 0; x < 10; x++)
			diffHeuristic.AddHeuristic();
	}
	exit(0);
}
// Compare:
// * 10 N memory
// * 100 compressed heuristics
void RunExperiments(ScenarioLoader *sl, int memory)
{
	std::vector<graphState> aPath;
	
	Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
	m->Scale(sl->GetNthExperiment(0).GetXScale(), 
			 sl->GetNthExperiment(0).GetYScale());
	Graph *g = GraphSearchConstants::GetGraph(m);
	
	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	GraphMapInconsistentHeuristic diff1(m, g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	diffHeuristic.SetMode(kRandom);
	diff1.SetPlacement(kAvoidPlacement);
	diff1.SetMode(kMax);
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);
	GraphEnvironment gEnv2(g, &diff1);
	gEnv2.SetDirected(true);
	
	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;
	
	Timer t;
	
	for (int x = 0; x < memory; x++)
		diffHeuristic.AddHeuristic();
	diffHeuristic.SetNumUsedHeuristics(memory);
	diffHeuristic.Compress();
	diff1.AddHeuristic();
	diff1.SetNumUsedHeuristics(1);
	
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		
//		if (e.GetBucket() < 100)
//			continue;
		
		graphState start, goal;
		start = m->GetNodeNum(e.GetStartX(), e.GetStartY());
		goal = m->GetNodeNum(e.GetGoalX(), e.GetGoalY());
		
		
		printf("%d\t", e.GetBucket());
		
		
		// 10N memory -- 10 heuristics
//		diffHeuristic.SetNumUsedHeuristics(1);
//		diffHeuristic.SetMode(kMax);
//		taNew.SetUseBPMX(0);
//			
//		t.StartTimer();
//		taNew.GetPath(&gEnv, start, goal, aPath);
//		t.EndTimer();		
//		printf("%dmx\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
//			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
//		diffHeuristic.SetNumUsedHeuristics(memory);
//		diffHeuristic.SetMode(kCompressed);
//		taNew.SetUseBPMX(0);
		
//		t.StartTimer();
//		taNew.GetPath(&gEnv, start, goal, aPath);
//		t.EndTimer();		
//		printf("%dcmp\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
//			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
//		diffHeuristic.SetNumUsedHeuristics(memory);
//		diffHeuristic.SetMode(kCompressed);
		taNew.SetUseBPMX(0);
		
		t.StartTimer();
		taNew.GetPath(&gEnv2, start, goal, aPath);
		t.EndTimer();		
		printf("%dbx1\t%lld\t%f\t%f\t", diff1.GetNumUsedHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
		taNew.SetUseBPMX(0);
		
		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%dbx1\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));

		taNew.SetUseBPMX(1);
		
		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%dbx1\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
//		diffHeuristic.SetNumUsedHeuristics(memory);
//		diffHeuristic.SetMode(kCompressed);
//		taNew.SetUseBPMX(1000);
//		
//		t.StartTimer();
//		taNew.GetPath(&gEnv, start, goal, aPath);
//		t.EndTimer();		
//		printf("%dbxi\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
//			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
		printf("\n");
		fflush(stdout);
	}
	exit(0);
}

// Compare:
// * 10 N memory
// * 1...10 regular lookups
// * 1...9 random lookups with and without BPMX
// Only compare on longest problems
void RunExperiments5(ScenarioLoader *sl)
{
	std::vector<graphState> aPath;
	
	Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
	m->Scale(sl->GetNthExperiment(0).GetXScale(), 
			 sl->GetNthExperiment(0).GetYScale());
	Graph *g = GraphSearchConstants::GetGraph(m);
	
	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	diffHeuristic.SetMode(kRandom);
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);
	
	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;
	
	Timer t;
	
	for (int x = 0; x < 10; x++)
		diffHeuristic.AddHeuristic();
	//diffHeuristic.SetNumUsedHeuristics(diffHeuristic.GetNumHeuristics()/10);
	
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		
		if (e.GetBucket() != 127)
			continue;
		
		graphState start, goal;
		start = m->GetNodeNum(e.GetStartX(), e.GetStartY());
		goal = m->GetNodeNum(e.GetGoalX(), e.GetGoalY());
		
		
		printf("%d\t", e.GetBucket());
		
		
		for (int y = 1; y <= 10; y++)
		{
			// N memory -- 1 heuristic
			diffHeuristic.SetNumUsedHeuristics(y);
			diffHeuristic.SetMode(kMax);
			taNew.SetUseBPMX(0);
			
			t.StartTimer();
			taNew.GetPath(&gEnv, start, goal, aPath);
			t.EndTimer();		
			printf("%dmx\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
				   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		}
		
		for (int y = 1; y <= 9; y++)
		{
			for (int z = 0; z <= 1; z++)
			{
				// N memory -- 1 heuristic
				diffHeuristic.SetNumUsedHeuristics(y);
				diffHeuristic.SetMode(kRandom);
				taNew.SetUseBPMX(z);
				
				t.StartTimer();
				taNew.GetPath(&gEnv, start, goal, aPath);
				t.EndTimer();	
				if (z==0)
					printf("%drnd\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
						   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
				else
					printf("%drdb\t%lld\t%f\t%f\t", diffHeuristic.GetNumUsedHeuristics(),
						   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
			}
		}
		printf("\n");
		
	}
	exit(0);
}


// Compare:
// * 1 random lookup of 10 with BPMX
// * 1 random lookup of 10 without BPMX
// * octile
// * max of 10 heuristics
// * 1 lookup in compressed heuristic
void RunExperiments4(ScenarioLoader *sl)
{
	std::vector<graphState> aPath;
	
	Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
	m->Scale(sl->GetNthExperiment(0).GetXScale(), 
			 sl->GetNthExperiment(0).GetYScale());
	Graph *g = GraphSearchConstants::GetGraph(m);
	
	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	diffHeuristic.SetMode(kRandom);
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);
	
	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;

	Timer t;
	
	for (int x = 0; x < 10; x++)
		diffHeuristic.AddHeuristic();
	//diffHeuristic.SetNumUsedHeuristics(diffHeuristic.GetNumHeuristics()/10);

	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		
		graphState start, goal;
		start = m->GetNodeNum(e.GetStartX(), e.GetStartY());
		goal = m->GetNodeNum(e.GetGoalX(), e.GetGoalY());
	
		
		printf("%d\t", e.GetBucket());
		
		// N memory -- 1 heuristic
		diffHeuristic.SetNumUsedHeuristics(1);
		diffHeuristic.SetMode(kMax);
		taNew.SetUseBPMX(0);

		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%df\t%lld\t%f\t%f\t", diffHeuristic.GetNumHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
		// N memory -- 10 compressed heuristics no BPMX
		diffHeuristic.SetNumUsedHeuristics(10);
		diffHeuristic.SetMode(kCompressed);
		taNew.SetUseBPMX(0);

		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%dc\t%lld\t%f\t%f\t", diffHeuristic.GetNumHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
		// N memory -- 10 compressed heuristics BPMX 1
		diffHeuristic.SetNumUsedHeuristics(10);
		diffHeuristic.SetMode(kCompressed);
		taNew.SetUseBPMX(1);

		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%dcb1\t%lld\t%f\t%f\t", diffHeuristic.GetNumHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
		// N memory -- 10 compressed heuristics BPMX(°)
		diffHeuristic.SetNumUsedHeuristics(10);
		diffHeuristic.SetMode(kCompressed);
		taNew.SetUseBPMX(1000);
				
		t.StartTimer();
		taNew.GetPath(&gEnv, start, goal, aPath);
		t.EndTimer();		
		printf("%dcbi\t%lld\t%f\t%f\n", diffHeuristic.GetNumHeuristics(),
			   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
		
	}
	exit(0);
}

// experiments with 0, 10, ..., 100 differential heuristics
// 10% of them are used at each step with and without BPMX
void RunExperiments2(ScenarioLoader *sl)
{
	std::vector<graphState> aPath;
	
	Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
	m->Scale(sl->GetNthExperiment(0).GetXScale(), 
			 sl->GetNthExperiment(0).GetYScale());
	Graph *g = GraphSearchConstants::GetGraph(m);
	
	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	diffHeuristic.SetPlacement(kAvoidPlacement);
	diffHeuristic.SetMode(kRandom);
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);
	
	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;
	
	Timer t;
	
	for (int z = 0; z < 1; z++)
	{
		for (int x = 0; x < 10; x++)
			diffHeuristic.AddHeuristic();
		diffHeuristic.SetNumUsedHeuristics(diffHeuristic.GetNumHeuristics()/10);
		
		for (int x = 0; x < sl->GetNumExperiments(); x++)
		{
			Experiment e = sl->GetNthExperiment(x);
			//			if (e.GetBucket() != 127)
			//			{ continue; }
			
			graphState start, goal;
			start = m->GetNodeNum(e.GetStartX(), e.GetStartY());
			goal = m->GetNodeNum(e.GetGoalX(), e.GetGoalY());
			
			//			taNew.SetUseBPMX(false);
			//			Timer t;
			//			t.StartTimer();
			//			taNew.GetPath(&gEnv, start, goal, aPath);
			//			t.EndTimer();
			//			
			//			printf("%d\t%d.%d\t%d\t%f\t\t%f\n", e.GetBucket(), diffHeuristic.GetNumHeuristics(), diffHeuristic.GetNumHeuristics()/10,
			//				   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
			
			for (int y = 1; y < 6; y++)
			{
				if (y == 5)
					taNew.SetUseBPMX(1000);
				else
					taNew.SetUseBPMX(y);
				t.StartTimer();
				taNew.GetPath(&gEnv, start, goal, aPath);
				t.EndTimer();
				
				printf("%d\t%d.%d\t%lld\t%f\tBPMX\t%f\n", e.GetBucket(), diffHeuristic.GetNumHeuristics(), y,
					   taNew.GetNodesExpanded(), t.GetElapsedTime(), gEnv.GetPathLength(aPath));
			}
		}
	}
	exit(0);
}
