
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
#include "Inconsistency2.h"
#include "SearchUnit.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "Propagation.h"
#include "GraphEnvironment.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
//int GraphSizeN = 10;

std::vector<GraphSimulation *> unitSims;

unsigned int fig = 1;
unsigned int N = 5;
unsigned int vid = 1;
double delta = 0;

graphState from,to;
std::vector<graphState> thePath;
Graph* grp=0;
Map* mp=0;
GraphEnvironment* env = 0;

Prop* ALG = 0;

bool done = false;

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
		grp = GraphSearchConstants::GetGraph(mp);
		env = new GraphEnvironment(grp, new GraphMapInconsistentHeuristic(mp, grp));
		
		while(from==to) {
			from = grp->GetRandomNode()->GetNum();
			to = grp->GetRandomNode()->GetNum();
		}
	}
	else if (fig == 1) 
	{
		grp = PropUtil::graphGenerator::genFig1(N);
		from = N;
		to = 0;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, to));
	}
	else 
	{
		grp = PropUtil::graphGenerator::genFig2(N);
		from = 0;
		to = 2*N - 1;
		env = new GraphEnvironment(grp, new GraphLabelHeuristic(grp, to));
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
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

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
		//	ALG->DoSingleSearchStep(thePath);

		if(ALG)
		{
			ALG->OpenGLDraw();
		}
	}
	unitSims[windowID]->OpenGLDraw(windowID);
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
//			if (CFOR == 0)
//			{
//				CFOR = new CFOptimalRefinement();
//				while (!CFOR->InitializeSearch(unitSims[windowID]->GetEnvironment()->GetMapAbstraction(),
//																			 unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->GetAbstractGraph(0)->GetRandomNode(),
//																			 unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->GetAbstractGraph(0)->GetRandomNode()))
//				{}
//			}
//			if (CFOR->DoOneSearchStep())
//				printf("DONE!!!\n");

			if(ALG == 0) 
			{
				ALG = new Prop(vid,delta);
				ALG->InitializeSearch(env,grp,from,to,thePath);
			}

			if(!done && ALG->DoSingleSearchStep(thePath)) 
			{
				printf("\nDone! Nodes expanded=%ld, Nodes touched=%ld, Reopenings=%d.\n",ALG->GetNodesExpanded(),ALG->GetNodesTouched(),ALG->GetReopenings());
				printf("Algorithm %s, solution cost=%lf, solution edges=%d.\n", ALG->algname,ALG->GetSolutionCost(),(int)thePath.size());
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
			ALG = new Prop(0);
			ALG->GetPath(env,grp,from,to,thePath);
			printf("\n");

			ALG = new Prop(1);
			ALG->GetPath(env,grp,from,to,thePath);
			printf("\n");

			ALG = new Prop(2,delta);
			ALG->GetPath(env,grp,from,to,thePath);
			printf("\n");

			ALG = new Prop(3);
			ALG->GetPath(env,grp,from,to,thePath);
			printf("\n");

			ALG = new Prop(4);
			ALG->GetPath(env,grp,from,to,thePath);
			printf("\n");

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

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
//	for (int x = 0; x < ((mod==kShiftDown)?(50):(1)); x++)
//	{
//		if (unitSim->getUnitGroup(1) == 0)
//		{
//			unitSim->addUnitGroup(new SharedAMapGroup(unitSim));
//			unitSim->setmapAbstractionDisplay(2);
//		}
//		int xx1, yy1, xx2, yy2;
//		unitSim->getRandomLocation(xx1, yy1);
//		unitSim->getRandomLocation(xx2, yy2);
//		
//		unit *u, *u2 = new unit(xx2, yy2, 0);
//		
//		praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//		//aStar *pra = new aStar();
//		
//		unitSim->addUnit(u2);
//		u = new SearchUnit(xx1, yy1, u2, pra);
//		// just set the group of the unit, and it will share a map with those
//		// units.
//		unitSim->getUnitGroup(1)->addUnit(u);
//		unitSim->addUnit(u);
//		u->setSpeed(0.5); // time to go 1 distance						
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
//	mouseTracking = false;
//	if (button == kRightButton)
//	{
//		switch (mType)
//		{
//			case kMouseDown:
//				unitSim->GetMap()->getPointFromCoordinate(loc, px1, py1);
//				//printf("Mouse down at (%d, %d)\n", px1, py1);
//				break;
//			case kMouseDrag:
//				mouseTracking = true;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
//				break;
//			case kMouseUp:
//			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
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
