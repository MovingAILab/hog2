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
#include "RandomUnits.h"
#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
//#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "Map2DHeading.h"
#include "GenericSearchUnit.h"
#include "Directional2DEnvironment.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 20;
int gStepsPerFrame = 2;
double searchWeight = 0;
bool screenShot = false;
bool record = false;
std::vector<UnitSimulation<xyhLoc, xyhAct, Map2DHeading> *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

TemplateAStar<xyhLoc, xyhAct, Map2DHeading> a1;
TemplateAStar<xyhLoc, xyhAct, Map2DHeading> a2;
Map2DHeading *ma1 = 0;
Map2DHeading *ma2 = 0;


std::vector<xyhLoc> path;


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
		map = new Map("/Users/nathanst/hog2/maps/dao/den312d.map");
//		map = new Map("/Users/nathanst/hog2/maps/local/verysmall.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/local/weight.map");
		//map = new Map("weight.map");
//		map = new Map(mazeSize, mazeSize);
//		MakeMaze(map, 5);
//		map->Scale(512, 512);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);

	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyhLoc, xyhAct, Map2DHeading>(new Map2DHeading(map));
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Start/stop recording movie.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Clear Units", "Clear all units.", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Compression", "Rotate Compression being shown in heuristic", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Displayed Heuristic", "Rotate which heuristic is shown", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	
	InstallCommandLineHandler(MyCLHandler, "-makeMaze", "-makeMaze x-dim y-dim corridorsize filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-makeRoom", "-makeRoom x-dim y-dim roomSie filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-makeRandom", "-makeRandom x-dim y-dim %%obstacles [0-100] filename", "makes a randomly filled with obstacles");
	InstallCommandLineHandler(MyCLHandler, "-resize", "-resize filename x-dim y-dim filename", "Resizes map to specified dimensions and saves");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-buildProblemSet", "-buildProblemSet filename", "Build problem set with the given map.");
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems3", "-problems3 filename", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-screen", "-screen <map>", "take a screenshot of the screen and then exit");
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
	unitSims[windowID]->OpenGLDraw();
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
	if (record)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/MIG/MIG%d%d%d", (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-screen" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		screenShot = true;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		Map map(argument[1]);
		map.Scale(512, 512);
		map.Save(argument[2]);
		//buildProblemSet();
		//doExport();
		exit(0);
		return 2;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r':
			if (record)
				unitSims[windowID]->GetEnvironment()->drawWeights = true;
			record = !record;
			break;
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
		case '|': unitSims[windowID]->ClearAllUnits();
			unitSims[windowID]->GetEnvironment()->ClearAllCosts(); break;
		case '{':
			break;
		case '}':
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

void ExportGraph(Map *m, Map2DHeading *env);
void ExportGraph(Map *m);

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	//ExportGraph(unitSims[windowID]->GetEnvironment()->GetMap(), unitSims[windowID]->GetEnvironment());
	ExportGraph(unitSims[windowID]->GetEnvironment()->GetMap());
}

void ExportGraph(Map *m, Map2DHeading *env)
{
	// export map as graph
	typedef std::unordered_map<uint64_t, int, Hash64> HT;

	HT table;
	Graph *g = new Graph();
	// build nodes
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int h = 0; h < 8; h++)
				{
					xyhLoc l;
					l.x = x;
					l.y = y;
					l.h = h;
					table[env->GetStateHash(l)] = g->AddNode(new node("n"));
					//					std::cout << table[env->GetStateHash(l)] << " : " << l << std::endl;
				}
			}
		}
	}
	// build edges
	std::vector<xyhLoc> succ;
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int h = 0; h < 8; h++)
				{
					xyhLoc l;
					l.x = x;
					l.y = y;
					l.h = h;
					env->GetSuccessors(l, succ);
					int from = table[env->GetStateHash(l)];
					int to;
					for (unsigned int t = 0; t < succ.size(); t++)
					{
						to = table[env->GetStateHash(succ[t])];
						g->AddEdge(new edge(from, to, env->GCost(l, succ[t])));
					}
				}
			}
		}
	}
	// export edges
	printf("%d nodes %d edges\n", g->GetNumNodes(), g->GetNumEdges());
	std::cout << "nodes:" << std::endl;
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int h = 0; h < 8; h++)
				{
					xyhLoc l;
					l.x = x;
					l.y = y;
					l.h = h;
					std::cout << table[env->GetStateHash(l)] << "\t" << l.x << "\t" << l.y << std::endl;
					//					std::cout << table[env->GetStateHash(l)] << " : " << l << std::endl;
				}
			}
		}
	}
	std::cout << "edges:" << std::endl;
	for (unsigned int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		printf("%d\t%d\t%1.2f\n", e->getFrom(), e->getTo(), e->GetWeight());
	}
}

void ExportGraph(Map *m)
{
	Directional2DEnvironment *env = new Directional2DEnvironment(m, kVehicle, kOctileHeuristic);

	// export map as graph
	typedef std::unordered_map<uint64_t, int, Hash64> HT;
	
	HT table;
	Graph *g = new Graph();
	// build nodes
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int s = -1; s <= 3; s++)
				{
					for (int h = 0; h < 16; h++)
					{
						xySpeedHeading l;
						l.x = x;
						l.y = y;
						l.rotation = h;
						l.speed = s;
						table[env->GetStateHash(l)] = g->AddNode(new node("n"));
						//					std::cout << table[env->GetStateHash(l)] << " : " << l << std::endl;
					}
				}
			}
		}
	}
	// build edges
	std::vector<xySpeedHeading> succ;
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int s = -1; s <= 3; s++)
				{
					for (int h = 0; h < 16; h++)
					{
						xySpeedHeading l;
						l.x = x;
						l.y = y;
						l.rotation = h;
						l.speed = s;
						env->GetSuccessors(l, succ);
						int from = table[env->GetStateHash(l)];
						int to;
						for (unsigned int t = 0; t < succ.size(); t++)
						{
							to = table[env->GetStateHash(succ[t])];
							g->AddEdge(new edge(from, to, env->GCost(l, succ[t])));
						}
					}
				}
			}
		}
	}
	// export edges
	printf("%d nodes %d edges\n", g->GetNumNodes(), g->GetNumEdges());
	std::cout << "nodes:" << std::endl;
	for (int x = 0; x < m->GetMapWidth(); x++)
	{
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				for (int s = -1; s <= 3; s++)
				{
					for (int h = 0; h < 16; h++)
					{
						xySpeedHeading l;
						l.x = x;
						l.y = y;
						l.rotation = h;
						l.speed = s;
						std::cout << table[env->GetStateHash(l)] << "\t" << l.x << "\t" << l.y << std::endl;
						//					std::cout << table[env->GetStateHash(l)] << " : " << l << std::endl;
					}
				}
			}
		}
	}
	std::cout << "edges:" << std::endl;
	for (unsigned int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		printf("%d\t%d\t%1.2f\n", e->getFrom(), e->getTo(), e->GetWeight());
	}
}

xyLoc queryLoc;
bool validQuery = false;
bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
	static point3d startLoc;
	switch (button)
	{
		case kRightButton: printf("Right button\n"); break;
		case kLeftButton: printf("Left button\n"); break;
		case kMiddleButton: printf("Middle button\n"); break;
	}
	mouseTracking = false;
	if ((button == kRightButton) && (mType == kMouseDown))
	{
		unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
		xyhLoc t;
		t.x = px1; t.y = py1; t.h = 4;
		
		
		GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading> *gsu;
		gsu = new GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading>(t, t, &a2);
		unitSims[windowID]->AddUnit(gsu);
		gsu->SetColor(0.0, 1.0, 0.0);
		unitSims[windowID]->GetEnvironment()->SetCost(t, -1.0, 5);
		printf("Added unit at (%d, %d)\n", px1, py1);
		return true;
//		if (validQuery == false)
//		{
//			validQuery = true;
//			queryLoc.x = px1;
//			queryLoc.y = py1;
//		}
//		else {
//			// compute angle, assuming facing upwards at given location
//			float heading = atan2(px1-queryLoc.x, py1-queryLoc.y);
//			int conv = (360.0*heading/(2*3.1415)+180+270); // -90 for face left
//			conv = conv%360;
//			if (conv > 180)
//				conv = 360-conv;
//			printf("Relative heading: %d\n", conv);
//			for (float P = -1; P <= 1; P += 0.5)
//			{
//				//float cost = (P<0)?((1.0-(float)conv/90.0)*(-P)):(((float)conv/90.0-1.0)*(P));
//				float cost = -P*cos(2*3.1415*conv/360.0);
//				printf("P : %1.2f   C : %1.2f\n", P, cost);
//			}
//		}
		//printf("Added heuristic at %d, %d\n", px1, px2);
		return false;
	}
	if (button == kLeftButton)
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
				
				if (ma1 == 0)
				{
					ma1 = new Map2DHeading(unitSims[windowID]->GetEnvironment()->GetMap());
				}
				if (ma2 == 0)
				{
					ma2 = new Map2DHeading(unitSims[windowID]->GetEnvironment()->GetMap());
				}
				
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				//a2.SetWeight(1.8);
				xyhLoc s1;
				xyhLoc g1;
				s1.x = px1; s1.y = py1; s1.h = 0;
				g1.x = px2; g1.y = py2; g1.h = 4;
				
				ma1->SetFourConnected();
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
				runningSearch1 = false;
				runningSearch2 = false;
//				unitSims[windowID]->ClearAllUnits();
//				unitSims[windowID]->GetEnvironment()->ClearAllCosts();
				xyhLoc forbid = g1;
				forbid.x-=1;
				forbid.h =0;
				unitSims[windowID]->GetEnvironment()->SetCost(forbid, -1.0, 10);
				forbid.x+=15;
				forbid.y+=5;
				forbid.h =2+4;
				//unitSims[windowID]->GetEnvironment()->SetCost(forbid, -0.2);
				std::cout << "Avoiding target: " << forbid << std::endl;
//				for (int x = -4; x < 2; x++)
//				{
//					forbid.x = g1.x+x;
//					for (int y = -3; y <= 3; y++)
//					{
//						forbid.y = g1.y+y;
//						if (x >= 0 && y == 0)
//							continue;
//						for (int rot = 0; rot < 8; rot++)
//						{
//							forbid.h = rot;
//							unitSims[windowID]->GetEnvironment()->SetCost(forbid, 5.0);
//						}
//					}
//				}
				
				GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading> *gsu;
				gsu = new GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading>(s1, g1, &a2);
				gsu->SetSpeed(0.25);
				unitSims[windowID]->AddUnit(gsu);
				g1.h = 0;
				g1.x -= 1;
				gsu = new GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading>(g1, g1, &a2);
				unitSims[windowID]->AddUnit(gsu);
				gsu->SetColor(0.0, 1.0, 0.0);

//				gsu = new GenericSearchUnit<xyhLoc, xyhAct, Map2DHeading>(forbid, forbid, &a2);
//				unitSims[windowID]->AddUnit(gsu);
//				gsu->SetColor(0.0, 1.0, 0.0);
				//(state &start, state &goal, GenericSearchAlgorithm<state,action,environment> *alg);

			}
				break;
		}
		return true;
	}
	return false;
}
