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
//#include "ContractionHierarchy.h"
#include "BFS.h"
#include "MNAgentPuzzle.h"
#include "Map2DConstrainedEnvironment.h"
#include "CBSUnits.h"

MNAgentEnvironment env;
MNAgentPuzzleState start(4, 4);
MNAgentPuzzleState goal(4, 4);

Map2DConstrainedEnvironment *m2e = 0;
MapEnvironment *me = 0;
Map *map = 0;
bool mouseDoesPath = false;
TemplateAStar<xytLoc, tDirection, Map2DConstrainedEnvironment> astar;
std::vector<xytLoc> thePath;
UnitSimulation<xyLoc, tDirection, MapEnvironment> *sim = 0;

int px1 = -1, px2, py1, py2;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}

void InitStart()
{
//	map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
//	map->SetTileSet(kWinter);
//	m2e = new Map2DConstrainedEnvironment(map);

//	map = new Map(6, 3);
//	map->SetTerrainType(2, 0, kOutOfBounds);
//	map->SetTerrainType(3, 0, kOutOfBounds);
//	map->SetTerrainType(2, 2, kOutOfBounds);
//	map->SetTerrainType(3, 2, kOutOfBounds);
//	me = new MapEnvironment(map);
//	
//	sim = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(me);
//	sim->SetStepType(kLockStep);
//	CBSGroup *group = new CBSGroup(me);
//
//	sim->AddUnitGroup(group);
//	xyLoc s1(1, 0), g1(4, 0);
//	CBSUnit *u1 = new CBSUnit(s1, g1);
//	u1->SetColor(1.0, 0.0, 0.0);
//	
//	xyLoc s2(0, 1), g2(5, 1);
//	CBSUnit *u2 = new CBSUnit(s2, g2);
//	u2->SetColor(0.0, 1.0, 0.0);
//	
//	xyLoc s3(1, 2), g3(4, 2);
//	CBSUnit *u3 = new CBSUnit(s3, g3);
//	u3->SetColor(0.0, 0.0, 1.0);
//	
//	group->AddUnit(u1);
//	group->AddUnit(u2);
//	group->AddUnit(u3);
//
//	sim->AddUnit(u1);
//	sim->AddUnit(u2);
//	sim->AddUnit(u3);
	
	
	map = new Map(4, 4);
	map->SetTerrainType(0, 0, kOutOfBounds);
	map->SetTerrainType(0, 3, kOutOfBounds);
	map->SetTerrainType(3, 0, kOutOfBounds);
	map->SetTerrainType(3, 3, kOutOfBounds);
	me = new MapEnvironment(map);
	
	sim = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(me);
	sim->SetStepType(kLockStep);
	CBSGroup *group = new CBSGroup(me);
	
	sim->AddUnitGroup(group);
	xyLoc s1(1, 0), g1(2, 3);
	CBSUnit *u1 = new CBSUnit(s1, g1);
	u1->SetColor(1.0, 0.0, 0.0);
	
	xyLoc s2(0, 1), g2(3, 2);
	CBSUnit *u2 = new CBSUnit(s2, g2);
	u2->SetColor(0.0, 1.0, 0.0);
	
	group->AddUnit(u1);
	group->AddUnit(u2);
	
	sim->AddUnit(u1);
	sim->AddUnit(u2);

	
//	start.BlockCell(2, 2);
//	start.BlockCell(1, 1);
//	start.AddAgent(0, 0);
//	start.AddAgent(1, 0);
//	start.AddAgent(2, 0);
//	start.AddAgent(3, 0);
//	start.AddAgent(0, 1);
//	start.AddAgent(0, 2);
//
//	goal.BlockCell(2, 2);
//	goal.BlockCell(1, 1);
//	goal.AddAgent(0, 2);
//	goal.AddAgent(0, 1);
//	goal.AddAgent(3, 0);
//	goal.AddAgent(2, 0);
//	goal.AddAgent(1, 0);
//	goal.AddAgent(0, 0);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Add viewport", "Add another viewport", kAnyModifier, '\t');

	InstallKeyboardHandler(MyDisplayHandler, "Toggle path/add constraints", "Toggle between right-click causing a pathfind or an adding of constraints", kAnyModifier, 't');

	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");

	
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
		InitStart();
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (m2e)
		m2e->OpenGLDraw();
	if (sim)
		sim->OpenGLDraw();
	if (px1 != -1)
	{
		xyLoc a(px1, py1), b(px2, py2);
		xytLoc c, d;
		c.l = a; c.t = 0;
		d.l = b; d.t = 0;
		m2e->GLDrawLine(c, d);
	}
	astar.OpenGLDraw();
	if (thePath.size() > 0)
	{
		glLineWidth(5.0);
		m2e->GLDrawPath(thePath);
		glLineWidth(1.0);
	}
	//	if (viewport == 0)
//		env.OpenGLDraw(start);
//	else
//		env.OpenGLDraw(goal);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-map") == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);

		return 2;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0': case '1': case '2': case '3': case '4': case '5':
			case '6': case '7': case '8': case '9':
			break;
		case 't': mouseDoesPath = !mouseDoesPath; break;
		case '\t': SetNumPorts(windowID, 1+(GetNumPorts(windowID))%4); break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<MNAgentPuzzleState> p;
	env.GetSuccessors(start, p);
	printf("Start has %d neighbors:\n", p.size());
	for (unsigned int x = 0; x < p.size(); x++)
		std::cout << p[x];
	
	BFS<MNAgentPuzzleState, tAgentAction> bfs;
	bfs.GetPath(&env, start, goal, p);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	//CBSGroup *g = new CBSGroup();
	sim->StepTime(1.0);
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (m2e == 0)
		return false;
	if (button == kRightButton)
	{
		if (mouseDoesPath)
		{
			if (mType == kMouseDown)
			{
				map->GetPointFromCoordinate(loc, px1, py1);
				py2 = py1;
				px2 = px1;
			}
			else if (mType == kMouseDrag)
			{
				map->GetPointFromCoordinate(loc, px2, py2);
			}
			else if (mType == kMouseUp)
			{
				xytLoc start, goal;
				start.l.x = px1;
				start.l.y = py1;
				start.t = 0;
				goal.l.x = px2;
				goal.l.y = py2;
				goal.t = 0;
				astar.GetPath(m2e, start, goal, thePath);
				px1 = -1;
			}
		}
		else {
			int px, py;
			map->GetPointFromCoordinate(loc, px, py);
			xytLoc constraint;
			constraint.l.x = px;
			constraint.l.y = py;
			for (int t = 0; t < 30; t++)
			{
				constraint.t = t;
				m2e->AddConstraint(constraint);
			}
		}
		return true;
	}
	return false;
}

