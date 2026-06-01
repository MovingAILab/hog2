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
#include "OptimisticSearch.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include <string>
#include "RHRUnit.h"

enum mode {
	kFindPathWA15 = 0,
	kFindPathWA30 = 1,
	kFindPathOptimistic = 2
};

UnitMapSimulation *u;
MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

bool running = false;

int stepsPerFrame = 1;

void StartSearch();
Map *ReduceMap(Map *inputMap);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Increase Speed", "Increase the number of steps per frame", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Decrease Speed", "Increase the number of steps per frame", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Maze", "Load maze", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Default map", "Load default map", kAnyModifier, 'd');
	
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
		
		//Map *map = new Map(300, 300);
		//MakeRandomMap(map, 40);
		//MakeMaze(map, 25);
		//BuildRandomRoomMap(map, 25, 60);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");

//		Map *m2 = ReduceMap(map);
//		delete map;
//		map = m2;

		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		u = new UnitMapSimulation(me);
		u->SetStepType(kUniTime);
//		start.x = 0xFFFF;
//		start.y = 0xFFFF;
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	//me->OpenGLDraw();
	for (int x = 0; x < stepsPerFrame; x++)
		u->StepTime(0.5);
	u->OpenGLDraw();
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
		case '}':
		{
			break;
		}
		case ']':
		{
			stepsPerFrame *= 2;
		}
			break;
		case '[':
		{
			stepsPerFrame /= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
		}
			break;
		case '|':
		{
			u->ClearAllUnits();
		}
			break;
		case 'd':
		{
			delete u;
			delete me->GetMap();
			delete me;

			Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
			map->SetTileSet(kWinter);
			me = new MapEnvironment(map);
			u = new UnitMapSimulation(me);
			u->SetStepType(kUniTime);

			break;
		}
		case 'm':
		{
			delete u;
			delete me->GetMap();
			delete me;
			
			Map *map = new Map(40, 40);
			MakeMaze(map, 1);
			map->SetTileSet(kWinter);
			me = new MapEnvironment(map);
			u = new UnitMapSimulation(me);
			u->SetStepType(kUniTime);
			
			break;
		}
		case '0':
//		case '1': edgeCost = 1.0; te.AddLine("Adding edges; New edges cost 1"); m = kAddEdges; break;
//		case '2': edgeCost = 2.0; te.AddLine("Adding edges; New edges cost 2"); m = kAddEdges; break;
//		case '3': edgeCost = 3.0; te.AddLine("Adding edges; New edges cost 3"); m = kAddEdges; break;
//		case '4': edgeCost = 4.0; te.AddLine("Adding edges; New edges cost 4"); m = kAddEdges; break;
//		case '5': edgeCost = 5.0; te.AddLine("Adding edges; New edges cost 5"); m = kAddEdges; break;
//		case '6': edgeCost = 6.0; te.AddLine("Adding edges; New edges cost 6"); m = kAddEdges; break;
//		case '7': edgeCost = 7.0; te.AddLine("Adding edges; New edges cost 7"); m = kAddEdges; break;
//		case '8': edgeCost = 8.0; te.AddLine("Adding edges; New edges cost 8"); m = kAddEdges; break;
//		case '9': edgeCost = 9.0; te.AddLine("Adding edges; New edges cost 9"); m = kAddEdges; break;
		case '\t':
			
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			running = !running;
			break;
		case 'o':
		{
			if (running)
			{
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
	static RHRUnit *unit = 0;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				u->AddUnit(new RHRUnit(x, y));
			}
			return true;
		}
		case kMouseDrag:
		{
//			int x, y;
//			me->GetMap()->GetPointFromCoordinate(loc, x, y);
//			if (me->GetMap()->GetTerrainType(x, y) == kGround)
//			{
//				goal.x = x;
//				goal.y = y;
//			}
			break;
		}
		case kMouseUp:
		{
//			if (unit != 0)
//				unit->Set
//			int x, y;
//			me->GetMap()->GetPointFromCoordinate(loc, x, y);
//			if (me->GetMap()->GetTerrainType(x, y) == kGround)
//			{
//				goal.x = x;
//				goal.y = y;
//				printf("UnHit (%d, %d)\n", x, y);
//			}
//
//			StartSearch();
			return true;
		}
	}
	return false;
}

void StartSearch()
{
}
