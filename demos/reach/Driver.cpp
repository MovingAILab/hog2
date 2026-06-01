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
#include "MapOverlay.h"
#include <string>
#include "Reach.h"
#include "CanonicalReach.h"
#include "SVGUtil.h"

MapEnvironment *me = 0;
CanonicalGrid::CanonicalGrid *cge = 0;
//MapOverlay *mo = 0;
//MapOverlay *canmo = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
void ComputeReach(std::vector<double> &reach);
void ComputeCanonicalReach(std::vector<double> &reach);
std::vector<double> regularReach;
std::vector<double> canonicalReach;
Reach *reach;
CanonicalReach *canReach;
bool mapChange = true;

xyLoc start, goal;
bool showCanonical = false;

int whichReach = 0;
int whichReachDraw = 0;
int stepsPerFrame = 1;
bool recording = false;
bool running = false;

void LoadMap(Map *m);
void LoadBigMap(Map *m);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Run A* faster", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Run A* slower", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "No reach", "Don't use reach", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "Reach", "Use reach", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Can. Reach", "Use canonical reach", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');

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
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		Map *map = new Map(1,1);
		//LoadMap(map);
		LoadBigMap(map);
//		Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
//		Map *map = new Map("/Users/nathanst/hog2/maps/dao/brc300d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak202d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map"); // round map
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		
		map->SetTileSet(kWinter);
		
		me = new MapEnvironment(map);
		reach = new Reach(me, false);
		cge = new CanonicalGrid::CanonicalGrid(map);
		canReach = new CanonicalReach(cge, false);
//		ComputeReach(regularReach);
//		ComputeCanonicalReach(canonicalReach);

	
//		{
//			std::fstream svgFile;
//			svgFile.open("/Users/nathanst/Desktop/reach-can.svg", std::fstream::out | std::fstream::trunc);
//			svgFile << me->SVGHeader();
//			svgFile << me->SVGDraw();
//			svgFile << canmo->SVGDraw();
//			svgFile << "</svg>";
//			svgFile.close();
//		}
//		{
//			std::fstream svgFile;
//			svgFile.open("/Users/nathanst/Desktop/reach-reg.svg", std::fstream::out | std::fstream::trunc);
//			svgFile << me->SVGHeader();
//			svgFile << me->SVGDraw();
//			svgFile << mo->SVGDraw();
//			svgFile << "</svg>";
//			svgFile.close();
//		}

	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	static int frame = 0;
	frame++;
	if (!reach->DoneComputing())
	{
		double p = 0;
		for (int x = 0; x < 2; x++)
			p = reach->IncrementalCompute();
		if (0 == frame%20)
		{
			std::string s = std::to_string(100.0*p)+"% done";
			submitTextToBuffer(s.c_str());
		}
	}
	if (!canReach->DoneComputing())
	{
		double p = 0;
		for (int x = 0; x < 2; x++)
			p = canReach->IncrementalCompute();
		if (0 == frame%20)
		{
			std::string s = " "+std::to_string(100.0*p)+"% done";
			appendTextToBuffer(s.c_str());
		}
	}

	Graphics::Display &display = getCurrentContext()->display;

	if (mapChange == true)
	{
		display.StartBackground();
		me->Draw(display);

		if (whichReachDraw == 1)
			reach->Draw(display);
		else if (whichReachDraw == 2)
			canReach->Draw(display);
		
		display.EndBackground();
		mapChange = false;
	}
	
	if (start.x != static_cast<uint16_t>(-1) && start.y != static_cast<uint16_t>(-1))
	{
		me->SetColor(0.0, 1.0, 0.0);
		me->Draw(display, start);
		me->SetColor(1.0, 0, 0.0);
		me->Draw(display, goal);
		me->DrawLine(display, start, goal);
	}

	if (running)
	{
		astar.Draw(display);

		for (int x = 0; x < stepsPerFrame; x++)
			if (path.size() == 0)
				astar.DoSingleSearchStep(path);

		if (path.size() != 0)
		{
			me->SetColor(Colors::green);
			for (int x = 1; x < path.size(); x++)
			{
				me->DrawLine(display, path[x-1], path[x], 4);
			}
		}

	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/reach-%d%d%d%d%d.svg",
				(frameCnt/10000)%10, (frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		MakeSVG(GetContext(windowID)->display, fname, 1200, 1200, 0, "", true);
		printf("Saved %s\n", fname);
		frameCnt++;
	}

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
		case ']':
		{
			if (stepsPerFrame < 16384)
				stepsPerFrame *= 2;
		}
			break;
		case '[':
		{
			stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
		}
			break;
		case '|':
		{
		}
			break;
		case 'w':
			break;
		case 'r':
			recording = !recording;
			break;
		case '0': break;
		case '1': whichReach = 0; break;
		case '2': whichReach = 1; break;
		case '3': whichReach = 2; break;
//		case '4': edgeCost = 4.0; te.AddLine("Adding edges; New edges cost 4"); m = kAddEdges; break;
//		case '5': edgeCost = 5.0; te.AddLine("Adding edges; New edges cost 5"); m = kAddEdges; break;
//		case '6': edgeCost = 6.0; te.AddLine("Adding edges; New edges cost 6"); m = kAddEdges; break;
//		case '7': edgeCost = 7.0; te.AddLine("Adding edges; New edges cost 7"); m = kAddEdges; break;
//		case '8': edgeCost = 8.0; te.AddLine("Adding edges; New edges cost 8"); m = kAddEdges; break;
//		case '9': edgeCost = 9.0; te.AddLine("Adding edges; New edges cost 9"); m = kAddEdges; break;
		case '\t':
			printf("Hit tab!\n");
			whichReachDraw = (whichReachDraw+1)%3;
			mapChange = true;
			break;
		case 'p':
			//running = !running;
			break;
		case 'o':
		{
			if (running)
			{
				astar.DoSingleSearchStep(path);
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
				start.x = x;
				start.y = y;
				goal = start;
				printf("Hit (%d, %d)\n", x, y);
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
			}
			break;
		}
		case kMouseUp:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
				printf("UnHit (%d, %d)\n", x, y);
			}

			astar.InitializeSearch(me, start, goal, path);
			astar.SetConstraint(0);
			if (whichReach == 1)// && reach->DoneComputing())
				astar.SetConstraint(reach);
			if (whichReach == 2)// && canReach->DoneComputing())
				astar.SetConstraint(canReach);
			running = true;
			return true;
		}
	}
	return false;
}

void LoadMap(Map *m)
{
	m->Scale(33, 57);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@TT@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@TTTTTTTTT..TTTTTTTT@@@@@@@@@@@@@TTTTTTTT....TTTTTTTT@@@@@@@@@@@@TTTTTT...........TTTT@@@@@@@@@@@TTTTTTT...........TTTT@@@@@@@@@@TTTTTTT............TTTT@@@@@@@@@TTTTTTT...............TTT@@@@@@@@TTTTT.....TT..........TTT@@@@@@@@TTTTT....TTT.........TTT@@@@@@@@@TTTTT....TTT........TTTT@@@@@@@@@TTTTT....TT.........TTTT@@@@@@@@@TTTTT....TTT........TTTT@@@@@@@@@TTTTT....TTT.........TTT@@@@@@@@@TT@TTT................TTT@@@@@@@@TTTTTTT...............TTT@@@@@@@@@TTTTTTT..............TTT@@@@@@@@@@TTTTTTT.............TTT@@@@@@@@@@@TTTTTTT............TT@@@@@@@@@@@@@TTTTT.............TT@@@@@@@@@@@@@@TTTTT............TT@@@@@@@@@@@@@@@TTTT............TTT@@@@@@@@TTTTTTTTTT............TTTTTTTTTTTTTTTTTTTTT............TTTTTTTTTTTTTTTTTTTTT............TTTTT........TTTTTTT.............TT@TT..........TTT......TTT......TT@TT...................TTT......TT@TTTTT................TTT......TT@TTTTT................TTT......TTTTTTTTT...............TTTT.....TTTTTTTTT...............TTTTTTTTTTTTTTTTTT................TTTTTTTTTT@@T@TTT................TTTTTTTTTT@@T@TTT.................TT....TTT@@T@TTT.......................TTT@@TTTTT......TTTTT............TTT@TTTTTT......TTTTT............TTTTTTTTTT.......TTTT............TTTTTTTTTT......TTTTT............TTTTTTTTTT......TTTTT............TTTT@TTTTT.......TTT.............TTT@@TTTTT.......................TTT@@TTT.........................TTT@@TTT.........................TTT@TTTT........................TTTT@TTTT........................TTTTTTTTT........................TTTTTTTTTT.......................TTTTTTTTTTT......................TTTT@TTTTTTT...............TT...TTTTT@TTTTTTTTTTTTT.........TT...TTTTT@TTTTTTTTTTTTT........TTTTTTTTTTTTTTTTTTTTTTTTT........TTTTTTTTTTTTTTTTTTTTTTTTTTT...TTTTTTTTTTTTTTTTT@@@@TTTTTTTTT...TTTTTTT@@@@@TTT";
	int which = 0;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				m->SetTerrainType(x, y, kGround);
			else if (map[which] == 'T')
				m->SetTerrainType(x, y, kTrees);
			else
				m->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
}

void LoadBigMap(Map *m)
{
	m->Scale(194, 194);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@TTTTTTTTTTTTTT.TTTTTTT@@@@@@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@TT@@@@@@@@@@@TTTTTTTTTTTTTT.TTTTTTT@@@@@@@@@@TTT@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@TTTTTTTTTTTTTT.TTTTTTT@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@TT@@@@@@@@@@@@@@@@TTTTTTT........TTTTTTT@@@@@@@@@@@@@@@TT@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT........TTTTTTT@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT........TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT........TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..T..............T..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@TTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@TT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T....................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT..TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@TTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTT...TTTTT.TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT..TTTT@@TTTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTT......TT.TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT....TTTTTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTT.......TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT........TTTT.TTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTT.......TTT.TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@TTT...........TTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTT...........TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@TTTTT...........TTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@TT.....................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT..........TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@TTTTTT............TTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@T.......................T@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTTTTTTTT.........TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@TTT................TTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@TT.......................T@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT.........TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TTTT...................TTTT@TTT@@@@@@@@@@@@@@@@@@@@@@TT........................T@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTT.TTTTTT..........TTTTTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTT...........T.........TTTT@TT@@@@@@@@@@@@@@@@@@@@@@TTT...........TT..........TT@@@@@@@@@@@@@@@@@@@@@@@@TT@TT.....TTT........TT.TTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT.......TTTT.........TTTT@TTT@@@@@@@@@@@@@@@@@@@@@TT...........TTTT.........TTT@@@@@@@@@@@@@@@@@@@@@@TT@TTTTT.............TTTT@T.TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT........TTTT..........TTTT@TT@@@@@@@@@@@@@@@@@@@@TT............TTTTT........TTT@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT..........TTTTTT...TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT........TTTT..........TTTTTTTT@@@@@@@@@@@@@@@@@@@TT..........TTTTTTTT........T@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTT........TTTTTTTT...TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...TTT.........TT............TTTT@TT@@@@@@@@@@@@@@@@@@@T...........TTTTTTTT........T@@@@@@@@@@@@@@@@@@@@@TTT@TTTTTTTTT.......TTTTTTTTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT................T..............TTTT@TT@@@@@@@@@@@@@@@@TTT...........TTTTTTT.........TT@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT........TTTTTTTTTT...TTTTT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......................TTTT......TTTT@TTT@@@@@@@@@@@@@@TTTT...........TTTTTTT..........T@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTT........TTTTTTTTTT....TT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT......................TTTTT.......TTTT@TT@@@@@@@@@@@@@TTTTT...........TTTTTTT........TTTT@@@@@@@@@@@@@@@@@@TT@TT.TTTTTTTTT...TT....TTTTTTTTTT.......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......................TTTT........TTTT@TTT@@@@@@@@@@@TTTTTT...........TTTTTTT........TTTTT@@@@@@@@@@@@@@@@TT@TTT...TTTTTTT..TTTTT...TTTTTTTTT.........T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT........................TTT.........TTTT@TT@@@@@@@@TTTT.TTTT............TTTTT.........TTTTTTT@@@@@@@@@@@@@@TT@TTT.....TT.....TTTTTT...TTTTTTTTT.......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.....................................TTTTTTT@@@@@@TTT.................................TTTT..TTTTT@@@@@@@@@TT@TTTTTT...........TTTTTT...TTTTTTTT.......TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT..............................TTTT....TTTT@TTT@@@TTTTT.........................................TTTTT@@@@@@TTT@TTTTTTT..........TTTTTTT...TTTTTT........TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.............................TTTTT.....TTTTTTTTTTT..............................................TT.TTTTTTTTTTTTTTTTTTT........TTTTTTTTT...TTTT............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.........TTT..................TTTT......TTTTTTTTTT....................................................TTTTTTTTTTTTTTTTT........TTTTTTTTTT...TT.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT........TTTTT.................TTTT......TTTTTTTTT........TT............................................TTTTTTTTTTTTTTTT.........TT.TTTTTTT..................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT........TTTT............................TTTTTTTTT.......T...............................................TTTTTTTTTTTT.T.............TTTTTTT...................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT........TTT.............................TTTTTTTTT..TT..T...T............................................TTTTTTTTTTTT...............TTTTTTTT..............TTT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TT........................................TTTTTTTTTTTTTTT....T............................................TTTTTTTTTTTT...............TTT.TTT..............TTTT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT..............TT........................TTTTTTTTTTTTT....T............................................TTTTTTTTTTTTT................T...T................TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT............TTTT.......................TTTTTTT@TTTT....T.............................................TTTTT.TTTTTTT................................TT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.............TTTT.......................TTTTTTTTTTTT...T...............................................TTTT..TTTTTTT.............................TTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@TTT...........TTTT.............................TTT@TTT..TT................................................TT....TTTTTTT...........................TTTTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@TTT...........TT..............................TTTT@TTTTTTT.....TTTT...................TT........................TTTTTTT.........................TTTTTTTT..TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@TTTTT...............TT.........................TTT@TTTTTTT.....TTTT.T......TTTT.....TTTTTTTT.....................TTTTTT.........................TTTTTTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@T@TTT@@@@@TTTTTT............TTTT........................TTTT@TTTTTT....TTTTTTT.....TTTTTT....TTTTTTTT......................TTTT.....................TT...TTTTTTTTT..TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@TTTTT..........TTTTT........................TTTT@TTTTTT....TTTTTTT...TTTTTTTTTT...TTTTTT@T......T...............TT.....................TTTT...TTTTTTTT...TTTTT.T@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT@@@TTTTT........TTTT..........................TTTT@TTTTT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.....TTTT...................TT...............TTTTT..TTTTTTTTT...TTTT..T@@@@@@@@@@@@@@@@@@@TTTTTTT...TTTTT@@@TTTT........T...........................TTTTTTTTTTTT.....TTTTTTTTTTTTTTTTTTTTTTTTTTT.......TTTTT.................TTTT.............TTTTTT...TTTTTTTTT..TTTTT..T@@@@@@@@@@@@@@@@@@TTTT.......TTTTTT@@TTTTT.................................TTT@@TT@TTTTT.TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT....TTTTTT................TTTTTT.TTT........TTTTTTT...TTTTTTTT...TTT..TT@@@@@@@@@@@@@@@@@TTT...........TTTTT@@@TTTTT...............................TT@@@@TTTTTTTTTTTTTTTTTTT@@@TTTTT@@@@TTTTTTTTTTTTTTTTTTT.................TTTTTTTTTTT.......TTTTTTTT..TTTTTTTT.........TT@@@@@@@@@@@@@@@@TT..............TTTTT@@@TTTTT.............................TT@@@@TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTT..................TTTTTTTTT........TTTTTTTT...TTTTTTTT........TT@@@@@@@@@@@@@@@TTT................TTTTT@@TTTTTT...........................TT@@@@TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT...................TTTTTTT..TT.........TTTTT..TTTTTTTT.........TT@@@@@@@@@@@@@TTT...................TTTTT@@TTTTTTTTTT.....................TTT@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT....................TTTTTTTTTTT........TTTTT...TTTTT...........TT@@@@@@@@@@@@@TT......................TTTT@@@TTTTTTTTTT....................TTT@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.....................TTTTTTTTTT.........TTTTT...TTT.............TT@@@@@@@@@@@TTT.......................TTTTT@@@@TTTTTTTT....................TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.....................TTTTTTTTT..........TTTT...................TTT@@@@@@@@@@TT.........................TTTTTT@@TTTTTTTTT....................TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.....................TTTTTTTTT.........TTT.....................TT@@@@@@@@@TTT............................TTTT@TTTTTT@TT...................TTTTTTT.TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.....................TTTTTTTT.................................TTT@@@@@@@@TTTT.......TT................TTTTTTTTTTTTTTTT...................TTTTTT...TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.....................TTTTTTT...................TTTT...........TT@@@@@@@TTTTT......TTT................TTT.TTTTTTTTTTTT....................TTTT....TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT....................TTTTTTT.TT...............TTTTT..TTT.......TT@@@@@TTTTTT.....TTTT.....TT...........T..TTTTTTTTTTT.............................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTTTT.........TTT...TTTTT..TTTT......TTT@@@@TTTT.......TTTT....TTT..............TTTTTTTTTTT...............TT.............TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTTT.........TTTT...TTT...TTTT.......TT@@@TTTTT..TT....TTT....TTTT....TT........TTTTTTTTT...............TTTT............TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTT.........TTTT.........TTTT......TTTT@@T@TTT..TTT....T....TTTT....TTT........TTTTTTTTT...........TTTTTTTT.............TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTT...........TTTT..TT...............TTTT@TT@TT...TTT..........TTT....TTT.....T....TTTTT..........TTTTTTTTTTT.............TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT..................TTTTTTTT.........T...TTTT.................TTTT@TTTTTTTT................TTTT...TTTT..................TTTTTTTTTTT..............TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT...........TTTTTTTTT............TTTT.................TTTTTTTTTTTT...................TT...TTTT.................TTTTTTTTTTT................TTT@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT...........TTTTTTTTT............TTT.................TTTTTTT...TTT........................TTT..................TTTTTTTTTT..................TT@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.............TTTTTTT..............T..................TTTTTTT...TTT.............................................TTTTTTTTT...................TTT@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.............TTTTT....................................TTTTTT...................................................TTTTTTTTTT...................TTTT@@TTTTTTTTTTTTT@@@TTTT@@@@@@@@@@@@@@@@@@@@TTTTTTTT.............TTTTT....................................TTTTT....................................................TTTTTTTTTTT..................TTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@TTTTTTT...............TTTT.....................................TTTT..TTT...............................................TTTTTTTTTTTTT................TTTTTTTTT.........TTTTTTTT@@@@@@@@@@@@@@@@@@@@@TTTTTT...............TTTT.............................TTT..TTTTTTT..TTT...T...........................................TTTTTT@@@@TTTTT..............TTTT.................TTTTT@@@@@@@@@@@@@@@@@@@@@@@TTTT...............TT................TTT............TTT..TTTTTTT...TTT.TTTT.........................................TTTTT@@@@@@@TTTTT..............TT..................T.TTT@@@@@@@@@@@@@@@@@@@@@@@TTT.............................TT..TTTTTT......TTTTTTTTTTTTTT....TTT.TTT....TT....................................TTTTT@@@@@@@@@TTTTT...................................TTT@@@@@@@@@@@@@@@@@@@@@@TTTT...........................TTTT.TTTTTT......TTTTTTTTTTTTTT........TTT....TTTT.................................TTTTTT@@@@@@@@@@TTTTTT.................................TTTT@@@@@@@@@@@@@@@@@@@@@TTTT........................TTTTTTTTTTTTTT......TTTTTTTTTTTTTT........TT.....TTTT...TT.............................TTTTT@@@@@@@@@@@@TTTTT.................................TTT@@@@@@@@@@@@@@@@@@@@@TTTT.................TTT..TTTTTTTTTTTTTTTT......TTTTT.TTT@@TTT...............TTTT...TTTT.......................TTTTTTTT@@@@@@@@@@@@@@@TTTTTTT..............................TTTTTTT@@@@@@@@@@@@@@@@@TTTT...............TTTT..TTTTTTTTTTT.TT..........TTT.TTT@@TTT...............TT.....TTTT...TT..................TTTTTTTT@@@@@@@@@@@@@@@@@TTTTT...............................TTTTTT@@@@@@@@@@@@@@@@@TTTT...............TTTTTTTTTTTTT.................TTT.TTT@@TTT......................TTT.....TTT.................T@TTTTT@@@@@@@@@@@@@@@@@TTTTT................................TTTTT@@@@@@@@@@@@@@@@@TTTT...............TTTTTTTTT.....................TTT.TTT@@TTTT.....................TT.....TTTT..................TTTTT@@@@@@@@@@@@@@@@@@TTTTT................................TTTT@@@@@@@@@@@@@@@@@@TTTT...............TTTTT................................TTTTTT............................TTTT..................TTTTT@@@@@@@@@@@@@@@@@@@TTT..................................TT@@@@@@@@@@@@@@@@@@@@TTTT..............TTTTT................................TTTTTT.............................T....................TTTTT@@@@@@@@@@@@@@@@@@@TT...................................TTT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT...............................TTTTTTT.................................................TTTTT@@@@@@@@@@@@@@@@@@TTT....................TT..............TT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT...............................TTTTTTTT...............................................TTTTT@@@@@@@@@@@@@@@@@@@TTT....................TT..............TT@@@@@@@@@@@@@@@@@@@@TTT...............TTTTT...T.T..T....TTTTTTTTTTTTTT.TTT@@TT@TTTT..........................TTTTTT.............TTTTTT@@@@@@@@@@@@@@@@@@@TT.............TTT....TTT..............TTT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT..TTTTTT.TT.TTTTTTTTTTTTTT.TTT@@TTTTTTTT........................TTTTTTTT...........TTTTTTT@@@@@@@@@@@@@@@@@@@TT.............TTT......................TT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT..TTTTTT.TT.TTTTTTTTTTTTTT.TTTTTTTTTTTTTT......................TTTTTTTTTT..........TTTTTTT@@@@@@@@@@@@@@@@@TTTT..............T.......................TT@@@@@@@@@@@@@@@@@@TTTTTT..............TTTTTT...T@@T....TTTTTTTTTTTTTT.TTTTTTTTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.........TTTTTTTTT@@@@@@@@@@@@@@@TTTTT......................................TTTTT@@@@@@@@@@@@@@@TTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT........TTTTTTTTT@@@@@@@@@@@@@@@TTTTT.....................................TTTTT@@@@@@@@@@@@@@@@TTTTTTTTT...........TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@TT........TTTTTTTTT@@@@@@@@@@@@@@@TTTTT.....................................TTTTT@@@@@@@@@@@@@@@@TTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@TT........TTTTTTTTT@@@@@@@@@@@@@@@TTTT........................T..............TTTT@@@@@@@@@@@@@@@@TTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.........TTTTTTTT@@@@@@@@@@@@@@@@@TTT......................TTT.............TTT@@@@@@@@@@@@@@@@@@TTTTT..............TTTTTT..TTT.T.TTTTTTT.T.TTTTTTT.TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@TTT..........TTTTTTT@@@@@@@@@@@@@@@@@@@TT...............TT.....TTT.............TTT@@@@@@@@@@@@@@@@@@TTTT...............TTTTT...TTT...TTTTTTT...TTTTTTT...TTTTT@@TTTTT......................TTT@@@TTTT...........TTTTTT@@@@@@@@@@@@@@@@@@@TT...............TTT.....T......TTT.....TT@@@@@@@@@@@@@@@@@@@TTTT..............TTTTT....TTT...TTTTTTT...TTTTTTT...TTTTT@TTTT.........................TTTTTTTT.............TTTTT@@@@@@@@@@@@@@@@@@@TT..............TTT............TTTTT...TTT@@@@@@@@@@@@@@@@@@@TTT...............TTTTT....TTT.TTTTTTTTT...TTTTTTT...TTTTTTTTT............................TTTT...............TTTTTT@@@@@@@@@@@@@@@@@@TT..............................TTTTT..TTT@@@@@@@@@@@@@@@@@@TTTT...............TTTTT....TTT.TTTTTTTTT...TTTTTTT...TTTTTTTT.......TTT......................................TTTTTT@@@@@@@@@@@@@@@@@@TTT..............................TTTTT.TT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT....TTT.TTTTTTTTT...TTTTTTT...TTTTTTT......TTTTT.......................................TTTTT@@@@@@@@@@@@@@@@@@TTT...............................TTTTTTT@@@@@@@@@@@@@@@@@@@TTTT..............TTTTT.....TTT.TTTTTTTTT...TTTTTTT....TTTTTT.....TTTTTT........................................TTTT@@@@@@@@@@@@@@@@@@@TTTT..............................TTTTT@@@@@@@@@@@@@@@@@@@TTTT...............TTTTT................................TTTTT.....TTTT...........................................TTTTT@@@@@@@@@@@@@@@@@TTTTT.....................T.........TTTTT@@@@@@@@@@@@@@@@@@TTTT...............TTTTT.................................TTTT.....TTT............................................TTTTT@@@@@@@@@@@@@@@@@TTTTT....................TTT.........TTTTT@@@@@@@@@@@@@@@@@TTTT..............TTTTT..................................TTTT....................................................TTTTT@@@@@@@@@@@@@@@@@@TTTTT...................TTTT.........TTTTT@@@@@@@@@@@@@@@@TTT...............TTTTT.................................TTTTT....................................................TTTTTT@@@@@@@@@@@@@@@@@@@TTTT..................TTTTT.........TTTTT@@@@@@@@@@@@@@TTTT.................TT.................TTT..............TTTTT.....................................................TTTTT@@@@@@@@@@@@@@@@@@@@TTTT..................TTTTT.........TTTTT@@@@@@@@@@@@@TTTT...................................TTTT..............TTTTT............................TTTT...................TTTTTTT@@@@@@@@@@@@@@@@@@@@@@TTT..T...............TTTTT.........TTTTT@@@@TT@@@@@@TTTT................................TT.TTTT...TTTT.........TTT.......................TTTTTTTTT..................TTTT@TTT@@@@@@@@@@@@@@@@@@@@@@TTTTTT................TTTTT.........TTTTT@@T@@T@@@@@TTT.................................TTTTTTT..TTTTTTT......TTTT..................TTTTTTTTTTTTTT.................TTT@@@TTTT@@@@@@@@@@@@@@@@@@@@@@TTTTTT...............TTTTTT.........TTTTTT@@@@T@@TTTTT...............TT................TTTTTTT..TTTTTTT...TT..TTT..............TTTTTTTTTTTTTTTTTT.................TT@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@TTTTTTTT....TT...TTTTTTTTTT.........TTTTT@@@@@TTTTTTT...............TTTT..............TTTTTT...TTTTTTT...TTTTTTT..........TTTTTTTTTTTTTTTTTTTTTTT...............TTT@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTT@TTTTT.........TTTTT@@@@TTTTTTT..............TTTTT...............TTTTT...TTTTTTT..TTTTTTTT..........TTTTTTTTTTTTTTTTTTTT..................TT@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@TTT@@@TTTTTTTTTTT@@@@@@TTTTT.......TTTTTTT@@@TTTTTTT..............TTTTT.............TTTTTTT..TTTTTTTT..TTTTTTTT..........TTTTTTTTTTTTTTTT......................TTT@@TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@TTTTT.......TTTTTTT@@TTTTTTTTT............TTTTT............TTTTTTT...TTTTTTT...TTTTTTTTTTTT.......TTTTTTTTTT.........................TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@TTTTT.......TTTTTT@@TTTTTTTTT............TTTTTT...........TTTTTTT...TTTTTTT...TTTTTTTTTTTTT......TTTTTT.............................TTTTTTT@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@TTTTT.T.....TTTTT@@@TTTTTTTT...........TTTTTTTT...........TTTTTT..TTTTTTTT..TTTTTTTTTTTTTT......T..................................TTTTTT@TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT.....TTTTT@@TTTTTTTT...........TTTTTTTT............TTTTT..TTTTTTT...TTTT@TTTTTTTTT..................................TTTTT..TTTT@@TTT.....TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT.....TTTT@TT..................TTTTTTTT............TTTT...TTTTTTT...TTTT@TT@TTTTTT................................TTTTTTTTTTT@@TTTT.......TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT.....TTTTT..................TTTTTTT..............TTTT...TTTTTTT..TTTTTTT@@TTTTT................................TTTT@@TTTTT@TTTT..........TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT......TT...................TTTTTTTT....................TTTTTTT..TTTTTTT@@@TTT................................TTTTTTTTTT@TTTT.............TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTTTTT....TT...................TTTTTTTTT....................TTTTTT...TTTTTT@@@@TTT................................TTTTTTTTTTTTT................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@TTTTTTT..TT...................TTTTTTTTTT.......................TTT...TTTTTT@@@@@TTT..............................TTTTTTTTTTTTT..................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@T@TTTT.TT....................TTTTTTTTT..............................TTT@T@@@@@@@TTT...........................TTTTTTTTTTTTT.....................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@TTTTT....................TTTTTTTT...........T....................TTTT@@@@@@@@TTT..........................TTTTTTTTTTTTTT......................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@TT....................TTTTTTTTT..........TTTT..................TTTT@@@@@@@@@TT.........................TTTTTTTTTTTTTT........................TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@TT....................TTTTTTTTTT.........TTTTT..................TTT@@@@@@@@@@TTT......................TTTTTTTTTTTTTTT..........................TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT....................TTTTTTTTT...........TTTT....................TT@@@@@@@@@@@TTT......................TTTTTTTTTT...............................TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT....................TTTTTTTTTTT.........TTTTT...TTT...........TTTT@@@@@@@@@@@@@TTT...................TTTTTTTTTT.................................TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT...................TTTTTTT@TTTT.........TTTT...TTTTTT.........TTT@@@@@@@@@@@@@@TTT.TTTT..............TTTTTTTT.................TT................TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT..................TTTTTTTTTTTTT........TTTTT..TTTTTTTT........TTT@@@@@@@@@@@@@@@TTTTTTTTTT...........TTTTTT...................TTTT..............TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTT..................TTTTTTTTT...........TTTTT...TTTTTTTT........TT@@@@@@@@@@@@@@@@TTTT@TTTTT...TT.T.....TTTT....................TTTT.............TTTTTTTTTTTTTTTTTTTT@@@TTTTTT@@@TTTTTTTTTTTTTTTTTTTT.................TTTTTTTTTT..........TTTTT..TTTTTTTTT...T.....T@@@@@@@@@@@@@@@@@TT@@@@@TTTTTTTTTT............................TTTTT...........TTTTTT.....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.....TTTTT.................TTTT..TTTT.........TTTTT...TTTTTTTT...TTTT..T@@@@@@@@@@@@@@@@@@TT@@@@@@TTTTTTTTTT..........................TTTTTT...........TTTTTT........TTTTTTTTTTTTTTTTTTTTTTTTT.........TTTTTT.................TT....TT..........TTTTT..TTTTTTTTT..TTTTT..T@@@@@@@@@@@@@@@@@@@T@@@@@@TTTTTTTTTT.........................TT@@@@TT...........TTTT.............TTTTTTTTTTTTTTTTTT.............TTTTT..................................TTTTT...TTTTTTTT...TTTTT.T@@@@@@@@@@@@@@@@@@@@@T@@@@@TTTTTTTTT........................TTT@@@@@@TT..........TTTT.................TTTTTTTTTT..................TT............TTTT....................TTTT...TTTTTTTTT..TTTTT..@@@@@@@@@@@@@@@@@@@@@@T@TTTTTTTTTTTT........................TTT@@@@@@@@TTT..........T....................TTTTTT.................................TTTTTT.....................T....TTTTTTTT...TTTTT.T@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTT.......................TTT@@@@@@@@TTT................................TTTT.................................TTTTTTT.................TT......TTTTTTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@TTT......................TTT@@@@@@@@@@TTT............................................................TT.....TTTTTTT.................TTTT.....TTTTTTTT..TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT@@@@TT...................T.TT@@@@@@@@@@@@@@TT.TTTT.....................................................TTTT...TTTTTTTTT................TTTT......TTTTTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@TT................TTTTT@@@@@@@@@@@@@@@@TTTTTTTT...................................................TTTTT.TTTTTTTTTTTTTT............TTTT........TTT...TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@TT..............TTTTTT@@@@@@@@@@@@@@@@@TTTTTT....................................................TTTTTTTTTTTT..TTTTTTT............TT...............TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@TTT..............TTTTT@@@@@@@@@@@@@@@@TTTTTTT....................................................TTTTTTTTTTT....TTTTTT................TTT.........TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@TTT.............TTTTT@@@@@@@@@@@@@@@@TTTTTTT......................................................TTTTTTTTT......TTTTTTTT........TT..TTTTT........TTTT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@TT..........TTTT@T@@@@@@@@@@@@@@@@@TTTTTTT.......................................................TTTTTTTT........TTTTTTT.......TTTT.TTTTT..........TT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@TT.......TTTT@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.......................................................TTTTTTT.........TTTTTT.......TTTT...TTT.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@T.......TTT@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.......................................................TTTTTTTTT........TTTT..TT....TTTT.................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@TTTTT.TTTT@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT........................................................TTTTTTTTT.........TTTTTTT.........................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.........................................................TTTTTTT@TT........TTTTTTT............TT.........TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TT...........................TTT..T............................TTT@TTTTT.........TTTTT............TTTT.......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT..........................TTTTT.TTT...........................TT@TTTTTT.........TTTTT...........TTTT.......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TT...........................TTTTTTTTTT.........................TTTT@TTTTTT........TTTTT...........TTTT......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT............................TTTTTTTT....................TTTTTTTTTTT@TTTTT........TTTT.............TT.......TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TT.............................TTTTTTTT............TT.....TTTTT@@@@@TT@TTTTTT........TT......................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTT.............................TTTTTTTT............TTTTTTTTTTT@@@@@@TTT@TTTTT...............................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT...............................TTTTTTT...........TTTTTT@@@@@@@@@@@@@TT@TTTTTTT.............................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TT................................TTTTTT............TTTTT@@@@@@@@@@@@@@TTT@TTTTTTT...........................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT.................................TTTT..............TTT@@@@@@@@@@@@@@@@TTTTTTTTTT...........TT..............TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TT....................................................TTT@@@@@@@@@@@@@@@@@TT@TTTTTTT.........TTTT........TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT....................................................TT@@@@@@@@@@@@@@@@@@TTT@TTTTTT........TTTTTTT......TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT.....................................................T@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT.......TTTTTTTT....TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TT@@@@@@@@@@@@@@@@@@@@@@@TT@TTT.....................................................T@@@@@@@@@@@@@@@@@@@@TTT@TTTTTT.......TTTTTTTT....TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@TTT@TT......................................................T@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT.....TTTTTTTT.....TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@TT@TT......................................................TTT@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT.....TTTTTTT.TT..TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@TTT@TT......................................................TT@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.....TTTTTTTTTT.TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@TT@TT.......................................................TT@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTT.....TTTTTTTTTT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@TT@TTT.......................................................T@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTTTT.....TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@TTT@TT........................................................T@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTT......TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@TT@TTT........................................................T@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTTTT........TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@TTT@TT........................................................TT@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT..TT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@TT@TT.......................................................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTT.TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@TTTT@TT................TTT....................................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTT..TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT.................TTT....................................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT.TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.................TT.......TT............................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT...............TTT......TTT.............................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT................TTT......TTT..............................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT................TTT......TT...............................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT...............TT.......TT...............................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.............TTT......TTT...............................T@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T...........TTT......TTT...............................T@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.........TT.......TT...............................TTT@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT......TTT......TTT...............................TTT@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T.....TTT......TTT................................T@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT...TTT......TTT................................T@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.TT.......TT.................................T@@@@@@@@@@@@TT@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@......TTT...............TTTTTTTT..........T@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..TTT...............TTTTTTTT.........TTT@@@@@@TT@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...............TTTTTTTT.........TTT@@@@TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
	int which = 0;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				m->SetTerrainType(x, y, kGround);
			else if (map[which] == 'T')
				m->SetTerrainType(x, y, kTrees);
			else
				m->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
}
