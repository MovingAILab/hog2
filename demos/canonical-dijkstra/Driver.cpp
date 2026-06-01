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
#include "CanonicalDijkstra.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include "CanonicalGrid.h"
#include <string>

MapEnvironment *me = 0;
CanonicalGrid::CanonicalGrid *grid;
CanonicalDijkstra canDijkstra;
std::vector<xyLoc> path;

xyLoc start, goal;

//mode m = kFindPathWA15;

int stepsPerFrame = 1;
bool recording = false;
bool running = false;
bool mouseDrag = false;
bool saveNextProblem = false;
void StartSearch();
Map *ReduceMap(Map *inputMap);

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
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save next problem", kAnyModifier, 's');

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
		
		//Map *map = new Map(300, 300);
		//MakeRandomMap(map, 40);
		//MakeMaze(map, 25);
		//BuildRandomRoomMap(map, 25, 60);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/local/empty_64.map");
		Map *map = new Map("/Users/nathanst/hog2/maps/dao/den407d.map");
		
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		start.x = 0xFFFF;
		start.y = 0xFFFF;
		grid = new CanonicalGrid::CanonicalGrid(map);
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	me->OpenGLDraw();
	
	if (mouseDrag)
	{
		me->SetColor(0.0, 1.0, 0.0);
		me->OpenGLDraw(start);
		me->SetColor(0.5, 0.5, 0.5);
		me->GLDrawLine(start, goal);
	}
	
	{
		glLineWidth(1);
		CanonicalGrid::xyLoc gLoc(start.x, start.y);
		std::deque<CanonicalGrid::xyLoc> queue;
		queue.push_back(gLoc);
		std::vector<CanonicalGrid::xyLoc> v;
		std::vector<bool> visited(me->GetMap()->GetMapHeight()*me->GetMap()->GetMapWidth());
		while (!queue.empty())
		{
			grid->GetSuccessors(queue.front(), v);
			for (auto &s : v)
			{
				if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
				{
					grid->SetColor(0.5, 0.5, 0.5);
					queue.push_back(s);
				}
				else {
					grid->SetColor(1.0, 0.5, 0.5);
				}
				grid->GLDrawLine(queue.front(), s);
				visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
			}
			queue.pop_front();
		}
	}
	
	if (running)
	{
		canDijkstra.OpenGLDraw();

		if (path.size() == 0)
		{
			for (int x = 0; x < stepsPerFrame && path.size() == 0; x++)
				canDijkstra.DoSingleSearchStep(path);
		}
		else {
			me->SetColor(0, 1, 0);
			glLineWidth(10);
			for (int x = 1; x < path.size(); x++)
			{
				me->GLDrawLine(path[x-1], path[x]);
			}
			glLineWidth(1);
		}
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
		case '{':
		{
			stepsPerFrame /= 2;
			if (stepsPerFrame < 1)
				stepsPerFrame = 1;
			break;
		}
		case '}':
		{
			stepsPerFrame *= 2;
			break;
		}
		case ']':
		{
		}
			break;
		case '[':
		{
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
		case '0':
		case '\t':
			
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			if (stepsPerFrame > 0)
				stepsPerFrame = 0;
			else
				stepsPerFrame = 1;
			break;
		case 'o':
		{
			if (running)
			{
				canDijkstra.DoSingleSearchStep(path);
			}
		}
			break;
		case '?':
		{
		}
			break;
		case 's':
			saveNextProblem = true;
			submitTextToBuffer("Next problem will be saved as SVG animation");
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
				mouseDrag = true;
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
			mouseDrag = false;
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
				printf("UnHit (%d, %d)\n", x, y);
			}

			StartSearch();
			return true;
		}
	}
	return false;
}

std::string DrawCanonicalOrdering(bool gray)
{
	std::string str;
	CanonicalGrid::xyLoc gLoc(start.x, start.y);
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
					me->SetColor(0.75, 0.75, 0.75);
				else
					me->SetColor(0.0, 0.0, 0.0);
				queue.push_back(s);
			}
			else {
				if (gray)
					me->SetColor(1.0, 0.5, 0.5);
				else
					me->SetColor(1.0, 0.0, 0.0);
			}
			//grid->GLDrawLine(queue.front(), s);
			str += me->SVGDrawLine({queue.front().x, queue.front().y}, {s.x, s.y});
			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
		}
		queue.pop_front();
	}
	return str;
}

void SaveProblem()
{
	std::string prefix("/Users/nathanst/Desktop/anim/");
	canDijkstra.InitializeSearch(me, start, goal, path);
	int frame = -1;

	while (true)
	{
		frame++;
		{
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/cdijkstra/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << me->SVGHeader();
			svgFile << me->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << canDijkstra.SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		if (canDijkstra.DoSingleSearchStep(path))
		{
			frame++;
			std::fstream svgFile;
			char name[1024];
			sprintf(name, "%s/cdijkstra/%d%d%d%d.svg", prefix.c_str(), (frame/1000)%10, (frame/100)%10, (frame/10)%10, frame%10);
			svgFile.open(name, std::fstream::out | std::fstream::trunc);
			svgFile << me->SVGHeader();
			svgFile << me->SVGDraw();
			svgFile << DrawCanonicalOrdering(true);
			svgFile << canDijkstra.SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
			break;
		}
	}

}

void StartSearch()
{
	if (start == goal)
		return;
	if (me->GetMap()->GetTerrainType(start.x, start.y) == kUndefined ||
		me->GetMap()->GetTerrainType(goal.x, goal.y) == kUndefined)
		return;
	
	if (saveNextProblem)
	{
		SaveProblem();
		submitTextToBuffer("");
	}
	saveNextProblem = false;
	canDijkstra.InitializeSearch(me, start, goal, path);
	running = true;
	stepsPerFrame = 0;
}
