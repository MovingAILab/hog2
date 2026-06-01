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
#include "Racetrack.h"
#include "MapGenerators.h"
#include "TemplateAStar.h"
#include "RoundRobin.h"

RacetrackState start, goal;
Racetrack *race;
TemplateAStar<RacetrackState, RacetrackMove, Racetrack> astar;
RoundRobin<RacetrackState, RacetrackMove, Racetrack> rr;
std::vector<RacetrackState> path1, path2;
int steps = 1;
void ConfigSearch();

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1024, 512);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Numbers", "Will get events for numbers 0-9", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset search", kAnyModifier, 'r');

	// Command-line argument, argument with parameters, description
	InstallCommandLineHandler(MyCLHandler, "-test", "-test <id>", "Sample command-line handler.");
	
	// Gets window events (created/destroyed)
	InstallWindowHandler(MyWindowHandler);
	
	// Gets mouse events - handles all windows
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
		// Basic port layout
		// 1 port with (0,0) in the middle. Top left is (-1, -1), bottom right is (1, 1)
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		// Default text line at top of window is hidden
		setTextBufferVisibility(false);
		
		// Other init
		Map *m = new Map(100, 100);
		MakeMaze(m, 10);
		for (int y = 0; y < 100; y++)
		{
			for (int x = 0; x < 100; x++)
			{
				if (m->GetTerrainType(x, y) != kGround)
					m->SetTerrainType(x, y, kObstacle);
			}
		}
		m->SetTerrainType(97, 97, 97, 99, kEndTerrain);
		m->SetTerrainType(98, 97, 98, 99, kEndTerrain);
		m->SetTerrainType(99, 97, 99, 99, kEndTerrain);
		m->SetTerrainType(1, 1, kStartTerrain);
		srandom(0);
		race = new Racetrack(m);
		start = {1, 1, 0, 0};
		double w = 2.0;
		Node<RacetrackState>::weight = w;
		// XDP
		astar.SetPhi([=](double h,double g){return (g+(2*w-1)*h+sqrt((g-h)*(g-h)+4*w*g*h))/(2*w);});
		// XUP
		//astar.SetPhi([=](double h,double g){return (g+h+sqrt((g+h)*(g+h)+4*w*(w-1)*h*h))/(2*w);});
		//astar.SetWeight(w);
		astar.InitializeSearch(race, start, start, path1);
		rr.InitializeSearch(race, start, start);

		// w = 5.0
//		Astar (phi) 685 expanded. Solution cost 87
//		Astar (phi) 1327 expanded. Solution cost 85 (XDP)
//		Astar (phi) 512 expanded. Solution cost 88 (XUP)
//		Round robin 1185 expanded. Solution cost 111 (2 queues)
//		Round robin 6714 expanded. Solution cost 105 (3 queues)

		// w = 3.0
//		Astar (phi) 2303 expanded. Solution cost 84
//		Astar (phi) 7960 expanded. Solution cost 83 XDP
//		Astar (phi) 3608 expanded. Solution cost 86 XUP
//		Round robin 1185 expanded. Solution cost 111 (2 queues)
//		Round robin 5232 expanded. Solution cost 117 (3 queues)

		// w = 2.0
//		Astar (phi) 27720 expanded. Solution cost 78
//		Astar (phi) 104283 expanded. Solution cost 77 XUP
//		Astar (phi) 28551 expanded. Solution cost 78 XDP
//		Round robin 86674 expanded. Solution cost 90 (2 queues)
//		Round robin 63714 expanded. Solution cost 95 (3 queues)
		
		// w = 1.5
//		Astar (phi) 139911 expanded. Solution cost 76
//		Astar (phi) 182461 expanded. Solution cost 76 XUP
//		Astar (phi) 79271 expanded. Solution cost 76 XDP
//		Round robin 153097 expanded. Solution cost 90 (2 queues)
//		Round robin 235716 expanded. Solution cost 88 (3 queues)

		// w = 1.25
//		Astar (phi) 184807 expanded. Solution cost 76
//		Astar (phi) 199302 expanded. Solution cost 76 XUP
//		Astar (phi) 162410 expanded. Solution cost 76 XDP
//		Round robin 271573 expanded. Solution cost 82 (2 queues)
//		Round robin 244152 expanded. Solution cost 82 (3 queues)

		// w = 1.1
//		Astar (phi) 198920 expanded. Solution cost 76
//		Astar (phi) 200212 expanded. Solution cost 76 XUP
//		Astar (phi) 194084 expanded. Solution cost 76 XDP
//		Round robin 239673 expanded. Solution cost 76 (2 queues)
//		Round robin 249138 expanded. Solution cost 77 (3 queues)
		
		// w = 1
//		Astar (phi) 200212 expanded. Solution cost 76 XDP/XUP
//		Round robin 200264 expanded. Solution cost 76 (2 queues)
//		Round robin 213108 expanded. Solution cost 76 (3 queues)
	}
}

void ConfigSearch()
{
	path1.clear();
	path2.clear();

	start = {1, 1, 0, 0};
	double w = 2.0;
	Node<RacetrackState>::weight = w;
	// XDP
	astar.SetPhi([=](double h,double g){return (g+(2*w-1)*h+sqrt((g-h)*(g-h)+4*w*g*h))/(2*w);});
	// XUP
	//astar.SetPhi([=](double h,double g){return (g+h+sqrt((g+h)*(g+h)+4*w*(w-1)*h*h))/(2*w);});
	//astar.SetWeight(w);
	astar.InitializeSearch(race, start, start, path1);
	rr.InitializeSearch(race, start, start);
}

point3d p1, p2;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	// Get reference to display
	Graphics::Display &d = GetContext(windowID)->display;
	
	// Do any drawing here
	//	d.FillRect({{-1, -1}, {1, 1}}, Colors::lightyellow);
	race->Draw(d);
	
	if (viewport == 0)
	{
		for (int x = 0; x < steps; x++)
		{
			if (path1.size() == 0)
			{
				if (astar.DoSingleSearchStep(path1))
				{
					std::cout << "Astar (phi) " << astar.GetNodesExpanded() << " expanded. Solution cost " << race->GetPathLength(path1) << "\n";
				}
			}
			if (path2.size() == 0)
			{
				if (rr.DoSingleSearchStep(path2))
				{
					std::cout << "Round robin " << rr.GetNodesExpanded() << " expanded. Solution cost " << race->GetPathLength(path2) << "\n";
				}
			}
		}
	}
	if (viewport == 0)
		astar.Draw(d);
	if (viewport == 1)
		rr.Draw(d);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-test" ) == 0 )
	{
		printf("Running in test mode.");
		if (maxNumArgs <= 1)
			return 1;
		printf("Got argument '%s'\n", argument[1]);
		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r':
			ConfigSearch();
			break;
		case 'a':
			printf("Hit 'a'\n");
			break;
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
		{
			int whichNumber = key-'0';
			steps = 1<<(whichNumber+2);//10*(whichNumber)+1;
			printf("Hit %c\n", key);
			break;
		}
		default:
			break;
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button == kLeftButton)
	{
		switch (mType)
		{
			case kMouseDown:
				printf("Mouse down\n");
				p1 = loc;
				p2 = loc;
				break;
			case kMouseDrag:
				printf("Mouse drag\n");
				p2 = loc;
				break;
			case kMouseUp:
				printf("Mouse up\n");
				p2 = loc;
				break;
			case kMouseMove: // You have to request this event explicitly if you want to receive it
				break;
		}
		return true; // handled
	}
	return false; // not handled
}
