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
#include "GraphEnvironment.h"
#include <string>
#include "Racetrack.h"
#include "TemplateAStar.h"
#include "MapGenerators.h"

bool recording = false;
bool running = false;

Map *m = 0;
Racetrack *r = 0;
RacetrackState s;
RacetrackMove v;
int frame = 0;

RacetrackState start;
RacetrackState end;
std::vector<RacetrackState> path;
std::vector<RacetrackState> yourPath;
std::vector<RacetrackState> bestPath;
std::vector<RacetrackState> legalSuccessors;
bool mapUpdated = true;
TemplateAStar<RacetrackState, RacetrackMove, Racetrack> astar;
// -------------- MAIN FUNCTION ----------- //
int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to start state", kAnyModifier, 'r');

//	InstallKeyboardHandler(MyDisplayHandler, "Up", "Accelerate upwards", kAnyModifier, kUpArrow);
//	InstallKeyboardHandler(MyDisplayHandler, "Down", "Accelerate downwards", kAnyModifier, kDownArrow);
//	InstallKeyboardHandler(MyDisplayHandler, "Left", "Accelerate left", kAnyModifier, kLeftArrow);
//	InstallKeyboardHandler(MyDisplayHandler, "Right", "Accelerate right", kAnyModifier, kRightArrow);
	// --- WASD handlers --- //
//	InstallKeyboardHandler(MyDisplayHandler, "Up", "Accelerate upwards", kAnyModifier, 'w');
//	InstallKeyboardHandler(MyDisplayHandler, "Left", "Accelerate Left", kAnyModifier, 'a');
//	InstallKeyboardHandler(MyDisplayHandler, "Down", "Accelerate downwards", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Right", "Accelerate Right", kAnyModifier, 'd');
//	InstallKeyboardHandler(MyDisplayHandler, "Continue", "Don't accelerate", kAnyModifier, ' ');

	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Solve optimally", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Random map", "New random map", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Change map", "Select map type", kAnyModifier, '0', '6');

	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseDrag|kMouseDown|kMouseUp));
}

void MakeMap(int mapType = -1)
{
	int mapSize = 35;
	int which = mapType;
	if (mapType == -1)
	{
		which = random()%7;
	}
	switch (which)
	{
		case 0:
		case 1:
		case 2:
		{
			m = new Map(mapSize, mapSize);
			MakeMaze(m, (which+1)*2);
			for (int y = 0; y < m->GetMapHeight(); y++)
				for (int x = 0; x < m->GetMapWidth(); x++)
					if (m->GetTerrainType(x, y) == kOutOfBounds)
						m->SetTerrainType(x, y, kObstacle);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, 0, kStartTerrain);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, m->GetMapHeight()-1, kEndTerrain);
		}
			break;
		case 3:
		{
			m = new Map(mapSize, mapSize);
			MakeRandomMap(m, 20);
			for (int y = 0; y < m->GetMapHeight(); y++)
				for (int x = 0; x < m->GetMapWidth(); x++)
					if (m->GetTerrainType(x, y) == kOutOfBounds)
						m->SetTerrainType(x, y, kObstacle);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, 0, kStartTerrain);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, m->GetMapHeight()-1, kEndTerrain);
			break;
		}
		case 4:
		case 5:
		case 6:
		{
			m = new Map(mapSize, mapSize);
			BuildRandomRoomMap(m, which*3);
			for (int y = 0; y < m->GetMapHeight(); y++)
				for (int x = 0; x < m->GetMapWidth(); x++)
					if (m->GetTerrainType(x, y) == kOutOfBounds)
						m->SetTerrainType(x, y, kObstacle);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, 0, kStartTerrain);
			for (int x = 0; x < m->GetMapWidth(); x++)
				m->SetTerrainType(x, m->GetMapHeight()-1, kEndTerrain);
		}
			break;
	}
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
		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);

		srandom((unsigned int)time(0));
		MakeMap();
		
		r = new Racetrack(m);
		r->Reset(s);
		yourPath.push_back(s);
		s.xVelocity=1;
		start = {0,0,0,0};
		end = {0,0,0,0};
		r->GetSuccessors(s, legalSuccessors);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (mapUpdated)
	{
		mapUpdated = false;
		display.StartBackground();
		display.FillRect({-1, -1, 1, 1}, Colors::black);
		
		// Draw map
		r->Draw(display);
		display.EndBackground();
	}

//	astar.Draw(display);
	
	// draw successors (under actual car)
	r->SetColor(Colors::yellow);
	for (auto i : legalSuccessors)
		r->Draw(display, i);

	// Draw "racecar"
	if (path.size() <= 1)
	{
		r->SetColor(Colors::black);
		r->Draw(display, s); //Draws the state of the racetrack
	}
	else { // draw car on solution path}
		r->SetColor(Colors::black);
		r->Draw(display, path[0], path[1], (frame%20)/20.0f);
		frame++;
		if (0 == frame%20)
		{
			path.erase(path.begin());
		}

//		// Draw solution path, if it has been found
//		r->SetColor(Colors::blue);
//		for (int x=1; x < path.size(); x++)
//		{
//			r->DrawLine(display, path[x-1], path[x], 1);
//		}
	}

	if (yourPath.size() > 1)
	{
		r->SetColor(Colors::darkblue);
		for (int x=1; x < yourPath.size(); x++)
		{
			r->DrawLine(display, yourPath[x-1], yourPath[x], 2);
		}
	}
	if (bestPath.size() > 1)
	{
		r->SetColor(Colors::darkred);
		for (int x=1; x < bestPath.size(); x++)
		{
			r->DrawLine(display, bestPath[x-1], bestPath[x], 2);
		}
	}

	// Draw Mouse move to illustrate legality
	if (start != end)
	{
		auto act = r->GetAction(start, end);
//		std::cout << start << " to " << end << " via " << act << "\n";
		bool legal = r->Legal(start, act);
		if (legal && act.hitGoal == true)
			r->SetColor(Colors::lightgreen);
		else if (legal)
			r->SetColor(Colors::darkgreen);
		else
			r->SetColor(Colors::red);
		r->DrawLine(display, start, end, 1.0);
	}

	if (r->GoalTest(s, s))
	{
		display.FillRect({-1+0.01f, -0.1f, 1-0.01f, 0.1f}, Colors::lighterblue);
		display.FrameRect({-1+0.01f, -0.1f, 1-0.01f, 0.1f}, Colors::lightblue, 0.01f);
		display.DrawText("Passed Finish Line", {0, 0}, Colors::purple, 0.15f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		std::string s = "Your moves: "+std::to_string(yourPath.size()-1);
		std::string s2 = "Optimal moves: "+std::to_string(bestPath.size()-1);
		display.DrawText(s.c_str(), {-1+0.02f, 0.1}, Colors::darkblue, 0.04f, Graphics::textAlignLeft, Graphics::textBaselineBottom);
		display.DrawText(s2.c_str(), {1-0.02f, 0.1}, Colors::darkred, 0.04f, Graphics::textAlignRight, Graphics::textBaselineBottom);
	}
	else if (legalSuccessors.size() == 0)
	{
		display.FillRect({-1+0.01f, -0.1f, 1-0.01f, 0.1f}, Colors::lighterred);
		display.FrameRect({-1+0.01f, -0.1f, 1-0.01f, 0.1f}, Colors::lightred, 0.01f);
		display.DrawText("You Crashed!", {0, 0}, Colors::purple, 0.15f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		display.DrawText("Reset to try again", {0.0f, 0.1f}, Colors::darkred, 0.04f, Graphics::textAlignCenter, Graphics::textBaselineBottom);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}



uint64_t random64()
{
	uint64_t r1 = random();
	uint64_t r2 = random();
	return (r1<<32)|r2;
}



void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key) // handles keypresses that change display
{
	switch (key)
	{
		case 'r':
			// TODO: Reset to start state
			r->Reset(s);
			s.xVelocity=1;
			r->GetSuccessors(s, legalSuccessors);
			path.clear();
			yourPath.clear();
			yourPath.push_back(s);
			bestPath.clear();
			break;
		case kUpArrow:
		case 'w': // y velocity goes up
			if (r->GoalTest(s, s))
				break;
			v.xDelta = 0;
			v.yDelta = -1;
			if (r->Legal(s, v))
				r->ApplyAction(s, v);
			r->GetSuccessors(s, legalSuccessors);
			break;
		case kDownArrow:
		case 's':
			if (r->GoalTest(s, s))
				break;
			v.xDelta = 0;
			v.yDelta = 1;
			if (r->Legal(s, v))
				r->ApplyAction(s, v);
			r->GetSuccessors(s, legalSuccessors);
			break;
		case kLeftArrow:
		case 'a':
			if (r->GoalTest(s, s))
				break;
			v.xDelta = -1;
			v.yDelta = 0;
			if (r->Legal(s, v))
				r->ApplyAction(s, v);
			r->GetSuccessors(s, legalSuccessors);
			break;
		case kRightArrow:
		case 'd':
			if (r->GoalTest(s, s))
				break;
			v.xDelta = 1;
			v.yDelta = 0;
			if (r->Legal(s, v))
				r->ApplyAction(s, v);
			r->GetSuccessors(s, legalSuccessors);
			break;
		case ' ':
			if (r->GoalTest(s, s))
				break;
			v.xDelta = 0;
			v.yDelta = 0;
			if (r->Legal(s, v))
				r->ApplyAction(s, v);
			r->GetSuccessors(s, legalSuccessors);
			break;
		case 'o':
		{
			r->Reset(s);
			s.xVelocity = 1;
//			float bound = 10.5;
//
//			// WA*
//			astar.SetWeight(bound);
//			astar.GetPath(r, s, s, path);
//			std::cout << "WA*(" << std::to_string(bound) << "): " << astar.GetNodesExpanded() << " nodes, length " << r->GetPathLength(path) << "\n";
//
//			// PWXU
//			astar.SetPhi([=](double h,double g){return (g < (2*bound-1) * h)?(g/(2*bound-1) + h):(g/bound+h/bound);});
//			astar.GetPath(r, s, s, path);
//			std::cout << "PWXU(" << std::to_string(bound) << "): " << astar.GetNodesExpanded() << " nodes, length " << r->GetPathLength(path) << "\n";
//
//			// PWXD
//			astar.SetPhi([=](double h,double g){return (h>g)?(g+h):(g/bound+h*(2*bound-1)/bound);});
//			astar.GetPath(r, s, s, path);
//			std::cout << "PWXD(" << std::to_string(bound) << "): " << astar.GetNodesExpanded() << " nodes, length " << r->GetPathLength(path) << "\n";
//
//			// XDP
//			astar.SetPhi([=](double x,double y){return (y+(2*bound-1)*x+sqrt((y-x)*(y-x)+4*bound*y*x))/(2*bound);});
//			astar.GetPath(r, s, s, path);
//			std::cout << "XDP(" << std::to_string(bound) << "): " << astar.GetNodesExpanded() << " nodes, length " << r->GetPathLength(path) << "\n";

			// A*
			astar.SetWeight(1);
			astar.GetPath(r, s, s, path);
			std::cout << "A*: " << astar.GetNodesExpanded() << " nodes, length " << r->GetPathLength(path) << "\n";

			break;
		}
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
			MakeMap('6'-key);
			r->UpdateMap(m);
			r->Reset(s);
			s.xVelocity=1;
			
			path.clear();
			yourPath.clear();
			yourPath.push_back(s);
			bestPath.clear();
			mapUpdated = true;
			r->GetSuccessors(s, legalSuccessors);
			break;
			
		case 'm':
			MakeMap();
			r->UpdateMap(m);
			r->Reset(s);
			s.xVelocity=1;

			path.clear();
			yourPath.clear();
			yourPath.push_back(s);
			bestPath.clear();
			mapUpdated = true;
			r->GetSuccessors(s, legalSuccessors);
			break;
		default:
			break;
		
	}
	
	
}


/*
 * Code runs when user clicks or moves mouse in window
 *
 * Application does not currently need mouse support
 */
bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	int x, y;
	m->GetPointFromCoordinate(loc, x, y);

	switch (mType)
	{
		case kMouseDrag:
		{
			if (button == kRightButton)
			{
				end.xLoc = x;
				end.yLoc = y;
				start.xVelocity = end.xLoc - start.xLoc;
				start.yVelocity = end.yLoc - start.yLoc;
			}
		}
			break;
		case kMouseDown:
		{
			if (button == kRightButton)
			{
				start.xLoc = x;
				start.yLoc = y;
				start.xVelocity = 0;
				start.yVelocity = 0;
				end.xLoc = x;
				end.yLoc = y;
				end.xVelocity = 0;
				end.yVelocity = 0;
//				std::cout << "h(" << start << ") = " << r->HCost(start) << "\n";
//				astar.GetPath(r, start, start, path);
//				if (r->GetPathLength(path) < r->HCost(start))
//				{
//					printf("Error - heuristic is overestimating! (Solution: %f)\n", r->GetPathLength(path));
//				}
			}
			else if (button == kLeftButton)
			{
				for (auto i : legalSuccessors)
				{
					if (i.xLoc == x && i.yLoc == y)
					{
						s = i;
						r->GetSuccessors(s, legalSuccessors);
						yourPath.push_back(s);
						if (r->GoalTest(s, s))
						{
							RacetrackState tmp;
							r->Reset(tmp);
							tmp.xVelocity = 1;
							astar.SetWeight(1);
							astar.GetPath(r, tmp, tmp, bestPath);
						}
						break;
					}
				}
			}
		}
			break;
		default:
		{
			start = {0, 0, 0, 0};
			end = {0, 0, 0, 0};
		}
			break;
	}
	return true;
}

