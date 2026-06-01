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
#include "NBS.h"
#include "fMM.h"
#include "SVGUtil.h"

enum game {
	kNotInGame = 0,
	kUnidirectional = 1,
	kBidirectional = 2,
	kBidirectionalTwo = 3
};

enum mode {
	kSetStartGoal = 0,
	kMark = 1,
	kMark2 = 2,
	kFindPath = 3,
	kDrawObstacles = 4
};

enum algorithm {
	kDijkstra = 0,
	kAStar = 1,
	kFMM = 2,
	kNBS = 3
};


MapEnvironment *me = 0;
ZeroHeuristic<xyLoc> z;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
TemplateAStar<xyLoc, tDirection, MapEnvironment> AStarForward, AStarBackward;
fMM<xyLoc, tDirection, MapEnvironment> fmm;
NBS<xyLoc, tDirection, MapEnvironment> nbs;
std::vector<xyLoc> path;
std::vector<xyLoc> hd;
std::vector<xyLoc> farStates;
bool failedBidirectional = false;
bool mapChange = true;

xyLoc start, goal, mark, mark2;

game gameMode = kNotInGame;
mode m = kDrawObstacles;
algorithm a = kDijkstra;

bool recording = false;
bool running = false;
bool necessary = false;
int stepsPerFrame = 1;

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
	InstallKeyboardHandler(MyDisplayHandler, "Mark", "Mark state on map", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Set", "Set start and goal", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find Path", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Draw", "Draw Obstacles", kAnyModifier, 'd');

	InstallKeyboardHandler(MyDisplayHandler, "Necessary", "Draw necessary expansions", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Do single expansion step", kAnyModifier, 'o');

	InstallKeyboardHandler(MyDisplayHandler, "Dijkstra", "Choose dikjstra", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Choose A*", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "fMM", "Choose fMM", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "NBS", "Choose NBS", kAnyModifier, '4');
	InstallKeyboardHandler(MyDisplayHandler, "Map", "Load bigger map", kAnyModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "Map", "Load smaller map", kAnyModifier, '9');

	InstallKeyboardHandler(MyDisplayHandler, "]", "Double steps per frame", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "[", "Half steps per frame", kAnyModifier, '[');

	InstallKeyboardHandler(MyDisplayHandler, "Record", "Toggle SVG recording", kAnyModifier, 'r');

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
//		printf("Window %ld created\n", windowID);
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/hrt201d.map");
		const int mapSize = 35;
		const int offset = 4;
		Map *map = new Map(mapSize, mapSize);
		for (int x = 0; x < mapSize; x++)
			for (int y = 0; y < mapSize; y++)
				map->SetTerrainType(x, y, kOutOfBounds);
		for (int x = offset; x < mapSize-offset; x++)
			for (int y = offset; y < mapSize-offset; y++)
				map->SetTerrainType(x, y, kGround);
		map->SetTerrainType(offset, offset, mapSize-offset, offset, kTrees);
		map->SetTerrainType(offset, mapSize-offset, mapSize-offset, mapSize-offset, kTrees);
		map->SetTerrainType(offset, offset, offset, mapSize-offset, kTrees);
		map->SetTerrainType(mapSize-offset, offset, mapSize-offset, mapSize-offset, kTrees);
		map->SetTerrainType(.75*mapSize, .25*mapSize, .75*mapSize, .75*mapSize, kTrees);
		submitTextToBuffer("Modify the map however you want; then choose a start/goal");
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		me->SetDiagonalCost(1.5);
		start.x = start.y = 0xFFFF;
		mark.x = 0xFFFF;
		mark2.x = 0xFFFF;
		printf("Done creating window\n");
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (mapChange)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		mapChange = false;
	}
		
	if (!(start.x == 0xFFFF || running))
	{
		//me->Draw(display, start);
		me->SetColor(1, 0, 0);
		me->Draw(display, start);
		me->SetColor(0, 0, 1);
		me->Draw(display, goal);
		me->SetColor(0.5, 0.5, 0.5);
		me->DrawArrow(display, start, goal, 3);
	}
	
	if (running)
	{
		if (a != kNBS)
		{
			fmm.Draw(display);
			for (int x = 0; x < stepsPerFrame; x++)
			{
				if (path.size() == 0)
					fmm.DoSingleSearchStep(path);
				else {
					// TODO: fmm is returning the goal twice in the path
//					printf("Found path: ");
//					for (auto s : path)
//						std::cout << s << " ";
//					std::cout << "\n";
					break;
				}
			}
		}
		else {
			nbs.Draw(display);
			for (int x = 0; x < stepsPerFrame; x++)
				if (path.size() == 0)
					nbs.DoSingleSearchStep(path);
		}

		if (path.size() != 0) {
			me->SetColor(0, 0, 1);
			for (int x = 1; x < path.size(); x++)
			{
				me->DrawLine(display, path[x-1], path[x], 3);
			}

			if (necessary)
			{
				double opt = me->GetPathLength(path);
				//printf("Measuring opt as %f\n", opt);
				for (int x = 0; x < AStarForward.GetNumItems(); x++)
				{
					const auto &i = AStarForward.GetItem(x);
					if (i.where == kClosedList && fless(i.g+i.h, opt))
					{
						me->SetColor(Colors::black);
						me->DrawAlternate(display, i.data);
					}
				}
			}
		}
		// Win condition for unidirectional search
		if (gameMode == kUnidirectional && path.size() != 0)
		{
			gameMode = kNotInGame;
			// Check to see if successful
			AStarOpenClosedDataWithF<xyLoc> item;
			double opt = me->GetPathLength(path);
			if (AStarForward.GetClosedItem(mark, item))
			{
				if (fless(item.g+item.h, opt))
				{
					submitTextToBuffer("You win; marked state was a necessary expansion! Other answers shown.");
					necessary = true;
				}
				else if (fequal(item.g+item.h, opt)) {
					submitTextToBuffer("Sorry; while your state was expanded, it doesn't have f<C*; try again!");
					gameMode = kUnidirectional;
					a = kAStar;
					m = kMark;
				}
			}
			else { // failed - not expanded
				submitTextToBuffer("Sorry, your state was not expanded; try another state!");
				gameMode = kUnidirectional;
				a = kAStar;
				m = kMark;
			}
		}
		// Always lose the second game
		if (gameMode == kBidirectional && path.size() != 0)
		{
			m = kMark;
			submitTextToBuffer("Sorry, your state was not expanded; try another!");
			failedBidirectional = true;
		}
		// Win condition for 2 states
		if (gameMode == kBidirectionalTwo && path.size() != 0)
		{
			// Win if both states are on optimal path
			auto m1 = find(path.begin(), path.end(), mark);
			auto m2 = find(path.begin(), path.end(), mark2);
			if (m1 == path.end() || m2 == path.end())
			{
				submitTextToBuffer("Good guess, but there is a better answer; try again!");
				m = kMark;
//				mark = mark2 = xyLoc();
			}
			else if (m1 != path.end() && m2 != path.end())
			{
				submitTextToBuffer("Fantastic! You chose two states on the optimal path; you win!");
				gameMode = kNotInGame;
			}
		}
	}

	if (mark.x != 0xFFFF)
	{
		me->SetColor(0, 0, 1);
		me->DrawAlternate(display, mark);
	}
	if (mark2.x != 0xFFFF)
	{
		me->SetColor(0.5, 0, 1);
		me->DrawAlternate(display, mark2);
	}

	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/bidir-%d%d%d%d.svg",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		
		MakeSVG(display, fname, 800, 800);
		
		printf("Saved %s\n", fname);
		frameCnt++;
//		if (path.size() == 0)
//		{
//			MyDisplayHandler(windowID, kNoModifier, 'o');
//		}
//		else {
//			recording = false;
//		}
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

const char *GetAlgName()
{
	switch (a)
	{
		case kDijkstra: return "Dijkstra";
		case kAStar: return "AStar";
		case kFMM: return "Oracle";
		case kNBS: return "NBS";
	}
	return "Unknown";
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'o':
			if (a != kNBS)
				fmm.DoSingleSearchStep(path);
			else
				nbs.DoSingleSearchStep(path);
			break;
		case 'm':
			if (mod == kShiftDown)
			{
				m = kMark2;
				submitTextToBuffer("Mode: Mark Second State!");
			}
			else {
				m = kMark;
				submitTextToBuffer("Mode: Mark State");
			}
			//running = false;
			necessary = false;
			break;
		case 'r':
			recording = !recording;
			break;
		case 's':
			m = kSetStartGoal;
			submitTextToBuffer("Mode: Set Start/Goal");
			necessary = false;
			break;
		case 'd':
			m = kDrawObstacles;
			submitTextToBuffer("Mode: Draw Obstacles");
			necessary = false;
			break;
		case 'p':
			m = kFindPath;
			running = false;
			necessary = false;
			submitTextToBuffer("Mode: Find path (");
			appendTextToBuffer(GetAlgName());
			appendTextToBuffer(") [Click to start search]");
			break;
		case 'n':
			necessary = !necessary;
			break;
		case '0':
			me->GetMap()->Load("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
			break;
		case '9':
		{
			const int mapSize = 35;
			const int offset = 4;
			Map *map = me->GetMap();
			map->Scale(mapSize, mapSize);
			for (int x = 0; x < mapSize; x++)
				for (int y = 0; y < mapSize; y++)
					map->SetTerrainType(x, y, kOutOfBounds);
			for (int x = offset; x < mapSize-offset; x++)
				for (int y = offset; y < mapSize-offset; y++)
					map->SetTerrainType(x, y, kGround);
			map->SetTerrainType(offset, offset, mapSize-offset, offset, kTrees);
			map->SetTerrainType(offset, mapSize-offset, mapSize-offset, mapSize-offset, kTrees);
			map->SetTerrainType(offset, offset, offset, mapSize-offset, kTrees);
			map->SetTerrainType(mapSize-offset, offset, mapSize-offset, mapSize-offset, kTrees);
			map->SetTerrainType(.75*mapSize, .25*mapSize, .75*mapSize, .75*mapSize, kTrees);
		}
			break;
		case '1':
			if (start == goal)
			{
				submitTextToBuffer("Please choose a unique start and goal before getting started");
				break;
			}
			gameMode = kUnidirectional;
			a = kAStar;
			m = kMark;
			mark = mark2 = xyLoc();
			running = false;
			necessary = false;
			submitTextToBuffer("Click a state that you think all unidirectional algorithms must expand when solving this problem");
			break;
		case '2':
			if (start == goal)
			{
				submitTextToBuffer("Please choose a unique start and goal before getting started");
				break;
			}
			gameMode = kBidirectional;
			a = kFMM;
			m = kMark;
			mark = mark2 = xyLoc();
			running = false;
			necessary = false;
			submitTextToBuffer("Click a state that you think all bidirectional algorithms must expand when solving this problem");
			break;
		case '3':
			if (!failedBidirectional)
			{
				submitTextToBuffer("Try the first bidirectional search game first!");
				break;
			}
			if (start == goal)
			{
				submitTextToBuffer("Please choose a unique start and goal before getting started");
				break;
			}
			submitTextToBuffer("Choose two states and see if the oracle can avoid both!");
			gameMode = kBidirectionalTwo;
			a = kFMM;
			m = kMark;
			mark = mark2 = xyLoc();
			running = false;
			necessary = false;

			break;
		case '4':
			a = kNBS;
			m = kFindPath;
			running = false;
			necessary = false;
			submitTextToBuffer("Mode: Find path (");
			appendTextToBuffer(GetAlgName());
			appendTextToBuffer(") [Click to start search]");
			break;
		case '[':
			stepsPerFrame /= 2;
			break;
		case ']':
			stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
			break;
			
	}
	
}

void SetStartGoalHandler(uint16_t x, uint16_t y, tMouseEventType mType)
{
	static bool currentlyDrawing = false;

	switch (mType)
	{
		case kMouseDown:
		{
			currentlyDrawing = true;
			running = false;
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				start.x = x;
				start.y = y;
				goal = start;
				//printf("Hit (%d, %d)\n", x, y);
			}
		}
		case kMouseDrag:
		{
			if (me->GetMap()->GetTerrainType(x, y) == kGround && currentlyDrawing)
			{
				//printf("drag (%d, %d)\n", x, y);
				goal.x = x;
				goal.y = y;
			}
			break;
		}
		case kMouseUp:
		{
			currentlyDrawing = false;
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				goal.x = x;
				goal.y = y;
//				AStarForward.GetPath(me, start, goal, path);
//				AStarBackward.GetPath(me, goal, start, path);
			}
		}
	}
}


void DrawHandler(uint16_t x, uint16_t y, tMouseEventType mType)
{
	static bool currentlyDrawing = false;
	static bool draw = true;
	static int lastx = 0, lasty = 0;
	if (mType == kMouseUp)
	{
		currentlyDrawing = false;
		return;
	}
	if (me->GetMap()->GetTerrainType(x, y) == kOutOfBounds)
		return;
	mapChange = true;
	if (mType == kMouseDown)
	{
		currentlyDrawing = true;
		if (me->GetMap()->GetTerrainType(x, y) == kGround)
			draw = true;
		else
			draw = false;
		if (draw)
			me->GetMap()->SetTerrainType(x, y, kTrees);
		else
			me->GetMap()->SetTerrainType(x, y, kGround);
	}
	else if (mType == kMouseDrag && currentlyDrawing)
	{
		if (draw)
			me->GetMap()->SetTerrainType(x, y, lastx, lasty, kTrees);
		else
			me->GetMap()->SetTerrainType(x, y, lastx, lasty, kGround);
	}
	lastx = x; lasty = y;
}


void GetPathHandler(tMouseEventType mType)
{
	if (mType != kMouseUp)
		return;

	AStarForward.GetPath(me, start, goal, path);
	AStarBackward.GetPath(me, goal, start, path);

	Heuristic<xyLoc> *h;
	switch (a)
	{
		case kDijkstra: fmm.SetFraction(1); h = &z; break;
		case kAStar: fmm.SetFraction(1); h = me; break;
		case kFMM:
		{
			// find optimal path length
			double optLen;
			AStarForward.GetClosedListGCost(goal, optLen);
			
			// find distance to marked state
			double fraction = 0.5;
			double markedDistanceForward;
			bool foundForward = AStarForward.GetClosedListGCost(mark, markedDistanceForward);
			double markedDistanceBackward;
			bool foundBackward = AStarBackward.GetClosedListGCost(mark, markedDistanceBackward);
			std::cout << "Marked: " << mark << "\n";
			if (foundForward && foundBackward)
			{
				printf("Found both directions\n");
				if (fequal(markedDistanceForward+markedDistanceBackward, optLen))
				{
					fraction = markedDistanceForward/optLen;
					printf("On optimal - setting %1.10f\n", fraction); // 11 of 27.5
				}
				else {
					printf("Off optimal\n");
					fraction = (markedDistanceForward/optLen+1-markedDistanceBackward/optLen)/2.0;
				}
//				fraction = markedDistanceForward/(markedDistanceForward+markedDistanceBackward);
			}
			else if (foundForward) // g/fraction > optLen ; fraction < g/optLen
			{
				printf("Found forward cost %f\n", markedDistanceForward);
				fraction = markedDistanceForward/optLen-0.001;
			}
			else if (foundBackward)
			{
				printf("Found backward cost %f\n", markedDistanceBackward);
				fraction = 1-markedDistanceBackward/optLen+0.001;
			}
//			printf("Marked state forward: %1.2f")
			// set the fraction to be the fraction of that state along the optimal path
			fmm.SetFraction(fraction);
			h = me;
			break;
		}
		case kNBS:
			nbs.InitializeSearch(me, start, goal, me, me, path);
			running = true;
			return;
	}
	fmm.InitializeSearch(me, start, goal, h, h, path);
	running = true;
}

void MarkPointHandler(uint16_t x, uint16_t y, tMouseEventType mType)
{
	if (m == kMark)
	{
		mark.x = x;
		mark.y = y;
		mark2 = xyLoc();
		if (gameMode != kNotInGame && mType == kMouseUp)
		{
			if (gameMode == kBidirectionalTwo)
			{
				running = false;
				submitTextToBuffer("Choose a second state!");
				m = kMark2;
			}
			else if (gameMode == kBidirectional) {
				submitTextToBuffer("Oracle selected bidirectional algorithm. Running...");
				GetPathHandler(kMouseUp);
				m = kFindPath;
			}
			else {
				submitTextToBuffer("Solving with unidirectional search...");
				GetPathHandler(kMouseUp);
				m = kFindPath;
			}
		}
	}
	else if (m == kMark2)
	{
		mark2.x = x;
		mark2.y = y;
		if (gameMode == kBidirectionalTwo && mType == kMouseUp)
		{
			submitTextToBuffer("Running search to see if you are correct...");
			GetPathHandler(kMouseUp);
			m = kFindPath;
		}
	}
}

bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button != kLeftButton)
		return false;
	int x, y;
	me->GetMap()->GetPointFromCoordinate(loc, x, y);
	switch (m)
	{
		case kSetStartGoal: SetStartGoalHandler(x, y, mType); break;
		case kMark2:
		case kMark: MarkPointHandler(x, y, mType); break;
		case kFindPath: GetPathHandler(mType); break;
		case kDrawObstacles: DrawHandler(x, y, mType); break;
	}
	return true;
}
	
	
	
	
