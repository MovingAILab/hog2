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
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include <string>
#include <sstream>
#include "MapGenerators.h"
#include "SVGUtil.h"
#include "GridHeuristics.h"
#include "BOBAStar.h"

/**
 TODO: Delete the following code
 */
int p1Score = 0;
int p2Score = 0;
bool ready = true;
bool p1Hit = false;
bool p2Hit = false;
void MyGameHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r':
			ready = true;
			p1Hit = p2Hit = false;
			break;
		case '1': p1Score -= 100; break;
		case '2': p1Score += 100; break;
		case '9': p2Score -= 100; break;
		case '0': p2Score += 100; break;
		case 'z':
			if (ready)
			{
				ready = false;
				p1Hit = true;
			}
			break;
		case '/':
			if (ready)
			{
				ready = false;
				p2Hit = true;
			}
			break;
	};
}



enum mode {
	kAddDH = 0,
	kIdentifyLowHeuristic = 1,
	kIdentifyHighHeuristic = 2,
	kFindPath = 3,
	kMeasureHeuristic = 4
};

enum mapType {
	kRandomMap5 = 0,
	kRandomMap10 = 1,
	kRoomMap8 = 2,
	kRoomMap16 = 3,
	kMazeMap1 = 4,
	kMazeMap2 = 5,
	kMazeMap25 = 6,
	kDAMap = 7
};

class AvoidBorderEnvironment : public MapEnvironment {
public:
	AvoidBorderEnvironment(Map *m)
	:MapEnvironment(m) {
		SetDiagonalCost(1.5);
		costs.resize(GetMaxHash());
		maxDist = 1;
		std::vector<xyLoc> q1, q2;
		std::vector<xyLoc> succ;
		// mark borders as depth 1
		for (int x = 0; x < this->GetMaxHash(); x++)
		{
			xyLoc l;
			this->GetStateFromHash(x, l);
			if (m->GetTerrainType(l.x, l.y) == kGround && this->GetNumSuccessors(l) != 8)
			{
				costs[x] = maxDist;
				q2.push_back(l);
			}
		}
		while (q2.size() > 0)
		{
			q1.swap(q2);
			q2.clear();
			maxDist++;
			while (!q1.empty())
			{
				xyLoc l = q1.back();
				q1.pop_back();
				GetSuccessors(l, succ);
				for (auto s : succ)
				{
					if (costs[GetStateHash(s)] == 0)
					{
						costs[GetStateHash(s)] = maxDist;
						q2.push_back(s);
					}
				}
			}
		}
	}
	double GCost(const xyLoc &node1, const xyLoc &node2) const
	{ return MapEnvironment::GCost(node1, node2) + std::max(maxDist-costs[GetStateHash(node1)], maxDist-costs[GetStateHash(node2)]); }
	double HCost(const xyLoc &node1, const xyLoc &node2) const { return 0; }
	void Draw(Graphics::Display &d)
	{
		for (int x = 0; x < GetMaxHash(); x++)
		{
			xyLoc l;
			GetStateFromHash(x, l);
			int dist = costs[x];
			Graphics::rect r;
			GLdouble px, py, t, rad;
			map->GetCoord(l.x, l.y, px, py, t, rad);
			if (map->GetTerrainType(l.x, l.y) != kGround)
				continue;
			r.left = (float)(px-rad);
			r.top = (float)(py-rad);
			r.right = (float)(px+rad);
			r.bottom = (float)(py+rad);
			
			rgbColor c = Colors::white;
			c.g = c.g*((float)dist/maxDist);
			c.b = c.b*((float)dist/maxDist);
			c.r = 0.5*c.r*((float)dist/maxDist)+0.5;
			d.FillRect(r, c);
		}
	}
private:
	std::vector<int> costs;
	int maxDist;
};

template <class state, class action, class environment>
class ReverseSearchHeuristic : public Heuristic<state> {
public:
	ReverseSearchHeuristic(environment *e)
	{
		env = e;
		astar.SetStopAfterGoal(false);
	}
	double HCost(const state &node1, const state &node2) const
	{
		if (node2 == goal)
		{
			double gcost;
			astar.GetClosedListGCost(node1, gcost);
			return gcost;
		}
		else {
			goal = node2;
			astar.GetPath(env, node2, node1, path);
			double gcost;
			astar.GetClosedListGCost(node1, gcost);
			return gcost;
		}
	}
private:
	environment *env;
	mutable state goal;
	mutable TemplateAStar<state, action, environment> astar;
	// actual path not used
	mutable std::vector<state> path;
};

std::vector<xyLoc> path;

int selectedGoal = -1;
graphState nodeToDraw = -1;
xyLoc stateToDraw;
xyLoc start, goal;
MapEnvironment *me = 0;
Map *map=0;
std::vector<std::pair<graphState, graphState>> pivots;
void LoadMap(Map *m);
void StartSearch();

bool selectingPath = false;
bool recording = false;
bool running = false;
bool mapChange = true;
bool graphChanged = true;
int stepsPerFrame = 1;

AvoidBorderEnvironment *me2 = 0;
BOBAStar<xyLoc, tDirection, MapEnvironment> *b = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
ReverseSearchHeuristic<xyLoc, tDirection, MapEnvironment> *fh1 = 0;
ReverseSearchHeuristic<xyLoc, tDirection, AvoidBorderEnvironment> *fh2 = 0;

ReverseSearchHeuristic<xyLoc, tDirection, MapEnvironment> *bh1 = 0;
ReverseSearchHeuristic<xyLoc, tDirection, AvoidBorderEnvironment> *bh2 = 0;

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
	InstallKeyboardHandler(MyGameHandler, "Score", "Score", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyGameHandler, "Reset", "Reset", kAnyModifier, 'r');
	InstallKeyboardHandler(MyGameHandler, "p1", "p1", kAnyModifier, 'z');
	InstallKeyboardHandler(MyGameHandler, "p2", "p2", kAnyModifier, '/');

	InstallKeyboardHandler(MyDisplayHandler, "Load Map", "Load map", kAnyModifier, '0', '7');

	InstallKeyboardHandler(MyDisplayHandler, "Lerp", "restart lerp", kAnyModifier, 'l');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear DH", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Speed Up", "Increase speed of A* search", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slow Down", "Decrease speed of A* search", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Add DH", "Switch to mode for add/study DH placement", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Show DH", "Toggle drawing the DH", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Measure", "Measure Heuristic", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find path using current DH", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Test High", "Test to find state pairs with high heuristic value", kAnyModifier, 'h');
	InstallKeyboardHandler(MyDisplayHandler, "Test Low", "Test to find state pairs with low heuristic value", kAnyModifier, 'l');

	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDown|kMouseDrag));
}

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

void SaveSVG(Graphics::Display &d, int port = 1)
{
	const std::string baseFileName = "/Users/nathanst/Pictures/hog2/BOBA_";
	static int count = 0;
	std::string fname;
	do {
		fname = baseFileName+std::to_string(count)+".svg";
		count++;
	} while (fileExists(fname.c_str()));
	printf("Save to '%s'\n", fname.c_str());
	MakeSVG(d, fname.c_str(), 1024, 1024, port);
}

void StartSearch()
{
	selectingPath = false;
	stepsPerFrame = 1;
	b->InitializeSearch(me, me2, fh1, fh2, bh1, bh2, start, goal, path);
}

void CreateMap(mapType which)
{
	if (map)
		delete map;
	if (me)
		delete me;
	if (me2)
		delete me2;
	delete fh1;
	delete fh2;
	delete bh1;
	delete bh2;

	nodeToDraw = -1;
	
	static int seed = 20230712;
	srandom(seed++);
	
	map = new Map(75,75);
	switch (which)
	{
		case kRandomMap10:
			map = new Map(150,150);
			MakeRandomMap(map, 10);
			break;
		case kRandomMap5:
			map = new Map(150,150);
			MakeRandomMap(map, 5);
			break;
		case kRoomMap8:
			BuildRandomRoomMap(map, 8);
			break;
		case kRoomMap16:
			BuildRandomRoomMap(map, 16);
			break;
		case kMazeMap1:
			MakeMaze(map, 4);
			break;
		case kMazeMap2:
			MakeMaze(map, 8);
			break;
		case kMazeMap25:
			MakeMaze(map, 25);
			break;
		case kDAMap:
			LoadMap(map);
			break;
	}
	
	me = new MapEnvironment(map);
	me->SetDiagonalCost(1.5);
	me2 = new AvoidBorderEnvironment(map);
	fh1 = new ReverseSearchHeuristic<xyLoc, tDirection, MapEnvironment>(me);
	fh2 = new ReverseSearchHeuristic<xyLoc, tDirection, AvoidBorderEnvironment>(me2);
	bh1 = new ReverseSearchHeuristic<xyLoc, tDirection, MapEnvironment>(me);
	bh2 = new ReverseSearchHeuristic<xyLoc, tDirection, AvoidBorderEnvironment>(me2);
	mapChange = true;

	do {
		start.x = random()%map->GetMapWidth();
		start.y = random()%map->GetMapHeight();
	} while (map->GetTerrainType(start.x, start.y) != kGround);
	do {
		goal.x = random()%map->GetMapWidth();
		goal.y = random()%map->GetMapHeight();
	} while (map->GetTerrainType(goal.x, goal.y) != kGround);

	//	h1 = new GridEmbeddingEnvironment(Map *m, Heuristic<xyLoc> *h)
	if (b == 0)
		b = new BOBAStar<xyLoc, tDirection, MapEnvironment>();

	StartSearch();
	
//	astar.SetHeuristic(h1);
//	astar.InitializeSearch(me, {1,1}, {30, 30}, path);
//	astar.SetHeuristic(h2);
//	astar.InitializeSearch(me2, {1,1}, {30, 30}, path);
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
		ReinitViewports(windowID, {-1.0f, -1.f, 0.f, 0.f}, kScaleToSquare);
		AddViewport(windowID, {-1.0f, 0.0f, 0.0f, 1.0f}, kScaleToSquare); // kTextView
		AddViewport(windowID, {0.05f, -0.95f, 0.95f, 0.95f}, kScaleToSquare); // kTextView

//		CreateMap(kRoomMap8);
		CreateMap(kMazeMap25);

		/**
		 TODO: Delete the following code
		 */
		ReinitViewports(windowID, {-1.0f, -1.f, 1.f, 1.f}, kScaleToSquare);
		setTextBufferVisibility(false);
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	
	/**
	 TODO: Delete the following code
	 */
	display.FillRect({-1, -1, 1, 1}, Colors::black);
	std::string s1 = "Team 1: "+std::to_string(p1Score);
	std::string s2 = "Team 2: "+std::to_string(p2Score);
	display.DrawText(s1.c_str(), {-1.0, -0.9}, Colors::white, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText(s2.c_str(), {1.0, -0.9}, Colors::white, 0.1f, Graphics::textAlignRight, Graphics::textBaselineTop);
	if (ready != true)
	{
		if (p1Hit)
		{
			display.FillRect({-1, 0, 0, 1}, Colors::green);
//			display.FillRect({0, 0, 1, 1}, Colors::red);
			display.DrawText("1", {-0.5, 0.5}, Colors::black, 0.3f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		}
		if (p2Hit)
		{
//			display.FillRect({-1, 0, 0, 1}, Colors::red);
			display.FillRect({0, 0, 1, 1}, Colors::green);
			display.DrawText("2", {0.5, 0.5}, Colors::black, 0.3f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		}
	}
	return;
	
	switch (viewport) {
		case 0:
		{
			// Draw map & search
			me2->Draw(display);
			if (selectingPath)
			{
				me->DrawLine(display, start, goal, 3);
			}
			else {
				for (int x = 0; x < stepsPerFrame; x++)
				{
					if (b->DoSingleSearchStep(path))
					{
						recording = false;
						stepsPerFrame = 0;
						break;
					}
				}
				if (stepsPerFrame > 0)
					b->Draw(display, 0);
				b->DrawAllPaths(display);
				if (selectedGoal != -1)
					b->DrawGoal(display, selectedGoal, Colors::lightgreen, 10.0f);
			}
			break;
		}
		case 1:
		{
			// Draw map & search
			me2->Draw(display);
			if (selectingPath)
			{
				me->DrawLine(display, start, goal, 3);
			}
			else {
//				for (int x = 0; x < stepsPerFrame; x++)
//				{
//					if (b->DoSingleSearchStep(path))
//					{
//						recording = false;
//						stepsPerFrame = 0;
//						break;
//					}
//				}
				if (stepsPerFrame > 0)
					b->Draw(display, 1);
				b->DrawAllPaths(display);
				if (selectedGoal != -1)
					b->DrawGoal(display, selectedGoal, Colors::lightgreen, 10.0f);
			}
			break;
		}
		case 2:
		{
			b->DrawFrontier(display, selectedGoal);
			// Draw bi-objective plot
		}
	}
	if (viewport == 1)
	
	if (viewport == 0 && recording)
	{
		std::string fname = "/Users/nathanst/Pictures/hog2/BOA_";
		static int count = 0;
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(GetContext(windowID)->display, (fname+std::to_string((count/1000)%10)+std::to_string((count/100)%10)+std::to_string((count/10)%10)+std::to_string(count%10)+".svg").c_str(), 600, 600, 0);
		count++;
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
{ return;
	switch (key)
	{
		case 'r':
			recording = !recording;
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
			CreateMap((mapType)(key-'0'));
			break;
		case '[':
		{
			stepsPerFrame /= 2;
//			std::string s = std::to_string(stepsPerFrame)+" steps per frame";
//			submitTextToBuffer(s.c_str());
		}
			break;
		case ']':
		{
			if (stepsPerFrame <= 16384)
				stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
//			std::string t = std::to_string(stepsPerFrame)+" steps per frame";
//			submitTextToBuffer(t.c_str());
		}
			break;
		case '|':
		case 'a':
			break;
		case 'm':
			break;
		case 'd':
			break;
		case 'p':
		{
		}
			break;
		case 'h':
			break;
	}
}





bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (viewport == 0)
	{
		switch (mType)
		{
			case kMouseDown:
			{
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if (me->GetMap()->GetTerrainType(x, y) == kGround)
				{
					selectingPath = true;
					start.x = goal.x = x;
					start.y = goal.y = y;
//					std::cout << goal << "\n";
				}
			}
				break;
			case kMouseDrag:
			{
				if (!selectingPath)
					break;
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if (me->GetMap()->GetTerrainType(x, y) == kGround)
				{
					goal.x = x;
					goal.y = y;
//					std::cout << goal << "\n";
				}
			}
				break;
			case kMouseUp:
			{
				if (!selectingPath)
					break;
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if (me->GetMap()->GetTerrainType(x, y) == kGround)
				{
					goal.x = x;
					goal.y = y;
//					std::cout << goal << "\n";
				}
				StartSearch();
			}
				break;
			default: break;
		}
	}
	if (viewport == 1)
	{
		selectedGoal = b->GetClosestGoal(loc, 0.1f);
	}
	return true;
}

void LoadMap(Map *m)
{
	m->Scale(194, 205);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT.TTTT..TTT...TTTTTTTT.....TTTTT......TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.TTTT.TTTT..TTT...TTTTTTTT.....TTTTT..TTT...TT@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT.TTTT...T...TTT...TTTTTTTT.....TTTTTTTTTT...TT@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...TTTT...TTTT.......TTT...TTTTTTTT.....TTTTTTTTTTT..TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT..........TTTT..TTT..TTT...TTTTTTTT.....TTTTTT.TTTT..TTTTT.TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT............TTTTTTT..T....TTTTTTTT.....TT.TT...TTT...TTT...TTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT............TTTTTTT..T....TTTTTTTT.....TT....TTTTTT..........TTTTT.TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT..TTT......TTTT......T....TTTTTTTT.....T.....TTTTTT.....T.....TTT....T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT..TTTTTT....TTTTTT..TTTTT......T....TTTTTTTT.....T......TTT.....TTTT.............T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT....TTT......TTTTTT..TT........TTT...TTTTTTTT.....T.........TT...TTTT..............TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......T.......TTTTTTT............TTT...............TTT........TT....TTTT...............TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.............................TT....T................TTT........TT....TTTT................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.TTT........................TTT....T................TTT....TT...T.....TTT...............TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT.......................TTTT....T.................T.....TT.........TT.............TT.TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTT......................TTTT....T.................T.....TTT......................TTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.TTTTTTTTTTTT.....TTT.....................TTT................T.....TT......................TTTTTT@TTTTTT.T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...TTTT@TTTTTTT....TTTT...................TTTT................T.............................TTTTT@TTTTTT...TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT....TTTTT@TTTTTTT....TTTT...................TTTT...............TTT......................TT...TTTTTT@TT.......TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......TTTTTT@TTTTTTT....TTTT..................TTTT...............TTTT.....................TTTTTTTTTT@TTTT......TT@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT.........TTTTT@TTTTTT..TTTTTT...................TTTT..............TTTTT.....................TTTTTTTTTT@TTTT......TTT@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...........TTTTT@TTT....TTTTT...................TTTT...............TTTTT.....................TTTT@@TTT@TTTTT........TTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...............TTTTTTT..TTTTT.......................T.................TTTT..TT................TTTTT@@TTTTTTTT...........TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT.....TT..........TTTT@TT..TTTTT..........................................TTT....................TTTTT@@TT@TTTTTT...........TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......TTTT.........TTTT@TTTTTTTT..........................................TT.....................TTTTT@TT@TTTTTTT............TTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT......TTTTTTT.TT......TT@TTTT.TT....................TT............................................TTTT@@TT@TTTTTTT............TTTTTT.TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT......TTTTTTTTTTT...TTTTT@TTT.......................TTT..........................................TTTTT@TT@TTTTTTTT.............TTT...TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT......TTTTTTTTTTTTTTTTTTT@TTT.......................TTT.......................TT.................TTTTTTTT@TT.TTTTT..............T....TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT...TTTTTTTTTT.TTTTTTTTTTTT@TT................................................TTT.................TTTT@TT@TT..TTT........................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT...TTTTTTTTTT...TTTTTTTTTTTTTTT................................................TT................TTTTTTT@TTT.TTTTTT.....................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT....TTTTTTTTTT.....TTTTTTTTTTT@TT...TT.............................................................TTTTTTT@TT..TTTTT......................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT......TTTTTTTTT.....TTTTTTTTTTTTT@TT..TTT..............T............TTTTTTT..........................TTTTTT@TTT....TTT........................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT........TTTTTTTTT.....TTTTTTTTTTTTT@TTT.TTT............TTTT.........TTTTTTTTT..........................TTTTTT@TT................................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T..........TTTTTTTT.........TTTT.TTTTT@TT.TTT............TTTT.........TTTTTTTTTT..........T....TTTT..TTTTTTTTT@TTT...........................T....TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT...........TTTTTTT..........T....TTTT@TTT....TT.......TTTTTT......TTTTTTTTTTTTTT........TTTT.TTTTTTTTTTTTTTTT@TT.................................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T............TTTT.................TTTTTT@TT....TTTT.....TTTTTT......TTTTTTTTTTTTTT........TTTTT@TTT@TTTTTTTTTT@TT..................................TT..T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT................................TTTTTTTTTTT..TT@TT...TTTTT.........TTTTTTTTTTTTTTT......TTTTTT@@@@@@TTT@TTTT@TTT........................T...T.........TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T................................TTTTTTTTT@TTTTTTT@@TTTTT............TTTTTTTTTTTTTTT.......TTTTTTT@@@@@@@TTTTT@TT........................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT...............................TTTTTTTTTT@TTTTT@@@TTT..............TTTTTTTTTTTTTTT.............TTTTT@@@TTTTTTTT........................................TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT................................TTTTTTTTT@TTTT@@TTTTT..............TT.TTTTTTTTTT.................TTT@@@@TTTTTT..........................................T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@TT................................TTTTTTTTTTTTTTTTT......................TTTTTTTTT..................TTTTTTTTTTTT........................................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTT............................TTTTTTTTTTTTTTTTTT........................TTTTTTTT.....................TTTTTTTT.......................................TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTT............................TTTTTTT@TTTTTTTTT.........................T.............................TTTTTTTT......................................TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT.TTT..........................TTTT@@TTTTTTTT.......................................................TTTTTTTT...............TTT.....................TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT..........................TTTTTTTTTTTTTT.......................................................TTTTTTTTT...........TTTTTTT.......................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTT........................TTTTTTTTTTTTTT......................................................TTTTTTTTTT.........TTTTTTTTT........................TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTT............................TTTTTTT.........................................................TT@TT.TTTT........TTTTTTTTTT.........................TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTT...........................TTTTTTT.........................................................TT@TT..TT..........TTTTTTTTT.........................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTT............................TTTTTTT........................................TT................TT@TT..............TTTT..............................TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT.TT...........................TTTTTTT....................................TT...TTT..............TT@TT..............TTTT....TTT........................TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTT.............................TTTTTTT....................................TTTTTTTTT....TTT......TTT@TT..............TTTTTT.TTTTT.........................TTTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@TTTTTTT.............................TTTTTT.................................TTTTTTTTTTTTT....TTTT.....TT@TT.............TTTTTTTTTTTTTTT........................TTTTT@@@@@@@@@@@@@@@@@@@@@@@T@TTTTTT@@TTTT...............................TTTT..................................TTTTTTTTTTTT.....TTT.....TT@TTT............TTTTTTTTTTTTTTTTT........................TTTTT@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT@TTTTT..............TTTT............TT............TT..................TTTTTTTTTTTTTTT.....TTT.....TT@TT.............TTTTTTTTTTTTTTTTTT...........................TT@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT@TTTT..TTT.....TTTTTTT.........................TTTT.............TTTTTTTTTTTTTTTTTTT....TTTTTT..TT@TT..............TTTTT@TTTTTTTTTTTT..........TT................T@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@TTTT@@TTTTTTT...TTTTTTTTT.TTT.TTT.................TTTT.........TTTTTTTTTTTTTTTTTTTTTTT....TTTTTT..TT@TT...............TTTT@@@TTTTTTTTTT........TTTTT...............TT@@@@@@@@@@@@@@@@@@TTTTTTTTTT@@@TTTT@@TTTTT..TTTTTTTTTTTTTTTTTTT................TTTT.....TTTTTTTTTTTTTTTTTTTTTTTTTT.....TTTTT..TT@TT...................TTTT@TTTTTTTTT.....TTTTTTTT.............TTTT@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT@TTTTTT@TTTTT..TTTTTTTTTTTTTTTTTTT..............TTTTTTTTTTTTTTTTTTTTT@@@@@TTTTTTTTTT....TTTTTTTTTTTTT....................TTTT@TTTTTTTTT...TTTTTTTTTT............TTTTT@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTT@TTTT.TTTTTTTT@@@TTTTTTT..............TTTTTTTTTTTTTTTTTT@@@@@@@@@TTTTTTTTTT....TTTTTTTTTTTT....................TTTTTT@TTTTTTTT...TTTTTTTTT..............TTTT@@@@@@@@@@@@@@@TTTTTTTT@TTTTTTTTTT.TTTT@@TTTTTTTTTTT@@TTTTTTTTT.............TTTTTTTTTTTTTTT@@@@@@@@@@@@@@TTTTTTTT....TTTTTTTTTTTTT...TT..............TTTTT@TTTTTTTTTT.TTTTTTTTTT...............TTTT@@@@@@@@@@@@@TT..TTT..TTTTTTTTT.....TTTT@@TTTTTTTT@@TTTTTTTTTT.............TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@TTTTTT.....TTTTTTTTTTTTT..................TTTTTTTTTTTTTTT@@TTTTTTTTTTT...............TTTTT@@@@@@@@@@@@T....T...TTTTTTTTT.......TTTT@TTTTTTT@TTTTTTTTTTT............TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@TTTTTT....TTTTTTTTTTTTTTT.................TTT@@@TTTTTTTTTTTTTTTTTTTTTT................TTTT@@@@@@@@@@@TT........TTTTTTTTT.....TTTTTTT@@TTTTTTTTTTTTTTTTT.............TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@TTT@TT....TTTTTTTTTTTTTTTT...............TT@@@@@TTTTTTTTTTTTTTTTTTTT.T................TTTTT@@@@@@@@@@T..........TTTTTTT......TTT@@TTTT@@TTTTTTTTTTTTTT...............TTTTTTTTTT@@@@@@@@@@@@@@@@@@@TT@@@.....TTTTTTTTTTTTTTTTT............TTT@@@@@@@TTTTTTTTTTTTT.TT....TT..............TTTTTT@@@@@@@@@T............TTTTT.....TT.TTT@@@TTTTTTTTTTTTTTTTTT...............TTTTTTTTTTT@@@@@@@@@@@@@@@@@TTT@@@@@@@.TTTTTTTTTTTTTTTTT...........TTT@@@@@@@@TTTTTTTTTTTTT.......TTT.............TTTTTTT@@@@@@@TT.............TTT.....TTT....TTTTTTTTTTTTTT.....TT...............TTTTT.TTTTT@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@TTTTTTTTTTTTTT...........TTT@@@@@@@@TTTTTTTTTTTTT.......TT...............TTTTTTT@@@@@@TT....................TTTT....TTTTTTTTTTTTTT.....TT................TTT..TTTTTT@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@TTTTTTTT..TTT...........TT@@@@@@@@@@TTTTTTTTTTTT.........TT...............TTTTTT@@@@@TTT....................TTTT....TTTTTTTTTTTTT.............TT..........T....TTTTTT@@@@@@@@@@@@@@@TTTTTTTT@@@@@@TTTT.......T...........TT@@@@@@@@@@@@TTTTTTTTTTT.........TT................TT.TTT@@@@TTT....................TTTT....TTTTTTTTTTTTT............TTTTT.TT...........TTTTT@@@@@@@@@@@@@@@TTTTTTTTTTTT@TTTT..............TT..TTT@@@@@@@@@@@@@TTTTTTTTTTT..........T...................TTT@@@TTTT......................T.....TTTTTTTTTTT..............TTTTTTTTT..........TTTTTT@@@@@@@@@@@@@TTTTTT@@TTTTTTTTTT.............TTTTTTTT@@@@@@@@@@@@@TTTTTT..TTT..........TTT.................TTTT@TTTTT............................TTTTTTTTTTT..............TTTTTTTTTT..........TTTTT@@@@@@@TTTTTTTTTTTT@@@@@@TTTTTT............TTTTTTTT@@@@@@@@@@@@@@@T@@T...............TTTTT................TTTT@TTTTT...........................TTTTTTTTTTTT...............TTTTTTTTT...........TTTTT@@@@@@TTTT@@TTTTTT@@@@@@@TTTT.............TTTTTTTT@@@@@@@@@@@@@@@@@@TTTTT...........TTTTT..................TTTTTTTT...........................TTTT...TTTTTTTT...........TTTTTTTTT............TTTTTTTTTTTTTT@TTTTTTTT@@@@@TTTTT...............TTTTTTTTT@@@@@@@@@@@@@@@@TTTTT...........TTTTT...................TTTTTTT............................TTT..TTTTTTTTT..........TTTTTTTTT..............TTTTTTT@@@@@TTTTTTTTTTTTTTTTTTT.................TTTTTTTTT@@@@@@@@@@@@@@@TTTTT............TTTT.................TTTTTTTTT.............................TT..TTTTTTTTT..........TTTTTTTT...............TTTTTTT@@@TTTTTTTTTTTTTTTTTTT...................TTTTTTTTT@@@@@@@@@@@@@@@TT....................................TTTTTTT...............................TT..TTTTTTTT...........TTTTTTTTT...............TTTTT@@@TTTTTTTTTTTTTT.......................TTTTTTTTTTT@@@@@@@@@@@@@@@@T..................................TT@@TTT........TTTT.........................TTTTT..T...........TTTTTTTTTTT...............TTTTT@TTTTTTTTTTTTT......................TTTTTTTTTTTTT@@@@@@@@@@@@@@@@TT.................T...............TT@@TTT........TTTT.........................TTTT...............TTTTTTTTTTTT..................TTTTTTTT..TTTTT.....................TTTTTTTTTTTTTT@@@@@@@@@@@@@@@@TT......TTT.......T................TT@@@TT........TTTT.TT.......................TT................TTTTTTTTTTTTTT...................TTT......TT....................TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@TT..TTTTTT........................TT@@@TT........TTTTTTT.......................................TTTTTT@@@TTTTTTTTT..............................................TTTTTTTTTT@@@@TTTT@@@@@@@@@@@@@@@@TTTTTTTTTT.........................TT@@@TT........TTTTTTTT......................................TTTTT@@@@@@TTTTTTTT.............................................TTTTTTTTT@@@@@@TTTT@@@@@@@@@@@@@@@TTTTTTT............................TT@@@TT........TTTTTTTT......................................TTTTT@@@@@@@TTTTTTTT...........................................TTTTTTTT@@@@@@@@TTTT@@@@@@@@@@@@@TTTTTTT..............................TT@@@TT..............T....................TT.................TTTTT@@@@@@@@@TTTTTT...........................................TTTTTT@@@@@@@@@@TTTT@@@@@@@@@@@@TTTTTTT............................T..TT@@@TT.................TT............TTT.TTTT...............TTTT@@@@@@@@@@@@TTTT..........................................TTTTT@@@@@@@@@@@@TTTT@@@@@@@TTTTTTTTTTT..........................TTTTT.TT@@@TT.................TT............TTTTTTTT...............TTTT@@@@@@@@@@@@TTT...........................................TTTTT@@@@@@@@@@@@@TTTTTT@TTTTTT.................................TTTTTT.TT@@@TT........TTT......TTT.......TTTTTTTTTTTT...............TTTT@@@@@@@@@@@@TTT..............................................T@@@@@@@@@@@@@@TTTTTTTT.....................................TTTTTTT.TT@@@TT........TTT......TTT.....TTTTTTTTTTTTTT...............TTTT@@@@@@@@@@@TTTTT.............................................T@@@@@@@@@@@@@@TTTTTT......................................TTTTTT...TT@@@TT........TTT.TTTTTTT.....TTTTTTTTTTTTTTTTTTT...........TTT@@@@@@@@@@@@TTTTT.............................................TT@@@@@@@@@@@@@@TTTTT.....................................TTTT.TT...TT@@@TTTTT.........TTTTTTT....TTTTTTTTTTTTTTTTTTTT.........TTTTT@@@@@@@@@@@TTTTTT.........................TTT..................TT@@@@@@@@@@@@@TTTT@T....................................TTTT......TT@@@TTTTTTTT......TTTTTTT....TTTTTTTTTTTTTTTTTTT..........TTTTT@@@@@@@@@@@TTTTTT........................TTTT.TTT.............TTT@@@@@@@@@@@@@TTTTTTTT..................................TTTTT.....TT@@@TTTTTTTT......TTTTTTT.....TTTTTTTTTTTTTTTTTTT.........TTTTT@@@@@@@@@@@TTTTTT.................TTTTTTTTTTTTTT..............TTT@@@@@@@@@@@@@TTTTTTTT.................................TTT.T......TT@@@TTTTTTTT......TTTTTTTTTT..TTTTTTTTTTTTTT@TTTT.........TTTT@@@@@@@@@@@@TTTTTT.................TTTTTTTTTTTTTT...............TT@@@@@@@@@@@@@@TTTTTTT...........................................TTT@@@TTTTTTTTT.TTT.TTTTTTTTTTT..TTTTTTTTTTTT@@TTTTT........TTTT@@@@@@@@@@@@T@TTTT................TTTTTTTTTTTTTTT................T@@@@@@@@@@@@@@TTTTTT..........................................TTTT@@@@TTTTTTTTTTTTT.TTTTTTTTTTT...TTTTTTTTTTT@@TTT@T........TTTT@@@@@@@@@@@@T@TTTTT...............TTTTTTTTTTTTTT.................T@@@@@@@@@@@@@@TTTT@T..............TTTT...............TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@TT...TTTTTTTTT@@@@@@@@@TTT@TTTTT..............TTTTTTTTTTTTTT..................TT@@@@@@@@@@@@TTTTTTTTTT......TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@TTT@TTTTT..............TTTTTTTTTTTTTT..................TTTTTTTTTTTT@@TTTTTTTTTTT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@TTTTTTTTTTTTTTTTTTTTTTT@@@@@TTTTTTTTTTTTTTT@@@@@@@@@TTT@TTTTT.............TTTTTTTTTTTTTTT.................TTTTTTTTTTTTTTTTTTTTTTTTTT....TTTTTTTTTTTT@@@@@@@@T@@@@@@@@@@@@@@@@@TTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@TTT@TTTTT............TTTTTTTTTTTTTTT..................TTTTTTTTTTTTTTTTTTTTTTTTTT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@TT@@TTTT............TTTTTTTTTTTTTTT...................TTTTTTTTT@@TTTTTTTTTTTTTT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@TT@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@T@@TTTT............TTTTTTTTTTTTTTT....................TTTTTTTTTTTTT@TTTTT...............TTTTTTTTTTTTTTTTTTT.......TT@@@@TTTTTT@@@@@@@TTT@@TTTTTTTTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTTTTTT@@@@@@@@@@@@T@@TTTTT............TTTTTTTTTTTTTTT....................T@TTTTTTTTTTTTTTT.................TTTTTTT...TTTTTTTT.......TT@@@@@@@@TT@@@@@@@TTT@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@TT@TTTT..............TTTTTTTTTTTTTT.....................TTTTTTTTTTTTTTTT.................TTTTTTTT..TTTTTTTT......TTT@@@@@@@@TT@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.....TTTTTTTTTTTTTT@@@@@@@@@@@@TTTTTTTT..............TTTTTTTTTTT.......................TTTTT@@TTTTTTTTT...................TTTTTT...TTTTTTT......TT@@@@@@@@@TT@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT...T..TTTTTTTTTTTTTTT@@@@@@@@@@@TTTTTTTT.............TTTTTTTTTTT.......................TTTTTT@@TTTTTTTTT....................TTTTT....TTTTTT......TT@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.TTT...TTTT...TTTTTTTTTTTTT@@@@@@@@@@@TTT@TTTT.............TTTTTTTTTTT.................TTT..TTTTTT@@TTTTTTTTT......................TTT......TTTTT......TTT@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.......TTTT....TTTTTTTTTTTT@@@@@@@@@@@@TT@TTTT...............TTTTTTTTT...............TTTTT..TTTTTT@TTTTTTTTT........................TTTTTT..............TT@@@@@@@@TTTTTTTTTTTT...TTTT.....................TTTT.....TT@TTTTTTTT@@@@@@@@@@@@T@TTTTT.................TTTTTT..............TTTTTTTTTTTTT@TTTTTTT............................TTTTTTTTT..........TTT@@@@@@@TT.......................................TTT......T@TTTTTTTT@@@@@@@@@@@@T@TTTT..................TTTTTT.............TTTTTTTTTTTTT@TTTTTTTT...........................TTTT@TTTTTTT........TTT@@@@@@@TT........................................T.......TTTTTTTTTT@@@@@@@@@@@@TTT@TT...................................TTTTTTTTTTTTTTTTTTTTTTTT...........................TTTTTTTTTTTT.........TTT@@@@@@TT................................................TTTTTTTTTT@@@@@@@@@@@@TTTTT...................................TTTTTT@TTTTT@@TTTTTTTTTTT...............................TTTTTTTT.........TTTT@@@@@TTTTTTTTTTTTTT.....................................TTTTTTTTTT@@@@@@@@@@TTTTTT.................................TTTTTTTTTTTTTTTT@TTTTTTTTTTTT................................TTTT...........TTTTT@@@TTTTTTTTTTTTTT...................................TTT@TTTTTTTT@@@@@@@@@TTTTTTTTT..............................TTTTTTT@TTTTTTTTTTTTTTTTTTTTTTT................................................TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.................TTTTTTTTTTTT@@@@@@@TTTTTTTT@TTT...........................TTTTTTTTTT@@TTT@TTTTTTTTTTTTTTTTT...........................................T.T...TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT.................TTTTTTTTTTTT@@@@@TTTTTTTTTTTTTTTT........................TTTTTTTTTT@@TT@@@@@TTTTTTTTTTTTTT.........................................T..T.T.......TTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTT..................TTTTTTTTTTTTT@@TTTTTTTTTTTTTTTTTTTT....................TTTTTTTTTT@TTT@@@@@@@@TTTTTTTTTTTTTT.......................T.............T....TTT........TTTTTTTTTTTTTTTTTTTTT..............................TTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@TTTTTTTTTTTTTTT....................T@T.................TTT........TTTTTTTTTTTTTTTTTTTTT...............................TTTTTTTTTTTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@TTTTTTTTTTTTT....................TT.................TTTT........TTTTTTTTTTTTTTTTTT.TT...............................TTTTTTTTTTTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@TTTTTTTTTTT..............TTTTT.....................TTT@T......TTTTTTTTTTTTTTTTTT...................................TTTTTTTTTTTTTTTT@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@TTTTTTTT@@@@@@@@@@@@@@@@TTTTTTTTTTT..............TTTTTT....................TTTTTT.....TTTTTTTTTTTTTTTTT..........................TT........TT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@TTTT@@@@@@@@@@@@@@@@@TTTTTTTTTT..............TTTTTTT.....TT.............TTTTTT...TTTTTTTTTTTTTTTTTTT..........................TT..........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@TTTTTTTTTTT..............TTTTTTT....TTTTT............TTTT....TTTTTTTTTTTTTTTTTTT........................TTTTT..........TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@T@TTTTTT@@@@@@@@@@@@@@@@TTTTT..TTTT..................TTT.....TTTTTTT...TT...............TTTTTTTTTTTTTTT.........................TTTTTTTTTTTT....TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@TTTTTT@TT@@@@@@@@@@@@@@@@TTTTT.....TT..........................TTTTTTT..TTTT.......TT....TTT@TTTTTTTT.............................TTTTTTTTTTTT......TTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@TTTTTT@TTT@TTTTTT@@@@@@@@@@@@@@@@@@@@@TTTTT...................................TTTTT..TT@@TTT....TTT..TTTT@@TTTTTTTT.............................TTTTTTTTTTTTT......TTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTT...............TTT.TT@@@@TT....TT..TTTTT@@@TTTTTTT............................TTTTTTTTTTTTTT.......TTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTTT............TTTT@@@@@@TT........TTT@@@@@TTTTT....................TT.......TTTTTTTTTTTTT.........TTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTTTTTT.........TT@TTTTTTTTTT........TT@@@@@@TTTTT.............................TTTTTTTTTTTTT..........TTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT....................TTTTTTT@TTTTT.........T@TTTTT.............TT@@@@@@@TTTT.............................TTTTTTTTTTTTT...........TTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.....................TTTTTTTT@@TTTTT.......TTTTTT..............T@@@@@@@@TT........................TTT...TTTTTTTTTTTTTTT...........TTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.....................TTTTTTTTT@@@@TTTTTTTTT..TT.......TTTTTT...TT@@@@@@@@@T......................TTTTT.TTTT@TTTTTTTTTTTT............TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT.....................TTTTTTTTTTTT@@@@TTTT@TTTT......TTTTTTTTTTTTT@@@@@@@@@@@T...TT...............TTTTTTTTT@@TTTTTTTTTTTT..............TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT.....................TTTT@@@@@@TT@@@@@TTT@@@TTTT...TTTTT@@@@TTTT@@@@@@@@@@@@TT.TTTTT...........TTTTTTTTT@@TTTTTTTTT@TTT.................TTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT....................TTT@@@@@@@@@T@TT@@TTT@@@@TT..TTT@@@@@@@@@TT@@@@@@@@@@@@@T.TTTTTTTT......TTTTTTTTTT@TTTTTTTTTTTTT...................TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT..................TTT@@@@@@@@@@@TTTT@@@TTT@@@T.TTT@@@@@@@@@@T@@@@@@@@@@@@@@TTTTTTTTTTTT...TTTTTTTTTT@TTTTTTTTTTTTT....................TTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTT..................TTT@@@@@@@@@@@@@@TT@@TTTTTTTTTT@@@@@@@@@@TT@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTT............................TTTTTTTTTTTTTTTTTT@@@@@@TTT@@@@@@TTTTTTTTTTTTTTTTTTTT..................TTT@@@@@@@@@@@@@@@@TTTTTT@TT@TTT@@@@@@@@@@T@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTT@TTTTTTTTTTTTT.............................TTTTTTTTTTTTTTTTTTTTTT@TTTTTT@@TTTTTTTTTTTTTTTTTTTTTT..................TT@@@@@@@@@@@@@@@@@@@TTTTTT@@TTT@@@@@@@@@@TT@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT..............................TTTTTTTTT@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT................TTTT@@@@@@@@@@@@@@@@@@@@@TTTTT@@TT@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@T@@@@TTTTTTTTTT@TTTTTTTTTTTTT................................TTTTTTTTTTTTT.TTTTTTTTTTTTTTTTTTTTTTT..TTTTTTTTTTTTT.........TTT...TTTTT@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT@TTTTTTTTTTTTTT.................................TTTTTTTTTTTT..TTT..TTTTTTTTTTTTTTT.....TTTTTTTTTTTTT........TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@T@TTTTTTTT@TTTTTTTT.TTTTT.....................................TTTTTT......TTT......TTTTTTT..........TTTTTTT.TT......TTT.TTTTTT@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@TT@@T@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTT....TT...................................................TTT......TTTTTT............TTTT...........TTTTTTTTT@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@T@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@TTTTTTT......................................................................TT............................TTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTT@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT...........................................TTT......................................................TTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@TTT..........................................TTTTTTT..................................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@TT..........................................TTTTTTTTT.................................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@TTT....TTTT..................................TTTTTTTTTT................................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TTT....TTTT.................................TTTTTTTTTTT...............................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TTT....TTTT................................TTTTTTTTTTTT...............................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@TT........................................TTTTTTTTTTTT..............................................TTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTT......................................TTTTTTTTTTTTTT............................................TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TT.....................................TTTTTTTTTTTTTTTTTTT.......................TTT...........TTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTT....................................TTTTTTTTTTTTTTTTTTTT...............TTTTTTTTTTTT.......TTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TT..................................TTTTTTTTTTTTTTTTTTTTT.....TTT.......TTTTTTTTTTTTTTTT....TTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTT...........................TTTTTTTTTT@TTTT@TTTTTTTTTT....TTTTTTT...TTTTTTTTTTTTTTTTT.....TTTTTTTT....TTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TTT........................TTTTTTTTTTTTTTTTTTTTTTT..TTT.....TTTTTTTTTTTTTTTTTTTTTTTTTTTT....TTTT......TT@TT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTTTTTTTTT............TTTTT@TTTTTTTTTT@TTTTTTTTTT..........TTTTTTTTTTTTTTTTTTTTTTTTTTTT..............TTT@TT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@TTT@TT@TTT..........TTTTTTTTTTTTTTTTT@TTTTTTTTTT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTT............TT@TTTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTTTTT.........TTTTTTTTTTTTTTTTTT@TTTTTTTTTT..............TTTTTTTTTTTTTTTTTTTTTT..............TTTTTTT@TT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTTTT@@TT.......TT@TTTTTTTTTTTT@TT@TTTTTTTTTTT.............T....TTTTTTTTTTTTTT.................TTTTTTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTT@TTT......TT@@TTTTTTT@@@@@TTT@TT@TTTTTT...............TT..TTTTTTTTTTTTTT..................TTTTTTTTT@TT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@TTTTTTTTT......TT@TTTTTTTTT@@@@TT@TT@@TTTTT.................TT..TTTTTTTTTTTTT...................TTTTTTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@TTT...TT@TTTTTTT@TT@@@TT@TTT@TTTTTT....................TTTTTTTTTTT@@@T...................TTTTTTTTTTT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT@TTTTT@@TTTTTTT@TT@@@TT@TT@@TT@@TT...................TTTTTTTTTTT@@@@@T...................TTTTTTTT@TT@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT@@TT@@TTTTTTTT@TT@@TT@TTT@@T@TTTT...................TTTTTTTTTTT@@@@@T....................TTTTTTT@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTT@TTTTTTT@@@@@@TTTTTTTT@TT@TTT@TT@@@TT@TT....................TTTTTTTTTTT@@@@TT......................TTTTTT@TT@TT@@@@@@@@@@@@@@@@@@@@@@@@TT@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT@@@TTTTTTTTT@TT@TT@TTT@@@T@TTT...................TTTTTTTTTTTTTT@T........................TTTTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT@TTTTTTTTTT@@@TTTTTT@@@TTTTTT.................TTTTTTTTTTTTTT..T..........................TTTTTTTTT@TTT@@@@@@@@@@@@@@@@@@@@@@TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTT@@@TT@TT@@@@TT@TTT...........T....TTTTTTTTTTTTTTT.TT...........................TTTTTTTTT@TT@@@@@@@@@@@@@@@@@@@@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TT@TTTTTTT@@TTTTTTTT@@TT@TTT@@@@TTTTTT........TTTT..TTTTTTTTTTT@@T@TT...............................TTTTTTTT@TTT@@@@@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@T@@TTTT@@TT@TT@@@@@TTTTT........TTTTTTTTTTTTTTTTTTTTT@TTTT..............................TTTTTTTT@TT@@@@@@@@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@@@TTT@@TT@TTTT@@@TTTTTT........TTTTTT@TTTTTTTTTTT@@T@TTTT.T.............................TTTTTTTTTTT@@@@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT@@TT@@TTT@TTTT@@@TTTTTT.........TTTTTTTT@TT@TTTTTTTT@TTTT.TT.............................TTTTTTT@TTT@@@@@@@@@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTT@TT@TTTT@@@TTTTTTT.........TTTTTTTT@TT@TTTT@@TTTTTTTTTTT.............................TTTTTTT@TT@@@@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTT@T@@TTTTT........TTTTTTTTTTT@@@@@@@@@TTTTTTTTTTT..............................TTT.TT@TTT@@@@@@@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT@TTTT@@@@TTTTT........TTTTTTTTTTTT@TTT@@@@TTTTTT@TTTT................................T...TT@TTTT@@@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTT@@@@TTTT.........TTTTTTTTTTTTTTTTTTTTTTTTTT@@TT.....................................TT@TTTT@@TTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTT@@@@@TT...........TTTTTTTTTTTTTTTTTTTTTTTTTTTT.....................................T..TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTT@@@TTTT............TTTTTTTTTTTT.TTTTTTTTTTTTTT...TTT....TTTTT.....................TTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT@@TTTTT.............TTTTTTTTTTT.TTTTTTTTTTTTTT.TTTTT....TTTTT......................T@TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTT@@TTTTTT............TTTTTTTT....TTTTTTTTTTTTTT.TTTTT....TTTTT.....................TT@@TTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@TT@TTT............TTTTTT......TTTTTTTTTTTTTT.TTTTTT...TTTTT.....................TT@@TT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT@@TTTT.............TT.........TTTTTTTTTTTTTT..TTTTT...TTTT.....................TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTT........................TTTTTTTTTTTTTTT..TTTTTT..TTTT................TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.........................TTTTTTTTTTTTTTT...TTTTT..TTTT..TTTTTTTT......TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT........................TTTTTTTTTTTTTTT...TTTTTT.TTTT..TTTTTTTT....TTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT.....................TTTTTTTTTTTTTTTT....TTTTT.TTTT..TTTTTTTT.TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT...................TTTTTTTTTTTTTTTT....TTTTT.TTTT..TTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTT..T...............TTTTTTTTTTTTTTTTT....TTTTTTTTT..TTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTT......TTT......TTTTTTTTTTTTTTTT....TTTTTTTTT........TTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTT..TT.TTTTT.......TTTTTTTTTTTTT.....TTTTTTTTT....TT..TTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTT......TTTTTTTTTTTTTT......TTT@TTTT..TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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
