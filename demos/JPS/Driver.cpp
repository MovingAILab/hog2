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
#include "CanonicalGrid.h"
#include "CanonicalDijkstra.h"
#include "JPS.h"

enum mode {
	kBasicCanonical = 0,
	kBasicCanonicalJumpPoint = 7,
	kFullCanonical = 1,
	kCanonicalAStar = 2,
	kBoundedJPS = 3,
	kJPS = 4,
	kCanonicalDijkstra = 5,
	kAStar = 6
};

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
CanonicalGrid::CanonicalGrid *grid;
CanonicalDijkstra canDijkstra;
JPS *jps;
std::vector<xyLoc> jpsPath, astarPath, dijPath;
std::vector<CanonicalGrid::xyLoc> jumpPoints;

xyLoc start={0,0}, goal;

mode m = kBasicCanonical;
void AddDH(xyLoc where);
void LoadMap(Map *m);

bool recording = false;
bool running = false;
bool mapChange = true;

int stepsPerFrame = 1;

std::vector<int> buttons;
typedef enum : int {
	kBasicButton = 0,
	kFullButton = 1,
	kBasicJPButton = 2,
	kAStarButton = 3,
	kCAStarButton = 4,
	kJPSButton = 5,
	kBJPS4Button = 6,
	kCDijkstraButton = 7,
	kFasterButton = 8,
	kSlowerButton = 9,
	kPauseButton = 10,
	kStepButton = 11,
} buttonID;
void ActivateModeButton(buttonID bId);
void SetupGUI(int windowID);

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
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Speed Up", "Increase speed of A* search", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slow Down", "Decrease speed of A* search", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Algorithm", "Select: 1. CA* 2. BJPS(4), 3. JPS 4. CDijkstra 5. A*", kAnyModifier, '1', '5');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Step 1 expansion", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Pause", "Set alg speed to 0", kAnyModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "Basic", "Draw basic canonical ordering", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Basic+JP", "Draw basic canonical ordering with jump points", kAnyModifier, 'j');
	InstallKeyboardHandler(MyDisplayHandler, "Full", "Draw full canonical ordering", kAnyModifier, 'f');

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
		SetNumPorts(windowID, 2);
		// Left half of screen
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		// Right half of screen (buttons)
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		SetupGUI(windowID);

		Map *map = new Map(1,1);
		LoadMap(map);
		
		me = new MapEnvironment(map);
		grid = new CanonicalGrid::CanonicalGrid(map);
		jps = new JPS(map);
	}
}

int frameCnt = 0;

void DrawSim(Graphics::Display &display)
{
	if (mapChange == true)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		mapChange = false;
	}
	switch (m)
	{
		case kBasicCanonical:
			if (start.x == 65535) break;
			grid->SetColor(Colors::darkgray);
			grid->DrawBasicOrdering(display, {start.x, start.y});
			return;

		case kBasicCanonicalJumpPoint:
			if (start.x == 65535) break;
		{
			grid->SetColor(Colors::darkgray);
			grid->DrawBasicOrdering(display, {start.x, start.y});
			grid->GetFirstJumpPoints({start.x, start.y}, jumpPoints);
			grid->SetColor(Colors::darkgray);
			for (auto &s : jumpPoints)
				grid->Draw(display, s);
		}
			return;
		case kFullCanonical:
			if (start.x == 65535) break;
			grid->SetColor(Colors::darkgray);
			grid->DrawOrdering(display, {start.x, start.y});
			return;
			
		case kCanonicalAStar:
		case kBoundedJPS:
		case kJPS:
		{
			grid->SetColor(Colors::green);
			grid->DrawOrdering(display, {start.x, start.y});
			
			jps->Draw(display);
			
			for (int x = 0; x < stepsPerFrame; x++)
				if (jpsPath.size() == 0)
					jps->DoSingleSearchStep(jpsPath);
			
			if (jpsPath.size() != 0)
			{
				me->SetColor(Colors::blue);
				for (int x = 1; x < jpsPath.size(); x++)
				{
					me->DrawLine(display, jpsPath[x-1], jpsPath[x], 5);
				}
			}
		}
			break;
		case kCanonicalDijkstra:
			canDijkstra.Draw(display);
			
			for (int x = 0; x < stepsPerFrame; x++)
				if (dijPath.size() == 0)
					canDijkstra.DoSingleSearchStep(dijPath);
			
			if (dijPath.size() != 0)
			{
				me->SetColor(Colors::blue);
				for (int x = 1; x < dijPath.size(); x++)
				{
					me->DrawLine(display, dijPath[x-1], dijPath[x], 5);
				}
			}
			break;
		case kAStar:
			astar.Draw(display);
			
			for (int x = 0; x < stepsPerFrame; x++)
				if (astarPath.size() == 0)
					astar.DoSingleSearchStep(astarPath);
			
			if (astarPath.size() != 0)
			{
				me->SetColor(Colors::blue);
				for (int x = 1; x < astarPath.size(); x++)
				{
					me->DrawLine(display, astarPath[x-1], astarPath[x], 5);
				}
			}
			break;
	}
	
	if (!running)
	{
		if (start.x != 0)
		{
			me->SetColor(Colors::red);
			me->DrawLine(display, start, goal, 3);
		}
	}

//	if (m == kBasicCanonical || m == kFullCanonical || m == kBasicCanonicalJumpPoint)
//		else if (m != kCanonicalDijkstra)
//	}
}

void SetupGUI(int windowID)
{
	const float borderPaddingX = 0.1f;
	const float borderPaddingY = 0.35f;
	const float verticalSep = 0.25f;
	const float buttonHeight = 0.1f;

	// Choosing canonical ordering
	float horizontalSep = 0.2f;
	float buttonWidth = 0.6f;
	float top = -1+borderPaddingY;
	float bot = top+buttonHeight;
	Graphics::roundedRect coButton({-1+0.3f, top, -1+0.3f+buttonWidth, bot}, 0.01f);
	Graphics::rect offsetRect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	int b = CreateButton(windowID, 1, coButton, "Basic", 'b', 0.01f, Colors::black, Colors::black,
					Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	coButton.r += offsetRect;
	b = CreateButton(windowID, 1, coButton, "Full", 'f', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	buttonWidth = 1.1f;
	top = bot+0.1f;
	bot = top+buttonHeight;
	Graphics::roundedRect coBothButton({-1+0.45f, top, -1+0.45f+buttonWidth, bot}, 0.01f);
	b = CreateButton(windowID, 1, coBothButton, "Basic And Jump Points", 'j', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Choosing algorithm
	horizontalSep = 0.075f;
	buttonWidth = 0.55f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect algButton({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, algButton, "A*", '5', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "Canonical A*", '1', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "JPS", '3', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	horizontalSep = 0.1f;
	buttonWidth = 0.85f;
	top = bot+0.1f;
	bot = top+buttonHeight;
	Graphics::roundedRect alg2Button({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, alg2Button, "Bounded JPS(4)", '2', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	alg2Button.r += offsetRect;
	b = CreateButton(windowID, 1, alg2Button, "Canonical Dijkstra", '4', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Simulation settings
	horizontalSep = 0.2f;
	buttonWidth = 0.6f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect speedButton({-1+0.3f, top, -1+0.3f+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, speedButton, "Run Faster", ']', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, 1, speedButton, "Run Slower", '[', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	top = bot+0.1f;
	bot = top+buttonHeight;
	Graphics::roundedRect simButton({-1+0.3f, top, -1+0.3f+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, simButton, "Pause", '0', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	simButton.r += offsetRect;
	b = CreateButton(windowID, 1, simButton, "Step", 'o', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
}

void DrawGUI(Graphics::Display &display)
{
	const float e = 0.03f;
	display.FillRect({-1, -1, 1, 1}, Colors::darkgray);
	display.FillRect({-1+e, -1+e, 1-e, 1-e}, Colors::lightgray);
	display.DrawText("Canonical Orderings: ", {-1+0.1f, -0.8f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Algorithms: ", {-1+0.1f, -0.25f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Algorithm Control: ", {-1+0.1f, 0.3f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
}

void ActivateModeButton(buttonID bId)
{
	SetButtonFillColor(buttons[kBasicButton], bId == kBasicButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBasicJPButton], bId == kBasicJPButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kFullButton], bId == kFullButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kAStarButton], bId == kAStarButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kCAStarButton], bId == kCAStarButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBJPS4Button], bId == kBJPS4Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kJPSButton], bId == kJPSButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kCDijkstraButton], bId == kCDijkstraButton ? Colors::yellow : Colors::white);
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	if (viewport == 0)
		DrawSim(display);
	else
		DrawGUI(display);
	
//	if (recording && viewport == GetNumPorts(windowID)-1)
//	{
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/astar-%d%d%d%d",
//				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
//		SaveScreenshot(windowID, fname);
//		printf("Saved %s\n", fname);
//		frameCnt++;
//		if (path.size() == 0)
//		{
//			MyDisplayHandler(windowID, kNoModifier, 'o');
//		}
//		else {
//			recording = false;
//		}
//	}
//	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void InitializeSearches()
{
	astar.InitializeSearch(me, start, goal, astarPath);
	if (m == kCanonicalAStar)
		jps->SetJumpLimit(0);
	else if (m == kBoundedJPS)
		jps->SetJumpLimit(4);
	else if (m == kJPS)
		jps->SetJumpLimit(-1);
	jps->InitializeSearch(me, start, goal, jpsPath);
	canDijkstra.InitializeSearch(me, start, goal, dijPath);
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[':
		{
			stepsPerFrame /= 2;
			std::string s = std::to_string(stepsPerFrame)+" steps per frame";
			submitTextToBuffer(s.c_str());
		}
			break;
		case ']':
		{
			if (stepsPerFrame <= 16384)
				stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
			std::string t = std::to_string(stepsPerFrame)+" steps per frame";
			submitTextToBuffer(t.c_str());
		}
			break;
		case '0':
		{
			stepsPerFrame = 0;
			std::string t = std::to_string(stepsPerFrame)+" steps per frame";
			submitTextToBuffer(t.c_str());
		}
			break;
		case '1':
			m = kCanonicalAStar;
			ActivateModeButton(kCAStarButton);
			submitTextToBuffer("Running Canonical A*");
			jpsPath.resize(0);
			if (running)
			{
				InitializeSearches();
			}
			else {
				start = {0,0};
			}
			break;
		case '2':
			m = kBoundedJPS;
			ActivateModeButton(kBJPS4Button);
			submitTextToBuffer("Running Bounded JPS(4)");
			jpsPath.resize(0);
			if (running)
			{
				InitializeSearches();
			}
			else {
				start = {0,0};
			}
			break;
		case '3':
			m = kJPS;
			ActivateModeButton(kJPSButton);
			submitTextToBuffer("Running JPS");
			jpsPath.resize(0);
			if (running)
			{
				InitializeSearches();
			}
			else {
				start = {0,0};
			}
			break;
		case '4':
			m = kCanonicalDijkstra;
			ActivateModeButton(kCDijkstraButton);
			submitTextToBuffer("Running Canonical Dijkstra");
			dijPath.resize(0);
			if (running)
			{
				InitializeSearches();
			}
			else {
				start = {0,0};
			}
			break;
		case '5':
			m = kAStar;
			ActivateModeButton(kAStarButton);
			submitTextToBuffer("Running (classic) A*");
			astarPath.resize(0);
			if (running)
			{
				InitializeSearches();
			}
			else {
				start = {0,0};
			}
			break;
		case 'o':
			if (m == kCanonicalDijkstra) { // canonical dijkstra
				if (dijPath.size() == 0)
					canDijkstra.DoSingleSearchStep(dijPath);
			}
			else if (m == kAStar)
			{
				if (astarPath.size() == 0)
					astar.DoSingleSearchStep(astarPath);
			}
			else if (m != kCanonicalDijkstra)
			{
				if (jpsPath.size() == 0)
					jps->DoSingleSearchStep(jpsPath);
			}
			break;
		case 'b':
			submitTextToBuffer("Click/drag to show basic canonical ordering");
			m = kBasicCanonical;
			ActivateModeButton(kBasicButton);
			break;
		case 'j':
			submitTextToBuffer("Click/drag to show basic canonical ordering with jump points");
			m = kBasicCanonicalJumpPoint;
			ActivateModeButton(kBasicJPButton);
			break;
		case 'f':
			submitTextToBuffer("Click/drag to show full canonical ordering");
			m = kFullCanonical;
			ActivateModeButton(kFullButton);
			break;
	}
}

void GetPathHandler(tMouseEventType mType, point3d loc)
{
	int x, y;
	me->GetMap()->GetPointFromCoordinate(loc, x, y);
	xyLoc tmp(x, y);
	if (me->GetMap()->GetTerrainType(x, y) != kGround)
		return;
	
	switch (mType)
	{
		case kMouseDown: goal = start = tmp; running = false; break;
		case kMouseDrag: goal = tmp; break;
		case kMouseUp:
		{
			goal = tmp;
			InitializeSearches();
			running = true;
		}
	}
}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	// Prevent button clicks from affecting simulation.
	if (viewport != 0)
		return false;

	int x, y;
	me->GetMap()->GetPointFromCoordinate(loc, x, y);
	xyLoc tmp(x, y);
	if (me->GetMap()->GetTerrainType(x, y) != kGround)
		return false;

	switch (m)
	{
		case kBasicCanonical:
		case kFullCanonical:
		case kBasicCanonicalJumpPoint:
		{
			start = tmp;
		}
			break;
		case kCanonicalAStar:
		case kBoundedJPS:
		case kJPS:
		case kCanonicalDijkstra:
		case kAStar:
			GetPathHandler(mType, loc); break;
	}
	return true;
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
