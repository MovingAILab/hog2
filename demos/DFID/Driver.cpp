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
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "RandomUnit.h"
#include "IDAStar.h"
#include "Timer.h"
#include "IncrementalDFID.h"
#include "IncrementalBFS.h"
#include "SVGUtil.h"

bool recording = false;
bool running = false;

int drawMode = 2; // bit 0 [dfid] bit 1 [dfs] bit 2 [bfs]

std::vector<int> buttons;
typedef enum : int {
	kRunButton = 0,
	kPauseButton = 1,
	kDFSButton = 2,
	kDFIDButton = 3,
	kBFSButton = 4,
	kBF2Button = 5,
	kBF3Button = 6,
	kBF4Button = 7,
	kBF5Button = 8,
	kBF6Button = 9,
	kBF7Button = 10,
	kBF8Button = 11,
	kBF9Button = 12,
} buttonID;
void ActivateBFButton(buttonID bId);
void SetupGUI(int windowID);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1440, 1080);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset run", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save SVG", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "DFS", "Toggle DFS", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "DFID", "Toggle DFID", kAnyModifier, 'i');
	InstallKeyboardHandler(MyDisplayHandler, "BFS", "Toggle BFS", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

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
		SetNumPorts(windowID, 2);
//		ReinitViewports(windowID, Graphics::rect{-1.f, -1.f, 1.f, 1.f}, kScaleToFill);
		// Bottom part of screen (for buttons)
		ReinitViewports(windowID, {-1, 0, 1, 1}, kScaleToSquare);
		// Top part of screen (overlapping, but on top -> allows for larger text size for button viewport)
		AddViewport(windowID, {-1, -1, 1, 0.5f}, kScaleToSquare);
		SetupGUI(windowID);
		
		{
			char txt[] = "Algorithms: ";
			submitTextToBuffer(txt);
			switch(drawMode)
			{
				case 0: appendTextToBuffer("none"); break;
				case 1: appendTextToBuffer("DFID"); break;
				case 2: appendTextToBuffer("DFS"); break;
				case 3: appendTextToBuffer("DFS+DFID"); break;
				case 4: appendTextToBuffer("BFS"); break;
				case 5: appendTextToBuffer("DFID+BFS"); break;
				case 6: appendTextToBuffer("DFS+BFS"); break;
				case 7: appendTextToBuffer("DFS+DFID+BFS"); break;
			}
		}

	}
}

#include "NaryTree.h"


double v = 1;

NaryTree tree(3, 5);
IncrementalDFID<NaryState, NaryAction> dfid(0);
IncrementalDFID<NaryState, NaryAction> dfs(10);
IncrementalBFS<NaryState, NaryAction> bfs;
std::vector<NaryState> path;

NaryState goal = tree.GetLastNode();

int frameCnt = 0;

void DrawSim(Graphics::Display &display, unsigned long windowID, unsigned int viewport)
{
	display.FillRect({-1.5, -1.0, 1.5, 1}, Colors::white);
	tree.SetColor(Colors::black);
	if (running)
	{
		MyDisplayHandler(windowID, kNoModifier, 'o');
	}
//	printf("Width: %d; height %d\n", GetContext(windowID)->globalCamera.viewWidth, GetContext(windowID)->globalCamera.viewHeight);
//	tree.SetWidthScale(double(GetContext(windowID)->globalCamera.viewWidth)/GetContext(windowID)->globalCamera.viewHeight);
	tree.SetWidthScale(1.5);
	tree.Draw(display);
	tree.SetColor(0.0, 1.0, 1.0);
	tree.Draw(display, goal);
	
	tree.SetColor(1, 0, 0);
	if (drawMode&0x1)
		dfid.Draw(display);
	tree.SetColor(0, 0, 1);
	if (drawMode&0x2)
		dfs.Draw(display);
	tree.SetColor(0.75, 0.5, 0.75);
	if (drawMode&0x4)
		bfs.Draw(display);

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/dfid-%d-%d%d%d%d", tree.GetBranchingFactor(), (frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		//SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		frameCnt++;
	}
	return;
}

void SetupGUI(int windowID)
{
	const float borderPaddingX = 1.4f;
	const float borderPaddingY = 0.2f;
	const float verticalSep = 0.1f;
	const float buttonHeight = 0.1f;

	// Basic controls
	float horizontalSep = 0.1f;
	float buttonWidth = 0.4f;
	float top = 0+borderPaddingY;
	float bot = top+buttonHeight;
	Graphics::roundedRect controlButton({-2.25f+borderPaddingX, top, -2.25f+borderPaddingX+buttonWidth, bot}, 0.01f);
	Graphics::rect offsetRect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	int b = CreateButton(windowID, 0, controlButton, "Run", 'r', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	controlButton.r += offsetRect;
	b = CreateButton(windowID, 0, controlButton, "Pause", 'p', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Choose search algorithm
	horizontalSep = 0.1f;
	buttonWidth = 0.4f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	Graphics::roundedRect algButton({-2.25f+borderPaddingX, top, -2.25f+borderPaddingX+buttonWidth, bot}, 0.01f);
	b = CreateButton(windowID, 0, algButton, "DFS", 'd', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 0, algButton, "DFID", 'i', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 0, algButton, "BFS", 'b', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;

	// Choose branching factor
	horizontalSep = 0.1f;
	buttonWidth = 0.15f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect bfButton({-2.25f+borderPaddingX, top, -2.25f+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 0, bfButton, "2", '2', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "3", '3', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "4", '4', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "5", '5', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "6", '6', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "7", '7', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "8", '8', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
	b = CreateButton(windowID, 0, bfButton, "9", '9', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	bfButton.r += offsetRect;
}

void ActivateBFButton(buttonID bId)
{
	SetButtonFillColor(buttons[kBF2Button], bId == kBF2Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF3Button], bId == kBF3Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF4Button], bId == kBF4Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF5Button], bId == kBF5Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF6Button], bId == kBF6Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF7Button], bId == kBF7Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF8Button], bId == kBF8Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kBF9Button], bId == kBF9Button ? Colors::yellow : Colors::white);
}

void DrawGUI(Graphics::Display &display)
{
	const float e = 0.03f;
	display.FillRect({-2.25f, 0, 2.25f, 1}, Colors::darkgray);
	display.FillRect({-2.25f+e, 0+e, 2.25f-e, 1-e}, Colors::lightgray);
	display.DrawText("Control: ", {-2.25f+0.1f, 0.2f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Search Algorithm: ", {-2.25f+0.1f, 0.4f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Branching Factor: ", {-2.25f+0.1f, 0.6f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (viewport == 0)
		DrawGUI(display);
	else
		DrawSim(display, windowID, viewport);
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
		case 'i':
		{
			drawMode ^= 0x1;
			frameCnt = 0;
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			SetButtonFillColor(buttons[kDFIDButton], drawMode&0x1 ? Colors::yellow : Colors::white);
			break;
		}
		case 'd':
		{
			drawMode ^= 0x2;
			frameCnt = 0;
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			SetButtonFillColor(buttons[kDFSButton], drawMode&0x2 ? Colors::yellow : Colors::white);
			break;
		}
		case 'b':
		{
			drawMode ^= 0x4;
			frameCnt = 0;
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			SetButtonFillColor(buttons[kBFSButton], drawMode&0x4 ? Colors::yellow : Colors::white);
			break;
		}
		case '{':
		{
			goal = tree.GetParent(goal);
			break;
		}
		case ']':
		{
			drawMode = (drawMode+2)&0x7;
		}
		case '[':
		{
			drawMode = (drawMode+7)&0x7;
		}
			break;
		case 's':
		{
			Graphics::Display d;
			d.FillRect({-1.0, -1.0, 1.0, 1}, Colors::black);
			tree.SetWidthScale(1.0);
			tree.Draw(d);
			MakeSVG(d, "/Users/nathanst/tree.svg", 1024, 512);
		}
			break;
		case '|':
			frameCnt = 0;
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		case 'r':
//			recording = !recording; running = true;
			running = true;
			break;
		case '0':
		{
		}
			break;
		case '1':
		{
		}
			break;
		case '2':
		{
			frameCnt = 0;
			tree = NaryTree(2, 8);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF2Button);
			break;
		}
		case '3':
		{
			frameCnt = 0;
			tree = NaryTree(3, 5);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF3Button);
			break;
		}
		case '4':
		{
			frameCnt = 0;
			tree = NaryTree(4, 4);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF4Button);
			break;
		}
		case '5':
		{
			frameCnt = 0;
			tree = NaryTree(5, 4);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF5Button);
			break;
		}
		case '6':
		{
			frameCnt = 0;
			tree = NaryTree(6, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF6Button);
			break;
		}
		case '7':
		{
			frameCnt = 0;
			tree = NaryTree(7, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF7Button);
			break;
		}
		case '8':
		{
			frameCnt = 0;
			tree = NaryTree(8, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF8Button);
			break;
		}
		case '9':
		{
			frameCnt = 0;
			tree = NaryTree(9, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			ActivateBFButton(kBF9Button);
			break;
		}
			break;
		case '\t':
			break;
		case 'p':
			running = !running;
			break;
		case 'o':
		{
			if (drawMode&0x1)
				dfid.DoSingleSearchStep(&tree, 0, goal, path);
			if (drawMode&0x2)
				dfs.DoSingleSearchStep(&tree, 0, goal, path);
			if (drawMode&0x4)
				bfs.DoSingleSearchStep(&tree, 0, goal, path);
		}
			break;
		default:
			break;
	}
	
	{
		char txt[] = "Algorithms: ";
		submitTextToBuffer(txt);
		switch(drawMode)
		{
			case 0: appendTextToBuffer("none"); break;
			case 1: appendTextToBuffer("DFID"); break;
			case 2: appendTextToBuffer("DFS"); break;
			case 3: appendTextToBuffer("DFS+DFID"); break;
			case 4: appendTextToBuffer("BFS"); break;
			case 5: appendTextToBuffer("DFID+BFS"); break;
			case 6: appendTextToBuffer("DFS+BFS"); break;
			case 7: appendTextToBuffer("DFS+DFID+BFS"); break;
		}
	}

}


bool MyClickHandler(unsigned long , int viewport, int x, int y, point3d loc, tButtonType , tMouseEventType e)
{
	// Prevent button clicks from affecting simulation.
	if (viewport == 0)
		return false;

	if (e == kMouseDown)
	{
		goal = tree.GetClosestNode(loc.x, loc.y);
		frameCnt = 0;
		dfid.Reset();
		dfs.Reset();
		bfs.Reset();
	}
	return true;
}
