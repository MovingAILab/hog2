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
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "GridHeuristics.h"

bool mouseTracking = false;
bool runningSearch1 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int gStepsPerFrame = 1;
double searchWeight = 0;
bool reexpand = false;
void LoadMap(Map *m, int which);
void StartSearch();

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
MapEnvironment *me = 0;
Map *m;
Heuristic<xyLoc> *searchHeuristic = 0;
EuclideanDistance euclid;
std::vector<xyLoc> path;
GridEmbedding *dh;
void SetupGUI(int windowID);
std::vector<int> buttons;
typedef enum : int {
	kDikstraButton = 0,
	kAStarButton = 1,
	kWAStar2Button = 2,
	kWAStar10Button = 3,
	kOctileHeuristic = 4,
	kEuclideanHeuristic = 5,
	kDifferentialHeuristic = 6,
	kBaldursGate = 7,
	kDragonAge = 8,
	kS1 = 9,
	kS10 = 10,
	kS100 = 11,
	kS1000 = 12,
	kNoReOpen = 13,
	kReOpen = 14,
} buttonID;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1500, 1000);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Reopen On", "Turn on re-openings.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reopen Off", "Turn off re-openiengs.", kNoModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Dijkstra", "Use Dijkstra", kNoModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Optimal A*", kNoModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "WA*(2)", "A* with weight of 2", kNoModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "WA*(10)", "A* with weight of 10", kNoModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "WA*(100)", "A* with weight of 100", kNoModifier, '4');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Increase simulation speed", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Decrease simulation speed", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "1/frame", "Expand 1/frame", kAnyModifier, '6');
	InstallKeyboardHandler(MyDisplayHandler, "10/frame", "Expand 10/frame", kAnyModifier, '7');
	InstallKeyboardHandler(MyDisplayHandler, "100/frame", "Expand 100/frame", kAnyModifier, '8');
	InstallKeyboardHandler(MyDisplayHandler, "1000/frame", "Expand 1000/frame", kAnyModifier, '9');

	InstallKeyboardHandler(MyDisplayHandler, "BG Map", "Load BG Map", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "DAO Map", "Load BG Map", kAnyModifier, 'd');

	InstallKeyboardHandler(MyDisplayHandler, "Octile", "Octile Heuristic", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Euclidean", "Euclidean Heuristic", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "DH", "Differential Heuristic", kAnyModifier, 'h');

	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
		runningSearch1 = false;
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		// at 1500x1000 => [X:-750->+250 U 250->750] [Y:-500 -> 500]
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		// 2x2 square
		ReinitViewports(windowID, {-1, -1, 0.333f, 1}, kScaleToSquare);
		// 1x2 square
		AddViewport(windowID, {0.333f, -1, 1.0, 1}, kScaleToSquare);
		// Needs to be called before load map, which resets some buttons
		SetupGUI(windowID);
		setTextBufferVisibility(false);
		m = new Map(1, 1);
		LoadMap(m, 0);
	}
	
}
bool updateBackground = true;
void DrawSim(Graphics::Display &display)
{
	if (updateBackground == true)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		updateBackground = false;
	}

	if (runningSearch1)
	{
		for (int x = 0; x < gStepsPerFrame; x++)
		{
			if (a1.DoSingleSearchStep(path))
			{
//					printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
//						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded(),
//						   a1.GetNumOpenItems());
				printf("%" PRId64 " unique expansions, %" PRId64 " total expansions\n", a1.GetUniqueNodesExpanded(), a1.GetNodesExpanded());
				runningSearch1 = false;
				break;
			}
		}
	}
	a1.Draw(display);

	if (mouseTracking)
	{
		me->SetColor(Colors::blue);
		me->DrawLine(display,xyLoc(px1, py1), xyLoc(px2, py2), 10);
	}
	

	if (path.size() != 0)
	{
		me->SetColor(Colors::blue);
		for (int x = 1; x < path.size(); x++)
		{
			me->DrawLine(display, path[x-1], path[x], 10);
		}

	}
}

void SetupGUI(int windowID)
{
	setTextBufferVisibility(false);
	// 4 algorithms
	Graphics::roundedRect algButton({-1+0.1f, -1.65f, -1+0.1f+0.4f, -1.55f}, 0.01f);
	Graphics::rect offsetRect(0.467f, 0, 0.467f, 0);
	int b;
	b = CreateButton(windowID, 1, algButton, "Dijkstra", '0', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "A*", '1', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "WA*(2)", '2', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "WA*(10)", '3', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	
	Graphics::roundedRect heuristicButton({-1+0.1f, -1.30f, -1+0.1f+0.5f, -1.2f}, 0.01f);
	offsetRect = Graphics::rect(0.65f, 0, 0.65f, 0);
	b = CreateButton(windowID, 1, heuristicButton, "Octile", 'o', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	heuristicButton.r += offsetRect;
	b = CreateButton(windowID, 1, heuristicButton, "Euclidean", 'e', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	heuristicButton.r += offsetRect;
	b = CreateButton(windowID, 1, heuristicButton, "Differential", 'h', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	Graphics::roundedRect mapButton({-1+0.1f, -0.95f, 0-0.1f, -0.85f}, 0.01f);
	offsetRect = Graphics::rect(1.0f, 0, 1.0f, 0);
	b = CreateButton(windowID, 1, mapButton, "Baldurs Gate", 'b', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	mapButton.r += offsetRect;
	b = CreateButton(windowID, 1, mapButton, "Dragon Age: Origins", 'd', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	Graphics::roundedRect speedButton({-1+0.1f, -0.6f, -1+0.1f+0.4f, -.5f}, 0.01f);
	offsetRect = Graphics::rect(0.467f, 0, 0.467f, 0);
	b = CreateButton(windowID, 1, speedButton, "1", '6', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, 1, speedButton, "10", '7', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, 1, speedButton, "100", '8', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, 1, speedButton, "1000", '9', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	Graphics::roundedRect reopeningsButton({-1+0.1f, -0.25f, 0-0.1f, -0.15f}, 0.01f);
	offsetRect = Graphics::rect(1.0f, 0, 1.0f, 0);
	b = CreateButton(windowID, 1, reopeningsButton, "Off", 't', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	reopeningsButton.r += offsetRect;
	b = CreateButton(windowID, 1, reopeningsButton, "On", 'r', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

}

void DrawGUI(Graphics::Display &display)
{
	const float e = 0.03f;
//	display.FillRect({-1, -1, 1, 1}, Colors::darkgray);
	display.FillRect({-1, -2, 1, 2}, Colors::darkgray);
	display.FillRect({-1+e, -2+e, 1-e, 2-e}, Colors::lightgray);
	display.DrawText("Algorithm: ", {-1+0.1f, -1.8f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Heuristic: ", {-1+0.1f, -1.45f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Map: ", {-1+0.1f, -1.10f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Expansions/frame: ", {-1+0.1f, -0.75f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Node re-openings: ", {-1+0.1f, -0.4f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (viewport == 0)
		DrawSim(display);
	else
		DrawGUI(display);
}




void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	char messageString[255];
	switch (key)
	{
		case 'r':
			reexpand = true;
			a1.SetReopenNodes(true);
//			if (a1.GetReopenNodes())
//				submitTextToBuffer("Reopenings enabled");
//			else
//				submitTextToBuffer("Reopenings disabled");
			SetButtonFillColor(buttons[kNoReOpen], Colors::white);
			SetButtonFillColor(buttons[kReOpen], Colors::yellow);
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			break;
		case 't': // turn off
			reexpand = false;
			a1.SetReopenNodes(false);
			SetButtonFillColor(buttons[kNoReOpen], Colors::yellow);
			SetButtonFillColor(buttons[kReOpen], Colors::white);

//			if (a1.GetReopenNodes())
//				submitTextToBuffer("Reopenings enabled");
//			else
//				submitTextToBuffer("Reopenings disabled");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			break;
		case '0':
			searchWeight = 0;
			submitTextToBuffer("Algorithm: Dijkstra");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kDikstraButton], Colors::yellow);
			SetButtonFillColor(buttons[kAStarButton], Colors::white);
			SetButtonFillColor(buttons[kWAStar2Button], Colors::white);
			SetButtonFillColor(buttons[kWAStar10Button], Colors::white);
			break;
		case '1':
			searchWeight = 1;
			submitTextToBuffer("Algorithm: A*");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kDikstraButton], Colors::white);
			SetButtonFillColor(buttons[kAStarButton], Colors::yellow);
			SetButtonFillColor(buttons[kWAStar2Button], Colors::white);
			SetButtonFillColor(buttons[kWAStar10Button], Colors::white);
			break;
		case '2':
			searchWeight = 2;
			submitTextToBuffer("Algorithm: WA*(2)");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kDikstraButton], Colors::white);
			SetButtonFillColor(buttons[kAStarButton], Colors::white);
			SetButtonFillColor(buttons[kWAStar2Button], Colors::yellow);
			SetButtonFillColor(buttons[kWAStar10Button], Colors::white);
			break;
		case '3':
			searchWeight = 10;
			submitTextToBuffer("Algorithm: WA*(10)");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kDikstraButton], Colors::white);
			SetButtonFillColor(buttons[kAStarButton], Colors::white);
			SetButtonFillColor(buttons[kWAStar2Button], Colors::white);
			SetButtonFillColor(buttons[kWAStar10Button], Colors::yellow);
			break;
		case '4':
			searchWeight = 100;
			submitTextToBuffer("Algorithm: WA*(100)");
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			break;
		case '6':
			gStepsPerFrame = 1;
			SetButtonFillColor(buttons[kS1], Colors::yellow);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::white);
			SetButtonFillColor(buttons[kS1000], Colors::white);
			break;
		case '7':
			gStepsPerFrame = 10;
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::yellow);
			SetButtonFillColor(buttons[kS100], Colors::white);
			SetButtonFillColor(buttons[kS1000], Colors::white);
			break;
		case '8':
			gStepsPerFrame = 100;
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::yellow);
			SetButtonFillColor(buttons[kS1000], Colors::white);
			break;
		case '9':
			gStepsPerFrame = 1000;
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::white);
			SetButtonFillColor(buttons[kS1000], Colors::yellow);
			break;
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 100;
			else if (searchWeight == 100.0)
				searchWeight = 0;
			a1.SetWeight(searchWeight);
			printf("Search weight is %1.2f\n", searchWeight);
			break;
		case '[':
			gStepsPerFrame /= 2;
			sprintf(messageString, "Speed: %d steps per frame", gStepsPerFrame);
			submitTextToBuffer(messageString);
			break;
		case ']':
			if (gStepsPerFrame <= 16384)
				gStepsPerFrame *= 2;
			if (gStepsPerFrame == 0)
				gStepsPerFrame = 1;
			sprintf(messageString, "Speed: %d steps per frame", gStepsPerFrame);
			submitTextToBuffer(messageString);
			break;
//		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'b':
		{
			LoadMap(m, 0);
			SetButtonFillColor(buttons[kDragonAge], Colors::white);
			SetButtonFillColor(buttons[kBaldursGate], Colors::yellow);
		}
			break;
		case 'd':
		{
			LoadMap(m, 1);
			SetButtonFillColor(buttons[kDragonAge], Colors::yellow);
			SetButtonFillColor(buttons[kBaldursGate], Colors::white);
		}
			break;
		case 'e':
		{
			searchHeuristic = &euclid;
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::yellow);
			SetButtonFillColor(buttons[kOctileHeuristic], Colors::white);
			SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);
		}
			break;
		case 'o':
		{
			searchHeuristic = me; // octile
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
			SetButtonFillColor(buttons[kOctileHeuristic], Colors::yellow);
			SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);
		}
			break;
		case 'h':
		{
			searchHeuristic = dh; // octile
			if (runningSearch1 || path.size() > 0)
				StartSearch();
			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
			SetButtonFillColor(buttons[kOctileHeuristic], Colors::white);
			SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::yellow);
		}
			break;
		default:
			break;
	}
}

void StartSearch()
{
	a1.SetStopAfterGoal(true);
	xyLoc s1;
	xyLoc g1;
	s1.x = px1; s1.y = py1;
	g1.x = px2; g1.y = py2;
	
	a1.SetWeight(searchWeight);
	a1.SetReopenNodes(reexpand);
	me->SetEightConnected();
	a1.SetHeuristic(searchHeuristic);
	a1.InitializeSearch(me, s1, g1, path);
	mouseTracking = false;
	runningSearch1 = true;
}

bool MyClickHandler(unsigned long windowID, int viewport, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (viewport == 1)
		return false;
	if (button == kLeftButton)
	{
		switch (mType)
		{
			case kMouseDown:
			{
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if ((me->GetMap()->GetTerrainType(x, y)>>terrainBits) != (kGround>>terrainBits))
					break;

				px1 = px2 = x;
				py1 = py2 = y;
				path.resize(0);
				mouseTracking = true;
			}
				break;
			case kMouseDrag:
			{
				if (!mouseTracking)
					break;
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if ((me->GetMap()->GetTerrainType(x, y)>>terrainBits) == (kGround>>terrainBits))
				{
					px2 = x;
					py2 = y;
				}
			}
				break;
			case kMouseUp:
			{
				if (!mouseTracking)
					break;
				int x, y;
				me->GetMap()->GetPointFromCoordinate(loc, x, y);
				if ((me->GetMap()->GetTerrainType(x, y)>>terrainBits) == (kGround>>terrainBits))
				{
					px2 = x;
					py2 = y;
				}
				StartSearch();
			}
				break;
		}
		return true;
	}
	return false;
}


void DAOMap(Map *m)
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

void BGMap2(Map *m)
{
	m->Scale(148, 139);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@....@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@.@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@.@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@...@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@....@..@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@..@.......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@........@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...................@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@.@@@@@@@...............@@@@@@@@..........@@@@@..@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@.......@@@@@.............@@@@@@@@..........@@@@@...@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@.........@@@@@...........@@@@@@@@........@@@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@..........@@@@@@@@.........@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........@@@@@@@.........@@@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@.........@@@@@@@.........@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@..........@@@@@@.........@@@@@@.....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.............@..@.........@@@@@@.....@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@.........................@@@@@@......@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........................@@@@@@.....@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@..........................@@@@@@.....@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@.................@@@@@@.....@@@@...@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@.........@@@@...............@@@@@@@....@@@@...@@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@.............@@@@@......@@@@...@@@@@@.@...@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@@............@@@@@......@@@@...@@@@@@@@@..@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@.......@@@@@@@@............@@@@......@@@@....@@@@@@@@..@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@@@@@@.............@@@.......@@.....@@@@@@@..@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@.......@@@@@@@@................@@................@@.@..@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@.......@@@@@@@@..........@.............................@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@........@@@@@@@..........@@@...........................@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@..@@@@.........@@@@@@..........@@@@@.........................@@@@..@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@..........@@@@@..........@@@@@@........................@@@@..@@@@@@....@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@............@@@..........@@@@@@@.......................@@@@..@@@@@@@...@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@.........................@@@@@@........................@@@@..@@@@@@@...@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@......................@@@@@@....................@@...@@@....@@@@@...@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@.....................@@@@@@....................@@@@...@......@@@...@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@.....@.....@@@@@@...................@@@@@@....................@@@@@@..............@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@.....@@@.....@@@@@..................@@@@@@@........@.@@.......@@@..@@@............@@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.....@@@@.................@@@@@..........@@@@@@.....@@@....@@@..........@@@@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.....@..................@@@@@...........@@@@@@....@@@......@@@........@@@@@@@@@@@@.......@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@........@@@...........@@@@@...........@@@@@@....@@@........@@@......@@@@.@@@@@@@@@.....@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@........@@@@.........@@@@@@............@@@@....@@@..........@@@....@@@@..@.@@@@@@@....@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@..@......@@@@@.......@@@@@@@.............@@....@@@............@@@...@@@.....@@@@@@....@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@..@@@.......@@@@.....@@@@@@@@@.................@@@..............@@....@.......@@@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@...@@@@@@@@...................@@...............@@@............@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.......@@@@.@@@@@@@@...................@@.........@@......@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@.......@@@@@@@@@@@@...................@@@.........@@@.....@@@...............@@@@.............@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@.......@@@@@@@@@@@@......@@...........@@@@..........@@......@...............@@@@.............@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................@@@@@@@@@@@@......@@@@.........@@@@@@.........@@.....................@@@@.............@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@......@@@@@@........@@@@@@@........@@@.....@@............@@@@.............@@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@.....@@@@@@@@@......@@@@@@@@@@....@@@@.....@@...........@@@@.............@@@@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@....@@@@@@@@@@@......@@@@@@@@@@@@@@..@@.................@@@.............@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@@@@@....@@@@@@@@@@@@@@@......@@@@@@@@@@.....@@..................@.............@@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@@@@@.....@@@@@@@@@@@@@@@.......@@@@@@@@.......@...............................@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@@......@@@@@@@@@@@@@@..........@@@@@.......................................@@@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@.@@........@@@@@@@@@@@@............@@@.......................................@@@@@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@.....@@.......@.......................................@@@@@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@.....@@@@@............................................@@@@@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@......@@@@@@@..........................................@@@@@@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@.....@@@@@@@@@........................................@@@@@@@@..........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@....@@@@@@@@@@@......................................@@@@@@@@..........@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@@....@@@@@@@@@@@@....................................@@@@@@@@..........@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@.....@@@@@@@@@@@@@@@..................................@@@@@@@@............@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@.....@@@@@@@@@@@@@@@..................................@@@@@@@@..............@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@......@@@@@@@@@@@@....................................@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@...........@@@@@@@@@@.....@@...............................@@.@@@.....@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@...........@@@@@@@@@.....@@@@...............................@........@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@.............@@@@@@@.....@@@@@@......................................@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@.....@@@@@@@@......................................@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@....@@@@@@@@@@......................................@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@...@@@@@@@@@@@@......................................@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@....@@@@@@@@@@@@@@......................................@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@......@@@@@@@@@@@@@@................................@......@@@@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....@@@@@@@@@@@@@@.............@...................@@......@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@.@@@......@@@@@@@@@@@@.............@@@...................@@......@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@...........@@@@@@@@@@.............@@@@@...................@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@@...........@@@@@@@@@.............@@@@@@...........@@.......@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@.............@@@@@@@@.............@@@@@@@..........@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.............@@@@@@............@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@@............@@@@@@..............@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@...........@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.....@.......@@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@......@@.....@@@@@@@@...........@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@.......@@@...@@@@@@@@.............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@...........@@@@@@@@@@@@...............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@...........@@@@@@@@@@@.......@@........@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@............@@@@@@@@@@.......@@@@........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@@@........@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@............@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@..............@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@...................@@@@......@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....................@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@.......................@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@.........................@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@.@@...........@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@..............@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
	int which = 0;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				m->SetTerrainType(x, y, kGround);
			else
				m->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
}


void LoadMap(Map *m, int which)
{
	assert(m != 0);
	if (which == 0)
		BGMap2(m);
	else if (which == 1)
		DAOMap(m);
	runningSearch1 = false;
	updateBackground = true;
	mouseTracking = false;
	a1.Reset();
	path.resize(0);
	m->SetTileSet(kWinter);
	if (me == 0)
	{
		me = new MapEnvironment(m);
		me->SetDiagonalCost(1.5);
	}
	searchHeuristic = me;
	SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
	SetButtonFillColor(buttons[kOctileHeuristic], Colors::yellow);
	SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);

	delete dh;
	dh = new GridEmbedding(me, 10, kLINF);
	for (int x = 0; x < 10; x++)
		dh->AddDimension(kDifferential, kFurthest);

}

