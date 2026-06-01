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

bool recording = false;
bool mouseTracking = false;
bool runningSearch1 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int gStepsPerFrame = 0;
float searchWeight = 2;
bool reexpand = false;
void LoadMap(Map *m, int which);
void StartSearch();

float angle1 = float(PID4);
float angle2 = float(1.1f*PID3);
float dist1 = 0.8f, dist2 = 1.2f;
float maxSearchAngle = 0;

void SaveSVG(Graphics::Display &d, int port=-1);

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
MapEnvironment *me = 0;
Map *m;
Heuristic<xyLoc> *searchHeuristic = 0;
EuclideanDistance euclid;
std::vector<xyLoc> path;
GridEmbedding *dh;
void SetupGUI(int windowID);
void ReduceMap(Map *inputMap);

std::vector<int> buttons;
typedef enum : int {
	kDikstraButton = 0,
	kAStarButton = 1,
	kWAStar2Button = 2,
	kWAStar10Button = 3,
	kOctileHeuristic = 4,
	kDifferentialHeuristic = 5,
	kBaldursGate = 6,
	kDragonAge = 7,
	kS0 = 8,
	kS1 = 9,
	kS10 = 10,
	kS100 = 11,
	kStepSearch = 12,
//	kReOpen = 13,
} buttonID;

const int kMapViewport = 0;
const int kPhiViewport = 1;
const int kGUIViewport = 2;


int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 2000, 800);
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
	InstallKeyboardHandler(MyDisplayHandler, "0/frame", "Expand 0/frame", kAnyModifier, '6');
	InstallKeyboardHandler(MyDisplayHandler, "1/frame", "Expand 1/frame", kAnyModifier, '7');
	InstallKeyboardHandler(MyDisplayHandler, "10/frame", "Expand 10/frame", kAnyModifier, '8');
	InstallKeyboardHandler(MyDisplayHandler, "100/frame", "Expand 100/frame", kAnyModifier, '9');

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
		ReinitViewports(windowID, {-1, -1, -0.2f, 1}, kScaleToSquare);
		AddViewport(windowID, {-0.2f, -1, 0.6f, 1}, kScaleToSquare);
		// 1x2 square
		AddViewport(windowID, {0.6f, -1, 1.0f, 1}, kScaleToSquare);
		// Needs to be called before load map, which resets some buttons
		SetupGUI(windowID);
		setTextBufferVisibility(false);
		m = new Map(1, 1);
		LoadMap(m, 0);
	}
	
}

bool StepSearch()
{
	float g = a1.GetOpenItem(0).g;
	float h = a1.GetOpenItem(0).h;
	maxSearchAngle = std::max(maxSearchAngle, atan2f(g, h));
	if (a1.DoSingleSearchStep(path))
	{
//					printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
//						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded(),
//						   a1.GetNumOpenItems());
		printf("%" PRId64 " unique expansions, %" PRId64 " total expansions\n", a1.GetUniqueNodesExpanded(), a1.GetNodesExpanded());
		runningSearch1 = false;
		return true;
	}
	return false;
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
			if (StepSearch())
				break;
		}
	}
	//a1.Draw(display);
	a1.DrawSuboptimality(display);

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
	b = CreateButton(windowID, kGUIViewport, algButton, "Dijkstra", '0', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	SetButtonActive(false, b);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, algButton, "A*", '1', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	SetButtonActive(false, b);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, algButton, "WA*(2)", '2', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, algButton, "WA*(10)", '3', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	SetButtonActive(false, b);
	buttons.push_back(b);

	
	Graphics::roundedRect heuristicButton({-1+0.1f, -1.30f, -1+0.1f+0.5f, -1.2f}, 0.01f);
	offsetRect = Graphics::rect(0.65f, 0, 0.65f, 0);
	b = CreateButton(windowID, kGUIViewport, heuristicButton, "Octile", 'o', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	heuristicButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, heuristicButton, "Differential", 'h', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
//	heuristicButton.r += offsetRect;
//	b = CreateButton(windowID, kGUIViewport, heuristicButton, "Differential", 'h', 0.01f, Colors::black, Colors::black,
//					 Colors::white, Colors::lightblue, Colors::lightbluegray);
//	buttons.push_back(b);

	Graphics::roundedRect mapButton({-1+0.1f, -0.95f, 0-0.1f, -0.85f}, 0.01f);
	offsetRect = Graphics::rect(1.0f, 0, 1.0f, 0);
	b = CreateButton(windowID, kGUIViewport, mapButton, "Random", 'b', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	mapButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, mapButton, "Maze", 'd', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	Graphics::roundedRect speedButton({-1+0.1f, -0.6f, -1+0.1f+0.4f, -.5f}, 0.01f);
	offsetRect = Graphics::rect(0.467f, 0, 0.467f, 0);
	b = CreateButton(windowID, kGUIViewport, speedButton, "0", '6', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, speedButton, "1", '7', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, speedButton, "10", '8', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	speedButton.r += offsetRect;
	b = CreateButton(windowID, kGUIViewport, speedButton, "100", '9', 0.01f, Colors::black, Colors::black,
					 Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	Graphics::roundedRect reopeningsButton({-1+0.1f, -0.25f, 0-0.1f, -0.15f}, 0.01f);
	offsetRect = Graphics::rect(1.0f, 0, 1.0f, 0);
	b = CreateButton(windowID, kGUIViewport, reopeningsButton, "Step", 't', 0.01f, Colors::black, Colors::black,
					 Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
//	reopeningsButton.r += offsetRect;
//	b = CreateButton(windowID, kGUIViewport, reopeningsButton, "On", 'r', 0.01f, Colors::black, Colors::black,
//					 Colors::white, Colors::lightblue, Colors::lightbluegray);
//	buttons.push_back(b);

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
	display.DrawText("Search Control: ", {-1+0.1f, -0.4f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
}

point3d HOGToLocal(point3d p)
{
	return point3d((p.x+1)*searchWeight/2.0f , (p.y-1)*searchWeight/-2.0);
}

point3d LocalToHOG(point3d p)
{
	return point3d(2.0f*(p.x/searchWeight-0.5), -2.0*(p.y/searchWeight-0.5));
}

void DrawPhi(Graphics::Display &display)
{
	point3d origin(-1, 1);
	float priority = 1;
	
	display.FillRect({-1, -2, 1, 2}, Colors::black);
	
	// draw bounding line
	point3d bl1(priority, 0.0f), bl2(0.0f, priority*searchWeight); // main suboptimality line
	point3d bl3(priority*searchWeight, 0.0f);//(0+2, priority*bound-2); //
	point3d bl4(priority*searchWeight/(2*searchWeight-1), 0.0f);//priority*bound-(2*bound-1));
	point3d bl2a(0.0f, priority);
	point3d bl2c(priority-priority*searchWeight/(2.0f*searchWeight-1.0f), priority*searchWeight);
	point3d m1(0.5f, 0.5f);
	point3d m2((searchWeight-1)/(2*searchWeight-2), -(searchWeight-1)/(2*searchWeight-2)+searchWeight);
	// WA* priority line
	//    display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1.5f/100.0f, Colors::yellow);
	// 45° upper bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(/*bl3*/m2), 1.5f/100.0f, Colors::lightgray);
	// 2w-1 upper bound line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(m2/*bl2c*/), 1.5f/100.0f, Colors::lightgray);
	
	// 45° lower bound line
	display.DrawLine(LocalToHOG(m1/*bl2a*/), LocalToHOG(bl1), 1.5f/100.0f, Colors::lightgray);
	// 2w-1 lower bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(/*bl4*/m1), 1.5f/100.0f, Colors::lightgray);
	
	//	for (int x = 0; x < data.size(); x++)
	//		display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::green);
	
	display.DrawLine(origin, {1, 1}, 1.f/100.0f, Colors::white);
	display.DrawLine(origin, {-1, -1}, 1.f/100.0f, Colors::white);
	
	// Now draw the lines people can change
	Graphics::point l1(cos(angle1)*searchWeight, sin(angle1)*searchWeight);
	Graphics::point l2(cos(angle2)*searchWeight, sin(angle2)*searchWeight);
	Graphics::point l_curr(cos(maxSearchAngle)*searchWeight, sin(maxSearchAngle)*searchWeight);
	display.DrawLine(origin, LocalToHOG(l1), 1.75f/100.0f, Colors::lightyellow);
	display.DrawLine(origin, LocalToHOG(l2), 1.75f/100.0f, Colors::lightred);
	display.DrawLine(origin, LocalToHOG(l_curr), 1.75f/100.0f, Colors::lightblue);
	
	Graphics::point d1 = l1;
	d1.normalise();
	d1 *= dist1;
	Graphics::point d2 = l2;
	d2.normalise();
	d2 *= dist2;
	display.FillCircle(LocalToHOG(d1), 0.025f, Colors::lightyellow);
	display.FillCircle(LocalToHOG(d2), 0.025f, Colors::lightred);
	
	display.DrawLine(LocalToHOG(point3d(1, 0)), LocalToHOG(d1), 1.75f/100.0f, Colors::purple);
	display.DrawLine(LocalToHOG(d1), LocalToHOG(d2), 1.75f/100.0f, Colors::purple);
	display.DrawLine(LocalToHOG(d2), LocalToHOG(point3d(0, searchWeight)), 1.75f/100.0f, Colors::purple);

	float w1, w2, w3;
	float k1, k2, k3;
	w1 = d1.y/(1-d1.x);
	w2 = (d2.y-d1.y)/(d1.x-d2.x);
	w3 = (searchWeight-d2.y)/(d2.x);
	// f1 = k1(g+w1*h);
	// 1 = k1 (0 + w1 * 1)
	// 1 = k1 (w1)
	k1 = 1/w1;
	// f2 = k2(g + w2*h)
	// 1 = k2 (d1.y + w2*d1.x)
	k2 = 1/(d1.y + w2*d1.x);
	// f3 = k3(g + w3*h)
	k3 = 1/(d2.y + w3*d2.x);
	a1.SetPhi(
			  [=](double h,double g)
			  { if (h <= 0) return (double)(g+h)/searchWeight;
				  if (g/h < d1.y/d1.x)
					  return k1*(g+w1*h);
				  if (g/h < d2.y/d2.x)
					  return k2*(g+w2*h);
				  return k3*(g+w3*h);
			  });
	
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (viewport == kMapViewport)
		DrawSim(display);
	else if (viewport == kPhiViewport)
		DrawPhi(display);
	else if (viewport == kGUIViewport)
	{
		DrawGUI(display);
		if (recording)
			SaveSVG(display);
	}
}




void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	char messageString[255];
	switch (key)
	{
		case 'r':
			recording = !recording;
//			reexpand = true;
//			a1.SetReopenNodes(true);
//			if (a1.GetReopenNodes())
//				submitTextToBuffer("Reopenings enabled");
//			else
//				submitTextToBuffer("Reopenings disabled");
//			SetButtonFillColor(buttons[kNoReOpen], Colors::white);
//			SetButtonFillColor(buttons[kReOpen], Colors::yellow);
//			if (runningSearch1 || path.size() > 0)
//				StartSearch();
			break;
		case 't':
			StepSearch();
//			reexpand = false;
//			a1.SetReopenNodes(false);
//			SetButtonFillColor(buttons[kNoReOpen], Colors::yellow);
//			SetButtonFillColor(buttons[kReOpen], Colors::white);

//			if (a1.GetReopenNodes())
//				submitTextToBuffer("Reopenings enabled");
//			else
//				submitTextToBuffer("Reopenings disabled");
//			if (runningSearch1 || path.size() > 0)
//				StartSearch();
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
			gStepsPerFrame = 0;
			SetButtonFillColor(buttons[kS0], Colors::yellow);
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::white);
			break;
		case '7':
			gStepsPerFrame = 1;
			SetButtonFillColor(buttons[kS0], Colors::white);
			SetButtonFillColor(buttons[kS1], Colors::yellow);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::white);
			break;
		case '8':
			gStepsPerFrame = 10;
			SetButtonFillColor(buttons[kS0], Colors::white);
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::yellow);
			SetButtonFillColor(buttons[kS100], Colors::white);
			break;
		case '9':
			gStepsPerFrame = 100;
			SetButtonFillColor(buttons[kS0], Colors::white);
			SetButtonFillColor(buttons[kS1], Colors::white);
			SetButtonFillColor(buttons[kS10], Colors::white);
			SetButtonFillColor(buttons[kS100], Colors::yellow);
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
//			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::yellow);
			SetButtonFillColor(buttons[kOctileHeuristic], Colors::white);
			SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);
		}
			break;
		case 'o':
		{
			searchHeuristic = me; // octile
			if (runningSearch1 || path.size() > 0)
				StartSearch();
//			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
			SetButtonFillColor(buttons[kOctileHeuristic], Colors::yellow);
			SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);
		}
			break;
		case 'h':
		{
			searchHeuristic = dh; // octile
			if (runningSearch1 || path.size() > 0)
				StartSearch();
//			SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
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
	maxSearchAngle = 0;
	mouseTracking = false;
	runningSearch1 = true;
}

bool CheckBound(float slope, float dist)
{
//	rise/run = slope;
//	rise = slope*run
//	rise^2+run^2 = dist^2
//	run^2*slope^2+run^2 = dist^2
//	run^2*(slope^2+1) = dist^2
//	run^2 = dist^2/(slope^2+1)
	
	float run = dist/sqrt(slope*slope+1);
	float rise = slope*run;
//	printf(":: rise %f run %f\n", rise, run);
	if (run > 1)
		return false;
	if (rise > searchWeight)
		return false;
	if (rise/(1.0f-run) < 1) // slope from bottom right
		return false;
	if (rise/(1.0f-run) > 2*searchWeight-1) // slope from bottom right
		return false;
	if ((searchWeight-rise)/(run) < 1) // slope from bottom right
		return false;
	if ((searchWeight-rise)/(run) > 2*searchWeight-1) // slope from bottom right
		return false;
	return true;
}

void HandlePhiClick(point3d loc, tButtonType button, tMouseEventType mType)
{
	if ((loc.x <= -1) || (loc.y >= 1))
		return;
	if (button == kLeftButton)
	{
		//loc = LocalToHOG(loc);
		float angle = atan2f(1-loc.y, loc.x+1);
		float slope = (1-loc.y)/(loc.x+1);
		float dist = (loc-point3d(-1, 1, 0)).length();

//		if (dist > searchWeight)
//			dist = searchWeight;
//		printf("angle: %f, dist: %f\n", angle/PID180, dist);
		if (!CheckBound(slope, dist))
			return;
		if (angle > angle2 && maxSearchAngle <= angle1)
		{
			angle2 = angle;
			dist2 = dist;
		}
		else if (angle < angle1)
		{
			if (maxSearchAngle <= 0)
			{
				angle1 = angle;
				dist1 = dist;
			}
			else if (maxSearchAngle < angle) // can move, but have to keep the same slope
			{
				float curry = sin(angle1)*dist1;
				float currx = cos(angle1)*dist1;
				float currWeight = curry/(1-currx);
				// sin(angle)*dist/(1-cos(angle)*dist) = currWeight
				// sin(angle)*dist = currWeight * (1-cos(angle)*dist)
				// sin(angle)*dist+currWeight*cos(angle)*dist = currWeight
				// dist*(sin(angle)+currWeight*cos(angle)) = currWeight
				dist1 = currWeight/(sin(angle)+currWeight*cos(angle));
				angle1 = angle;
			}
		}
		else if (fabs(angle2-angle) < fabs(angle1-angle) && maxSearchAngle <= angle1)
		{
			angle2 = angle;
			dist2 = dist;
		}
		else if (angle1 > maxSearchAngle)
		{
			float curry = sin(angle1)*dist1;
			float currx = cos(angle1)*dist1;
			float currWeight = curry/(1-currx);
			// sin(angle)*dist/(1-cos(angle)*dist) = currWeight
			// sin(angle)*dist = currWeight * (1-cos(angle)*dist)
			// sin(angle)*dist+currWeight*cos(angle)*dist = currWeight
			// dist*(sin(angle)+currWeight*cos(angle)) = currWeight
			float d = currWeight/(sin(angle)+currWeight*cos(angle));
			if (CheckBound(sin(angle)/cos(angle), d))
			{
				angle1 = angle;
				dist1 = d;
			}
		}
	}
}

bool MyClickHandler(unsigned long windowID, int viewport, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (viewport == kGUIViewport)
		return true;
	if (viewport == kPhiViewport)
	{
		HandlePhiClick(loc, button, mType);
		return true;
	}
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
				if (!(px1 == px2 && py1 == py2))
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
	{
		static int seed = 1;
		srandom(seed++);
		m->Scale(100, 100);
		m->SetRectHeight(0, 0, 100-1, 100-1, 0, kGround);
		MakeRandomMap(m, 20);//BGMap2(m);
//		ReduceMap(m);
	}
	else if (which == 1)
	{
		m->Scale(100, 100);
		MakeMaze(m, 2);
	}
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
//	SetButtonFillColor(buttons[kEuclideanHeuristic], Colors::white);
	SetButtonFillColor(buttons[kOctileHeuristic], Colors::yellow);
	SetButtonFillColor(buttons[kDifferentialHeuristic], Colors::white);

	delete dh;
	dh = new GridEmbedding(me, 10, kLINF);
	for (int x = 0; x < 10; x++)
		dh->AddDimension(kDifferential, kFurthest);

	// swamp isn't part of heuristic
	for (int x = 0; x < 100; x++)
		for (int y = 40; y < 60; y++)
		{
			if (m->GetTerrainType(x, y) == kGround)
				m->SetTerrainType(x, y, kSwamp);
		}

}

int LabelConnectedComponents(Graph *g);

void ReduceMap(Map *inputMap)
{
	Graph *g = GraphSearchConstants::GetGraph(inputMap);
	int biggest = LabelConnectedComponents(g);
	
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) != biggest)
			inputMap->SetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
									 g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY), kOutOfBounds);
	}
	delete g;
}

int LabelConnectedComponents(Graph *g)
{
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	int group = 0;
	std::vector<int> groupSizes;
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
		{
			group++;
			groupSizes.resize(group+1);

			std::vector<unsigned int> ids;
			ids.push_back(x);
			while (ids.size() > 0)
			{
				unsigned int next = ids.back();
				ids.pop_back();
				if (g->GetNode(next)->GetLabelL(GraphSearchConstants::kTemporaryLabel) != 0)
					continue;
				groupSizes[group]++;
				g->GetNode(next)->SetLabelL(GraphSearchConstants::kTemporaryLabel, group);
				for (int y = 0; y < g->GetNode(next)->GetNumEdges(); y++)
				{
					edge *e = g->GetNode(next)->getEdge(y);
					if (g->GetNode(e->getFrom())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getFrom());
					if (g->GetNode(e->getTo())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getTo());
				}
			}
		}
	}
	int best = 0;
	for (unsigned int x = 1; x < groupSizes.size(); x++)
	{
		printf("%d states in group %d\n", groupSizes[x], x);
		if (groupSizes[x] > groupSizes[best])
			best = x;
	}
	printf("Keeping group %d\n", best);
	return best;
	//	kMapX;
}

#include "FileUtil.h"
#include "SVGUtil.h"
void SaveSVG(Graphics::Display &d, int port)
{
	const std::string baseFileName = "/Users/nathanst/Pictures/SVG/DSWA_";
	static int count = 0;
	std::string fname;
	do {
		fname = baseFileName+std::to_string(count)+".svg";
		count++;
	} while (FileExists(fname.c_str()));
	printf("Save to '%s'\n", fname.c_str());
	MakeSVG(d, fname.c_str(), 2000, 800);
}
