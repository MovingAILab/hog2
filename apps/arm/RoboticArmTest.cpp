/*
 *  RoboticArmTest.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Common.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include <deque>
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "RoboticArmTest.h"
#include "FrontierBFS.h"

#include "RoboticArm.h"
#include "FileUtil.h"
#include "SVGUtil.h"

//#define RUN_RANDOM_TESTS
//#define HEURISTIC_TABLES
//#define MAX_DIST_HEUR_TABLES
#define FIXED_RANDOM_NUMBER_SEED 7

int activeArm = 0;
bool redrawBackground = true;
enum armMouseMode {
	kAddObstacle,
	kSetTarget,
	kManualMoveArm
};
armMouseMode mode = kAddObstacle;
const int numArms = 2;
RoboticArm *r = 0;
armAngles config;
armAngles goal;
bool validSearch = false;
unsigned int pathLoc = 0;
std::vector<armAngles> ourPath;
TemplateAStar<armAngles, armRotations, RoboticArm> astar;
float totalTime;
ArmToArmHeuristic *aa = 0;
void TestArms();
void TestArms2(bool h);
void BuildTipTables();
void Build4ArmDH();

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
int stepsPerFrame = 100;

std::vector<std::vector<bool> > configSpace;
void BuildConfigSpace(bool fast = false);

bool recording = false;

//std::vector<PuzzleSimulation *> unitSims;

int main(int argc, char* argv[])
{
	InstallHandlers();
	setvbuf(stdout, NULL, _IONBF, 0);
	RunHOGGUI(argc, argv, 1024, 512);
	return 0;
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int)
{
	if (r != 0)
		delete r;
	r = new RoboticArm(numArms, 1.0/(double)numArms);

//	r->AddObstacle(line2d(recVec(-0.32, -0.30, 0), recVec(0.32, -0.30, 0)));
//	r->AddObstacle(line2d(recVec(0.30, 0.32, 0), recVec(0.30, -0.32, 0)));
//	r->AddObstacle(line2d(recVec(0.28, 0.30, 0), recVec(0.32, 0.30, 0)));
//	r->AddObstacle(line2d(recVec(-0.30, -0.32, 0), recVec(-0.30, 0.32, 0)));
//	r->AddObstacle(line2d(recVec(-0.28, 0.30, 0), recVec(-0.32, 0.30, 0)));

#if 0
	r->AddObstacle(line2d(recVec(0.50, 0, 0), recVec(0.52, 0.02, 0)));
	r->AddObstacle(line2d(recVec(0.52, 0.02, 0), recVec(0.54, 0, 0)));
	r->AddObstacle(line2d(recVec(0.54, 0, 0), recVec(0.52, -0.02, 0)));
	r->AddObstacle(line2d(recVec(0.52, -0.02, 0), recVec(0.50, 0, 0)));
#endif
	
#if 0
	line2d l1({-0.37, -1.14, 0}, {-0.99, -0.11, 0});
	r->AddObstacle(l1);
	line2d l2({1.18, -0.47, 0}, {0.45, -0.02, 0});
	r->AddObstacle(l2);
	line2d l3({0.45, -0.02, 0}, {1.14, 0.40, 0});
	r->AddObstacle(l3);
	line2d l4({-1.13, 0.36, 0}, {-0.50, 1.13, 0});
	r->AddObstacle(l4);
	line2d l5({0.37, -0.74, 0}, {0.64, -1.16, 0});
	r->AddObstacle(l5);
	line2d l6({0.37, -0.74, 0}, {0.11, -1.13, 0});
	r->AddObstacle(l6);
	BuildConfigSpace();
#endif
	

	config.SetNumArms( numArms );
	for (int x = 0; x < numArms; x++)
		config.SetAngle( x, 2 );
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Increase search speed", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Decrease search speed", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyKeyHandler, "Config", "Build Configuration Space", kNoModifier, 'c');
	InstallKeyboardHandler(MyKeyHandler, "Remove Last", "Remove last obstacle from config space", kNoModifier, '|');
	InstallKeyboardHandler(MyKeyHandler, "Reset", "Reset to initial position", kNoModifier, 'r');
//	InstallKeyboardHandler(MyKeyHandler, "Save", "Save a screen capture", kNoModifier, '&');
	InstallKeyboardHandler(MyKeyHandler, "0-9", "select segment", kNoModifier, '0', '9');
	InstallKeyboardHandler(MyKeyHandler, "Rotate segment", "rotate segment CW", kNoModifier, 'a');
	InstallKeyboardHandler(MyKeyHandler, "Rotate segment", "rotate segment CCW", kNoModifier, 's');

	InstallKeyboardHandler(MyKeyHandler, "Use PWXD(1.5)", "", kNoModifier, 'z');
	InstallKeyboardHandler(MyKeyHandler, "Use XDP(1.5)", "", kNoModifier, 'x');
	InstallKeyboardHandler(MyKeyHandler, "Use Weighted A*(1.5)", "", kNoModifier, 'w');
	InstallKeyboardHandler(MyKeyHandler, "Use A*", "", kNoModifier, 'd');

	InstallKeyboardHandler(MyKeyHandler, "Target", "Set target", kNoModifier, 't');
	InstallKeyboardHandler(MyKeyHandler, "Obstacle", "Add obstacle", kNoModifier, 'o');
	InstallKeyboardHandler(MyKeyHandler, "Move Arm", "Manually move arm", kNoModifier, 'm');
//	InstallKeyboardHandler(MyKeyHandler, "JPS", "Build & test jps", kNoModifier, 'j');
//	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		CreateSimulation(windowID);
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		setTextBufferVisibility(false);
	}
}

void MyFrameHandler(unsigned long id, unsigned int viewport, void *)
{
	Graphics::Display &display = GetContext(id)->display;
	if (viewport == 0)
	{
		static int currFrame = 0;
		currFrame++;
		r->Draw(display);
		if (!validSearch)
		{
			if (ourPath.size() == 0)
			{
				//r->OpenGLDraw(config);
				r->Draw(display, config);
				if (mode == kManualMoveArm)
					r->Draw(display, config, activeArm, Colors::lightblue);
			}
			else {
				r->Draw(display,ourPath[(pathLoc++)%ourPath.size()]);
				//r->OpenGLDraw(ourPath[(pathLoc++)%ourPath.size()]);
			}
		}
		else {
			Timer t;
			t.StartTimer();
			for (int x = 0; x < stepsPerFrame; x++)
			{
				if (astar.DoSingleSearchStep(ourPath))
				{
					validSearch = false;
					if (ourPath.size() > 0)
					{
						totalTime += t.EndTimer();
						printf("Done! %lld nodes expanded; %1.1f nodes/sec\n",
							   astar.GetNodesExpanded(), (double)astar.GetNodesExpanded()/totalTime);
						config = ourPath.back();
						pathLoc = 0;
						break;
					}
				}
			}
			totalTime += t.EndTimer();
			if ((currFrame%500) == 499)
				printf("Currently generating %1.1f nodes/sec\n", (double)astar.GetNodesExpanded()/totalTime);
			if (astar.GetNumOpenItems() == 0)
			{
				validSearch = false;
			}
			if (validSearch)
			{
				armAngles next = astar.CheckNextNode();
				r->Draw(display, next);
				r->Draw(display, goal);
//				r->OpenGLDraw(next);
//				r->OpenGLDraw(goal);
			}
			//astar.GetPath(r, config, goal, ourPath);
		}
		if (mode == kAddObstacle)
			display.DrawText("Click to Add Obstacle", {-1, -1}, Colors::yellow, 0.05f, Graphics::textAlignLeft, Graphics::textBaselineTop);
		if (mode == kSetTarget)
			display.DrawText("Click to Set Target", {-1, -1}, Colors::yellow, 0.05f, Graphics::textAlignLeft, Graphics::textBaselineTop);
		if (mode == kManualMoveArm)
		{
			static char text[] = "Moving arm segment X";
			text[19] = '0'+activeArm;
			display.DrawText(text, {-1, -1}, Colors::yellow, 0.05f, Graphics::textAlignLeft, Graphics::textBaselineTop);
		}
		display.DrawText("Work space ", {1, -1}, Colors::lightblue, 0.05f, Graphics::textAlignRight, Graphics::textBaselineTop);
	}
	if (viewport == 1)
	{
		rgbColor c;

		if (redrawBackground)
		{
			display.StartBackground();
			// TODO: draw into background when config space changes
			display.FillRect({-1, -1, 1, 1}, Colors::black);
			if (configSpace.size() != 0)
			{
				float ratio = 2.0/configSpace.size();
				float size = 2.0/configSpace.size();
				for (int x = 0; x < configSpace.size(); x++)
				{
					int start = -1;
					for (int y = 0; y < configSpace[x].size(); y++)
					{
						if (configSpace[x][y] == false)
						{
							if (start == -1)
								start = y;
						}
						else {
							if (start != -1)
							{
								Graphics::rect r(-1+float(x)*size, -1+float(start)*size,
												 -1+float(x+1)*size, -1+float(y+1)*size);
								display.FillRect(r, Colors::red);
							}
							start = -1;
						}
					}
					if (start != -1)
					{
						Graphics::rect r(-1+float(x)*size, -1+float(start)*size,
										 -1+float(x+1)*size, -1+float(configSpace[x].size()+1)*size);
						display.FillRect(r, Colors::red);
					}
				}
			}
			display.EndBackground();
			redrawBackground = false;
		}
		
		// Draw cursors, etc
		{
			float size = 4.0f/512.0f;

			{
				double x, y;
				if (validSearch)
				{
					x = 2*astar.CheckNextNode().GetAngle(0)/1024.0-1;
					y = 2*astar.CheckNextNode().GetAngle(1)/1024.0-1;
					c = Colors::lightgreen;
				}
				else {
					if (ourPath.size() == 0)
					{
						x = 2*config.GetAngle(0)/1024.0-1;
						y = 2*config.GetAngle(1)/1024.0-1;
						c = Colors::white;
					}
					else {
						x = 2*ourPath[(pathLoc)%ourPath.size()].GetAngle(0)/1024.0-1;
						y = 2*ourPath[(pathLoc)%ourPath.size()].GetAngle(1)/1024.0-1;
						c = Colors::green;
					}
				}
				Graphics::rect rec(point3d(x, y), 1.5f*size);
				display.FillRect(rec, c);
				
			}
			// labeling axes (needs to be updated)
			display.DrawText("Segment 2 angle", {-1, -1}, Colors::yellow, 0.05f, Graphics::textAlignLeft, Graphics::textBaselineTop);
			display.DrawText("Segment 1 angle", {1, 1}, Colors::yellow, 0.05f, Graphics::textAlignRight, Graphics::textBaselineBottom);
			display.DrawText("Config space ", {1, -1}, Colors::lightblue, 0.05f, Graphics::textAlignRight, Graphics::textBaselineTop);
		}
	}
//	if (recording && viewport == GetNumPorts((int)id)-1)
//	{
//		static int index = 0;
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/robot-%d%d%d%d", index/1000, (index/100)%10, (index/10)%10, index%10);
//		
//		//SaveScreenshot((int)id, fname);
//		printf("%s\n", fname);
//		index++;
//	}

}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	CreateSimulation(0);
	TestArms2(maxNumArgs==1);
	//Build4ArmDH();
	//BuildTipTables();
	exit(0);
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t':
//			if (mod != kShiftDown)
//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//			else
//			{
//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//			}
//			break;
		case 'p': //unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
//			if (unitSims[windowID]->GetPaused())
//			{
//				unitSims[windowID]->SetPaused(false);
//				unitSims[windowID]->StepTime(1.0/30.0);
//				unitSims[windowID]->SetPaused(true);
//			}
			break;
		case ']': stepsPerFrame *= 2; break;//absType = (absType+1)%3; break;
		case '[': if (stepsPerFrame > 50) stepsPerFrame /= 2;
			break;
		default:
			break;
	}
}

armAngles GetRandomState()
{
	armAngles s, g;
	s.SetNumArms(2);
	s.SetAngle(0, random()%1024);
	s.SetAngle(1, random()%1024);
	return s;
}

armAngles GetRandomLegalState()
{
	armAngles s;
	s.SetNumArms(2);
	do {
		s.SetAngle(0, random()%1024);
		s.SetAngle(1, random()%1024);
	} while (!r->LegalState(s));
	return s;
}

void TestJPS()
{
	std::vector<armAngles> states;
	srandom(1234);
	const int totalStates = 10000;
	// 1. Generate 10000 states
	for (int x = 0; x < totalStates; x++)
	{
		states.push_back(GetRandomState());
	}
	Timer t;
	// 2. Check validity of each
	int numLegal = 0;
	t.StartTimer();
	for (int x = 0; x < totalStates; x++)
		numLegal += r->LegalState(states[x])?1:0;
	t.EndTimer();
	printf("%f elapsed; %d of %d legal\n", t.GetElapsedTime(), numLegal, totalStates);

	// 3. Add each to queue
	AStarOpenClosed<armAngles, AStarCompare<armAngles> > openClosedList;
	int numAdded = 0;
	t.StartTimer();
	for (int x = 0; x < totalStates; x++)
	{
		uint64_t objid;
		if (openClosedList.Lookup(r->GetStateHash(states[x]), objid) != kOpenList)
		{
			numAdded++;
			openClosedList.AddOpenNode(states[x], r->GetStateHash(states[x]), 0.0, 0.0);
		}
	}
	t.EndTimer();
	printf("%f elapsed added %d of %d to open\n", t.GetElapsedTime(), numAdded, totalStates);
	
//	t.StartTimer();
//	astar.GetPath(r, s, g, ourPath);
//	t.EndTimer();
//	printf("Astar %f %llu %llu %1.2f\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), r->GetPathLength(ourPath));
	
}

void MyKeyHandler(unsigned long wid, tKeyboardModifier mod, char key)
{
	if ((key >= '0') && (key <= '1'))
	{
		activeArm = key-'0';
		return;
	}
	if (key == 'c')
	{
		SetNumPorts((int)wid, 2);
		BuildConfigSpace();
	}
	if (key == '|')
	{
		r->PopObstacle();
		//		SetNumPorts((int)wid, 2);
		BuildConfigSpace();
	}
	if (key == '&')
	{
		int index = 0;
		char fname[255];
		while (1)
		{
			sprintf(fname, "/Users/nathanst/Pictures/SVG/robot-%d%d%d%d.svg", index/1000, (index/100)%10, (index/10)%10, index%10);
			
			if (!FileExists(fname))
				break;
			index++;
		}
		MakeSVG(GetContext(wid)->display, fname, 1024, 512, -1);
		printf("Saved: '%s'\n", fname);
	}
	if (key == 'r')
	{
		for (int x = 0; x < numArms; x++)
			config.SetAngle( x, 2 );
		ourPath.clear();
		//recording = !recording;
	}
	if (key == 'a' && mode == kManualMoveArm)
	{
		std::vector<armRotations> actions;
		
		armRotations rot;
		rot.SetRotation(activeArm, kRotateCW);
		r->GetActions(config, actions);
		for (unsigned int x = 0; x < actions.size(); x++)
		{
			if (rot == actions[x])
			{
				r->ApplyAction(config, rot);
				ourPath.resize(0);
			}
		}
		//		for (int x = 0; x < numArms; x++)
		//			printf(" %d:%d", x, config.GetAngle(x));
		//		printf("\n");
	}
	if (key == 's' && mode == kManualMoveArm)
	{
		std::vector<armRotations> actions;
		
		armRotations rot;
		rot.SetRotation(activeArm, kRotateCCW);
		r->GetActions(config, actions);
		for (unsigned int x = 0; x < actions.size(); x++)
		{
			if (rot == actions[x])
			{
				r->ApplyAction(config, rot);
				ourPath.resize(0);
			}
		}
		//		for (int x = 0; x < numArms; x++)
		//			printf(" %d:%d", x, config.GetAngle(x));
		//		printf("\n");
	}
	
	if (key == 't')
	{
		mode = kSetTarget;
		ourPath.clear();
	}
	if (key == 'o')
	{
		mode = kAddObstacle;
		ourPath.clear();
	}
	if (key == 'm')
	{
		mode = kManualMoveArm;
		ourPath.clear();
	}
	if (key == 't')
	{
		//		BuildTipTables();
		//		//TestArms();
		//		TestArms2(true);
		//		//TestArms2();
	}
	
	if (key == 'b')
	{
		if (aa == 0)
		{
			aa = new ArmToArmHeuristic(r, config);
			r->AddHeuristic(aa);
		}
		else
			aa->AddDiffTable();
	}
	if (key == 'j')
	{
		TestJPS();
	}
	if (key == 'd')
	{
		astar.SetWeight(1.0);
	}
	if (key == 'w')
	{
		astar.SetWeight(1.5);
	}
	if (key == 'x')
	{
		float proveBound = 1.5;
		astar.SetPhi([=](double x,double y){return (y+(2*proveBound-1)*x+sqrt((y-x)*(y-x)+4*proveBound*y*x))/(2*proveBound);});
	}
	if (key == 'z')
	{
		float proveBound = 1.5;
		astar.SetPhi([=](double h,double g){return (h>g)?(g+h):(g/proveBound+h*(2*proveBound-1)/proveBound);});
	}
}


bool drag = false;
Graphics::point s, e;

bool MyClickHandler(unsigned long , int viewport, int x, int y, point3d loc, tButtonType whichButton, tMouseEventType mouseEvent)
{
	if (viewport != 0)
	{
		return false;
	}
	if ((mouseEvent == kMouseDown) && mode == kAddObstacle)//(whichButton == kLeftButton))
	{
		validSearch = false;
		s.x = loc.x;
		s.y = loc.y;
		s.z = 0;
		e = s;
		drag = true;
		line2d l(s, e);
		r->AddObstacle(l);
		BuildConfigSpace(true);
	}
	if ((mouseEvent == kMouseDrag) && (mode == kAddObstacle) && drag)
	{
		r->PopObstacle();
		e.x = loc.x;
		e.y = loc.y;
		e.z = 0;
		line2d l(s, e);
		r->AddObstacle(l);
		BuildConfigSpace(true);
	}
	if ((mouseEvent == kMouseUp) && (mode == kAddObstacle) && drag)
	{
		r->PopObstacle();
		e.x = loc.x;
		e.y = loc.y;
		e.z = 0;
		line2d l(s, e);
		if (!(fequal(s.x, e.x) && fequal(s.y, e.y)))
		{
			printf("Adding obstacle (%1.2f, %1.2f) (%1.2f, %1.2f)\n", s.x, s.y, e.x, e.y);
			r->AddObstacle(l);
		}
		BuildConfigSpace();
	}
	if ((mouseEvent == kMouseDown) && (mode == kSetTarget))
	{
		goal.SetGoal(loc.x, loc.y);
		if (aa)
		{
			const std::vector<armAngles> &pos = aa->GetTipPositions(loc.x, loc.y);
			if (pos.size() > 0)
			{
				goal = pos[0];
				validSearch = astar.InitializeSearch(r, goal, config, ourPath);
				for (unsigned int t = 1; t < pos.size(); t++)
				{
					goal = pos[t];
					astar.AddAdditionalStartState(goal);
				}
			}
		}
		else {
			validSearch = astar.InitializeSearch(r, config, goal, ourPath);
		}
		
		if (validSearch)
		{
			std::cout << "Starting search between: " << config << " and " << goal << std::endl;
			totalTime = 0;
		}
	}
//	static point3d oldloc(0, 0, 0);
//
//	if (oldloc.x != 0)
//	{
//		r->AddObstacle(line2d(recVec(oldloc.x, oldloc.y, 0), recVec(loc.x, loc.y, 0)));
//		oldloc = loc;
//		return true;
//	}
//	oldloc = loc;
//	return false;

	return true;
}

void BuildConfigSpace(bool fast)
{
	int resolution = 512;
	if (fast)
	{
		resolution = 128;
	}
	int multiplier = 1024/resolution;

	configSpace.resize(0);
	configSpace.resize(resolution);
	for (int x = 0; x < resolution; x++)
		configSpace[x].resize(resolution);

	armAngles a;
	a.SetNumArms(2);
	for (int x = 0; x < 1024; x+=multiplier)
	{
		a.SetAngle(0, x);
		for (int y = 0; y < 1024; y+=multiplier)
		{
			a.SetAngle(1, y);
			configSpace[x/multiplier][y/multiplier] = r->LegalState(a);
		}
	}
	redrawBackground = true;
}


#if 0
void AddToMinHeap( std::vector<armAngles> &heap, armAngles &arm,
		   float *distances )
{
	unsigned i;

	// need to increase size, although we don't actually care
	// about the new content yet
	heap.push_back( arm );
	for ( i = heap.size(); i > 1; i >>= 1 ) {
		if ( ArmDistance( arm, distances )
		    >= ArmDistance( heap[ ( i >> 1 ) - 1 ], distances ) ) {
			break;
		}
		heap[ i - 1 ] = heap[ ( i >> 1 ) - 1 ];
	}
	heap[ i - 1 ] = arm;
}

armAngles GetFromMinHeap( std::vector<armAngles> &heap, float *distances )
{
	unsigned c; // child index
	armAngles ret;

	ret = heap[ 1 - 1 ];

	// item heap.size() starts as the root (1) but it
	// must be pushed down if it is too large
	c = 1 << 1;
	while( c <= heap.size() - 1 ) {
		if ( c + 1 <= heap.size() - 1
		    && ArmDistance( heap[ c - 1 ], distances )
		    > ArmDistance( heap[ c + 1 - 1 ], distances ) ) {
			// child c is bigger than child c+1, so use c+1
			++c;
		}
		if ( ArmDistance( heap[ heap.size() - 1 ], distances )
		    <= ArmDistance( heap[ c - 1 ], distances ) ) {
			// new root <= than child, so it stops moving down
			break;
		}

		// root is pushed down past child, so child
		// item is now the parent
		heap[ ( c >> 1 ) - 1 ] = heap[ c - 1 ];

		// root has a new potential location, get new child
		c <<= 1;
	}

	// we found the child of our final location, put item
	// in its place
	heap[ ( c >> 1 ) - 1 ] = heap[ heap.size() - 1 ];
	heap.pop_back();

	return ret;
}
#endif

void TestArms()
{
	assert(aa == 0);
	aa = new ArmToArmHeuristic(r, config, true);
	r->AddHeuristic(aa);
	
	std::vector<armAngles> starts;
	std::vector<armAngles> goals;
	
	// first, get lots of good problems
	printf("Generating problems\n");
	while (starts.size() < 500)
	{
		if ((starts.size()%500) == 0)
			printf("Generating problem %d\n", (int)starts.size());
		double x, y;
		x = random()%10000;
		x = 2*x/10000-1;
		y = random()%10000;
		y = 2*y/10000-1;
		//printf("Trying config from (%f, %f) for start\n", x, y);
		const std::vector<armAngles> &pos = aa->GetTipPositions(x, y);
		if (pos.size() > 0)
		{
			starts.push_back(pos[random()%pos.size()]);
		}		
		else
			continue;

		while (1)
		{
			x = random()%10000;
			x = 2*x/10000-1;
			y = random()%10000;
			y = 2*y/10000-1;
			//printf("Trying (%f, %f) for goal\n", x, y);
			const std::vector<armAngles> &pos2 = aa->GetTipPositions(x, y);
			if (pos2.size() > 0)
			{
				armAngles theGoal;
				theGoal.SetGoal(x, y);
				goals.push_back(theGoal);
				break;
			}
		}
	}
	printf("Done generating problems\n");
	
//	if (aa == 0)
//	{
//		aa = new ArmToArmHeuristic(r, config);
//		r->AddHeuristic(aa);
//	}
//	else
//		aa->AddDiffTable();

	double fTotalTime;
	double totalNodes;
	double totalHvalue;
	
//	aa->AddDiffTable();
//	aa->AddDiffTable();
//	aa->AddDiffTable();
	for (int total = 0; total <= -1; total++)
	{
		printf("Solving with %d heuristics\n", total);
		fTotalTime = 0;
		totalNodes = 0;
		totalHvalue = 0;
		for (unsigned int x = 0; x < starts.size(); x++)
		{
			armAngles theGoal;
			float x1, y1;
			goals[x].GetGoal(x1, y1);
			const std::vector<armAngles> &pos = aa->GetTipPositions(x1, y1);
			theGoal = pos[0];
			double localHvalue = r->HCost(starts[x], theGoal);
			validSearch = astar.InitializeSearch(r, theGoal, starts[x], ourPath);
			for (unsigned int t = 1; t < pos.size(); t++)
			{
				theGoal = pos[t];
				astar.AddAdditionalStartState(theGoal);
				localHvalue = min(localHvalue, r->HCost(starts[x], theGoal));
			}
			
			Timer t;
			t.StartTimer();
//			while (!astar.DoSingleSearchStep(ourPath))
//			{}
			totalHvalue += localHvalue;
			fTotalTime += t.EndTimer();
			totalNodes += astar.GetNodesExpanded();
			printf("%d\t%lld\t%f\t%f\n", x, astar.GetNodesExpanded(), t.GetElapsedTime(), localHvalue);
		}
		fTotalTime /= starts.size();
		totalNodes /= starts.size();
		totalHvalue /= starts.size();
		printf("time\t%f\tnodes\t%f\thcost\t%f\n", fTotalTime, totalNodes, totalHvalue);
		aa->AddDiffTable();
	}

	printf("Solving with no heuristics the normal way\n");
	fTotalTime = 0;
	totalNodes = 0;
	for (unsigned int x = 0; x < starts.size(); x++)
	{
		validSearch = astar.InitializeSearch(r, starts[x], goals[x], ourPath);
		
		Timer t;
		t.StartTimer();
		while (!astar.DoSingleSearchStep(ourPath))
		{}
		fTotalTime += t.EndTimer();
		totalHvalue += r->HCost(starts[x], goals[x]);
		totalNodes += astar.GetNodesExpanded();
		printf("%d\t%lld\t%f\t%f\n", x, astar.GetNodesExpanded(), t.GetElapsedTime(), r->HCost(starts[x], goals[x]));
	}
	fTotalTime /= starts.size();
	totalNodes /= starts.size();
	totalHvalue /= starts.size();
	printf("time\t%f\tnodes\t%f\n", fTotalTime, totalNodes);
	exit(0);
}

void TestArms2(bool h)
{
	if (h) printf("using heuristic\n");
	std::vector<armAngles> starts;
	std::vector<armAngles> goals;
	std::vector<armAngles> succ;
	srandom(101);
	for (int x = 0; x < 100; x++)
	{
		std::cout << "Building start " << x << std::endl;
		armAngles a;
		a.SetNumArms(numArms);
		//for (int y = 0; y < numArms; y++)
		//	a.SetAngle( y, 512 );
		//32, 1022, 2, 1022
		if (x%1)
		{
			a.SetAngle(0, 32);
			a.SetAngle(1, 1022);
			a.SetAngle(2, 2);
			a.SetAngle(3, 1022);
		}
		else {
			//988, 328, 1022, 444
			a.SetAngle(0, 988);
			a.SetAngle(1, 328);
			a.SetAngle(2, 1022);
			a.SetAngle(3, 444);
		}
		for (int y = 0; y < 35000; y++)
		{
			r->GetSuccessors(a, succ);
			a = succ[random()%succ.size()];
		}
		starts.push_back(a);
	}
	for (int x = 0; x < 100; x++)
	{
		std::cout << "Building goal " << x << std::endl;
		armAngles a;
		a.SetNumArms(numArms);
		for (int y = 0; y < numArms; y++)
			a.SetAngle( y, 512 );
		if (numArms == 3)
		{
			for (int y = 0; y < 35000; y++)
			{
				if (x%2)
				{
					a.SetAngle(0, 694);
					a.SetAngle(1, 1022);
					a.SetAngle(2, 154);
				}
				else {
					a.SetAngle(0, 330);
				a.SetAngle(1, 2);
				a.SetAngle(2, 870);
				}
			}
		}
		else {
			for (int y = 0; y < numArms; y++)
				a.SetAngle( y, 512 );
		}
		for (int y = 0; y < 35000; y++)
		{
			r->GetSuccessors(a, succ);
			a = succ[random()%succ.size()];
		}
		goals.push_back(a);
	}
	if (h)
	{
		std::cout << "Loading heuristics" << std::endl;
		ArmToArmCompressedHeuristic *a1 = new ArmToArmCompressedHeuristic(r, "4-arm_far3.diff");
		//		ArmToArmCompressedHeuristic *a2 = new ArmToArmCompressedHeuristic(r, "4-arm_far2.diff");
		r->AddHeuristic(a1);
		//	r->AddHeuristic(a2);
	}
	std::vector<int> use;
	use.push_back(1);
	use.push_back(2);
	use.push_back(3);

	use.push_back(5);
	use.push_back(6);
	use.push_back(7);
	use.push_back(8);
	use.push_back(9);
	use.push_back(10);
	use.push_back(11);


	use.push_back(14);
	use.push_back(15);
	use.push_back(16);
	use.push_back(17);
	use.push_back(18);
	use.push_back(20);
	use.push_back(21);
	use.push_back(22);
	use.push_back(23);
	use.push_back(24);
	use.push_back(25);
	use.push_back(27);
	use.push_back(28);
	use.push_back(29);
	use.push_back(30);
	use.push_back(31);
	use.push_back(32);
	use.push_back(33);
	use.push_back(34);
	use.push_back(36);
	use.push_back(37);
	use.push_back(38);
	use.push_back(39);
	use.push_back(41);
	use.push_back(42);
	use.push_back(43);
	use.push_back(44);
	use.push_back(46);
	use.push_back(48);
	use.push_back(53);
	use.push_back(54);
	use.push_back(55);
	use.push_back(56);
	use.push_back(57);
	use.push_back(58);
	use.push_back(59);
	use.push_back(61);
	use.push_back(62);
	use.push_back(63);
	use.push_back(64);
	use.push_back(65);
	use.push_back(66);
	use.push_back(67);
	use.push_back(70);
	use.push_back(71);
	use.push_back(72);
	use.push_back(73);
	use.push_back(75);
	use.push_back(76);
	use.push_back(77);
	use.push_back(78);
	use.push_back(79);
	use.push_back(81);
	use.push_back(82);
	use.push_back(83);
	use.push_back(86);
	use.push_back(87);
	use.push_back(88);
	use.push_back(89);
	use.push_back(90);
	use.push_back(91);
	use.push_back(92);
	use.push_back(93);
	use.push_back(94);
	use.push_back(95);
	use.push_back(96);
	use.push_back(97);
	use.push_back(99);


	astar.SetUseBPMX(1);
	printf("%d starts; %d goals\n", (int)starts.size(), (int)goals.size());
	for (unsigned int t = 0; t < use.size(); t+=1)
	{
		int x = use[t];
		config = starts[x];
		goal = goals[x];
		std::cout << "Searching " << starts[x] << " to " << goals[x] << std::endl;
		astar.InitializeSearch(r, starts[x], goals[x], ourPath);
		
		Timer tmr;
		tmr.StartTimer();
		int cnt = 1;
		while (!astar.DoSingleSearchStep(ourPath))

		{ if (((++cnt%100) == 0) && (tmr.EndTimer() > 300)) break; }

		tmr.EndTimer();
//		totalHvalue += r->HCost(starts[x], goals[x]);
//		totalNodes += astar.GetNodesExpanded();
		printf("%d\t%lld\t%lld\t%f\t%f\t%1.0f\n", x, astar.GetNodesExpanded(), astar.GetUniqueNodesExpanded(),
			   tmr.GetElapsedTime(), r->HCost(starts[x], goals[x]), r->GetPathLength(ourPath));
	}
	
}

void Build4ArmDH()
{
	//	return;

	std::vector<int> reduction, offset1;
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);
	//offset2.push_back(0);offset2.push_back(0);offset2.push_back(1);
	ArmToArmCompressedHeuristic *aah = new ArmToArmCompressedHeuristic(r, reduction, offset1);
	
	FrontierBFS<armAngles, armRotations> fbfs;
	printf("Performing frontier BFS!\n");
	std::cout << "Starting from " << config << std::endl;
//	std::vector<std::vector<armAngles> > cache;
	
	armAngles tmp = config;
	//988, 328, 1022, 444
	//32, 1022, 2, 1022
	tmp.SetAngle(0, 32); 
	tmp.SetAngle(1, 1022);
	tmp.SetAngle(2, 2);
	tmp.SetAngle(3, 1022);
	std::cout << "Adding heuristic from: " << tmp << std::endl;
	aah->BuildHeuristic(tmp);
	aah->Save("4-arm_far3.diff");
}

void WriteCache(int index, std::vector<armAngles> &values);
void BuildDHTables(std::vector<int> reduction, const char *baseName);

void BuildTipTables()
{
	if (r == 0)
		CreateSimulation(0);
	std::vector<int> reduction;

	reduction.clear();
	reduction.push_back(1);reduction.push_back(1);reduction.push_back(1);
	BuildDHTables(reduction, "3-arm_1V");

	reduction.clear();
	reduction.push_back(1);reduction.push_back(1);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_2V");
	
	reduction.clear();
	reduction.push_back(1);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_4V");

	reduction.clear();
	reduction.push_back(2);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_8V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_16V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_32V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(3);
	BuildDHTables(reduction, "3-arm_64V");
}

void BuildDHTables(std::vector<int> reduction, const char *baseName)
{
	std::vector<int> offset1, offset2, offset3, offset4;
	//reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);
	offset2.push_back(0);offset2.push_back(0);offset2.push_back(1);
	offset3.push_back(0);offset3.push_back(1);offset3.push_back(0);
	offset4.push_back(0);offset4.push_back(1);offset4.push_back(1);

	ArmToArmCompressedHeuristic *aah = new ArmToArmCompressedHeuristic(r, reduction, offset1);
	ArmToArmCompressedHeuristic *aah1 = new ArmToArmCompressedHeuristic(r, reduction, offset2);
	ArmToArmCompressedHeuristic *aah2 = new ArmToArmCompressedHeuristic(r, reduction, offset3);
	ArmToArmCompressedHeuristic *aah3 = new ArmToArmCompressedHeuristic(r, reduction, offset4);
	Timer t;
	t.StartTimer();
	
	FrontierBFS<armAngles, armRotations> fbfs;

	std::vector<armAngles> init, far;
	armAngles tmp = config;
	init.push_back(tmp);
	std::cout << "Building heuristic 1 from: " << std::endl;
	far.push_back(aah->BuildHeuristic(init)); // initial finding of far state; far[0] = far1

	std::cout << "Building heuristic 2: " << std::endl;
	far.push_back(aah1->BuildHeuristic(far)); // first real heuristic; far[1] = far2

	std::cout << "Building heuristic 3: " << std::endl;
	far.push_back(aah2->BuildHeuristic(far)); // first real heuristic; far[1] = far2
	
	std::cout << "Building heuristic 4: " << std::endl;
	far.push_back(aah3->BuildHeuristic(far)); // first real heuristic; far[1] = far2

	char name[255];
	sprintf(name, "%s_a.diff", baseName);
	aah->Save(name);
	sprintf(name, "%s_b.diff", baseName);
	aah1->Save(name);
	sprintf(name, "%s_c.diff", baseName);
	aah2->Save(name);
	sprintf(name, "%s_d.diff", baseName);
	aah3->Save(name);
	delete aah;
	delete aah1;
	delete aah2;
	delete aah3;
}

void WriteCache(int index, std::vector<armAngles> &values)
{
	return;
	char filename[255];
	sprintf(filename, "%d.tipIndex", index);
	FILE *f = fopen(filename, "a+");
	if (!f) assert(!"Couldn't open file!");
	for (unsigned int x = 0; x < values.size(); x++)
	{
		for (int y = 0; y < values[x].GetNumArms(); y++)
			fprintf(f, "%d ", values[x].GetAngle(y));
		fprintf(f, "\n");
	}
	fclose(f);
	values.resize(0);
}
