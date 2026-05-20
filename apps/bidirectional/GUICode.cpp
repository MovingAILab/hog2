//
//  GUICode.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include <vector>
#include <fstream>
#include <numeric>
#include "GUICode.h"
#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"
#include "MM.h"
#include "SVGUtil.h"
#include "NBSQueueGF.h"
#include "NBS.h"
#include "BSStar.h"
#include "FFBDS.h"
#include "DVCBS.h"
#include "WeightedVertexGraph.h"
#include "Timer.h"

Map *map = 0;
MapEnvironment *me = 0;
MapOverlay *mo;
xyLoc start, goal;

std::vector<int> counts;

bool mouseTracking = false;
bool mouseTracked = false;
bool drawSearch = true;
bool paused = false;
void SetupMapOverlay();

int gStepsPerFrame = 2;

TemplateAStar<xyLoc, tDirection, MapEnvironment> forward;
TemplateAStar<xyLoc, tDirection, MapEnvironment> backward;
FFBDS<xyLoc, tDirection, MapEnvironment> ff;

ZeroHeuristic<xyLoc> *z = new ZeroHeuristic<xyLoc>;
WeightedHeuristic<xyLoc> *w = 0;


MM<xyLoc, tDirection, MapEnvironment> mm;
NBS<xyLoc, tDirection, MapEnvironment> nbs;
TemplateAStar<xyLoc, tDirection, MapEnvironment> compare;
MM<xyLoc, tDirection, MapEnvironment> mm0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> compare0;
BSStar<xyLoc, tDirection, MapEnvironment> bs;
DVCBS<xyLoc, tDirection, MapEnvironment> dvcbs;

bool nbsSearchRunning = false;
bool compareSearchRunning = false;
bool mm0SearchRunning = false;
bool bsSearchRunning = false;
bool ffSearchRunning = false;
bool compare0SearchRunning = false;
bool dvcbsSearchRunning = false;
bool searchRan = false;
std::vector<xyLoc> path;
std::vector<xyLoc> goalPath;
bool recording = false;

std::fstream svgFile;
bool saveSVG = false;

enum bibfs {
	XX = 0,
	NN = 1,
	NF = 2,
	NR = 3,
	FN = 4,
	FF = 5,
	FR = 6,
	RN = 7,
	RF = 8,
	RR = 9
};

const char *bibfs_desc[10] = {
	"", "NN", "NF", "NR", "FN", "FF", "FR", "RN", "RF", "RR"
};



void InstallHandlers()
{
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler);
	InstallKeyboardHandler(MyKeyboardHandler, "Save SVG", "Export graphics to SVG File", kNoModifier, 's');
	InstallKeyboardHandler(MyKeyboardHandler, "Record", "Start/stop recording movie", kNoModifier, 'r');
	InstallKeyboardHandler(MyKeyboardHandler, "Draw", "Toggle drawing search", kNoModifier, 'd');
	InstallKeyboardHandler(MyKeyboardHandler, "Pause", "Toggle pause", kNoModifier, 'p');
	InstallKeyboardHandler(MyKeyboardHandler, "Go", "Go solve 100 random problems", kNoModifier, 'g');
	InstallKeyboardHandler(MyKeyboardHandler, "Step", "Single algorithm step", kNoModifier, 'o');
	InstallKeyboardHandler(MyKeyboardHandler, "Single Viewport", "Set to use a single viewport", kNoModifier, '1');
	InstallKeyboardHandler(MyKeyboardHandler, "Two Viewports", "Set to use two viewports", kNoModifier, '2');
	InstallKeyboardHandler(MyKeyboardHandler, "Three Viewports", "Set to use three viewports", kNoModifier, '3');
	InstallKeyboardHandler(MyKeyboardHandler, "Four Viewports", "Set to use four viewports", kNoModifier, '4');
	InstallKeyboardHandler(MyKeyboardHandler, "Slower", "Slow down visualization", kNoModifier, '[');
	InstallKeyboardHandler(MyKeyboardHandler, "Faster", "Speed up visualization", kNoModifier, ']');
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		mouseTracking = false;
		delete map;
		delete me;
		map = 0;
		me = 0;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		delete map;
		delete me;
		//		map = new Map("/Users/nathanst/hog2/maps/dao/brc000d.map");
//		map = new Map("/Users/nathanst/hog2/maps/dao/brc000d.map");
//		map = new Map("/Users/nathanst/hog2/maps/dao/brc203d.map");
//		map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/w_woundedcoast.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-1-0.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		if (0)
		{
			map = new Map(128, 128);
//			MakeMaze(map, 0.1f, 1.0f);
			MakeMaze(map, 0.95f, 1.0f);
//			MakeMaze(map, 0.25f, 1.0f);

//			MakeMaze(map, 1);
		}
		
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		me->SetDiagonalCost(1.5);
		w = new WeightedHeuristic<xyLoc>(me, 1.0);
	}
	
}

void StepAlgorithms()
{
	for (int x = 0; x < gStepsPerFrame/2; x++)
	{
		if (nbsSearchRunning)
		{
			nbsSearchRunning = !nbs.DoSingleSearchStep(path);
			if (!nbsSearchRunning)
				printf("NBS: %llu nodes expanded cost %1.1f\n", nbs.GetNodesExpanded(), me->GetPathLength(path));
		}
	}
	for (int x = 0; x < gStepsPerFrame/2; x++)
	{
		if (ffSearchRunning)
		{
			ffSearchRunning = !ff.DoSingleSearchStep(path);
			if (!ffSearchRunning)
				printf("FFBDS*: %llu nodes expanded\n", ff.GetNodesExpanded());
		}
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (mm0SearchRunning)
		{
			mm0SearchRunning = !mm0.DoSingleSearchStep(path);
			if (!mm0SearchRunning)
				printf("MM0: %llu nodes expanded\n", mm0.GetNodesExpanded());
		}
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (bsSearchRunning)
		{
			bsSearchRunning = !bs.DoSingleSearchStep(path);
			if (!bsSearchRunning)
				printf("BS*: %llu nodes expanded cost %1.1f\n", bs.GetNodesExpanded(), me->GetPathLength(path));
		}
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (compareSearchRunning)
		{
			compareSearchRunning = !compare.DoSingleSearchStep(path);
			if (!compareSearchRunning)
			{
				printf("A*: %llu nodes expanded cost %1.1f\n", compare.GetNodesExpanded(), me->GetPathLength(path));
			}
		}
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (compare0SearchRunning)
		{
			compare0SearchRunning = !compare0.DoSingleSearchStep(path);
			if (!compare0SearchRunning)
			{
				printf("BFS: %llu nodes expanded cost %1.1f\n", compare0.GetNodesExpanded(), me->GetPathLength(path));
			}
		}
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (dvcbsSearchRunning)
		{
			dvcbsSearchRunning = !dvcbs.DoSingleSearchStep(path);
			if (!dvcbsSearchRunning)
			{
				printf("BFS: %llu nodes expanded cost %1.1f\n", dvcbs.GetNodesExpanded(), me->GetPathLength(path));
			}
		}
	}

}

void MyKeyboardHandler(unsigned long windowID, tKeyboardModifier, char key)
{
	switch (key)
	{
		case 'g':
		{
			for (float p = 1; p <= 16.0; p *= 4)
			{
				Map *m = new Map(512, 512);
				MapEnvironment *env = new MapEnvironment(m);
				MakeMaze(m, (int)p);
				//				MakeMaze(m, p, 1.0f);

				const int total = 500;
				int better = 0;
				for (int x = 0; x < total; x++)
				{
//					MakeMaze(m, p, 1.0f);
					xyLoc a, b;
					while (true)
					{
						a.x = random()%m->GetMapWidth();
						a.y = random()%m->GetMapHeight();
						if (m->GetTerrainType(a.x, a.y) == kGround)
							break;
					}
					while (true)
					{
						b.x = random()%m->GetMapWidth();
						b.y = random()%m->GetMapHeight();
						if (m->GetTerrainType(b.x, b.y) == kGround)
							break;
					}
					BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment> bpa(a, b, env, env, env);
					if ((bpa.GetMinWork() != bpa.GetForwardWork()) && (bpa.GetMinWork() != bpa.GetBackwardWork()))
						better++;
				}
				printf("%1.2f : %d of %d\n", p, better, total);
				delete m;
				delete env;
			}
		}
		case 's':
		{
			svgFile.open("/Users/nathanst/Desktop/test.svg", std::fstream::out | std::fstream::trunc);
			saveSVG = true;
			break;
		}
		case 'r':
		{
			recording = !recording;
			break;
		}
		case 'p':
		{
			paused = !paused;
			break;
		}
		case 'o':
		{
			StepAlgorithms();
			break;
		}
		case 'd':
		{
			drawSearch = !drawSearch;
			break;
		}
		case '1':
		{
			SetNumPorts(windowID, 1);
			break;
		}
		case '2':
		{
			SetNumPorts(windowID, 2);
			break;
		}
		case '3':
		{
			SetNumPorts(windowID, 3);
			break;
		}
		case '4':
		{
			SetNumPorts(windowID, 4);
			break;
		}
		case '[':
		{
			gStepsPerFrame /= 2;
			if (gStepsPerFrame < 2)
				gStepsPerFrame = 2;
			break;
		}
		case ']':
		{
			gStepsPerFrame *= 2;
			break;
		}
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (saveSVG && viewport == 0)
	{
		svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*map->GetMapWidth()+10)+"\" height = \""+std::to_string(10*map->GetMapHeight()+10+100)+"\">";
		rgbColor black = {0, 0, 0};
		rgbColor white = {1, 1, 1};
		//svgFile << SVGDrawRect(0, 0, map->GetMapWidth(), map->GetMapHeight()+10+1, white);
		if (mo)
		{
			svgFile << mo->SVGDraw();

			for (int x = 1; x < 10; x++)
			{
				rgbColor r = mo->GetValueColor(x);
				//glColor3f(r.r, r.g, r.b);
				char num[16];
				sprintf(num, "%d", counts[x]);
				
				svgFile << SVGDrawRect(x*map->GetMapWidth()/11+0, map->GetMapHeight()+2, 5, 5, r);
				svgFile << SVGDrawText(x*map->GetMapWidth()/11+6, map->GetMapHeight()+4, bibfs_desc[x], black, 3);
				svgFile << SVGDrawText(x*map->GetMapWidth()/11+6, map->GetMapHeight()+7, num, black, 3);
			}

			svgFile << SVGDrawText(start.x+1, start.y+2, "start", black, 5);
			svgFile << SVGDrawText(goal.x+1, goal.y+2, "goal", black, 5);
		}
		else {
			svgFile << me->SVGDraw();
		}
		
		svgFile << "</svg>";


		saveSVG = false;
		svgFile.close();
	}

	//map->OpenGLDraw();
	if (mo)
	{
	  //mo->OpenGLDraw();
		for (int x = 1; x < 10; x++)
		{
		  /*
			rgbColor r = mo->GetValueColor(x);
			glColor3f(r.r, r.g, r.b);
			DrawBox(-1+0.2*x-1.0/40.0, -1-1.0/40.0, 0, 1.0/40.0);
			glColor3f(1.0, 1.0, 1.0);
			DrawText(-1+0.2*x+1.0/40.0, -1-1.0/40.0, -0.01, 1.0/10.0, bibfs_desc[x]);
			char num[16];
			sprintf(num, "%d", counts[x]);
			DrawText(-1+0.2*x+1.0/40.0, -1+1.0/40.0, -0.01, 1.0/10.0, num);
		  */
		}
	}
	if (mouseTracking)
	{
	  /*
		me->SetColor(1.0, 0, 0);
		glLineWidth(3.0);
		me->GLDrawLine(start, goal);
		glLineWidth(1.0);
		me->SetColor(1.0, 1.0, 1.0);
		me->GLDrawLine(start, goal);
	  */
	}
	if (mouseTracked && 0)
	{
	  /*
		me->SetColor(1.0, 1.0, 1.0);
		glLineWidth(1.0);
		me->GLLabelState(start, "start", map->GetMapHeight()/8.0);
		me->GLLabelState(goal, "goal", map->GetMapHeight()/8.0);
		glLineWidth(1.0);
	  */
	}

	if (drawSearch)
	{
		if (!paused)
		{
			StepAlgorithms();
		}
		if (searchRan)
		{
		  /*
		  if (viewport == 0)
				nbs.OpenGLDraw();
			else if (viewport == 1)
				dvcbs.OpenGLDraw();
			else if (viewport == 2)
				compare.OpenGLDraw();//bs.OpenGLDraw();
//			else if (viewport == 3)
//				mm.OpenGLDraw();
*/
		}
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/BI-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		//SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
	if (goalPath.size() > 0)
	{
		goal = goalPath.back();
		goalPath.pop_back();
		SetupMapOverlay();
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	//	return false;
	static point3d startLoc;
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
			delete mo;
			mo = 0;
			
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			start.x = x; start.y = y;
			goal = start;
			mouseTracking = true;
			mouseTracked = true;
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			goal.x = x; goal.y = y;
			mouseTracking = true;
			mouseTracked = true;
			return true;
		}
		case kMouseUp:
			if (mouseTracking)
			{
				int x, y;
				map->GetPointFromCoordinate(loc, x, y);
				goal.x = x; goal.y = y;

				//100, 59) to (176, 112
//				start.x = 100;
//				start.y = 59;
//				goal.x = 176;
//				goal.y = 112;
//
//				//102, 170) to (82, 11 //  100, 192) to (106, 60
//101, 143) to (93, 155
//				start.x = 101;
//				start.y = 143;
//				goal.x = 93;
//				goal.y = 155;
				
//				start.x = 75;
//				start.y = 33;
//				goal.x = 93;//78;
//				goal.y = 116;//132-10;
				
//				forward.GetPath(me,
//								{static_cast<uint16_t>(goal.x-17),
//									static_cast<uint16_t>(goal.y+22)}, goal, goalPath);
//				recording = true;
				if ((0))
				{
					{
						mm.GetPath(me, start, goal, me, me, path);
						std::fstream svgFile;
						me->SetColor(Colors::darkgray.r, Colors::darkgray.g, Colors::darkgray.b);
						svgFile.open("/Users/nathanst/mm.svg", std::fstream::out | std::fstream::trunc);
						svgFile << me->SVGHeader();
						svgFile << me->SVGDraw();
						svgFile << mm.SVGDraw();
						svgFile << "</svg>";
						svgFile.close();
					}
					{
						forward.GetPath(me, start, goal, path);
						std::fstream svgFile;
						me->SetColor(Colors::darkgray.r, Colors::darkgray.g, Colors::darkgray.b);
						svgFile.open("/Users/nathanst/forward.svg", std::fstream::out | std::fstream::trunc);
						svgFile << me->SVGHeader();
						svgFile << me->SVGDraw();
						svgFile << forward.SVGDraw();
						svgFile << "</svg>";
						svgFile.close();
					}
					{
						backward.GetPath(me, goal, start, path);
						std::fstream svgFile;
						me->SetColor(Colors::darkgray.r, Colors::darkgray.g, Colors::darkgray.b);
						svgFile.open("/Users/nathanst/backward.svg", std::fstream::out | std::fstream::trunc);
						svgFile << me->SVGHeader();
						svgFile << me->SVGDraw();
						svgFile << backward.SVGDraw();
						svgFile << "</svg>";
						svgFile.close();
					}
				}

				BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(start, goal, me, me, me);
				
				mouseTracking = false;
				//SetupMapOverlay();
				SetNumPorts(windowID, 3);
				compare.SetHeuristic(w);
				compare.InitializeSearch(me, start, goal, path);

				compare0.SetHeuristic(z);
				compare0.InitializeSearch(me, start, goal, path);
				nbs.InitializeSearch(me, start, goal, me, me, path);
				bs.InitializeSearch(me, start, goal, me, me, path);
				//mm.InitializeSearch(me, start, goal, z, z, path);
				mm.InitializeSearch(me, start, goal, me, me, path);
				mm0.InitializeSearch(me, start, goal, z, z, path);
				ff.InitializeSearch(me, start, goal, me, path);
				dvcbs.InitializeSearch(me, start, goal, me, me, path);
				nbsSearchRunning = true;
				compareSearchRunning = true;
//				mm0SearchRunning = true;
//				bsSearchRunning = true;
//				ffSearchRunning = true;
//				compare0SearchRunning = true;
				dvcbsSearchRunning = true;
				
				searchRan = true;
				return true;
			}
	}
	return false;
}

bibfs GetLocationClassification(xyLoc l, double optimal)
{
	double startDist, goalDist;
	forward.GetClosedListGCost(l, startDist);
	backward.GetClosedListGCost(l, goalDist);

	// Only Show NEAR in each direction
//	if (startDist < optimal/2)
//		return NF;
//	if (goalDist < optimal/2)
//		return FN;
//	return RR;

	// Only show N*, F*
//	if (startDist < optimal/2)
//		return NR;
//	if (startDist < optimal)
//		return NF;
//	if (startDist == optimal/2)
//		return NN;
//	return RR;

	if (0) // Draw MM & A* heuristic regions that don't overlap
	{
		bool MM = false;
		bool ASTAR = false;
		if (startDist <= optimal && me->HCost(l, goal)+startDist <= optimal)
			ASTAR = true;
		if ((startDist <= optimal/2 && me->HCost(l, goal)+startDist <= optimal) ||
			(goalDist <= optimal/2 && me->HCost(l, start)+goalDist <= optimal))
			MM = true;
		if (ASTAR && MM)
			return XX;
//			return NF;
		if (MM)
			return RN;
		if (ASTAR)
			return FN;
		else return XX;
	}
	
	if (0) // Draw MM heuristic regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			if (me->HCost(l, goal)+startDist > optimal || me->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
			//return XX;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			return XX;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			return XX;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			if (me->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
			return XX;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return XX;
			return FF;
		}
		else if (startDist <= optimal)
		{
			return XX;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			if (me->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}
	
	if (0) // Draw A* heuristic regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			//return XX;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return FN;
			return XX;
			return FF;
		}
		else if (startDist <= optimal)
		{
			if (me->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return XX;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}
	
	if (0) // DRAW just BFS regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			return FN;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			return FN;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			return FN;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			return FN;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return FN;
			return FF;
		}
		else if (startDist <= optimal)
		{
			return FN;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return XX;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}

	if (1)
	{
		// DRAW all regions
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return FF;
		}
		else if (startDist <= optimal)
		{
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return RF;
		}
		else {
			return RR;
		}
	}
}

void SetupMapOverlay()
{
	if (start.x >= map->GetMapWidth() || start.x < 0 || start.y >= map->GetMapHeight() || start.y < 0)
	{
		std::cout << "Invalid path: " << start << " to " << goal << "\n";
		return;
	}
	std::cout << "Doing map overlay from " << start << " to " << goal << "\n";
	counts.resize(0);
	counts.resize(10);
	delete mo;
	mo = new MapOverlay(map);
	mo->SetColorMap(MapOverlay::customColorMap);
	
	mo->SetColor(XX, Colors::black);

	mo->SetColor(NN, Colors::cyan);
	mo->SetColor(NF, Colors::lightblue);
	mo->SetColor(NR, Colors::blue);
	mo->SetColor(FN, Colors::lightgreen);
	mo->SetColor(RN, Colors::green);

	mo->SetColor(FF, Colors::cyan);
	mo->SetColor(FR, Colors::darkblue);
	mo->SetColor(RF, Colors::darkgreen);

	mo->SetColor(RR, Colors::darkgray);
	
	forward.SetStopAfterGoal(false);
	backward.SetStopAfterGoal(false);
	std::vector<xyLoc> path;
	forward.GetPath(me, start, goal, path);
	backward.GetPath(me, goal, start, path);
	
	double optimal;
	forward.GetClosedListGCost(goal, optimal);
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			if (map->GetTerrainType(x, y) == kGround)
			{
				xyLoc l(x, y);
				bibfs i = GetLocationClassification(l, optimal);
				counts[i]++;
				mo->SetOverlayValue(x, y, i);
			}
		}
	}
//	mo->SetD
	mo->SetTransparentValue(XX);
	mo->SetOverlayValue(start.x, start.y, 10);
	mo->SetOverlayValue(goal.x, goal.y, 10);
	for (int x = 0; x < counts.size(); x++)
	{
		switch (x)
		{
			case 1: printf("NN: %d\n", counts[x]); break;
			case 2: printf("NF: %d\n", counts[x]); break;
			case 3: printf("NR: %d\n", counts[x]); break;
			case 4: printf("FN: %d\n", counts[x]); break;
			case 5: printf("FF: %d\n", counts[x]); break;
			case 6: printf("FR: %d\n", counts[x]); break;
			case 7: printf("RN: %d\n", counts[x]); break;
			case 8: printf("RF: %d\n", counts[x]); break;
			case 9: printf("RR: %d\n", counts[x]); break;
			default: break;
		}
	}
}
#include "WeightedVertexGraph.h"
const char *strip(const char *str)
{
	static std::string s;
	s = "";
	const char *p1 = strrchr(str, '.');
	const char *p2 = strrchr(str, '/');
	s.append(p2+1, p1);
	return s.c_str();
}

void AnalyzeProblem(Map *m, int whichProblem, Experiment e, double weight)
{
	
	WeightedHeuristic<xyLoc> wh(me, weight);
	ZeroHeuristic<xyLoc> z;
	compare.SetHeuristic(&wh);
	forward.SetStopAfterGoal(false);
	backward.SetStopAfterGoal(false);
	std::vector<xyLoc> path;
	start.x = e.GetStartX();
	start.y = e.GetStartY();
	goal.x = e.GetGoalX();
	goal.y = e.GetGoalY();
	
	{
		Timer timer;
		NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1>> nbse1;
		NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 0>> nbse0;
		NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1>> nbs0e1;
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

		if (1)
		{
			nbs0e1.GetPath(me, start, goal, &z, &z, path);
			printf("NBS0e1 found path length %1.0f; %llu expanded; %llu necessary\n", me->GetPathLength(path),
				   nbs0e1.GetNodesExpanded(), nbs0e1.GetNecessaryExpansions());

			nbse0.GetPath(me, start, goal, me, me, path);
			printf("NBSe0 found path length %1.0f; %llu expanded; %llu necessary\n", me->GetPathLength(path),
				   nbse0.GetNodesExpanded(), nbse0.GetNecessaryExpansions());

			nbse1.GetPath(me, start, goal, me, me, path);
			printf("NBSe1 found path length %1.0f; %llu expanded; %llu necessary\n", me->GetPathLength(path),
				   nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions());

//			EuclideanDistance d;
//			BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(start, goal, me, &d, &d);
//			BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(start, goal, me, me, me);
		}
		
		if (0)
		{
			WeightedHeuristic<xyLoc> w(me, 0.5);
			
			std::string t = "/Users/nathanst/";
			t += strip(e.GetMapName());
			t += "_";
			t += std::to_string(whichProblem);
			t += ".svg";
			//		GetWeightedVertexGraph<xyLoc, tDirection, MapEnvironment>(start, goal, me, t.c_str());
			if (BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(start, goal, me, &w, &w) == 0)
				return;
			
			
			timer.StartTimer();
			nbs.GetPath(me, start, goal, &w, &w, path);
			timer.EndTimer();
			printf("NBSW found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", me->GetPathLength(path),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(), nbs.GetMeetingPoint());
		}

		if (0)
		{
			OffsetHeuristic<xyLoc> o(me, 25);
			if (BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(start, goal, me, &o, &o) == 0)
				return;
			timer.StartTimer();
			nbs.GetPath(me, start, goal, &o, &o, path);
			timer.EndTimer();
			printf("NBSO found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", me->GetPathLength(path),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		
		if (0)
		{
			ZeroHeuristic<xyLoc> z;
			timer.StartTimer();
			nbs.GetPath(me, start, goal, &z, &z, path);
			timer.EndTimer();
			printf("NBS0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", me->GetPathLength(path),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		return;
	}

	
	forward.GetPath(me, start, goal, path);
	backward.GetPath(me, goal, start, path);

	double optimal;
	forward.GetClosedListGCost(goal, optimal);

	compare.GetPath(me, start, goal, path);
	assert(me->GetPathLength(path) == optimal);
	//printf("A* path length: %1.2f\t", me->GetPathLength(path));
	mm.GetPath(me, start, goal, &wh, &wh, path);
	//printf("MM path length: %1.2f\n", me->GetPathLength(path));

	// full state space
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			for (int y = 0; y < m->GetMapHeight(); y++)
			{
				if (m->GetTerrainType(x, y) == kGround)
				{
					counts[GetLocationClassification(xyLoc(x, y), optimal)]++;
				}
			}
		}
		printf("MAP: %d\t", std::accumulate(counts.begin(), counts.end(), 1));
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
	printf("\nA*: %llu\t%llu\t", compare.GetNodesExpanded(), compare.GetNecessaryExpansions());
	// A*
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < compare.GetNumItems(); x++)
		{
			if (compare.GetItem(x).where == kClosedList)
				counts[GetLocationClassification(compare.GetItem(x).data, optimal)]++;
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
	// MM
	printf("\nMM: %llu\t%llu\t", mm.GetNodesExpanded(), mm.GetNecessaryExpansions());
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < mm.GetNumForwardItems(); x++)
		{
			if (mm.GetForwardItem(x).where == kClosedList)
				counts[GetLocationClassification(mm.GetForwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < mm.GetNumBackwardItems(); x++)
		{
			if (mm.GetBackwardItem(x).where == kClosedList)
				counts[GetLocationClassification(mm.GetBackwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
	printf("\n");
}

void AnalyzeMap(const char *map, const char *scenario, double weight)
{
	printf("Loading %s with scenario %s weight %1.2f\n", map, scenario, weight);
	ScenarioLoader s(scenario);
	Map *m = new Map(map);
	me = new MapEnvironment(m);
	me->SetDiagonalCost(1.5);
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
//		if (x+1 != 813)
//			continue;
		if (s.GetNthExperiment(x).GetDistance() <= 0)
			continue;
		printf("Problem %d of %d\n", x+1, s.GetNumExperiments());
		AnalyzeProblem(m, x, s.GetNthExperiment(x), weight);
	}
	exit(0);
}

void AnalyzeNBS(const char *map, const char *scenario, double weight)
{
	NBS<xyLoc, tDirection, MapEnvironment> nbs;
	BSStar<xyLoc, tDirection, MapEnvironment> bs;
	FFBDS<xyLoc, tDirection, MapEnvironment> ff;
	//NBS<xyLoc, tDirection, MapEnvironment, NBSQueueGF<xyLoc>, BDOpenClosed<xyLoc, NBSGLowHigh<xyLoc>, NBSFLowHigh<xyLoc>>> nbs;
	
	MM<xyLoc, tDirection, MapEnvironment> mm;
	MM<xyLoc, tDirection, MapEnvironment> mm0;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	
	printf("Loading %s with scenario %s\n", map, scenario);
	ScenarioLoader s(scenario);
	Map *m = new Map(map);
	me = new MapEnvironment(m);
	me->SetDiagonalCost(1.5);
//	me->SetDiagonalCost(ROOT_TWO);
	Timer t;
	ZeroHeuristic<xyLoc> z;
	// 406 is bad!
	for (int x = s.GetNumExperiments()-1; x >= 0; x--) // 547 to 540
	{
		if (fequal(s.GetNthExperiment(x).GetDistance(), 0))
			continue;
		xyLoc start, goal;
		start.x = s.GetNthExperiment(x).GetStartX();
		start.y = s.GetNthExperiment(x).GetStartY();
		goal.x = s.GetNthExperiment(x).GetGoalX();
		goal.y = s.GetNthExperiment(x).GetGoalY();
//		printf("Problem %d of %d from ", x, s.GetNumExperiments());
//		std::cout << start << " to " << goal << "\n";
		std::vector<xyLoc> correctPath;
		std::vector<xyLoc> mmPath;
		std::vector<xyLoc> mm0Path;
		std::vector<xyLoc> nbsPath;
		std::vector<xyLoc> bsPath;
		astar.SetHeuristic(me);

//		astar.GetPath(me, start, goal, correctPath);
//		printf("%d %1.1f A* nodes: %llu necessary %llu\n", x, me->GetPathLength(correctPath), astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
//		bs.GetPath(me, start, goal, me, me, bsPath);
//		printf("%d %1.1f BS nodes: %llu necessary %llu\n", x, me->GetPathLength(bsPath), bs.GetNodesExpanded(), bs.GetNecessaryExpansions());
//		mm.GetPath(me, start, goal, me, me, mmPath);
//		printf("%d %1.1f MM nodes: %llu necessary %llu\n", x, me->GetPathLength(mmPath), mm.GetNodesExpanded(), mm.GetNecessaryExpansions());
		nbs.GetPath(me, start, goal, me, me, nbsPath);
		printf("%d %1.1f NBS nodes: %llu necessary %llu meeting: %f\n", x, me->GetPathLength(nbsPath), nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetMeetingPoint());

//		ff.GetPath(me, start, goal, me, bsPath);
//		printf("%d %1.1f FF nodes: %llu necessary %llu\n", x, me->GetPathLength(nbsPath), ff.GetNodesExpanded(), 0);
//		mm0.GetPath(me, start, goal, &z, &z, mm0Path);
//		printf("%d %1.1f MM0 nodes: %llu necessary %llu\n", x, me->GetPathLength(mm0Path), mm0.GetNodesExpanded(), mm0.GetNecessaryExpansions());

//		printf("NBSNecessaryRatios: NBS/A* %1.2f NBS/BS %1.2f NBS/MM %1.2f NBS/MM0 %1.2f\n",
//			   (double)nbs.GetNecessaryExpansions()/astar.GetNecessaryExpansions(),
//			   (double)nbs.GetNecessaryExpansions()/bs.GetNecessaryExpansions(),
//			   (double)nbs.GetNecessaryExpansions()/mm.GetNecessaryExpansions(),
//			   (double)nbs.GetNecessaryExpansions()/mm0.GetNecessaryExpansions()
//			   );
//		printf("SelfNecessaryRatios: A* %1.2f BS %1.2f MM %1.2f NBS %1.2f MM0 %1.2f\n",
//			   (double)astar.GetNodesExpanded()/astar.GetNecessaryExpansions(),
//			   (double)bs.GetNodesExpanded()/bs.GetNecessaryExpansions(),
//			   (double)mm.GetNodesExpanded()/mm.GetNecessaryExpansions(),
//			   (double)nbs.GetNodesExpanded()/nbs.GetNecessaryExpansions(),
//			   (double)mm0.GetNodesExpanded()/mm0.GetNecessaryExpansions()
//			   );

//		std::cout << "A*\t" << astar.GetNodesExpanded() << "\tNBS:\t" << nbs.GetNodesExpanded() << "\tBS*:\t" << bs.GetNodesExpanded();
//		std::cout << "\tMM:\t" << mm.GetNodesExpanded() << "\n";
//		printf("NBS* total\t%llu\tnecessary\t%llu\tdoubles\t%llu\t", nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetDoubleExpansions());
//		printf("A* total\t%llu\tnecessary\t%llu\tratio\t%1.3f\n", astar.GetNodesExpanded(), astar.GetNecessaryExpansions(),
//			   (double)nbs.GetNecessaryExpansions()/astar.GetNecessaryExpansions());
		//if (!fequal)
		if (0)//(!fequal(me->GetPathLength(nbsPath), me->GetPathLength(correctPath)))
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << me->GetPathLength(nbsPath) << "\t" << me->GetPathLength(correctPath) << "\n";
			double d;
			for (auto x : correctPath)
			{
				astar.GetClosedListGCost(x, d);
				auto t = nbs.GetNodeForwardLocation(x);
				auto u = nbs.GetNodeBackwardLocation(x);
				std::cout << x << " is on " << t << " and " << u << "\n";
				std::cout << "True g: " << d;
				if (t != kUnseen)
					std::cout << " forward g: " << nbs.GetNodeForwardG(x);
				if (u != kUnseen)
					std::cout << " backward g: " << nbs.GetNodeBackwardG(x);
				std::cout << "\n";
			}
			exit(0);
		}
		
	}
	printf("Exiting with no errors\n");
	exit(0);
}

#include "RubiksCube.h"
void tmp()
{
	printf("---NBS*---\n");
	Timer t;
	RubiksCube cube;
	RubiksState start, goal;
	std::vector<RubiksState> thePath;
	thePath.clear();
	ZeroHeuristic<RubiksState> z;
	t.StartTimer();
	NBS<RubiksState, RubiksAction, RubiksCube> nbs;
	nbs.GetPath(&cube, start, goal, &z, &z, thePath);
	
	t.EndTimer();
	printf("%llu nodes expanded\n", nbs.GetNodesExpanded());
	printf("%llu neccesary nodes expanded\n", nbs.GetNecessaryExpansions());
	printf("Solution path length %1.0f\n", cube.GetPathLength(thePath));
	printf("%1.2f elapsed\n", t.GetElapsedTime());
}
