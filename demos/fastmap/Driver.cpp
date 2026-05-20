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

enum mode {
	kAddDH = 0,
	kIdentifyLowHeuristic = 1,
	kIdentifyHighHeuristic = 2,
	kFindPath = 3,
	kMeasureHeuristic = 4
};

enum embedding {
	kDH,
	kFastMap
};

enum mapType {
	kRandomMap10 = 0,
	kRandomMap20 = 1,
	kRoomMap8 = 2,
	kRoomMap16 = 3,
	kMazeMap1 = 4,
	kMazeMap2 = 5,
	kDAMap = 6
};

std::vector<xyLoc> path;

graphState nodeToDraw = -1;
xyLoc stateToDraw;
xyLoc start, goal;
GraphEnvironment *ge=0, *basege=0;
Graph *g=0, *base=0;
MapEnvironment *me = 0;
Map *map=0;
std::vector<std::pair<graphState, graphState>> pivots;

void LoadMap(Map *m);
void DoOneDimension(int label, embedding e = kFastMap);
void NormalizeGraph();
void DoAnimation();

bool recording = false;
bool running = false;
bool mapChange = true;
bool graphChanged = true;

const float lerpspeed = 0.003;
float lerp = -lerpspeed;
bool doLerp = true;

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
	InstallKeyboardHandler(MyDisplayHandler, "Load Map", "Load map", kAnyModifier, '0', '6');

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

	InstallMouseClickHandler(MyClickHandler);
}

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

void SaveSVG(Graphics::Display &d, int port = 1)
{
	const std::string baseFileName = "/Users/nathanst/Pictures/hog2/FastMap_";
	static int count = 0;
	std::string fname;
	do {
		fname = baseFileName+std::to_string(count)+".svg";
		count++;
	} while (fileExists(fname.c_str()));
	printf("Save to '%s'\n", fname.c_str());
	MakeSVG(d, fname.c_str(), 1024, 1024, port);
}

void CreateMap(mapType which)
{
	if (map)
		delete map;
	if (me)
		delete me;
	if (base)
		delete base;
	if (basege)
		delete basege;
	if (g)
		delete g;
	if (ge)
		delete ge;

	nodeToDraw = -1;
	
	static int seed = 20;
	srandom(seed++);

	map = new Map(80,80);
	switch (which)
	{
		case kRandomMap20:
			MakeRandomMap(map, 20);
			break;
		case kRandomMap10:
			MakeRandomMap(map, 10);
			break;
		case kRoomMap8:
			BuildRandomRoomMap(map, 8);
			break;
		case kRoomMap16:
			BuildRandomRoomMap(map, 16);
			break;
		case kMazeMap1:
			MakeMaze(map, 1);
			break;
		case kMazeMap2:
			MakeMaze(map, 2);
			break;
		case kDAMap:
			LoadMap(map);
			break;
	}
	
	me = new MapEnvironment(map);
	if (doLerp)
	{
		base = GraphSearchConstants::GetUndirectedGraph(map);
		basege = new GraphEnvironment(base);
	}
	g = GraphSearchConstants::GetUndirectedGraph(map);
	ge = new GraphEnvironment(g);
	ge->SetDirected(false);
	
	DoOneDimension(GraphSearchConstants::kXCoordinate, kDH);
	DoOneDimension(GraphSearchConstants::kYCoordinate, kDH);
	NormalizeGraph();
	
	
	ge->SetDrawEdgeCosts(false);
	ge->SetColor(Colors::white);
	if (doLerp)
	{
		basege->SetDrawEdgeCosts(false);
		basege->SetColor(Colors::white);
	}
	mapChange = true;
	graphChanged = true;
	
//	DoAnimation(); exit(0);
}

void DoOneDimension(int label, embedding e)
{
	bool first = false;
	if (pivots.size() == 0)
		first = true;
	graphState pivot1, pivot2;
	srandom(20230104);
	TemplateAStar<graphState, graphMove, GraphEnvironment> astarf;
	TemplateAStar<graphState, graphMove, GraphEnvironment> astarb;
	std::vector<graphState> p;
	node *n = g->GetRandomNode();
	graphState s, t;
	astarf.InitializeSearch(ge, n->GetNum(), 0, p);
	while (astarf.GetNumOpenItems() > 0)
	{
		t = astarf.GetOpenItem(0).data;
		astarf.DoSingleSearchStep(p);
		pivot2 = t;
	}
	astarb.InitializeSearch(ge, t, 0, p);
	while (astarb.GetNumOpenItems() > 0)
	{
		s = astarb.GetOpenItem(0).data;
		astarb.DoSingleSearchStep(p);
		pivot1 = s;
	}
	astarf.InitializeSearch(ge, s, 0, p);
	while (astarf.GetNumOpenItems() > 0)
	{
		astarf.DoSingleSearchStep(p);
	}
	if (e == kDH)
	{
		if (first)
			pivots.push_back({pivot1,pivot1});
		else
			pivots.push_back({pivot2,pivot2});
	}
	else if (e == kFastMap)
		pivots.push_back({pivot1, pivot2});
	// assign first coordinates
	//(dai + dab − dib)/2
	double dab, dai, dib;
	astarf.GetClosedListGCost(t, dab);
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		astarf.GetClosedListGCost(x, dai);
		astarb.GetClosedListGCost(x, dib);
		if (e == kFastMap)
			n->SetLabelF(label, (dai+dab-dib)/2);
		else if (e == kDH)
		{
			if (first)
				n->SetLabelF(label, dai);
			else
				n->SetLabelF(label, dib);
		}
//		n->SetLabelF(label, dab*(dai/(dai+dib)));
	}
	// update residuals
	if (e == kFastMap)
	{
		for (int x = 0; x < g->GetNumEdges(); x++)
		{
			edge *e = g->GetEdge(x);
			double w = fabs(g->GetNode(e->getFrom())->GetLabelF(label)-g->GetNode(e->getTo())->GetLabelF(label));
			e->setWeight(e->GetWeight()-w);
		}
	}
}

void NormalizeGraph()
{
	double maxx=0, maxy=0;
	double minx = me->GetMap()->GetMapWidth()*me->GetMap()->GetMapHeight();
	double miny = me->GetMap()->GetMapWidth()*me->GetMap()->GetMapHeight();
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		double tx = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		double ty = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		maxx = std::max(maxx, tx);
		minx = std::min(minx, tx);
		maxy = std::max(maxy, ty);
		miny = std::min(miny, ty);
	}
//	printf("X from %f to %f\n", minx, maxx);
//	printf("Y from %f to %f\n", miny, maxy);
	double scale = 2.0/std::max(maxx-minx, maxy-miny);
	double xOff = minx+(maxx-minx)/2;
	double yOff = miny+(maxy-miny)/2;

	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		double tx = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		double ty = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		tx = (tx-xOff)*scale;
		ty = (ty-yOff)*scale;
		n->SetLabelF(GraphSearchConstants::kXCoordinate, tx*0.95);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, ty*0.95);
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
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		ReinitViewports(windowID, {-1.0f, -1.f, 0.f, 1.f}, kScaleToSquare);
		AddViewport(windowID, {0.f, -1.f, 1.f, 1.f}, kScaleToSquare); // kTextView

//		CreateMap(kRoomMap8);
		CreateMap(kMazeMap1);
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	if (lerp < 1 && recording)
	{
		lerp += lerpspeed;
		if (lerp > 1)
			lerp = 1;
		graphChanged = true;
	}
	//else { lerp = 1; }
	
	if ((mapChange||graphChanged) == true && viewport == 0)
	{
		display.StartBackground();
		display.FillRect({-1, -1, 1, 1}, Colors::black);

		me->Draw(display);
//		basege->Draw(display);
		display.EndBackground();
		mapChange = false;
	}
	if (graphChanged == true && viewport == 1)
	{
		graphChanged = false;
		display.StartBackground();
		display.FillRect({-1, -1, 1, 1}, Colors::black);

		ge->SetDrawEdgeCosts(false);
		ge->SetColor(Colors::white);
		if (doLerp) {
			ge->SetColor(Colors::lightred);
			ge->SetNodeScale(200);
			ge->Draw(display, pivots[0].first);
			ge->Draw(display, pivots[0].second);
			ge->SetNodeScale(1);
			ge->SetColor(Colors::white);
			ge->DrawLERP(display, base, g, lerp);
			graphChanged = true;
		}
		else {
			ge->SetNodeScale(g->GetNumNodes()/20);
			ge->Draw(display);
		}
		display.EndBackground();
	}
	if (viewport == 0 && nodeToDraw != -1)
	{
		double x, y, z, r;
		me->GetMap()->GetCoord(stateToDraw.x, stateToDraw.y, x, y, z, r);
		Graphics::point p(x, y);
		display.FillCircle(p, 0.03, Colors::red);
	}
	if (viewport == 1 && nodeToDraw != -1)
	{
		node *n = g->GetNode(nodeToDraw);
		Graphics::point p(n->GetLabelF(GraphSearchConstants::kXCoordinate),
						  n->GetLabelF(GraphSearchConstants::kYCoordinate));
		display.FillCircle(p, 0.03, Colors::red);
		
	}
	if (viewport == 1 && recording)
	{
		std::string fname = "/Users/nathanst/Pictures/hog2/FastMap_";
		static int count = 0;
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(GetContext(windowID)->display, (fname+std::to_string((count/1000)%10)+std::to_string((count/100)%10)+std::to_string((count/10)%10)+std::to_string(count%10)+".svg").c_str(), 600, 600, 1);
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
{
	switch (key)
	{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
			CreateMap((mapType)(key-'0'));
			break;
		case 'l':
			lerp = 0;
		case '[':
		{
//			stepsPerFrame /= 2;
//			std::string s = std::to_string(stepsPerFrame)+" steps per frame";
//			submitTextToBuffer(s.c_str());
		}
			break;
		case ']':
		{
//			if (stepsPerFrame <= 16384)
//				stepsPerFrame *= 2;
//			if (stepsPerFrame == 0)
//				stepsPerFrame = 1;
//			std::string t = std::to_string(stepsPerFrame)+" steps per frame";
//			submitTextToBuffer(t.c_str());
		}
			break;
		case '|':
//			h.Clear();
			break;
		case 'a':
////			submitTextToBuffer("Click anywhere in the map to place a differential heuristic");
//			m = kAddDH;
			break;
		case 'm':
//			if (h.values.size() == 0)
//			{
//				submitTextToBuffer("Error: Must place DH first");
//				break;
//			}
//			m = kMeasureHeuristic;
			break;
		case 'd':
//			showDH = !showDH;
//			mapChanged = true;
			break;
		case 'p':
		{
//			if (showDH)
//			{
//				showDH = false;
//				mapChanged = true;
//			}
//			m = kFindPath;
//			start = goal = {0, 0};
//			path.resize(0);
//			submitTextToBuffer("Click and drag to find path");
		}
			break;
		case 'h':
//			if (h.values.size() == 0)
//			{
//				submitTextToBuffer("Error: Must place DH first");
//				break;
//			}
//			submitTextToBuffer("Select two of the points that have a high heuristic value in the current DH");
//			m = kIdentifyHighHeuristic;
//			FindSamplePoints();
			break;
	}
}






void GetMapLoc(tMouseEventType mType, point3d loc)
{
	int x, y;
	me->GetMap()->GetPointFromCoordinate(loc, x, y);
	nodeToDraw = me->GetMap()->GetNodeNum(x, y);
	stateToDraw.x = x;
	stateToDraw.y = y;
	printf("Hit (%d, %d)\n", x, y);
}

double dist(point3d loc, graphState s)
{
	double x = g->GetNode(s)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double y = g->GetNode(s)->GetLabelF(GraphSearchConstants::kYCoordinate);
	return (loc.x-x)*(loc.x-x)+(loc.y-y)*(loc.y-y);
}

void GetGraphLoc(tMouseEventType mType, point3d loc)
{
	graphState best = 0;
	for (int t = 0; t < g->GetNumNodes(); t++)
		if (dist(loc, t) < dist(loc, best))
			best = t;

	nodeToDraw = best;
	stateToDraw.x = g->GetNode(best)->GetLabelL(GraphSearchConstants::kMapX);
	stateToDraw.y = g->GetNode(best)->GetLabelL(GraphSearchConstants::kMapY);
}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (viewport == 0)
		GetMapLoc(mType, loc);
	if (viewport == 1)
		GetGraphLoc(mType, loc);
	return true;
}

void DoAnimation()
{
	basege->SetDrawEdgeCosts(false);
	basege->SetColor(Colors::black);
	Graphics::Display display;
	// 1. Choose pivot & draw on map
//	DoOneDimension(GraphSearchConstants::kXCoordinate);
//	DoOneDimension(GraphSearchConstants::kYCoordinate);
//	NormalizeGraph();
	
	// basic map
	display.StartFrame();
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	basege->SetNodeScale(1);
	basege->SetColor(Colors::black);
	basege->DrawLERP(display, base, base, 0);
	//basege->Draw(display);
	display.EndFrame();
	SaveSVG(display, 0);

	auto smooth = [](float a, float b, float mix)
	{ float tmp1 = mix*mix*mix; float tmp2 = (1-mix)*(1-mix)*(1-mix);
		mix = (1-mix)*tmp1+mix*(1-tmp2); return (1-mix)*a+mix*b;
	};
	
	// 2. Animate first dimension
	for (float x = 0; x <= 1; x += 0.005f)
	{
		display.StartFrame();
		display.FillRect({-1, -1, 1, 1}, Colors::white);
		ge->SetColor(Colors::black);
		ge->DrawLERP(display, base, g, x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(a, 0, mix); });
		ge->SetColor(Colors::red);
		ge->DrawLERP(display, base, g, pivots[0].first, pivots[0].first,
					 x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(a, 0, mix); });
		ge->DrawLERP(display, base, g, pivots[0].second, pivots[0].second,
					 x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(a, 0, mix); });

//					 [](float a, float b, float mix) { mix = mix*mix; return (1-mix)*a+mix*0;});
//					 [](float a, float b, float mix) { mix = (mix-a)/(b-a); return mix*mix*(3 - 2 * mix);}); // lerp y to 0
//					 [](float a, float b, float mix) { if (mix<0.5) return a; mix = (mix-0.5f)*2; return (1-mix)*a+mix*0;}); // lerp y to 0
		SaveSVG(display, 0);
	}

	// in-between animation
	display.StartFrame();
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	ge->SetColor(Colors::black);
	ge->DrawLERP(display, base, g, 1,
				 smooth,
				 [&smooth](float a, float b, float mix) { return smooth(a, 0, mix); });
	SaveSVG(display, 0);

	
	// 3. Choose new pivot - start over because of normalization
//	delete g;
//	g = GraphSearchConstants::GetUndirectedGraph(map);
//	delete ge;
//	ge = new GraphEnvironment(g);
//	ge->SetDirected(false);
//	DoOneDimension(GraphSearchConstants::kXCoordinate, kFastMap);
//	DoOneDimension(GraphSearchConstants::kYCoordinate, kFastMap);
//	NormalizeGraph();
	
	// 4. Animate next dimension
	for (float x = 0; x < 1; x += 0.005f)
	{
		display.StartFrame();
		ge->SetColor(Colors::black);
		display.FillRect({-1, -1, 1, 1}, Colors::white);
//		ge->DrawLERP(display, base, g, x);
		ge->DrawLERP(display, g, g, x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(0, b, mix); });
//					 [](float a, float b, float mix) { return (1-mix)*a+mix*b;},
//					 [](float a, float b, float mix) { return mix*b;}); // lerp y from 0

		ge->SetColor(Colors::red);
		ge->DrawLERP(display, g, g, pivots[1].first, pivots[1].first,
					 x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(0, b, mix); });
		ge->DrawLERP(display, g, g, pivots[1].second, pivots[1].second,
					 x,
					 smooth,
					 [&smooth](float a, float b, float mix) { return smooth(0, b, mix); });

		SaveSVG(display, 0);
	}
	display.StartFrame();
	ge->SetColor(Colors::black);
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	ge->DrawLERP(display, g, g, 1,
				 smooth,
				 [&smooth](float a, float b, float mix) { return smooth(0, b, mix); });
	SaveSVG(display, 0);
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
