#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include <string>
#include <sstream>
#include <fstream>
#include <numeric>
#include "MapGenerators.h"
#include "ScenarioLoader.h"
#include "GridHeuristics.h"
#include "SVGUtil.h"

using namespace std;
enum GUImode {
	kFindPath = 0,
	kExploreEmbedding = 1,
	kLerp = 2
};

enum whichHeuristic {
	kOH = 0,
	kDH = 1,
	kFM = 2,
	kFMDH = 3,
	kLastH = 4
};

const int kNumDimensions = 2;
Map *map = 0;
MapEnvironment *me = 0;
GridEmbedding *embeddingFM = 0;
GridEmbedding *embeddingDH = 0;
GridEmbedding *embeddingFMDH = 0;
bool mapChanged = true;
bool embeddingChanged = true;
bool recording = false;
void LoadMap(Map *m);
GUImode mode = kFindPath;
whichHeuristic heurToShow = kOH;
xyLoc start, goal;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
bool runningSearch = false;
int stepsPerFrame = 1;
std::vector<xyLoc> path;
Heuristic<xyLoc> h;
float lerp = 0;
const float defaultLerpSpeed = 1.0/200.0;
float lerpSpeed = defaultLerpSpeed;

enum mapType {
    kRandomMap10 = 0,
    kRandomMap20 = 1,
    kRoomMap8 = 2,
    kRoomMap16 = 3,
    kMazeMap1 = 4,
    kMazeMap2 = 5,
    kDAMap = 6,
    kSimpleMap = 7,
    kDA2Maps = 8
};

void SaveSVG(Graphics::Display &d, int port);

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
    InstallKeyboardHandler(MyDisplayHandler, "Load", "Load map", kAnyModifier, '0', '8');
    InstallKeyboardHandler(MyDisplayHandler, "Lerp", "restart lerp", kAnyModifier, 'l');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save a screen shot", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
    //InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
    //InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear DH", kAnyModifier, '|');
    InstallKeyboardHandler(MyDisplayHandler, "Speed Up", "Increase speed of A* search", kAnyModifier, ']');
    InstallKeyboardHandler(MyDisplayHandler, "Slow Down", "Decrease speed of A* search", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Prev Embedding", "Use prev embedding", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Next Embedding", "Use next embedding", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find path using current DH", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Discover", "Discover embedding mapping", kAnyModifier, 'd');

    InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-heuristicSpeed", "-heuristicSpeed [FM,DH,FMDH] <map> <scenario>", "Measure heuristic speed on all start/goal pairs in scenario");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve <dim> [FM,DH,FMDH] <map> <scenario>", "Solve a problems in the scenario using A* and the given heuristic with a <dim> dimensions");

    InstallWindowHandler(MyWindowHandler);

    InstallMouseClickHandler(MyClickHandler);
}

void CreateMap(mapType which)
{
    if (map)
        delete map;
    if (me)
        delete me;
	if (embeddingFM)
		delete embeddingFM;
	if (embeddingDH)
		delete embeddingDH;
	if (embeddingFMDH)
		delete embeddingFMDH;

    static int seed = 20;
    srandom(seed++);
    bool loadedmap=false;
    
    if (gDefaultMap[0] != 0){
		map = new Map(gDefaultMap);
		loadedmap=true;
    }
    else {
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
			case kDAMap:
				LoadMap(map);
				break;
            case kMazeMap1:
                MakeMaze(map, 1);
                break;
			default:
			case kMazeMap2:
				MakeMaze(map, 2);
				break;
            case kSimpleMap:
				MakeMaze(map, 4);
				//LoadSimpleMap(map);
                break;
            case kDA2Maps:
				MakeMaze(map, 8);
                //LoadMaps(map);
                break;
        }
    }
    me = new MapEnvironment(map);
	embeddingFMDH = new GridEmbedding(me, kNumDimensions, kL1);
	for (int x = 0; x < kNumDimensions-1; x++)
		embeddingFMDH->AddDimension(kFastMap, kFurthest);
	embeddingFMDH->AddDimension(kDifferential, kFurthest);

	embeddingFM = new GridEmbedding(me, kNumDimensions, kL1);
	for (int x = 0; x < kNumDimensions; x++)
		embeddingFM->AddDimension(kFastMap, kFurthest);

	embeddingDH = new GridEmbedding(me, kNumDimensions, kLINF);
	for (int x = 0; x < kNumDimensions; x++)
		embeddingDH->AddDimension(kDifferential, kFurthest);
	
	mapChanged = true;
	embeddingChanged = true;
	h.heuristics.resize(0);
	h.heuristics.push_back(me);
	switch (heurToShow)
	{
		case kOH:
			h.heuristics.push_back(me);
			break;
		case kDH:
			h.heuristics.push_back(embeddingDH);
			break;
		case kFMDH:
			h.heuristics.push_back(embeddingFMDH);
			break;
		case kFM:
			h.heuristics.push_back(embeddingFM);
			break;
		default: break;
	}
	
	h.lookups.push_back({kMaxNode, 1, 2});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	runningSearch = false;
	goal = start;
	//SaveSVG();
	//exit(0);
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
		setTextBufferVisibility(false);
        CreateMap(kRoomMap8);
    }
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
    Graphics::Display &display = getCurrentContext()->display;
	
    if (mapChanged == true && viewport == 0)
    {
        display.StartBackground();
        display.FillRect({-1, -1, 1, 1}, Colors::black);
        me->Draw(display);
		display.EndBackground();
		mapChanged = false;
    }
	if (mode == kExploreEmbedding)
	{
		if (viewport == 0)
		{
			me->SetColor(Colors::red);
			me->DrawAlternate(display, start);
			me->DrawAlternate(display, goal);
			me->SetColor(Colors::yellow);
			me->DrawLine(display, start, goal, 1);
//			if (heurToShow == kDH)
//			{
//				std::string s = std::to_string(embeddingDH->HCost(start, goal));
//				display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
//				embeddingDH->DrawPivots(display);
//				embeddingDH->DrawPivots(display, goal);
//			}

			if (me->GetMap()->GetTerrainType(goal.x, goal.y) == kGround)
			{
				display.FillRect({-1, -1, 1, -0.95}, Colors::darkgray);
				std::string s = "[Heuristic] Octile:"+to_string_with_precision(me->HCost(start, goal), 2)+
				" DH: "+to_string_with_precision(embeddingDH->HCost(start, goal), 2)+
				" FM: "+to_string_with_precision(embeddingFM->HCost(start, goal), 2)+
				" FMDH: "+to_string_with_precision(embeddingFMDH->HCost(start, goal), 2);
				display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
			}

		}
		switch (heurToShow)
		{
			case kOH:
				if (viewport == 0)
				{
//					if (me->GetMap()->GetTerrainType(goal.x, goal.y) == kGround)
//					{
//						display.FillRect({-1, -1, -0.5, -0.95}, Colors::darkgray);
//						std::string s = "Octile h-cost: "+to_string_with_precision(me->HCost(start, goal), 2);
//						display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
//					}
				}
				break;
			case kDH:
				if (viewport == 1)
				{
					embeddingDH->Draw(display, start);
					embeddingDH->Draw(display, goal);
				}
//				else {
//					display.FillRect({-1, -1, -0.5, -0.95}, Colors::darkgray);
//					if (me->GetMap()->GetTerrainType(goal.x, goal.y) == kGround)
//					{
//						std::string s = "DH h-cost: "+to_string_with_precision(embeddingDH->HCost(start, goal), 2);
//						display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
//					}
//				}
				break;
			case kFMDH:
				if (viewport == 1)
				{
					embeddingFMDH->Draw(display, start);
					embeddingFMDH->Draw(display, goal);
				}
//				else {
//					display.FillRect({-1, -1, -0.5, -0.95}, Colors::darkgray);
//					if (me->GetMap()->GetTerrainType(goal.x, goal.y) == kGround)
//					{
//						std::string s = "FMDH h-cost: "+to_string_with_precision(embeddingFMDH->HCost(start, goal), 2);
//						display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
//					}
//				}
				break;
			case kFM:
				if (viewport == 1)
				{
					embeddingFM->Draw(display, start);
					embeddingFM->Draw(display, goal);
				}
//				else {
//					display.FillRect({-1, -1, -0.5, -0.95}, Colors::darkgray);
//					if (me->GetMap()->GetTerrainType(goal.x, goal.y) == kGround)
//					{
//						std::string s = "FM h-cost: "+to_string_with_precision(embeddingFM->HCost(start, goal), 2);
//						display.DrawText(s.c_str(), {-1, -1}, Colors::yellow, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
//					}
//				}
				break;
			default: break;
		}
	}
	if (viewport == 0 && mode == kFindPath && start != goal && !runningSearch)
	{
		me->SetColor(Colors::blue);
		me->DrawLine(display, start, goal, 2);
	}
	if (viewport == 0 && runningSearch)
	{
		for (int x = 0; x < stepsPerFrame; x++)
			if (path.size() == 0)
				astar.DoSingleSearchStep(path);
		astar.Draw(display);
//		std::string s = std::to_string(astar.GetNodesExpanded())+" expanded";
//		submitTextToBuffer(s.c_str());
	}
	if (embeddingChanged == true && viewport == 1)
    {
        display.StartBackground();
        display.FillRect({-1, -1, 1, 1}, Colors::black);

		if (mode == kLerp)
		{
			switch (heurToShow)
			{
				case kOH: break;//me->Draw(display); break;
				case kDH: embeddingDH->Draw(display, lerp); break;
				case kFMDH: embeddingFMDH->Draw(display, lerp); break;
				case kFM: embeddingFM->Draw(display, lerp); break;
				default: break;
			}
		}
		else {
			switch (heurToShow)
			{
				case kOH: break;//me->Draw(display); break;
				case kDH: embeddingDH->Draw(display); break;
				case kFMDH: embeddingFMDH->Draw(display); break;
				case kFM: embeddingFM->Draw(display); break;
				default: break;
			}
		}

		display.EndBackground();
        embeddingChanged = false;
    }

	if (mode == kLerp && viewport == 1)
	{
		lerp += lerpSpeed;
		if (lerp > 1)
		{
			lerp = 1;
			//mode = kFindPath;
		}
		mapChanged = true;
		embeddingChanged = true;
	}

    if (viewport ==1 && recording)
    {
		SaveSVG(display, 1);
//		SaveSVG();
//        recording=false;
    }
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-heuristicSpeed" ) == 0 )
	{
		GridEmbedding *ge;
		MapEnvironment *me = new MapEnvironment(new Map(argument[2]));
		ScenarioLoader *sl = new ScenarioLoader(argument[3]);
		//-heuristicSpeed [FM,DH,FMDH] <map> <scenario>
		if (strcmp(argument[1], "FM") == 0)
		{
			ge = new GridEmbedding(me, 10, kL1);
			for (int x = 0; x < 10; x++)
				ge->AddDimension(kFastMap, kFurthest);
		}
		else if (strcmp(argument[1], "DH") == 0)
		{
			ge = new GridEmbedding(me, 10, kLINF);
			for (int x = 0; x < 10; x++)
				ge->AddDimension(kDifferential, kFurthest);
		}
		else if (strcmp(argument[1], "FMDH") == 0)
		{
			ge = new GridEmbedding(me, 10, kL1);
			for (int x = 0; x < 9; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);
		}
		else {
			printf("Invalid parameter '%s'\n", argument[1]);
			exit(0);
		}
		double h = 0;
		Timer t;
		// populating caches identically
		for (int x = 0; x < sl->GetNumExperiments(); x++)
		{
			xyLoc from(sl->GetNthExperiment(x).GetStartX(), sl->GetNthExperiment(x).GetStartY());
			xyLoc to(sl->GetNthExperiment(x).GetGoalX(), sl->GetNthExperiment(x).GetGoalY());
			ge->HCost(from, to);
		}
		t.StartTimer();
		for (int x = 0; x < sl->GetNumExperiments(); x++)
		{
			xyLoc from(sl->GetNthExperiment(x).GetStartX(), sl->GetNthExperiment(x).GetStartY());
			xyLoc to(sl->GetNthExperiment(x).GetGoalX(), sl->GetNthExperiment(x).GetGoalY());
			h += ge->HCost(from, to);
		}
		t.EndTimer();
		printf("%1.2f microseconds elapsed for %s heuristic; total value %f\n", 1000000*t.GetElapsedTime(), argument[1], h);
		exit(0);
	}
	if (strcmp(argument[0], "-solve" ) == 0 )
	{
		Timer t;
		int dim = atoi(argument[1]);
		GridEmbedding *ge;
		MapEnvironment *me = new MapEnvironment(new Map(argument[3]));
		ScenarioLoader *sl = new ScenarioLoader(argument[4]);
		Heuristic<xyLoc> h;
		//-heuristicSpeed [FM,DH,FMDH] <map> <scenario>
		if (strcmp(argument[2], "FM") == 0)
		{
			ge = new GridEmbedding(me, dim, kL1);
			for (int x = 0; x < dim; x++)
				ge->AddDimension(kFastMap, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "DH") == 0)
		{
			ge = new GridEmbedding(me, dim, kLINF);
			for (int x = 0; x < dim; x++)
				ge->AddDimension(kDifferential, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "FM_e") == 0)
		{
			ge = new GridEmbedding(me, dim, kL1);
			ge->AddDimension(kFastMap, kHeuristicError);
			for (int x = 0; x < dim-1; x++)
				ge->AddDimension(kFastMap, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "FMDH_e") == 0)
		{
			ge = new GridEmbedding(me, dim, kL1);
			ge->AddDimension(kFastMap, kHeuristicError);
			for (int x = 0; x < dim-2; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "FMDH") == 0)
		{
			ge = new GridEmbedding(me, dim, kL1);
			for (int x = 0; x < dim-1; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "DH+FM") == 0)
		{
			ge = new GridEmbedding(me, dim/2, kLINF);
			for (int x = 0; x < dim/2; x++)
				ge->AddDimension(kDifferential, kFurthest);
			h.heuristics.push_back(ge); // DH

			ge = new GridEmbedding(me, dim/2, kL1);
			for (int x = 0; x < dim/2; x++)
				ge->AddDimension(kFastMap, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 3});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.lookups.push_back({kLeafNode, 2, 2});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "DH+FMDH") == 0)
		{
			ge = new GridEmbedding(me, dim/2, kLINF);
			for (int x = 0; x < dim/2; x++)
				ge->AddDimension(kDifferential, kFurthest);

			h.heuristics.push_back(ge); // FM
			ge = new GridEmbedding(me, dim/2, kL1);
			for (int x = 0; x < dim/2-1; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);


			h.lookups.push_back({kMaxNode, 1, 3});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.lookups.push_back({kLeafNode, 2, 2});
			h.heuristics.push_back(ge);
			h.heuristics.push_back(me);
		}
		else if (strcmp(argument[2], "FMDH_e+DH") == 0)
		{
			ge = new GridEmbedding(me, dim/2, kLINF);
			for (int x = 0; x < dim/2; x++)
				ge->AddDimension(kDifferential, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 1}); // only uses base heuristic for error
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(me);
			h.heuristics.push_back(ge);

			ge = new GridEmbedding(me, dim, kL1);

			ge->AddDimension(kFastMap, kHeuristicError, &h);
			for (unsigned int x = 0; x < dim/2-2; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);

			h.heuristics.push_back(ge);
			h.lookups[0].numChildren = 3;
			h.lookups.push_back({kLeafNode, 2, 2});
		}
		else if (strcmp(argument[2], "DH+FMDH_e") == 0)
		{
			ge = new GridEmbedding(me, dim/2, kLINF);
			for (int x = 0; x < dim/2; x++)
				ge->AddDimension(kDifferential, kFurthest);

			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.push_back(me);
			h.heuristics.push_back(ge);

			ge = new GridEmbedding(me, dim, kL1);

			ge->AddDimension(kFastMap, kHeuristicError, &h);
			for (unsigned int x = 0; x < dim/2-2; x++)
				ge->AddDimension(kFastMap, kFurthest);
			ge->AddDimension(kDifferential, kFurthest);

			h.heuristics.push_back(ge);
			h.lookups[0].numChildren = 3;
			h.lookups.push_back({kLeafNode, 2, 2});
		}
		else if (strcmp(argument[2], "FM2DH_e") == 0)
		{
			h.lookups.push_back({kMaxNode, 1, 1});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.heuristics.push_back(me);

			for (unsigned int x = 0; x < dim/3; x++)
			{
				ge = new GridEmbedding(me, dim, kL1);
				ge->AddDimension(kFastMap, kHeuristicError, &h);
				ge->AddDimension(kFastMap, kFurthest);
				ge->AddDimension(kDifferential, kFurthest);
				h.lookups[0].numChildren = 2+x;
				h.lookups.push_back({kLeafNode, (x+1), (x+1)});
				h.heuristics.push_back(ge);
			}
		}
		else if (strcmp(argument[2], "FM2DH") == 0)
		{
			h.lookups.push_back({kMaxNode, 1, 1});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.heuristics.push_back(me);

			for (unsigned int x = 0; x < dim/3; x++)
			{
				ge = new GridEmbedding(me, dim, kL1);
				ge->AddDimension(kFastMap, kFurthest, &h);
				ge->AddDimension(kFastMap, kFurthest);
				ge->AddDimension(kDifferential, kFurthest);
				h.lookups[0].numChildren = 2+x;
				h.lookups.push_back({kLeafNode, (x+1), (x+1)});
				h.heuristics.push_back(ge);
			}
		}
		else {
			printf("Invalid parameter '%s'\n", argument[2]);
			exit(0);
		}

		astar.SetHeuristic(&h);
		for (int x = 0; x < sl->GetNumExperiments(); x++)
		{
			// We aren't checking connected components explicitly, but the code
			// has this information and trivially could.
			// So, we just skip the bucket 0 problems which may not be solvable
			if (sl->GetNthExperiment(x).GetBucket() == 0)
				continue;
			xyLoc from(sl->GetNthExperiment(x).GetStartX(), sl->GetNthExperiment(x).GetStartY());
			xyLoc to(sl->GetNthExperiment(x).GetGoalX(), sl->GetNthExperiment(x).GetGoalY());
			astar.GetPath(me, from, to, path);
			if (!fequal(me->GetPathLength(path), sl->GetNthExperiment(x).GetDistance(), 0.01))
			{
				printf("Problem %d: Got path cost %f, should have been %f\n", x,
					   me->GetPathLength(path), sl->GetNthExperiment(x).GetDistance());
				exit(1);
			}
			printf("%s Problem %d: %" PRId64 " nodes expanded\n", argument[2], x, astar.GetNodesExpanded());
		}
		exit(0);
	}

	
    if (strcmp( argument[0], "-map" ) == 0 )
    {
//        if (maxNumArgs <= 1)
//            return 0;
//        strncpy(gDefaultMap, argument[1], 1024);
//        strncpy(scenfile, argument[2], 1024);
//        cout<<gDefaultMap<<endl;
//        //Extracting the name of the map
//        for(int i=strlen(gDefaultMap);i>=0;i--)
//            if(gDefaultMap[i]=='/'){
//                strncpy(mapName, gDefaultMap+i+1, strlen(gDefaultMap)-1-i-4);
//                break;
//            }
//        strncpy(saveDirectory, argument[3], 1024);
//        return 2;
//    }
//    if (strcmp( argument[0], "-heuristicSpeed" ) == 0 )
//      {
//	// heuristicSpeed [FM,DH,FMDH] <map> <scenario
//	map = new Map(argument[2]);
//	ScenarioLoader *sl = new ScenarioLoader(argument[3]);
//        g = GraphSearchConstants::GetUndirectedGraph(map);
//        ge = new GraphEnvironment(g);
//        ge->SetDirected(false);
//        StoreEdgeWeights(kEdgeWeight+1);
//        StoreMapLocInNodeLabels();
//        FindLargestPart();
//	int nofp = 10;
//	GraphMapHeuristicE<graphState> octile(map, g);
//	EmbeddingHeuristic<graphState> embedding(g, GraphSearchConstants::kFirstData + nofp, nofp);
//	DifferentialHeuristic<graphState> dh(g, GraphSearchConstants::kFirstData + nofp, nofp);
//	GraphHeuristicContainerE <graphState> h(g);
//
//	if (strcmp(argument[1], "FMDH") == 0)
//	  {
//	    DoDimensions(GraphSearchConstants::kFirstData+ nofp, nofp, 1);
//	    h.AddHeuristic(&octile);
//	    h.AddHeuristic(&embedding);
//	    printf("Computing FMDH\n");
//	  }
//	else if (strcmp(argument[1], "FM") == 0)
//	  {
//	    DoDimensions(GraphSearchConstants::kFirstData+nofp, nofp, 0);
//	    h.AddHeuristic(&octile);
//	    h.AddHeuristic(&embedding);
//	    printf("Computing FM\n");
//	  }
//	else if (strcmp(argument[1], "DH") == 0)
//	  {
//	    DoDH(GraphSearchConstants::kFirstData + nofp, nofp);
//	    h.AddHeuristic(&octile);
//	    h.AddHeuristic(&dh);
//	    printf("Computing DH\n");
//	  }
//	Timer t;
//	float sum = 0;
//	t.StartTimer();
//	for (int j = 0; j < sl->GetNumExperiments(); j++)
//	  {
//	    Experiment e = sl->GetNthExperiment(j);
//            xyLoc start, goal;
//            start.x = e.GetStartX();
//            start.y = e.GetStartY();
//            goal.x = e.GetGoalX();
//            goal.y = e.GetGoalY();
//
//	    sum += h.HCost(map->GetNodeNum(start.x, start.y), map->GetNodeNum(goal.x, goal.y));
//
//	  }
//	t.EndTimer();
//	printf("%s %1.6fus elapsed sum: %f\n", argument[1], t.GetElapsedTime()*1000000, sum);
//	exit(0);
	}
//    // Not sure about this
    return 0;
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
        case '7':
        case '8':
            CreateMap((mapType)(key-'0'));
            break;
        case 'l':
			mode = kLerp;
			lerp = 0;
			runningSearch = false;
			path.resize(0);
            //lerp = 0;
			break;
		case '{':
			// back 2 then forward 1
			heurToShow = (whichHeuristic)(((int)heurToShow+kLastH-2)%((int)kLastH));
		case '}':
			heurToShow = (whichHeuristic)(((int)heurToShow+1)%((int)kLastH));
			if (runningSearch)
				astar.InitializeSearch(me, start, goal, path);
			embeddingChanged = true;
			mapChanged = true;
			switch (heurToShow)
			{
				case kOH:
					h.heuristics[1] = me;
					break;
				case kDH:
					h.heuristics[1] = embeddingDH;
					break;
				case kFMDH:
					h.heuristics[1] = embeddingFMDH;
					break;
				case kFM:
					h.heuristics[1] = embeddingFM;
					break;
				default: break;
			}
			break;
        case '[':
        {
            stepsPerFrame /= 2;
//            std::string s = std::to_string(stepsPerFrame)+" steps per frame";
//            submitTextToBuffer(s.c_str());
        }
            break;
        case ']':
        {
            if (stepsPerFrame <= 16384)
                stepsPerFrame *= 2;
            if (stepsPerFrame == 0)
                stepsPerFrame = 1;
//            std::string t = std::to_string(stepsPerFrame)+" steps per frame";
//            submitTextToBuffer(t.c_str());
        }
            break;
        case '|':
//            h.Clear();
            break;
        case 'a':
////            submitTextToBuffer("Click anywhere in the map to place a differential heuristic");
//            m = kAddDH;
            break;
        case 'm':
//            if (h.values.size() == 0)
//            {
//                submitTextToBuffer("Error: Must place DH first");
//                break;
//            }
//            m = kMeasureHeuristic;
            break;
        case 'd':
			mode = kExploreEmbedding;
			runningSearch = false;
            break;
        case 'p':
			mode = kFindPath;
			break;
		case 's':
		{
			Graphics::Display &d = getCurrentContext()->display;
			SaveSVG(d, 0);
			SaveSVG(d, 1);
		}
        case 'r':
            recording=!recording;
            break;
    }
}


void GetMapLoc(tMouseEventType mType, point3d loc)
{
    int x, y;
    me->GetMap()->GetPointFromCoordinate(loc, x, y);
	if (mode == kExploreEmbedding)
	{
		switch (mType)
		{
			case kMouseDown:
				start = xyLoc(x, y);
				goal = xyLoc(x, y);
				runningSearch = false;
				path.resize(0);
				break;
			case kMouseDrag:
			case kMouseUp:
				if (me->GetMap()->GetTerrainType(x, y) != kGround)
					return;
				goal = xyLoc(x, y);
				if (me->GetMap()->GetTerrainType(start.x, start.y) != kGround)
					start = goal;
				break;
			default: return;
		}
	}
	if (mode == kFindPath)
	{
		switch (mType)
		{
			case kMouseDown:
				start = xyLoc(x, y);
				goal = xyLoc(x, y);
				runningSearch = false;
				path.resize(0);
				break;
			case kMouseDrag:
				if (me->GetMap()->GetTerrainType(x, y) != kGround)
					return;
				goal = xyLoc(x, y);
				if (me->GetMap()->GetTerrainType(start.x, start.y) != kGround)
					start = goal;
				break;
			case kMouseUp:
			{
				if (me->GetMap()->GetTerrainType(start.x, start.y) != kGround)
					return;
				if (me->GetMap()->GetTerrainType(x, y) == kGround)
					goal = xyLoc(x, y);
				if (start != goal)
				{
					runningSearch = true;
					astar.SetHeuristic(&h);
					astar.InitializeSearch(me, start, goal, path);
				}
				break;
			}
			default: return;
		}
		
	}
}

double dist(point3d loc1, point3d loc2)
{
	return (loc1-loc2).length();
}

void GetGraphLoc(tMouseEventType mType, point3d loc)
{
	if (mode != kExploreEmbedding)
		return;
	
	point3d p, bestPoint;
	xyLoc best;
	for (int t = 0; t < me->GetMaxHash(); t++)
	{
		xyLoc s;
		me->GetStateFromHash(t, s);
		
		switch (heurToShow)
		{
			case kOH: return;//me->Draw(display); break;
			case kDH:
				p = embeddingDH->Lookup(s);
				break;
			case kFMDH:
				p = embeddingFMDH->Lookup(s);
				break;
			case kFM:
				p = embeddingFM->Lookup(s);
				break;
			default: break;
		}

		if (dist(loc, p) < dist(loc, bestPoint))
		{
			best = s;
			bestPoint = p;
		}
	}

	switch (mType)
	{
		case kMouseDown:
			start = best;
			goal = best;
			runningSearch = false;
			path.resize(0);
			break;
		case kMouseDrag:
		case kMouseUp:
			goal = best;
			break;
		default: return;
	}

}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (mode == kLerp)
	{
		switch (mType)
		{
			case kMouseDown:
				lerpSpeed = 0;
				break;
			case kMouseDrag:
				break;
			case kMouseUp:
				lerpSpeed = defaultLerpSpeed;
				break;
			default: break;
		}
		lerp = (loc.x+1)/2;
		if (lerp < 0) lerp = 0;
		if (lerp > 1) lerp = 1;
		return true;
	}
	if (viewport == 0)
        GetMapLoc(mType, loc);
    if (viewport == 1)
        GetGraphLoc(mType, loc);
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

void LoadSimpleMap(Map *m)
{
    m->Scale(4, 4);
    const char map[] = "................";
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

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

void SaveSVG(Graphics::Display &d, int port)
{
	const std::string baseFileName = "/Users/nathanst/Pictures/hog2/FMDH_";
	static int count = 0;
	std::string fname;
	do {
		fname = baseFileName+std::to_string(count)+".svg";
		count++;
	} while (fileExists(fname.c_str()));
	printf("Save to '%s'\n", fname.c_str());
	MakeSVG(d, fname.c_str(), 1024, 1024, port);
}
