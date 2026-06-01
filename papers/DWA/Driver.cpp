/*
 *  sample.cpp
 *  hog
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "DynamicWeightedGrid.h"
#include "SVGUtil.h"
#include "IndexOpenClosed.h"

DWG::TerrainType t = DWG::kRoad;

enum demoState {
	kDrawTerrain,
	kWaitPath,
	kDrawingPath,
	kDoPathfinding
};

enum costModel {
	kHuman,
	kHumanSpell,
	kDeer,
	kSheep
};

const int sectorSize = 16;

demoState mode = kDrawTerrain;//kWaitPath;//kDrawTerrain;
bool recording = false;
xyLoc start, goal;
int rate = 20;
DWG::DynamicWeightedGrid<sectorSize> *dwg = 0;//new DWG::DynamicWeightedGrid<sectorSize>(128, 128);
//DWG::DynamicWeightedGrid<sectorSize> *dwg = 0;
DWG::DynamicWeightedGridEnvironment *env;
TemplateAStar<DWG::abstractState, DWG::edge, DWG::DynamicWeightedGrid<sectorSize>> absAStar;
NBS<DWG::abstractState, DWG::edge, DWG::DynamicWeightedGrid<sectorSize>> absNBS;

TemplateAStar<xyLoc, tDirection, DWG::DynamicWeightedGridEnvironment, IndexOpenClosed<xyLoc>> astar;
typedef BDIndexOpenClosed<xyLoc,
						  NBSCompareOpenReady<xyLoc, BDIndexOpenClosedData<xyLoc>>,
                          NBSCompareOpenWaiting<xyLoc, BDIndexOpenClosedData<xyLoc>>> indexOpenClosed;
NBS<xyLoc, tDirection, DWG::DynamicWeightedGridEnvironment, NBSQueue<xyLoc, 1, true, indexOpenClosed>, indexOpenClosed> nbs;
std::vector<xyLoc> path;
std::vector<xyLoc> wpath;
std::vector<DWG::abstractState> absPath;
std::string mapName;

int radius = 0;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Trees", "Draw trees into terrain", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Swamp", "Draw swamp into terrain", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Road", "Draw road into terrain", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Ground", "Draw ground into terrain", kAnyModifier, 'g');
	InstallKeyboardHandler(MyDisplayHandler, "Impassable", "Draw impassable ground", kAnyModifier, 'i');
	InstallKeyboardHandler(MyDisplayHandler, "Water", "Draw water into terrain", kAnyModifier, 'w');

	InstallKeyboardHandler(MyDisplayHandler, "Rad 1", "Set Radius to 1", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "Rad 2", "Set Radius to 2", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Rad 3", "Set Radius to 3", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "Rad 5", "Set Radius to 5", kAnyModifier, '5');

	InstallKeyboardHandler(MyDisplayHandler, "Human", "Human cost structure", kAnyModifier, '7');
	InstallKeyboardHandler(MyDisplayHandler, "Jesus", "Human (walk on water) cost structure", kAnyModifier, '8');
	InstallKeyboardHandler(MyDisplayHandler, "Sheep", "Sheep cost structure", kAnyModifier, '9');
	InstallKeyboardHandler(MyDisplayHandler, "Deer", "Deer cost structure", kAnyModifier, '0');

	
	InstallKeyboardHandler(MyDisplayHandler, "Search", "Search for paths", kAnyModifier, '/');
	InstallKeyboardHandler(MyDisplayHandler, "Edit", "Draw terrain", kAnyModifier, '.');
	InstallKeyboardHandler(MyDisplayHandler, "Abs", "Toggle Drawing Abstraction", kAnyModifier, 'a');
//	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
//	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Search faster", kAnyModifier, ']');
//	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Search slower", kAnyModifier, '[');
//	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Search slower", kAnyModifier, '[');

//	InstallKeyboardHandler(MyTestHandler, "Text", "Randomly test problems", kAnyModifier, ',');

	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <map> <output>", "Make svg of map");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map <map in> <map out>", "Save map after reducing very small regions");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test <map>", "Test random problems on given map");
	InstallCommandLineHandler(MyCLHandler, "-memory", "-memory <map>", "Get memory used by map abstraction");
	InstallCommandLineHandler(MyCLHandler, "-timing", "-timing <map>", "Measure the time to repair the abstraction");

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
		SetNumPorts(windowID, 1);
		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);
		recording = false;
		dwg = new DWG::DynamicWeightedGrid<sectorSize>(128, 128);
		env = new DWG::DynamicWeightedGridEnvironment(128, 128);
		auto cost = env->GetCosts();
		for (auto &i : cost)
			i = 10;
		cost[DWG::kGround] = 1.5;
		cost[DWG::kRoad] = 1.0;
		cost[DWG::kSwamp] = 5.0;
		cost[DWG::kWater] = 4.0;
		cost[DWG::kTrees] = 2.0;
		env->SetCosts(cost);
		mode = kDrawTerrain;
		submitTextToBuffer("Mode: Drawing terrain");

//		dwg = new DWG::DynamicWeightedGrid<sectorSize>("/Users/nathanst/hog2/maps/weighted/Map_1.map");
//		env = new DWG::DynamicWeightedGridEnvironment("/Users/nathanst/hog2/maps/weighted/Map1.map");
		dwg->SetCosts(cost);
		dwg->SetDrawAbstraction(true);
	}
}

void SetCostModel(costModel m)
{
	switch (m)
	{
		case kHuman:
			env->SetCost(DWG::kRoad, 1.0);
			env->SetCost(DWG::kGround, 2.0);
			env->SetCost(DWG::kWater, 3.0);
			env->SetCost(DWG::kTrees, 4.0);
			env->SetCost(DWG::kSwamp, 4.0);
			break;
		case kHumanSpell:
			env->SetCost(DWG::kRoad, 1.0);
			env->SetCost(DWG::kWater, 1.0);
			env->SetCost(DWG::kGround, 2.0);
			env->SetCost(DWG::kSwamp, 4.0);
			env->SetCost(DWG::kTrees, 4.0);
			break;
		case kDeer:
			env->SetCost(DWG::kTrees, 1.0);
			env->SetCost(DWG::kGround, 2.0);
			env->SetCost(DWG::kRoad, 3.0);
			env->SetCost(DWG::kWater, 4.0);
			env->SetCost(DWG::kSwamp, 4.0);
			break;
		case kSheep:
			env->SetCost(DWG::kRoad, 1.0);
			env->SetCost(DWG::kGround, 1.0);
			env->SetCost(DWG::kTrees, 5.0);
			env->SetCost(DWG::kWater, 10.0);
			env->SetCost(DWG::kSwamp, 10.0);
			break;
	}
	dwg->SetCosts(env->GetCosts());
	submitTextToBuffer("Mode: Find paths");
	mode = kDrawingPath;
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = GetContext(windowID)->display;
	//p.OpenGLDraw(s);
	dwg->Draw(display);
	
	if (mode == kWaitPath && path.size() > 0)
	{
		for (size_t x = 1; x < path.size(); x++)
		{
			float x1, y1, r1;
			float x2, y2, r2;
			dwg->GetCoordinate(path[x-1], x1, y1, r1);
			dwg->GetCoordinate(path[x],  x2, y2, r2);
			display.DrawLine({x1, y1}, {x2, y2}, r1, Colors::red);
		}
	}
	if (mode == kWaitPath && absPath.size() > 0)
	{
		for (int x = 1; x < absPath.size(); x++)
		{
			float x1, y1, r1;
			float x2, y2, r2;
			xyLoc a, b;
			a = dwg->GetLocation(absPath[x-1]);
			b = dwg->GetLocation(absPath[x]);
			dwg->GetCoordinate(a, x1, y1, r1);
			dwg->GetCoordinate(b, x2, y2, r2);
			display.DrawLine({x1, y1}, {x2, y2}, 1.5f*r1, Colors::purple);
		}
	}
	if (0 && mode == kWaitPath && wpath.size() > 0)
	{
		for (size_t x = 1; x < wpath.size(); x++)
		{
			float x1, y1, r1;
			float x2, y2, r2;
			dwg->GetCoordinate(wpath[x-1], x1, y1, r1);
			dwg->GetCoordinate(wpath[x],  x2, y2, r2);
			display.DrawLine({x1, y1}, {x2, y2}, r1, Colors::yellow);
		}
	}
	if (mode == kDrawingPath)
	{
		float x1, y1, r1;
		float x2, y2, r2;
		dwg->GetCoordinate(start, x1, y1, r1);
		dwg->GetCoordinate(goal,  x2, y2, r2);
		display.DrawLine({x1, y1}, {x2, y2}, r1*0.5f, Colors::red);
	}
	if (mode == kDoPathfinding)
	{
//		for (int x = 0; x < rate; x++)
//		{
//			if (astar.DoSingleSearchStep(path) == true)
//			{
//				mode = kWaitPath;
//				printf("Path found length %f; %" PRId64 " nodes expanded\n",
//					   env->GetPathLength(path), astar.GetNodesExpanded());
//				break;
//			}
//		}
//		astar.Draw(display);
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		//recording = false;
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/DWA-%d%d%d%d.svg", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		MakeSVG(display, fname, 2048, 2048);
//		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-svg") == 0)
	{
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass argument: <map> <out>\n");
			exit(0);
		}
		DWG::DynamicWeightedGrid<128> dwg(argument[1]);
		Graphics::Display d;
		dwg.Draw(d);
		MakeSVG(d, argument[2], 2048, 2048);
		exit(0);
	}
	if (strcmp(argument[0], "-map") == 0)
	{
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass argument: <map in> <map out>\n");
			exit(0);
		}
		DWG::DynamicWeightedGrid<sectorSize> dwg(argument[1]);
		dwg.SaveMap(argument[2]);
		exit(0);
	}
	if (strcmp(argument[0], "-test") == 0)
	{
		if (maxNumArgs < 2)
		{
			printf("Error: didn't pass argument: <map>\n");
			exit(0);
		}
		dwg = new DWG::DynamicWeightedGrid<sectorSize>(argument[1]);
		env = new DWG::DynamicWeightedGridEnvironment(argument[1]);
		dwg->SetCosts(env->GetCosts());
		mapName = argument[1];
		MyTestHandler(0, kNoModifier, ',');

		exit(0);
	}
	if (strcmp(argument[0], "-memory") == 0)
	{
		if (maxNumArgs < 2)
		{
			printf("Error: didn't pass argument: <map>\n");
			exit(0);
		}
		auto a = new DWG::DynamicWeightedGrid<8>(argument[1]);
		printf("MEMORY(8): %" PRId64 " bytes; %d edges %d regions\n", a->EstimateMemoryInBytes(), a->GetNumEdges(), a->GetNumRegions());
		
		auto b = new DWG::DynamicWeightedGrid<16>(argument[1]);
		printf("MEMORY(16): %" PRId64 " bytes; %d edges %d regions\n", b->EstimateMemoryInBytes(), b->GetNumEdges(), b->GetNumRegions());

		auto c =  new DWG::DynamicWeightedGrid<32>(argument[1]);
		printf("MEMORY(32): %" PRId64 " bytes; %d edges %d regions\n", c->EstimateMemoryInBytes(), c->GetNumEdges(), c->GetNumRegions());
		exit(0);
	}
	if (strcmp(argument[0], "-timing") == 0)
	{
		if (maxNumArgs < 2)
		{
			printf("Error: didn't pass argument: <map>\n");
			exit(0);
		}
		dwg = new DWG::DynamicWeightedGrid<sectorSize>(argument[1]);

		int count = 0;
		Timer t;
		t.StartTimer();
		for (int x = sectorSize/2; x < dwg->GetWidth(); x+=sectorSize)
		{
			for (int y = sectorSize/2; y < dwg->GetWidth(); y+=sectorSize)
			{
				xyLoc l(x, y);
				dwg->SetTerrainType(l, DWG::kGround);
				count++;
			}
		}
		t.EndTimer();
		printf("%1.6fs elapsed\n", t.GetElapsedTime()/count);
		exit(0);
	}

	return 0;
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't': t = DWG::kTrees; mode = kDrawTerrain; break;
		case 's': t = DWG::kSwamp; mode = kDrawTerrain; break;
		case 'p': t = DWG::kRoad; mode = kDrawTerrain; break;
		case 'g': t = DWG::kGround; mode = kDrawTerrain; break;
		case 'i': t = DWG::kBlack; mode = kDrawTerrain; break;
		case 'w': t = DWG::kWater; mode = kDrawTerrain; break;
		case 'r': recording = !recording; break;
		case 'a': dwg->SetDrawAbstraction(!dwg->GetDrawAbstraction()); break;
		case '.':
			mode = kDrawTerrain;
			submitTextToBuffer("Mode: Drawing terrain");
			break;
		case '/':
			mode = kDrawingPath;
			submitTextToBuffer("Mode: Find paths");
			break;
		case '1': radius = 0; mode = kDrawTerrain; submitTextToBuffer("Mode: Drawing terrain"); break;
		case '2': radius = 1; mode = kDrawTerrain; submitTextToBuffer("Mode: Drawing terrain"); break;
		case '3': radius = 2; mode = kDrawTerrain; submitTextToBuffer("Mode: Drawing terrain"); break;
		case '5': radius = 6; mode = kDrawTerrain; submitTextToBuffer("Mode: Drawing terrain"); break;
		case '7': SetCostModel(kHuman); break;
		case '8': SetCostModel(kHumanSpell); break;
		case '9': SetCostModel(kSheep); break;
		case '0': SetCostModel(kDeer); break;
		case '[': rate /= 2; if (rate == 0) rate = 1; break;
		case ']': rate *= 2; break;
		default:
			break;
	}
}

uint64_t GetPathViaAbstraction(const xyLoc &start, const xyLoc &goal, std::vector<xyLoc> &result, uint64_t &firstSegment)
{
	result.clear();
	firstSegment = 0;
	
	static std::vector<xyLoc> tmp;
	uint64_t nodes = 0;
	// 1. Get Abstract states
	DWG::abstractState s = dwg->GetState(start);
	DWG::abstractState g = dwg->GetState(goal);
	// 2. Find abstract path
	//absAStar.SetWeight(1.2);
//	absAStar.GetPath(dwg, s, g, absPath);
//	nodes += absAStar.GetNodesExpanded();
	absNBS.GetPath(dwg, s, g, dwg, dwg, absPath);
	nodes += absNBS.GetNodesExpanded();
	// 3. Connect abstract regions
	xyLoc currStart = start;
//	astar.SetWeight(1.2);
	for (size_t x = 1; x+1 < absPath.size(); x++)
	{
		xyLoc end = dwg->GetLocation(absPath[x]);
		astar.GetPath(env, currStart, end, tmp);
		nodes += astar.GetNodesExpanded();
		if (x == 1)
			firstSegment = nodes;
		result.insert(result.end(), tmp.begin()+((x>1)?1:0), tmp.end());
		currStart = end;
	}
	astar.GetPath(env, currStart, goal, tmp);
	nodes += astar.GetNodesExpanded();
	result.insert(result.end(), tmp.begin()+1, tmp.end());
	astar.SetWeight(1);
	return nodes;
}

void MyTestHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	srandom(20190529);
	const int numProblems = 250;
	uint64_t astarNodes = 0;
	double optimalPath = 0;
	double suboptimalPath = 0;
	double suboptimalPath2 = 0;
	double suboptimalPath3 = 0;
	double absPath = 0;
	ZeroHeuristic<xyLoc> z;
	Timer t;
	printf("version 1\n");
	for (int x = 0; x < numProblems; x++)
	{
		start.x = random()%dwg->GetWidth();
		start.y = random()%dwg->GetHeight();
		goal.x = random()%dwg->GetWidth();
		goal.y = random()%dwg->GetHeight();

//		std::cout << x+1 << ". Searching from " << start << " to " << goal << "\n";
//		t.StartTimer();
//		nbs.GetPath(env, start, goal, env, env, path);
//		t.EndTimer();
//		printf("NBS: Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), nbs.GetNodesExpanded(), t.GetElapsedTime());
//		nbsNodes += nbs.GetNodesExpanded();
//
//		t.StartTimer();
//		nbs.GetPath(env, start, goal, &z, &z, path);
//		t.EndTimer();
//		printf("NBS0: Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), nbs.GetNodesExpanded(), t.GetElapsedTime());

		t.StartTimer();
		astar.GetPath(env, start, goal, path);
		t.EndTimer();
//		printf(" A*: Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), astar.GetNodesExpanded(), t.GetElapsedTime());
		astarNodes += astar.GetNodesExpanded();
		optimalPath += env->GetPathLength(path);

		printf("%d\t%s\t", (int)env->GetPathLength(path)/4, mapName.c_str());
		printf("%d\t%d\t%d\t%d\t%d\t%d\t%f\n",
			   dwg->GetWidth(), dwg->GetHeight(), start.x, start.y, goal.x, goal.y, env->GetPathLength(path));
		
//		astar.SetWeight(4.0);
//		t.StartTimer();
//		astar.GetPath(env, start, goal, path);
//		t.EndTimer();
//		printf("WA*(4): Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), astar.GetNodesExpanded(), t.GetElapsedTime());
//		wastarNodes += astar.GetNodesExpanded();
//		suboptimalPath += env->GetPathLength(path);
//		astar.SetWeight(1.0);
//
//		astar.SetWeight(3.0);
//		t.StartTimer();
//		astar.GetPath(env, start, goal, path);
//		t.EndTimer();
//		printf("WA*(3): Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), astar.GetNodesExpanded(), t.GetElapsedTime());
//		wastarNodes3 += astar.GetNodesExpanded();
//		suboptimalPath3 += env->GetPathLength(path);
//		astar.SetWeight(1.0);
//
//		astar.SetWeight(2.0);
//		t.StartTimer();
//		astar.GetPath(env, start, goal, path);
//		t.EndTimer();
//		printf("WA*(2): Path found length %f; %" PRId64 " nodes expanded; time: %1.6f\n",
//			   env->GetPathLength(path), astar.GetNodesExpanded(), t.GetElapsedTime());
//		wastarNodes2 += astar.GetNodesExpanded();
//		suboptimalPath2 += env->GetPathLength(path);
//		astar.SetWeight(1.0);
//
//		uint64_t tmp2;
//		t.StartTimer();
//		uint64_t tmpNodes = GetPathViaAbstraction(start, goal, path, tmp2);
//		t.EndTimer();
//		firstAbsNodes += tmp2;
//		absNodes += tmpNodes;
//		absPath += env->GetPathLength(path);
//		printf("ABS: Path found length %f; %" PRId64 " nodes expanded [%" PRId64 "]; time: %1.6f\n",
//			   env->GetPathLength(path), tmpNodes, tmp2, t.GetElapsedTime());
	}
//	printf("A*: %" PRId64 "\nNBS: %" PRId64 "\nWA*(2): %" PRId64 "\nWA*(3): %" PRId64 "\nWA*(4): %" PRId64 "\nABS: %" PRId64 " [%" PRId64 "]\n",
//		   astarNodes/numProblems, nbsNodes/numProblems,
//		   wastarNodes2/numProblems, wastarNodes3/numProblems, wastarNodes/numProblems,
//		   absNodes/numProblems, firstAbsNodes/numProblems);
//	printf("Optimal Path: %f\n", optimalPath/numProblems);
//	printf("WA*(2) Path: %f\n", suboptimalPath2/numProblems);
//	printf("WA*(4) Path: %f\n", suboptimalPath/numProblems);
//	printf("ABS Path: %f\n", absPath/numProblems);
}


bool MyClickHandler(unsigned long , int, int, point3d p, tButtonType , tMouseEventType mType)
{
	if (mode == kDrawTerrain)
	{
		int x, y;
		dwg->GetPointFromCoordinate(p, x, y);
		if (x == -1 || y == -1)
			return true;
		for (int ox = -radius; ox <= radius; ox++)
		{
			for (int oy = -radius; oy <= radius; oy++)
			{
				if (x+ox < 0 || y+oy < 0)
					continue;
				xyLoc l(x+ox, y+oy);
				if (l.x < dwg->GetWidth() && l.y < dwg->GetHeight())
				{
//					std::cout << l << "\n";
					env->SetTerrainType(l, t);
					dwg->SetTerrainType(l, t);
				}
			}
		}
	}
	else {
		int x, y;
		dwg->GetPointFromCoordinate(p, x, y);
		switch (mType)
		{
			case kMouseDown:
				start.x = x; start.y = y;
				goal = start;
				mode = kDrawingPath;
				break;
			case kMouseDrag:
				goal.x = x;
				goal.y = y;
				break;
			case kMouseUp:
				goal.x = x;
				goal.y = y;
				mode = kDoPathfinding;
//				std::cout << "Searching from " << start << " to " << goal << "\n";
//				nbs.GetPath(env, start, goal, env, env, path);
//				printf("NBS: Path found length %f; %" PRId64 " nodes expanded\n",
//					   env->GetPathLength(path), nbs.GetNodesExpanded());
//				astar.GetPath(env, start, goal, path);
//				printf(" A*: Path found length %f; %" PRId64 " nodes expanded\n",
//					   env->GetPathLength(path), astar.GetNodesExpanded());
//				astar.InitializeSearch(env, start, goal, path);
				
				if (1)
				{
					//dwg->ValidateEdges();
					DWG::abstractState s = dwg->GetState(start);
					DWG::abstractState g = dwg->GetState(goal);
					absAStar.GetPath(dwg, s, g, absPath);
					printf(" A* abstract path cost: %1.2f\n", dwg->GetPathLength(absPath));
					absNBS.GetPath(dwg, s, g, dwg, dwg, absPath);
					printf("NBS abstract path cost: %1.2f\n", dwg->GetPathLength(absPath));
				}

				if (1)
				{
					mode = kWaitPath;
					uint64_t total, init;
					total = GetPathViaAbstraction(start, goal, path, init);
					printf("ABS: %1.2f [%" PRId64 " - %" PRId64 "]\n", env->GetPathLength(path), total, init);

					nbs.GetPath(env, start, goal, env, env, path);
					printf("NBS: %1.2f [%d - %" PRId64 "]\n", env->GetPathLength(path), path.size(), nbs.GetNodesExpanded());

					astar.SetWeight(10);
					astar.GetPath(env, start, goal, wpath);
					printf("WA*: %1.2f [%d - %" PRId64 "]\n", env->GetPathLength(wpath), path.size(), astar.GetNodesExpanded());
					astar.SetWeight(1);
				}
				break;
			default: break;
		}
		return true;
	}
	
	return true;
}
