/*
 * This code demonstrates IBEX variants of BTS and BGS and can be used to duplicate
 * many of the experiments found in that paper.
 */
#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "GraphInconsistencyInstances.h"
#include "ImprovedBGS2.h"
#include "IncrementalBGS.h"
#include "Map2DEnvironment.h"
#include "ScenarioLoader.h"
#include <map>
#include "IBEX.h"
#include "Table.h"

const int kHeuristic = GraphSearchConstants::kTemporaryLabel;

bool recording = false;

/***** Graph Tests ******/

Graph *g;
GraphEnvironment *ge;

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return g->GetNode(a)->GetLabelL(kHeuristic);
	}
};

GraphDistHeuristic h;

/******* DH Tests ********/


struct dh {
	std::vector<double> depths;
	xyLoc startLoc;
};

class DifferentialHeuristic : public Heuristic<xyLoc> {
public:
	double HCost(const xyLoc &a, const xyLoc &b) const
	{
		//return distance(a, b);
		double v = e->HCost(a, b);
		int a_hash = e->GetStateHash(a);
		int b_hash = e->GetStateHash(b);
		int x  = (a.x+a.y)%(values.size());
		v = std::max(v, fabs(values[x].depths[a_hash]-values[x].depths[b_hash]));
		return v;
	/*	for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[a_hash]-values[x].depths[b_hash]));
		return v;
	*/
	}
	// Doesn't include the baseline heuristic
	double DHCost(const xyLoc &a, const xyLoc &b) const
	{
		//return distance(a, b);
		double v = 0;
		for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[e->GetStateHash(a)]-values[x].depths[e->GetStateHash(b)]));
		return v;
	}
	void Clear()
	{ values.resize(0); }
	void Add()
	{
		std::vector<xyLoc> path;
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		astar.SetStopAfterGoal(false);
		xyLoc where;
		if (values.size() == 0)
		{ // find first open location
		    Map *m = e->GetMap();
			for (int y = 0; y < m->GetMapHeight(); y++)
			{
				for (int x = 0; x < m->GetMapWidth(); x++)
				{
					if (m->GetTerrainType(x, y) == kGround)
					{
						where.x = x;
						where.y = y;
						y = m->GetMapHeight();
						x = m->GetMapWidth();
						astar.InitializeSearch(e, where, where, path);
					}
				}
			}
		}
		else { // get furthest point
			astar.InitializeSearch(e, values[0].startLoc, values[0].startLoc, path);
			for (int x = 1; x < values.size(); x++)
				astar.AddAdditionalStartState(values[x].startLoc);
		}

		while (astar.GetNumOpenItems() > 0)
		{
			where = astar.CheckNextNode();
			astar.DoSingleSearchStep(path);
		}
		
		//std::cout << "Adding DH at " << where << "\n";
		dh newDH;
		newDH.startLoc = where;
		newDH.depths.resize(e->GetMaxHash());
		astar.SetStopAfterGoal(false);
		astar.GetPath(e, where, where, path);
		
		for (int x = 0; x < astar.GetNumItems(); x++)
		{
			double cost;
			xyLoc v = astar.GetItem(x).data;
			if (!astar.GetClosedListGCost(v, cost)) {
				//printf("Error reading depth from closed list!\n");
			}
			else {
				int hash = e->GetStateHash(v);
				newDH.depths[hash] = cost;
			}
		}
		values.push_back(newDH);
	}
	MapEnvironment *e;
	std::vector<dh> values;
};

DifferentialHeuristic DH;



int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Test", "Basic test with MD heuristic", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');

//	InstallKeyboardHandler(WeightedTest, "Weighted STP Test", "Test the STP with weights", kNoModifier, 'w');
//	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
//	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-polygraph", "-polygraph <size> <algorithm>", "Runs worst-case inconsistency graph");
	InstallCommandLineHandler(MyCLHandler, "-grid", "-grid <map> <scenario> <algorithm>", "Runs the algorithm on the scenario file with Octile heuristic");
	InstallCommandLineHandler(MyCLHandler, "-inconsistent", "-inconsistent <map> <scenario> <algorithm>", "Runs the algorithm on the scenario file with a compressed DH heuristic");
	InstallCommandLineHandler(MyCLHandler, "-compareall", "-compareall <map> <scenario> ", "Runs all algorithms with consistent and inconsistent heurstics and generates a table");
	
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
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.DrawText("No visual display", {0, 0}, Colors::black, 0.1, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
}

void runProblemSet(char *theMap, char *scenario, char *algorithm)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	ImprovedBGS2<xyLoc, tDirection>  bgs;
	IncrementalBGS<xyLoc, tDirection>  ibex;
	IBEX::IBEX<xyLoc, tDirection, MapEnvironment, false> i(2, 8, 2, false);
	
	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	Timer t;
	std::vector<xyLoc> path;
	std::vector<tDirection> path2;
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetDistance() == 0)
			continue;
		
		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());

		if (strcmp(algorithm, "astar") == 0)
		{
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs") == 0)
		{
			t.StartTimer();
			bgs.SetK(9);
			bgs.GetPath(&e, from, to, &e, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), bgs.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "ibex") == 0)
		{
			t.StartTimer();
			ibex.GetPath(&e, from, to, &e, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), ibex.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "IBEX") == 0)
		{
			t.StartTimer();
			i.GetPath(&e, from, to, path2);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), i.GetNodesExpanded(), t.GetElapsedTime());
		}
/*
		if (fgreater(fabs(e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance()), 0.01))
		{
			std::cout << "From: " << from << " to " << to << "\n";
			printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), e.GetPathLength(path), s.GetNthExperiment(x).GetDistance(),
				  e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance());
			exit(1);
		}
		*/
	}
	
	exit(0);
}



void runProblemSetInconsistent(char *theMap, char *scenario, char *algorithm, char *K)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	IncrementalBGS<xyLoc, tDirection>  ibex;
	IBEX::IBEX<xyLoc, tDirection, MapEnvironment, false> i(2, 8, 2, false);
	searcher.SetReopenNodes(true);
	
	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	DH.e = &e;
	for (int x = 0; x < 10; x++)
		DH.Add();
	Timer t;
	std::vector<xyLoc> path;
	std::vector<tDirection> path2;
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetDistance() == 0)
			continue;

		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());

		if (strcmp(algorithm, "astar") == 0)
		{
			searcher.SetHeuristic(&DH);
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bpmx") == 0)
		{
			searcher.SetHeuristic(&DH);
			searcher.SetUseBPMX(1);
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs-bpmx") == 0)
		{
			ImprovedBGS2<xyLoc, tDirection>  bgs;
			t.StartTimer();
			bgs.SetUseBPMX();
			bgs.SetK(std::stoi(K));
			bgs.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs") == 0)
		{
			ImprovedBGS2<xyLoc, tDirection>  bgs;
			t.StartTimer();
			bgs.SetK(std::stoi(K));
			bgs.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
			if (fgreater(fabs(bgs.GetSolutionCost()-s.GetNthExperiment(x).GetDistance()), 0.01))
			{
				std::cout << "From: " << from << " to " << to << "\n";
				printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), e.GetPathLength(path), s.GetNthExperiment(x).GetDistance(),
					bgs.GetSolutionCost()-s.GetNthExperiment(x).GetDistance());
				exit(1);
			}
		}

		if (strcmp(algorithm, "ibex") == 0)
		{
			t.StartTimer();
			ibex.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), ibex.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "IBEX") == 0)
		{
			t.StartTimer();
			i.GetPath(&e, &DH, from, to, path2);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(from, path2), i.GetNodesExpanded(), t.GetElapsedTime());
		}
/*
		if (fgreater(fabs(e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance()), 0.01))
		{
			std::cout << "From: " << from << " to " << to << "\n";
			printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), e.GetPathLength(path), s.GetNthExperiment(x).GetDistance(),
				  e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance());
			exit(1);
		}
		*/
	}
	
	exit(0);
}

void RunPolygraph(int instanceSize, char *algorithm, char *K)
{
	std::vector<graphState> path;
	IncrementalBGS<graphState, graphMove>  ibex;
	std::vector<graphMove> path2;
	
	g = GraphInconsistencyExamples::GetPolyGraph(instanceSize);
	ge = new GraphEnvironment(g);
	ge->SetDirected(true);
	
	graphState from, to;
	from = 0;
	to = g->GetNumNodes()-1;
	//astar.SetUseBPMX(BPMX?1:0);
	//astar.InitializeSearch(ge, from, to, path);
	Timer t;

	if (strcmp(algorithm, "IBEX") == 0)
	{
		IBEX::IBEX<graphState, graphMove, GraphEnvironment, false> i(2, 5, 2, false);
		t.StartTimer();
		i.GetPath(ge, from, to, path2);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, i.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "astar") == 0)
	{
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
		astar.SetReopenNodes(true);
		astar.SetHeuristic(&h);
		t.StartTimer();
		astar.GetPath(ge, from, to, path);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, astar.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "bgs") == 0)
	{
		ImprovedBGS2<graphState, graphMove>  bgs;
		t.StartTimer();
		bgs.SetK(std::stoi(K));
		bgs.GetPath(ge, from, to, &h, path);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, bgs.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "ibex") == 0)
	{
		t.StartTimer();
		ibex.GetPath(ge, from, to, &h, path);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, ibex.GetNodesExpanded(), t.GetElapsedTime());
	}
	
}

void runAll(char *theMap, char *scenario)
{
	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	Timer t;
	DH.e = &e;
	for (int x = 0; x < 10; x++)
		DH.Add();
	
	
	double pathlength[24] = {0.0};
	uint64_t nodes[24] = {0};
	double time[24] = {0.0};
  
    Table<std::string, int, std::string> table({"Algorithm", "# node expansions", "time (in secs)"},
                                                            10);
	int n = s.GetNumExperiments();
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetDistance() == 0)
			continue;

		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());
		TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher1;
		searcher1.SetReopenNodes(true);
		searcher1.SetHeuristic(&DH);
	    std::vector<xyLoc> path1;
		t.StartTimer();
		searcher1.GetPath(&e, from, to, path1);
		t.EndTimer();
		//printf("result: %f\t%llu\t%f\n", e.GetPathLength(path1), searcher1.GetNodesExpanded(), t.GetElapsedTime());
		pathlength[0] += e.GetPathLength(path1);
		nodes[0] += searcher1.GetNodesExpanded();
		time[0] += t.GetElapsedTime();
    
	    TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher2;
		searcher2.SetReopenNodes(true);
		searcher2.SetHeuristic(&DH);
		searcher2.SetUseBPMX(1);
	    std::vector<xyLoc> path2;
		t.StartTimer();
		searcher2.GetPath(&e, from, to, path2);
		t.EndTimer();
		//printf("result: %f\t%llu\t%f\n", e.GetPathLength(path2), searcher2.GetNodesExpanded(), t.GetElapsedTime());
		pathlength[1] += e.GetPathLength(path2);
		nodes[1] += searcher2.GetNodesExpanded();
		time[1] += t.GetElapsedTime();

        IBEX::IBEX<xyLoc, tDirection, MapEnvironment, false> i(2, 8, 2, false);
		std::vector<tDirection> pathIbex;
		t.StartTimer();
		i.GetPath(&e, &DH, from, to, pathIbex);
		t.EndTimer();
		//printf("result: %f\t%llu\t%f\n", e.GetPathLength(from, pathIbex), i.GetNodesExpanded(), t.GetElapsedTime());
		pathlength[2] += e.GetPathLength(from, pathIbex);
		nodes[2] += i.GetNodesExpanded();
		time[2] += t.GetElapsedTime();

        for (int p = 0; p <= 9; p++)
		{
			ImprovedBGS2<xyLoc, tDirection>  bgs;
			std::vector<xyLoc> path3;
			t.StartTimer();
			bgs.SetK(p);
			bgs.GetPath(&e, from, to, &DH, path3);
			t.EndTimer();
			//printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
			pathlength[3+2*p] += bgs.GetSolutionCost();
			nodes[3+2*p] += bgs.GetNodesExpanded();
			time[3+2*p] += t.GetElapsedTime();
			if (fgreater(fabs(bgs.GetSolutionCost()-s.GetNthExperiment(x).GetDistance()), 0.01))
		    {
				std::cout << "From: " << from << " to " << to << "\n";
				printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), bgs.GetSolutionCost(), s.GetNthExperiment(x).GetDistance(),
					bgs.GetSolutionCost()-s.GetNthExperiment(x).GetDistance());
				exit(1);
		    }
			ImprovedBGS2<xyLoc, tDirection>  bgs_bpmx;
			std::vector<xyLoc> path4;
			t.StartTimer();
			bgs_bpmx.SetUseBPMX();
			bgs_bpmx.SetK(p);
			bgs_bpmx.GetPath(&e, from, to, &DH, path4);
			t.EndTimer();
			//printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
			pathlength[3+(2*p+1)] += bgs_bpmx.GetSolutionCost();
			nodes[3+(2*p+1)] += bgs_bpmx.GetNodesExpanded();
			time[3+(2*p+1)] += t.GetElapsedTime();
		}
		


		
	}
	table.addRow("A*",  nodes[0]/n, std::to_string(time[0]/n));
	table.addRow("BPMX",  nodes[1]/n, std::to_string(time[1]/n));
	table.addRow("IBEX",  nodes[2]/n, std::to_string(time[2]/n));
	table.addRow("BGS with no re-expansions",  nodes[3]/n, std::to_string(time[3]/n));
	table.addRow("BGS+BPMX with no re-expansions",  nodes[4]/n, std::to_string(time[4]/n));
	table.addRow("BGS with 1*budget",  nodes[5]/n, std::to_string(time[5]/n));
	table.addRow("BGS+BPMX with 1*budget", nodes[6]/n, std::to_string(time[6]/n));
	table.addRow("BGS-2",  nodes[7]/n, std::to_string(time[7]/n));
	table.addRow("BGS-BPMX-2",  nodes[8]/n, std::to_string(time[8]/n));
	table.addRow("BGS-3",  nodes[9]/n, std::to_string(time[9]/n));
	table.addRow("BGS-BPMX-3",  nodes[10]/n, std::to_string(time[10]/n));
	table.addRow("BGS-4",  nodes[11]/n, std::to_string(time[11]/n));
	table.addRow("BGS-BPMX-4",  nodes[12]/n, std::to_string(time[12]/n)); 
	table.addRow("BGS with 5*budget",  nodes[13]/n, std::to_string(time[13]/n));
	table.addRow("BGS+BPMX with 5*budget",  nodes[14]/n, std::to_string(time[14]/n));
	table.addRow("BGS-6",  nodes[15]/n, std::to_string(time[15]/n));
	table.addRow("BGS-BPMX-6", nodes[16]/n, std::to_string(time[16]/n));
	table.addRow("BGS-7", nodes[17]/n, std::to_string(time[17]/n));
	table.addRow("BGS-BPMX-7", nodes[18]/n, std::to_string(time[18]/n));
	table.addRow("BGS-8",  nodes[19]/n, std::to_string(time[19]/n));
	table.addRow("BGS-BPMX-8",  nodes[20]/n, std::to_string(time[20]/n));
	table.addRow("BGS with inf budget", nodes[21]/n, std::to_string(time[21]/n));
	table.addRow("BGS+BPMX with inf budget",  nodes[22]/n, std::to_string(time[22]/n));
	table.print(std::cout);
	exit(0);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-grid") == 0)
	{
		if (maxNumArgs > 3)
		{
			runProblemSet(argument[1], argument[2], argument[3]);
			exit(0);
		}
	}
	if (strcmp(argument[0], "-inconsistent") == 0)
	{
		if (maxNumArgs > 4)
		{
			runProblemSetInconsistent(argument[1], argument[2], argument[3], argument[4]);
			exit(0);
		}
	}
	if (strcmp(argument[0], "-polygraph") == 0)
	{
		if (maxNumArgs >= 4)
		{
			RunPolygraph(atoi(argument[1]), argument[2], argument[3]);
		}
	}


	if (strcmp(argument[0], "-compareall") == 0)
	{
		if (maxNumArgs >= 2)
		{
			runAll(argument[1], argument[2]);
		}
	}
	
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't': break;
		case 'r': recording = !recording; break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			break;
		case '\t':
			break;
		case 'p':
		{}
			break;
		case 'o':
		{}
			break;
		default:
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

