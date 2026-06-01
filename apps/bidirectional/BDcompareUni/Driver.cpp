
#include <iostream>
#include <vector>
#include <time.h>
#include <cstring>

#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"
#include "MM.h"
#include "NERO.h"
#include "WeightedHeuristic.h"

#define SQUARE_ROOT_OF2 1.414213562373

using std::cout;

bool LoadBenchmark(std::vector<int>& group, std::vector<int>& startx, std::vector<int>& starty, std::vector<int>& goalx, std::vector<int>& goaly,
	std::vector<double>& expectedCost, std::string fileName);

void run(std::string fileName, double weight, int teststart, int testend);

Map *map = 0;
MapEnvironment *me = 0;
MapOverlay *mo;
xyLoc start, goal;

std::vector<int> counts;


TemplateAStar<xyLoc, tDirection, MapEnvironment> forward;
TemplateAStar<xyLoc, tDirection, MapEnvironment> backward;

ZeroHeuristic<xyLoc> *z = new ZeroHeuristic<xyLoc>;
WeightedHeuristic<xyLoc> *wh = 0;


MM<xyLoc, tDirection, MapEnvironment> mm;

TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

BDOpenClosed<xyLoc, NEROCompare0<xyLoc>, NEROCompare1<xyLoc>> wfward, wbward;
NERO<xyLoc, tDirection, MapEnvironment> nerocompare(wfward, wbward);


std::vector<xyLoc> path;
std::vector<xyLoc> goalPath;



int main(int argc, char** argv)
{
	if (argc > 1 )
	{
		std::string fileName = argv[1];
		double weight = 1.0;
		if (argc > 2)
			weight = std::atof(argv[2]);

		int start = 1;
		int end = 1000000;
		if (argc > 3)
			start = std::atoi(argv[3]);
		if (argc > 4)
			end = std::atoi(argv[4]);
		if(weight<2)
			run(fileName, weight,start, end);
		else
		{
			for (double w = 0.0; w <= 1; w = w+0.1)
			{
				run(fileName, w,start,end);
			}
		}
	}




	else
	{
		std::cout << "Usage: " << argv[0] << " <filename> [weight] [teststart] [testend]\n";
	}


	return 0;
}





bool LoadBenchmark(std::vector<int>& group, std::vector<int>& startx, std::vector<int>& starty, std::vector<int>& goalx, std::vector<int>& goaly,
	std::vector<double>& expectedCost, std::string fileName)
{
	std::ifstream fin;
	fin.open(fileName);
	if (!fin.is_open())
	{
		std::cout << "fail to load benchmark file: " << fileName << "\n";
		return false;
	}

	startx.resize(0);
	starty.resize(0);
	goalx.resize(0);
	goaly.resize(0);
	expectedCost.resize(0);
	std::string str;
	int grp;
	//get rid of "version 1"
	fin >> str;
	fin >> str;
	while (!fin.eof())
	{
		//first 4 should be "group", "mapname", "map width", "map height"
		//which are not helpful for this assignment
		fin >> str;
		grp = std::stoi(str);
		group.push_back(grp);

		fin >> str;
		fin >> str;
		fin >> str;

		if (fin.eof())
			break;

		fin >> str;
		startx.push_back(std::stoi(str));
		fin >> str;
		starty.push_back(std::stoi(str));
		fin >> str;
		goalx.push_back(std::stoi(str));
		fin >> str;
		goaly.push_back(std::stoi(str));
		fin >> str;
		expectedCost.push_back(std::stod(str));
	}
	return true;
}

void run(std::string fileName, double weight, int teststart, int testend)
{
	std::cout << "file_name: " << fileName << "\n";
	std::cout << "weight: " << weight << "\n";

	std::string mapName = "/home/jingwei/Desktop/Shared/hog2/maps/";
	std::string scenName = "/home/jingwei/Desktop/Shared/hog2/scenarios/";

	mapName = mapName + fileName + ".map";
	scenName = scenName + fileName + ".map.scen";
	map = new Map(mapName.c_str());

	me = new MapEnvironment(map);
	me->SetDiagonalCost(SQUARE_ROOT_OF2);


	wh = new WeightedHeuristic<xyLoc>(me, weight);


	//load the benchmark
	std::vector<int> group, startx, starty, goalx, goaly;
	std::vector<double> expectedCost;
	if (!LoadBenchmark(group, startx, starty, goalx, goaly, expectedCost, scenName.c_str()))
		return ;


	clock_t startTime;
	clock_t endTime;
	clock_t clockTicksTaken;
	double timeInSeconds;

	double astarTime;
	double mmTime;
	double neroTime;

	double solutionCost;

	for (int i = teststart-1; i < std::min(testend,(int)(startx.size())); i++)
	{
		start.x = startx[i];
		start.y = starty[i];
		goal.x = goalx[i];
		goal.y = goaly[i];

		std::cout << "********************************\n"
			<< "test_case " << i + 1 << "\n"
			<< "group_number " << group[i] << "\n"
			<< "start: " << start
			<< " goal: " << goal << "\n";

		std::vector<xyLoc> correctPath;
		startTime = clock();
		astar.SetHeuristic(wh);
		astar.InitializeSearch(me, start, goal, correctPath);
		astar.GetPath(me, start, goal, correctPath);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		astarTime = clockTicksTaken / (double)CLOCKS_PER_SEC;

		
		startTime = clock();
		mm.InitializeSearch(me, start, goal, wh, wh, path);
		mm.GetPath(me, start, goal, wh, wh, path);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		mmTime = clockTicksTaken / (double)CLOCKS_PER_SEC;


		startTime = clock();
		nerocompare.InitializeSearch(me, start, goal, wh, wh, path);
		nerocompare.GetPath(me, start, goal, wh, wh, path);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		neroTime = clockTicksTaken / (double)CLOCKS_PER_SEC;

		if (nerocompare.GetSolutionCost() - expectedCost[i] > 0.01 || nerocompare.GetSolutionCost() - expectedCost[i] < -0.01)
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << nerocompare.GetSolutionCost() << "\t" << expectedCost[i] << "\n";
			double d;
			for (auto x : correctPath)
			{
				astar.GetClosedListGCost(x, d);
				auto t = nerocompare.GetNodeForwardLocation(x);
				auto u = nerocompare.GetNodeBackwardLocation(x);
				std::cout << x << " is on " << t << " and " << u << "\n";
				std::cout << "True g: " << d;
				if (t != kUnseen)
					std::cout << " forward g: " << nerocompare.GetNodeForwardG(x);
				if (u != kUnseen)
					std::cout << " backward g: " << nerocompare.GetNodeBackwardG(x);
				std::cout << "\n";
			}

			return ;
		}

		cout << "nodes:(A*,MM,NERO) \t" << astar.GetNodesExpanded() << "\t"
			<< mm.GetNodesExpanded() << "\t" << nerocompare.GetNodesExpanded() << "\n";
		cout << "time:(A*,MM,NERO) \t" << astarTime << "\t"
			<< mmTime << "\t" << neroTime << "\t" << "\n";
	}
}