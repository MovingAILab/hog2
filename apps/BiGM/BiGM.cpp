#include "Common.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include <string>
#include "ScenarioLoader.h"
#include <dirent.h>
#include "BidirectionalGreedyBestFirst.h"
#include "BitMap.h"
#include <algorithm>
#include "GreedyAnchorSearch.h"
//#include "TAS.h"
#include "TAS2.h"
#include "TASA2.h"
#include "SVGUtil.h"
#include "RAS.h"
#include "RASFast.h"
#include "DNode.h"
#include "TTBS.h"
#include "TASS4.h"
#include "FastTAS.h"


using namespace std;

struct dh {
	std::vector<double> depths;
	xyLoc startLoc;
	xyLoc farthest;
};

class DifferentialHeuristic : public Heuristic<xyLoc> {
public:
	int maxVal;
	double HCost(const xyLoc &a, const xyLoc &b) const
	{
		//return distance(a, b);
		double v = e->HCost(a, b);
		for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[e->GetStateHash(a)]-values[x].depths[e->GetStateHash(b)]));
		histogram[(int)(v*256.0/(maxVal+1))]++;
		return v;
	}
	int maxValue()
	{
		double v = 0;
		for (int x = 0; x < values.size(); x++)
			v = std::max(v, values[x].depths[e->GetStateHash(values[x].farthest)]);
		maxVal = (int)v;
		return (int)v;
	}
	
	MapEnvironment *e;
	std::vector<dh> values;
};

void AddDH(xyLoc start, MapEnvironment* me, DifferentialHeuristic &h)
{
	dh newDH;
	newDH.startLoc = start;
	newDH.depths.resize(me->GetMaxHash());
	std::vector<xyLoc> p;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	search.SetStopAfterGoal(false);
	search.GetPath(me, start, start, p);

	double maxCost = 0;
	
	for (int x = 0; x < search.GetNumItems(); x++)
	{
		double cost;
		xyLoc v = search.GetItem(x).data;
		if (!search.GetClosedListGCost(v, cost))
			printf("Error reading depth from closed list!\n");
		else {
			int hash = me->GetStateHash(v);
			newDH.depths[hash] = cost;
			if (cost > maxCost)
			{
				newDH.farthest = v;
				maxCost = cost;
			}
		}
	}
	
	h.values.push_back(newDH);
	//cout << maxCost << endl;
}

void BuildDH(int count, ScenarioLoader *sl, MapEnvironment* me, DifferentialHeuristic &h)
{
	for (int x = 0; x < 256; x++) h.histogram[x] = 0;
	h.e = me;
	Experiment exp = sl->GetNthExperiment(sl->GetNumExperiments() / 2);
	xyLoc start;
	start.x = exp.GetStartX();
	start.y = exp.GetStartY();
	AddDH(start, me, h);
	start = h.values[0].farthest;
	h.values.clear();
	for (int i = 0; i < count; i++)
	{
		AddDH(start, me, h);
		start = h.values[h.values.size() - 1].farthest;
	}
	cout << "MaxValue: " << h.maxValue() << endl;
}



void Test(const char* mapsDir, const char* path)
{
	DifferentialHeuristic h;
    ScenarioLoader *sl = new ScenarioLoader(path);
    Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	Experiment exp = sl->GetNthExperiment(200);

    xyLoc pivot1, pivot2, s1, s2;
	s1.x = exp.GetStartX();
    s1.y = exp.GetStartY();
	s2.x = exp.GetGoalX();
    s2.y = exp.GetGoalY();
    char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	BuildDH(2, sl, env, h);
	
}

vector<string> GetScenarios(string mapname)
{
	vector<string> res;
	string basePath = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS/scenarios/" + mapname;
	DIR *dr;
   	struct dirent *en;
   	dr = opendir(basePath.c_str()); //open all directory
   	if (dr) {
      	while ((en = readdir(dr)) != NULL) {
         	//cout << en->d_name << endl; //print all directory name
			if (string(en->d_name) != string(".") && string(en->d_name) != string(".."))
				res.push_back((basePath + "/" + en->d_name).c_str());
      	}
      	closedir(dr); //close all directory
   	}
	return res;
}

double phi(double h, double g)
{
    return h;
}

void DrawBitMap(Map *map, BitMapPic &pic, vector<xyLoc> path)
{
	//MapEnvironment *env = new MapEnvironment(map);
    for (size_t x = 0; x < map->GetMapWidth(); x++)
    {
        for (size_t y = 0; y < map->GetMapHeight(); y++)
        {
            //cout << x << " " << y << endl;
            if (map->GetTerrainType(x, y) == kGround)
			{
				pic.SetPixel(x, y, 255, 255, 255, 255);
			}
			else if (map->GetTerrainType(x, y) == kTrees)
			{
				pic.SetPixel(x, y, 0, 255/2, 0, 255);
			}
			else if (map->GetTerrainType(x, y) == kWater)
			{
				pic.SetPixel(x, y, 0, 0, 255, 255);
			}
			else if (map->GetTerrainType(x, y) == kSwamp)
			{
				pic.SetPixel(x, y, 0, 255 * 0.3, 255, 255);
			}
			else {
				pic.SetPixel(x, y, 0, 0, 0, 255);
			}
        }
    }
	for (int i = 0; i < path.size(); i++)
	{
		pic.SetPixel(path[i].x, path[i].y, 0, 0, 255, 255);
	}
	//std::string picPath = "./map.png";
    //pic.Save(picPath.c_str());
}


void BGBFS(string mapName, int scenarioIndex, int problemIndex, int problemCount, int numOfPivots, string outputdir)
{
	cout << "BGBFS" << endl;
	cout << "pivots: " << numOfPivots << endl;
 	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
	DifferentialHeuristic h;
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	BuildDH(numOfPivots, sl, env, h);
	ofstream myfile;
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
	string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + "(BGBFS)" + "_p" + to_string(numOfPivots) + ".txt";
  	myfile.open (s);
	cout << s << endl;
	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		if (i >= sl->GetNumExperiments())
			break;
		for (int x = 0; x < 256; x++) env->histogram[x] = 0;
		for (int x = 0; x < 256; x++) h.histogram[x] = 0;
  		//myfile << "Writing this to a file.\n";
		myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		//cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		start.x = exp.GetStartX();
    	start.y = exp.GetStartY();
		goal.x = exp.GetGoalX();
    	goal.y = exp.GetGoalY();
		BidirectionalGreedyBestFirst<xyLoc, tDirection, MapEnvironment> astar;
    	astar.SetPhi(phi);
    	astar.SetHeuristic(&h);
		std::vector<xyLoc> fPath, bPath;
    	Timer timer;
    	timer.StartTimer();
    	astar.GetPath(env, start, goal, fPath, bPath);
    	timer.EndTimer();
		//cout << i << endl;
    	//myfile << "Elapsed: " << timer.GetElapsedTime() << endl;
    	//myfile << "Expansions: " << astar.GetNodesExpanded() << endl;
    	//myfile << "Length: " << env->GetPathLength(fPath) + env->GetPathLength(bPath) << endl;
		//myfile << "Shortest path: " << exp.GetDistance() << endl;
		for (int i = 0; i < 256; i++)
			myfile << h.histogram[i] << " ";
		myfile << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}
	myfile.close();
	delete map;
	delete env;
	delete sl;
}

void GBFS(string mapName, int scenarioIndex, int problemIndex, int problemCount, int numOfPivots, string outputdir)
{
	cout << "GBFS" << endl;
	cout << "pivots: " << numOfPivots << endl;
	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
	DifferentialHeuristic h;
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	BuildDH(numOfPivots, sl, env, h);
	ofstream myfile;
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
  	string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + "(GBFS)" + "_p" + to_string(numOfPivots) + ".txt";
  	myfile.open (s);
	cout << s << endl;
	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		if (i >= sl->GetNumExperiments())
			break;
		for (int x = 0; x < 256; x++) env->histogram[x] = 0;
		for (int x = 0; x < 256; x++) h.histogram[x] = 0;
  		//myfile << "Writing this to a file.\n";
		myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		start.x = exp.GetStartX();
    	start.y = exp.GetStartY();
		goal.x = exp.GetGoalX();
    	goal.y = exp.GetGoalY();
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
    	astar.SetPhi(phi);
    	astar.SetHeuristic(&h);
		std::vector<xyLoc> thePath;
    	Timer timer;
    	timer.StartTimer();
    	astar.GetPath(env, start, goal, thePath);
    	timer.EndTimer();
		//cout << i << endl;
    	//myfile << "Elapsed: " << timer.GetElapsedTime() << endl;
    	//myfile << "Expansions: " << astar.GetNodesExpanded() << endl;
    	//myfile << "Length: " << env->GetPathLength(thePath) << endl;
		//myfile << "Shortest path: " << exp.GetDistance() << endl;
		for (int i = 0; i < 256; i++)
			myfile << h.histogram[i] << " ";
		myfile << endl;
		//cout << h.histogram << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}
	myfile.close();
	delete map;
	delete env;
	delete sl;
}

void BackGBFS(string mapName, int scenarioIndex, int problemIndex, int problemCount, int numOfPivots, string outputdir)
{
	cout << "GBFS" << endl;
	cout << "pivots: " << numOfPivots << endl;
	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
	DifferentialHeuristic h;
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	BuildDH(numOfPivots, sl, env, h);
	ofstream myfile;
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
  	string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + "(GBFS_B)" + "_p" + to_string(numOfPivots) + ".txt";
  	myfile.open (s);
	cout << s << endl;
	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		
		if (i >= sl->GetNumExperiments())
			break;
		for (int x = 0; x < 256; x++) env->histogram[x] = 0;
		for (int x = 0; x < 256; x++) h.histogram[x] = 0;
  		//myfile << "Writing this to a file.\n";
		myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		goal.x = exp.GetStartX();
    	goal.y = exp.GetStartY();
		start.x = exp.GetGoalX();
    	start.y = exp.GetGoalY();
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
    	astar.SetPhi(phi);
    	astar.SetHeuristic(&h);
		std::vector<xyLoc> thePath;
    	Timer timer;
    	timer.StartTimer();
    	astar.GetPath(env, start, goal, thePath);
    	timer.EndTimer();
		//cout << i << endl;
    	//myfile << "Elapsed: " << timer.GetElapsedTime() << endl;
    	//myfile << "Expansions: " << astar.GetNodesExpanded() << endl;
    	//myfile << "Length: " << env->GetPathLength(thePath) << endl;
		//myfile << "Shortest path: " << exp.GetDistance() << endl;
		for (int i = 0; i < 256; i++)
			myfile << h.histogram[i] << " ";
		myfile << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}
	myfile.close();
	delete map;
	delete env;
	delete sl;
}


void DrawTest(string scenarioPath, int problemIndex, int isRAS = 1)
{
	cout << "GAS" << endl;
	//vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
	const char* scenariosDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2/scenarios";
	string sp;
	sp = string(scenariosDir) + "/" + scenarioPath;
	cout << sp << endl;
    ScenarioLoader *sl = new ScenarioLoader(sp.c_str());
	Experiment last = sl->GetNthExperiment(0);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	string mn = last.GetMapName();
	double bgbfs_time = 0;

	cout << "---------- " << last.GetMapName() << " ---------- problem #" << problemIndex << endl;
	Experiment exp = sl->GetNthExperiment(problemIndex);
    xyLoc start, goal;
	start.x = exp.GetStartX();
    start.y = exp.GetStartY();
	goal.x = exp.GetGoalX();
    goal.y = exp.GetGoalY();
	cout << start << " " << goal << " " << exp.GetXScale() << " " << exp.GetYScale() << endl; 
	TAS<MapEnvironment, xyLoc> gas(env, start, goal, env, env, 10);
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
    astar.SetPhi(phi);
	std::vector<xyLoc> thePath;
	
    Timer t1;
    t1.StartTimer();
	if (isRAS)
	{	
		gas.GetPath(thePath);
		std::cout << gas.GetNodesExpanded() << std::endl;
	}
	else
	{
		astar.GetPath(env, start, goal, thePath);
		std::cout << astar.GetNodesExpanded() << std::endl;
	}
	t1.EndTimer();

	std::cout << t1.GetElapsedTime() << std::endl;	

	std::string s;
    s += env->SVGHeader();
    s += env->SVGDraw();
    xyLoc pos;
	if (isRAS)
	{
		for (int i = 0; i < gas.ff->loc.size(); i++)
		{
			if (gas.ff->loc[i].index == -1)
				continue;
			xyLoc pos;
			env->GetStateFromHash(i, pos);
			s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.0, 0.0});
			//s += SVGDrawRect(pos.x+1, pos.y+1, 1, 1, {1.0, 0.0, 0.0});
		}
		for (int i = 0; i < gas.bf->loc.size(); i++)
		{
			if (gas.bf->loc[i].index == -1)
				continue;
			xyLoc pos;
			env->GetStateFromHash(i, pos);
			s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.0, 0.0});
			//s += SVGDrawRect(pos.x+1, pos.y+1, 1, 1, {1.0, 0.0, 0.0});
		}
	}
	else
	{
		for (uint64_t i = 0; i < astar.openClosedList.size(); i++)
		{
			xyLoc pos = astar.openClosedList.Lookup(i).data;
			s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.0, 0.0});
			//s += SVGDrawRect(pos.x+1, pos.y+1, 1, 1, {1.0, 0.0, 0.0});
		}
	}
	for (int i = 0; i < thePath.size() -1; i++)
    {
        //s += SVGDrawRect(pos.x + 1, pos.y + 1, 1, 1, {1.0, 0.0, 0.0});
		auto p1 = thePath[i];
		auto p2 = thePath[i+1];
		s += SVGDrawLine(p1.x+1, p1.y+1, p2.x+1, p2.y+1, 1, {51.0/256.0, 204.0/256.0, 98.0/256.0});
    }
    s += "\n</svg>";
    auto svgPath = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/res.svg";
    FILE *f = fopen("/home/sepehr3/projects/def-nathanst-ab/sepehr3/res.svg", "w+");
	fputs(s.c_str(), f);
	fclose(f);

	delete map;
	delete env;
	delete sl;
}


void GASTest(string scenarioPath, int problemIndex, int problemCount, int samples)
{
	cout << "GAS" << endl;
	//vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
	const char* scenariosDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2/scenarios";
	string sp;
	sp = string(scenariosDir) + "/" + scenarioPath;
	cout << sp << endl;
    ScenarioLoader *sl = new ScenarioLoader(sp.c_str());
	Experiment last = sl->GetNthExperiment(0);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	//return;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	//ofstream myfile;
	string mn = last.GetMapName();
	//replace(mn.begin(), mn.end(), '/', '_');
  	//string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + "(GAS)" + "_p" + to_string(numOfPivots) + ".txt";
  	//myfile.open (s);
	//cout << s << endl;
	double gas_sum = 0;
	double gbfs_sum = 0;
	double bgbfs_sum = 0;

	double gas_time = 0;
	double gbfs_time = 0;
	double bgbfs_time = 0;


	double tas_sum = 0;
	double dnode20_sum = 0;
	double dnode40_sum = 0;
	double dnode100_sum = 0;

	double tas_time = 0;
	double dnode20_time = 0;
	double dnode40_time = 0;
	double dnode100_time = 0;

	int count = 0;

	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		if (i >= sl->GetNumExperiments())
			break;
		count++;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		start.x = exp.GetStartX();
    	start.y = exp.GetStartY();
		goal.x = exp.GetGoalX();
    	goal.y = exp.GetGoalY();
		//cout << start << " " << goal << " " << exp.GetXScale() << " " << exp.GetYScale() << endl; 
		TAS<MapEnvironment, xyLoc> gas(env, start, goal, env, env, samples);
		DNode<MapEnvironment, xyLoc> dnode20(env, start, goal, env, env, 20);
		DNode<MapEnvironment, xyLoc> dnode40(env, start, goal, env, env, 40);
		DNode<MapEnvironment, xyLoc> dnode100(env, start, goal, env, env, 100);
		BidirectionalGreedyBestFirst<xyLoc, tDirection, MapEnvironment> bgbfs;
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		bgbfs.SetPhi(phi);
    	astar.SetPhi(phi);
		std::vector<xyLoc> thePath, fPath, bPath, aPath;
		std::vector<xyLoc> thePath1, thePath2, thePath3;
		
		/*
		bgbfs.GetPath(env, start, goal, fPath, bPath);
		BitMapPic pic1(map->GetMapWidth(), map->GetMapHeight());
		DrawBitMap(map, pic1, aPath);
		for (unsigned int x = 0; x < bgbfs.f.openClosedList.size(); x++)
		{
			const auto &data = bgbfs.f.openClosedList.Lookat(x);
			if (data.where == kOpenList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 0, 255, 0, 255);
			}
			else if (data.where == kClosedList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 255, 0, 0, 255);
			}
		}
		for (unsigned int x = 0; x < bgbfs.b.openClosedList.size(); x++)
		{
			const auto &data = bgbfs.b.openClosedList.Lookat(x);
			if (data.where == kOpenList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 0, 255, 0, 255);
			}
			else if (data.where == kClosedList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 255, 0, 0, 255);
			}
		}
		for (int i = 0; i < fPath.size(); i++)
		{
			auto n = fPath[i];
			pic1.SetPixel(n.x, n.y, 0, 0, 255, 255);
		}
		for (int i = 0; i < bPath.size(); i++)
		{
			auto n = bPath[i];
			pic1.SetPixel(n.x, n.y, 0, 0, 255, 255);
		}
		pic1.SetPixel(start.x, start.y, 0, 255, 0, 255);
		pic1.SetPixel(goal.x, goal.y, 0, 255, 0, 255);
		cout << "BGBFS: " << bgbfs.GetNodesExpanded() << endl;
		pic1.Save("/home/sepehr3/projects/def-nathanst-ab/sepehr3/snapshots/frame_final1.png");
		*/


		/*
		if (!astar.InitializeSearch(env, start, goal, aPath))
  		{	
  			return;
  		}
  		while (!astar.DoSingleSearchStep(aPath))
		{
			
		}
		BitMapPic pic1(map->GetMapWidth(), map->GetMapHeight());
		DrawBitMap(map, pic1, aPath);
		for (unsigned int x = 0; x < astar.openClosedList.size(); x++)
		{
			const auto &data = astar.openClosedList.Lookat(x);
			if (data.where == kOpenList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 0, 255, 0, 255);
			}
			else if (data.where == kClosedList)
			{
				pic1.SetPixel(data.data.x, data.data.y, 255, 0, 0, 255);
			}
		}
		for (int i = 0; i < aPath.size(); i++)
		{
			auto n = aPath[i];
			pic1.SetPixel(n.x, n.y, 0, 0, 255, 255);
		}
		pic1.SetPixel(start.x, start.y, 0, 255, 0, 255);
		pic1.SetPixel(goal.x, goal.y, 0, 255, 0, 255);
		cout << "GBFS: " << astar.GetNodesExpanded() << endl;
		pic1.Save("/home/sepehr3/projects/def-nathanst-ab/sepehr3/snapshots/frame_final1.png");
		*/
		//gas.GetPath(thePath);
		//cout << gas.ff->env->GetPathLength(thePath) << endl;
		//cout << "GAS: " << gas.GetNodesExpanded() << endl;

		/*
		int step = 0;
		while (true)
		{
			if (gas.DoSingleSearchStep())
				break;
			step++;
			//if (step++ % 100 != 0)
			//	continue;
			
			BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
			DrawBitMap(map, pic, fPath);
			for (auto state : gas.ff->open)
			{
				pic.SetPixel(state.x, state.y, 0, 255, 0, 255);
			}
			for (auto state : gas.bf->open)
			{
				pic.SetPixel(state.x, state.y, 0, 255, 0, 255);
			}
			//for (auto state : gas.ff->closed)
			//{
			//	pic.SetPixel(state.x, state.y, 255, 0, 0, 255);
			//}
			//for (auto state : gas.bf->closed)
			//{
			//	pic.SetPixel(state.x, state.y, 255, 0, 0, 255);
			//}
			pic.SetPixel(start.x, start.y, 0, 0, 255, 255);
			pic.SetPixel(goal.x, goal.y, 0, 0, 255, 255);

			pic.SetPixel(gas.ff->anchor.x, gas.ff->anchor.y, 0, 0, 255, 255);
			pic.SetPixel(gas.bf->anchor.x, gas.bf->anchor.y, 0, 0, 255, 255);

			pic.SetPixel(gas.ff->anchor.x + 1, gas.ff->anchor.y, 0, 0, 255, 255);
			pic.SetPixel(gas.bf->anchor.x + 1, gas.bf->anchor.y, 0, 0, 255, 255);

			pic.SetPixel(gas.ff->anchor.x - 1, gas.ff->anchor.y, 0, 0, 255, 255);
			pic.SetPixel(gas.bf->anchor.x - 1, gas.bf->anchor.y, 0, 0, 255, 255);

			pic.SetPixel(gas.ff->anchor.x, gas.ff->anchor.y + 1, 0, 0, 255, 255);
			pic.SetPixel(gas.bf->anchor.x, gas.bf->anchor.y + 1, 0, 0, 255, 255);

			pic.SetPixel(gas.ff->anchor.x, gas.ff->anchor.y - 1, 0, 0, 255, 255);
			pic.SetPixel(gas.bf->anchor.x, gas.bf->anchor.y - 1, 0, 0, 255, 255);
			pic.Save(("/home/sepehr3/projects/def-nathanst-ab/sepehr3/snapshots/frame_" + to_string(step) + ".png").c_str());
			
		}
		
		BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		DrawBitMap(map, pic, fPath);
		for (auto state : gas.ff->open)
		{
			pic.SetPixel(state.x, state.y, 0, 255, 0, 255);
		}
		for (auto state : gas.bf->open)
		{
			pic.SetPixel(state.x, state.y, 0, 255, 0, 255);
		}
		for (auto hash : gas.ff->closed)
		{
			int y = hash / map->GetMapWidth();
			int x = hash % map->GetMapWidth();
			pic.SetPixel(x, y, 255, 0, 0, 255);
		}
		for (auto hash : gas.bf->closed)
		{
			int y = hash / map->GetMapWidth();
			int x = hash % map->GetMapWidth();
			pic.SetPixel(x, y, 255, 0, 0, 255);
		}
		gas.ExtractPath(thePath);
		double max = thePath.size();
		for (int i = 0; i < max; i++)
		{
			auto n = thePath[i];
			pic.SetPixel(n.x, n.y, 0, 0, 255, 255);
		}
		pic.SetPixel(start.x, start.y, 0, 255, 0, 255);
		pic.SetPixel(goal.x, goal.y, 0, 255, 0, 255);
		pic.Save("/home/sepehr3/projects/def-nathanst-ab/sepehr3/snapshots/frame_final.png");

		cout << gas.ff->env->GetPathLength(thePath) << endl;
		cout << "GAS: " << gas.GetNodesExpanded() << endl;
		*/

		
    	Timer t1, t2, t3, t4, t5, t6;
    	t1.StartTimer();
    	gas.GetPath(thePath);
		t1.EndTimer();
		t4.StartTimer();
    	dnode20.GetPath(thePath1);
		t4.EndTimer();
		t5.StartTimer();
    	dnode40.GetPath(thePath2);
		t5.EndTimer();
		t6.StartTimer();
    	dnode100.GetPath(thePath3);
		t6.EndTimer();

		//t2.StartTimer();
		//bgbfs.GetPath(env, start, goal, fPath, bPath);
		//t2.EndTimer();
		//t3.StartTimer();
		//astar.GetPath(env, start, goal, aPath);
		//t3.EndTimer();

		cout << "GAS:   " << gas.GetNodesExpanded() << " " << env->GetPathLength(thePath) << " " << t1.GetElapsedTime() << endl;
		cout << "DNode(20):   " << dnode20.GetNodesExpanded() << " " << env->GetPathLength(thePath1) << " " << t4.GetElapsedTime() << endl;
		cout << "DNode(40):   " << dnode40.GetNodesExpanded() << " " << env->GetPathLength(thePath2) << " " << t5.GetElapsedTime() << endl;
		cout << "DNode(100):   " << dnode100.GetNodesExpanded() << " " << env->GetPathLength(thePath3) << " " << t6.GetElapsedTime() << endl;
		//cout << "BGBFS: " << bgbfs.GetNodesExpanded() << " " << env->GetPathLength(fPath) + env->GetPathLength(bPath) << " " << t2.GetElapsedTime() << endl;
		//cout << "GBFS:  " << astar.GetNodesExpanded() << " " << env->GetPathLength(aPath) << " " << t3.GetElapsedTime() << endl;
		tas_sum += gas.GetNodesExpanded();
		dnode20_sum += dnode20.GetNodesExpanded();
		dnode40_sum += dnode40.GetNodesExpanded();
		dnode100_sum += dnode100.GetNodesExpanded();

		tas_time += t1.GetElapsedTime();
		dnode20_time += t4.GetElapsedTime();
		dnode40_time += t5.GetElapsedTime();
		dnode100_time += t6.GetElapsedTime();
		/*
		BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		cout << map->GetMapWidth() << " " <<  map->GetMapHeight() << endl;
		DrawBitMap(map, pic, fPath);
		pic.SetPixel(start.x, start.y, 0, 255, 0, 255);
		pic.SetPixel(goal.x, goal.y, 0, 255, 0, 255);
		pic.Save("/home/sepehr3/projects/def-nathanst-ab/sepehr3/snapshots/f.png");

		

		gas_sum += gas.GetNodesExpanded();
		gbfs_sum += astar.GetNodesExpanded();
		bgbfs_sum += bgbfs.GetNodesExpanded();

		gas_time += t1.GetElapsedTime();
		bgbfs_time += t2.GetElapsedTime();
		gbfs_time +=  t3.GetElapsedTime();
		*/
		//cout << i << endl;
    	//myfile << "Elapsed: " << timer.GetElapsedTime() << endl;
    	//myfile << "Expansions: " << astar.GetNodesExpanded() << endl;
    	//myfile << "Length: " << env->GetPathLength(thePath) << endl;
		//myfile << "Shortest path: " << exp.GetDistance() << endl;
		//cout << h.histogram << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}

	cout << "TAS: " << tas_sum / count << " " << tas_time / count << endl;
	cout << "DNode(20): " << dnode20_sum / count << " " << dnode20_time / count << endl;
	cout << "DNode(40): " << dnode40_sum / count << " " << dnode40_time / count << endl;
	cout << "DNode(100): " << dnode100_sum / count << " " << dnode100_time / count << endl;

	//cout << endl;
	//cout << "GAS (exp): " << gas_sum / sl->GetNumExperiments() << endl;
	//cout << "GBFS (exp): " << gbfs_sum / sl->GetNumExperiments() << endl;
	//cout << "BGBFS (exp): " << bgbfs_sum / sl->GetNumExperiments() << endl;
//
	//cout << endl;
	//cout << "GAS (time): " << gas_time / sl->GetNumExperiments() << endl;
	//cout << "GBFS (time): " << gbfs_time / sl->GetNumExperiments() << endl;
	//cout << "BGBFS (time): " << bgbfs_time / sl->GetNumExperiments() << endl;
	//myfile.close();
	delete map;
	delete env;
	delete sl;
}

void ExstensiveTest(string mapName, int scenarioIndex, int problemIndex, int problemCount, string outputdir)
{
	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	ofstream myfile;
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
  	string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + ".txt";
  	myfile.open (s);
	cout << s << endl;
	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		
		if (i >= sl->GetNumExperiments())
			break;
  		//myfile << "Writing this to a file.\n";
		myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		goal.x = exp.GetStartX();
    	goal.y = exp.GetStartY();
		start.x = exp.GetGoalX();
    	start.y = exp.GetGoalY();
		//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		//TAS<MapEnvironment, xyLoc> gas1(env, start, goal, env, env, 1);
		//RASFast<MapEnvironment, xyLoc> gas5(env, start, goal, env, env, 5);
		DNode<MapEnvironment, xyLoc> dnode10(env, start, goal, env, env, 10);
		DNode<MapEnvironment, xyLoc> dnode20(env, start, goal, env, env, 20);
		DNode<MapEnvironment, xyLoc> dnode50(env, start, goal, env, env, 50);
		DNode<MapEnvironment, xyLoc> dnode100(env, start, goal, env, env, 100);
		//RASFast<MapEnvironment, xyLoc> gas10(env, start, goal, env, env, 10);
		//BidirectionalGreedyBestFirst<xyLoc, tDirection, MapEnvironment> bgbfs;
    	//astar.SetPhi(phi);
		//bgbfs.SetPhi(phi);
		std::vector<xyLoc> thePath5, thePath10, aPath, fPath, bPath;
		std::vector<xyLoc> thePath1, thePath2, thePath3, thePath4;
    	Timer t1, t2, t3, t4;

		t1.StartTimer();
    	dnode10.GetPath(thePath1);
    	t1.EndTimer();
		t2.StartTimer();
    	dnode20.GetPath(thePath2);
    	t2.EndTimer();
		t3.StartTimer();
    	dnode50.GetPath(thePath3);
    	t3.EndTimer();
		t4.StartTimer();
    	dnode100.GetPath(thePath4);
    	t4.EndTimer();
    	//t1.StartTimer();
    	//astar.GetPath(env, start, goal, aPath);
    	//t1.EndTimer();
		//t2.StartTimer();
    	//bgbfs.GetPath(env, start, goal, fPath, bPath);
    	//t2.EndTimer();
		//t3.StartTimer();
    	//gas5.GetPath(thePath5);
    	//t3.EndTimer();
		//t4.StartTimer();
    	//gas10.GetPath(thePath10);
    	//t4.EndTimer();
		//cout << i << endl;
		//myfile << "GAS5: " << "Expansions: " << gas5.GetNodesExpanded() << " Time: " << t3.GetElapsedTime() << " Length: " << env->GetPathLength(thePath5) << endl;
		myfile << "DNode(10): " << "Expansions: " << dnode10.GetNodesExpanded() << " Time: " << t1.GetElapsedTime() << " Length: " << env->GetPathLength(thePath1) << endl;
		myfile << "DNode(20): " << "Expansions: " << dnode20.GetNodesExpanded() << " Time: " << t2.GetElapsedTime() << " Length: " << env->GetPathLength(thePath2) << endl;
		myfile << "DNode(50): " << "Expansions: " << dnode50.GetNodesExpanded() << " Time: " << t3.GetElapsedTime() << " Length: " << env->GetPathLength(thePath3) << endl;
		myfile << "DNode(100): " << "Expansions: " << dnode100.GetNodesExpanded() << " Time: " << t4.GetElapsedTime() << " Length: " << env->GetPathLength(thePath4) << endl;
		//myfile << "GAS10: " << "Expansions: " << gas10.GetNodesExpanded() << " Time: " << t4.GetElapsedTime() << " Length: " << env->GetPathLength(thePath10) << endl;
		//myfile << "GBFS: " << "Expansions: " << astar.GetNodesExpanded() << " Time: " << t1.GetElapsedTime() << " Length: " << env->GetPathLength(aPath) << endl;
		//myfile << "BGBFS: " << "Expansions: " << bgbfs.GetNodesExpanded() << " Time: " << t2.GetElapsedTime() << " Length: " << env->GetPathLength(fPath) + env->GetPathLength(bPath) << endl;

		myfile << "Shortest path: " << exp.GetDistance() << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}
	myfile.close();
	delete map;
	delete env;
	delete sl;
}


void DNodeTest(string scenarioPath, int problemIndex)
{
	cout << "GAS" << endl;
	//vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS";
	const char* scenariosDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS/scenarios";
	string sp;
	sp = string(scenariosDir) + "/" + scenarioPath;
	cout << sp << endl;
    ScenarioLoader *sl = new ScenarioLoader(sp.c_str());
	Experiment last = sl->GetNthExperiment(0);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	string mn = last.GetMapName();
	double bgbfs_time = 0;

	cout << "---------- " << last.GetMapName() << " ---------- problem #" << problemIndex << endl;
	Experiment exp = sl->GetNthExperiment(problemIndex);
    xyLoc start, goal;
	start.x = exp.GetStartX();
    start.y = exp.GetStartY();
	goal.x = exp.GetGoalX();
    goal.y = exp.GetGoalY();
	cout << start << " " << goal << " " << exp.GetXScale() << " " << exp.GetYScale() << endl; 
	TTBS<MapEnvironment, xyLoc> gas(env, start, goal, env, env);
    TAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
	std::vector<xyLoc> thePath, thePath2;
	
    Timer t1, t2;
    t1.StartTimer();
    gas.GetPath(thePath);
	t1.EndTimer();
    t2.StartTimer();
    tas.GetPath(thePath2);
    t2.EndTimer();
	std::cout << "Done!" << std::endl;
	std::string s;
    s += env->SVGHeader();
    s += env->SVGDraw();
    xyLoc pos;
	
    for (auto item : gas.ff->gValues)
    {
        xyLoc pos;
		env->GetStateFromHash(item.first, pos);
		s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.0, 0.5});
    }
    for (auto item : gas.bf->gValues)
    {
        xyLoc pos;
		env->GetStateFromHash(item.first, pos);
		s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.5, 0.0});
    }
    for (auto hash : gas.ff->closed)
    {
        xyLoc pos;
		env->GetStateFromHash(hash, pos);
		s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.0, 0.5});
    }
    for (auto hash : gas.bf->closed)
    {
        xyLoc pos;
		env->GetStateFromHash(hash, pos);
		s += SVGDrawCircle(pos.x+0.5+1, pos.y+0.5+1, 0.5, {1.0, 0.5, 0.0});
    }
	
	//std::cout<< "Flag 1" << std::endl;
	for (int i = 0; i < thePath.size() -1; i++)
    {
        //s += SVGDrawRect(pos.x + 1, pos.y + 1, 1, 1, {1.0, 0.0, 0.0});
		auto p1 = thePath[i];
		auto p2 = thePath[i+1];
		s += SVGDrawLine(p1.x+1, p1.y+1, p2.x+1, p2.y+1, 1, {0.0, 0.0, 1.0});
    }
	
	std::cout << "TAS: " << tas.GetNodesExpanded() << std::endl;
    std::cout << "TTBS: " << gas.GetNodesExpanded() << std::endl;
    s += "\n</svg>";
    auto svgPath = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/SVGs/res.svg";
    FILE *f = fopen("/home/sepehr3/projects/def-nathanst-ab/sepehr3/SVGs/res.svg", "w+");
	fputs(s.c_str(), f);
	fclose(f);

	delete map;
	delete env;
	delete sl;
}


void QuickTest(string scenarioPath, int anchorSelection)
{
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS";
	const char* scenariosDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS/scenarios";
	string sp;
	sp = string(scenariosDir) + "/" + scenarioPath;
	cout << sp << endl;
    ScenarioLoader *sl = new ScenarioLoader(sp.c_str());
	Experiment last = sl->GetNthExperiment(0);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	string mn = last.GetMapName();


    double avg1, avg2, avg3;
	double t1, t2, t3;
	avg1 = 0;
	avg2 = 0;
	avg3 = 0;
	t1 = 0;
	t2 = 0;
	t3 = 0;
    int count = 0;

    for (int i = 0; i < sl->GetNumExperiments(); i++)
    {
	    cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
	    Experiment exp = sl->GetNthExperiment(i);
        xyLoc start, goal;
	    start.x = exp.GetStartX();
        start.y = exp.GetStartY();
	    goal.x = exp.GetGoalX();
        goal.y = exp.GetGoalY();
	    cout << start << " " << goal << " " << exp.GetXScale() << " " << exp.GetYScale() << endl; 
	    TTBS<MapEnvironment, xyLoc> ttbs(env, start, goal, env, env);
        FastTAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
		TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc>> astar;
		astar.SetPhi(phi);
		//TASS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
		tas.SetAnchorSelection((kAnchorSelection)anchorSelection);
	    std::vector<xyLoc> thePath, thePath2, thePath3;
        Timer timer1, timer2, timer3;
        timer1.StartTimer();
        ttbs.GetPath(thePath);
	    timer1.EndTimer();
        timer2.StartTimer();
        tas.GetPath(thePath2);
        timer2.EndTimer();
		timer3.StartTimer();
		astar.GetPath(env, start, goal, thePath3);
		timer3.EndTimer();
		cout << "TTBS: " << ttbs.GetNodesExpanded() << " TAS: " << tas.GetNodesExpanded() << " GBFS: " << astar.GetNodesExpanded() << endl;

        avg1 += ttbs.GetNodesExpanded();
        avg2 += tas.GetNodesExpanded();
		avg3 += astar.GetNodesExpanded();
		t1 += timer1.GetElapsedTime();
		t2 += timer2.GetElapsedTime();
		t3 += timer3.GetElapsedTime();
        count++;

		delete tas.ff;
		delete tas.bf;
    }

	std::cout << "TTBS: " << avg1/count << " " << t1/count << std::endl;
	std::cout << "TAS: " << avg2/count << " " << t2/count << std::endl;
	std::cout << "GBFS: " << avg3/count << " " << t3/count << std::endl;
    

	delete map;
	delete env;
	delete sl;
}


void QuickTest2(string mapName, int scenarioIndex, int problemIndex, int problemCount, string outputdir, string alg)
{
	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2_AS";
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	ofstream myfile;
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
  	string s = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/GMResults/" + outputdir + "/" + mn + ".txt";
  	myfile.open (s);

    double avg,t;
	avg = 0;
	t = 0;
    int count = 0;

    for (int i = problemIndex; i < min(sl->GetNumExperiments(), problemIndex + problemCount); i++)
    {
	    myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
	    Experiment exp = sl->GetNthExperiment(i);
        xyLoc start, goal;
	    start.x = exp.GetStartX();
        start.y = exp.GetStartY();
	    goal.x = exp.GetGoalX();
        goal.y = exp.GetGoalY();
	    myfile << start << " " << goal << " " << exp.GetXScale() << " " << exp.GetYScale() << endl; 

		std::vector<xyLoc> thePath;
		thePath.clear();
		Timer timer;
		if (alg == "tas-t")
		{
			FastTAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
			tas.SetAnchorSelection(Temporal);
			timer.StartTimer();
			tas.GetPath(thePath);
			timer.EndTimer();
			myfile << "TAS-T: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << tas.pathRatio << endl;
			avg += tas.GetNodesExpanded();
			t += timer.GetElapsedTime();
			delete tas.ff;
			delete tas.bf;
		}
		else if (alg == "tas-a")
		{
			FastTAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
			tas.SetAnchorSelection(Anchor);
			timer.StartTimer();
			tas.GetPath(thePath);
			timer.EndTimer();
			myfile << "TAS-A: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << tas.pathRatio << endl;
			avg += tas.GetNodesExpanded();
			t += timer.GetElapsedTime();
			delete tas.ff;
			delete tas.bf;
		}
		else if (alg == "tas-af")
		{
			FastTAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
			tas.SetAnchorSelection(Anchor, Fixed);
			timer.StartTimer();
			tas.GetPath(thePath);
			timer.EndTimer();
			myfile << "TAS-AF: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << tas.pathRatio << endl;
			avg += tas.GetNodesExpanded();
			t += timer.GetElapsedTime();
			delete tas.ff;
			delete tas.bf;
		}
		else if (alg == "gbfs")
		{
			TemplateAStar<xyLoc, tDirection, MapEnvironment, IndexOpenClosed<xyLoc>> astar;
			astar.SetPhi(phi);
			timer.StartTimer();
			astar.GetPath(env, start, goal, thePath);
			timer.EndTimer();
			avg += astar.GetNodesExpanded();
			t += timer.GetElapsedTime();
			myfile << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << endl;
		}
		else if (alg == "ttbs-lifo")
		{
			TTBS<MapEnvironment, xyLoc> ttbs(env, start, goal, env, env, 1);
			timer.StartTimer();
        	ttbs.GetPath(thePath);
	    	timer.EndTimer();
			avg += ttbs.GetNodesExpanded();
			t += timer.GetElapsedTime();
			myfile << "TTBS: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << ttbs.pathRatio << endl;
			ttbs.Clear();
		}
		else if (alg == "ttbs-fifo")
		{
			TTBS<MapEnvironment, xyLoc> ttbs(env, start, goal, env, env, 0);
			timer.StartTimer();
        	ttbs.GetPath(thePath);
	    	timer.EndTimer();
			avg += ttbs.GetNodesExpanded();
			t += timer.GetElapsedTime();
			myfile << "TTBS: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << ttbs.pathRatio << endl;
			ttbs.Clear();
		}
		else if (alg == "dnr")
		{
			DNode<MapEnvironment, xyLoc> dnode(env, start, goal, env, env, 100);
			timer.StartTimer();
        	dnode.GetPath(thePath);
	    	timer.EndTimer();
			avg += dnode.GetNodesExpanded();
			t += timer.GetElapsedTime();
			myfile << "DNR: " << dnode.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << env->GetPathLength(thePath) << " " << dnode.pathRatio << endl;
			dnode.Clear();
		}
		count++;
    }
	myfile << "***************************************" << endl;
	myfile << avg/(double)count << " " << t/(double)count << endl;
	myfile.close();
	delete map;
	delete env;
	delete sl;
}

void TASATest(string mapName, int scenarioIndex, int problemIndex, int problemCount)
{
	vector<string> scenarios = GetScenarios(mapName);
	const char* mapsDir = "/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2";
    ScenarioLoader *sl = new ScenarioLoader(scenarios[scenarioIndex].c_str());
	Experiment last = sl->GetNthExperiment(sl->GetNumExperiments() - 1);
	char *_s = new char[strlen(mapsDir)+strlen(last.GetMapName())+1];
	strcpy(_s,mapsDir);
	strcat(_s,"/");
	strcat(_s,last.GetMapName());
	cout << _s << endl;
	Map *map = new Map(_s);
	MapEnvironment *env = new MapEnvironment(map);
	string mn = last.GetMapName();
	replace(mn.begin(), mn.end(), '/', '_');
	//cout << s << endl;
	for (int i = problemIndex; i < problemIndex + problemCount; i++)
	{
		
		if (i >= sl->GetNumExperiments())
			break;
  		//myfile << "Writing this to a file.\n";
		//myfile << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		cout << "---------- " << last.GetMapName() << " ---------- problem #" << i << endl;
		Experiment exp = sl->GetNthExperiment(i);
    	xyLoc start, goal;
		goal.x = exp.GetStartX();
    	goal.y = exp.GetStartY();
		start.x = exp.GetGoalX();
    	start.y = exp.GetGoalY();
		//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		//TAS<MapEnvironment, xyLoc> gas1(env, start, goal, env, env, 1);
		//RASFast<MapEnvironment, xyLoc> gas5(env, start, goal, env, env, 5);
		TAS<MapEnvironment, xyLoc> tas(env, start, goal, env, env, 10);
		TASA<MapEnvironment, xyLoc> tasa(env, start, goal, env, env, 10);

		std::vector<xyLoc> thePath5, thePath10, aPath, fPath, bPath;
		std::vector<xyLoc> thePath1, thePath2, thePath3, thePath4;
    	Timer t1, t2, t3, t4;

		t1.StartTimer();
    	tasa.GetPath(thePath1);
    	t1.EndTimer();
		t2.StartTimer();
    	tas.GetPath(thePath2);
    	t2.EndTimer();

    	//t1.StartTimer();
    	//astar.GetPath(env, start, goal, aPath);
    	//t1.EndTimer();
		//t2.StartTimer();
    	//bgbfs.GetPath(env, start, goal, fPath, bPath);
    	//t2.EndTimer();
		//t3.StartTimer();
    	//gas5.GetPath(thePath5);
    	//t3.EndTimer();
		//t4.StartTimer();
    	//gas10.GetPath(thePath10);
    	//t4.EndTimer();
		//cout << i << endl;
		//myfile << "GAS5: " << "Expansions: " << gas5.GetNodesExpanded() << " Time: " << t3.GetElapsedTime() << " Length: " << env->GetPathLength(thePath5) << endl;
		cout << "TASA: " << "Expansions: " << tasa.GetNodesExpanded() << " Time: " << t1.GetElapsedTime() << " Length: " << env->GetPathLength(thePath1) << endl;
		cout << "TAS:  " << "Expansions: " << tas.GetNodesExpanded() << " Time: " << t2.GetElapsedTime() << " Length: " << env->GetPathLength(thePath2) << endl;
		//myfile << "GAS10: " << "Expansions: " << gas10.GetNodesExpanded() << " Time: " << t4.GetElapsedTime() << " Length: " << env->GetPathLength(thePath10) << endl;
		//myfile << "GBFS: " << "Expansions: " << astar.GetNodesExpanded() << " Time: " << t1.GetElapsedTime() << " Length: " << env->GetPathLength(aPath) << endl;
		//myfile << "BGBFS: " << "Expansions: " << bgbfs.GetNodesExpanded() << " Time: " << t2.GetElapsedTime() << " Length: " << env->GetPathLength(fPath) + env->GetPathLength(bPath) << endl;
		//cout << fPath[0] << " " << fPath[fPath.size() - 1] << " " << bPath[0] << " " << bPath[bPath.size() - 1] << endl;
		//BitMapPic pic(map->GetMapWidth(), map->GetMapHeight());
		//fPath.insert( fPath.end(), bPath.begin(), bPath.end() );
		//DrawBitMap(map, pic, fPath);
		//cout << "---------------------------" << endl;
		
	}
	//myfile.close();
	delete map;
	delete env;
	delete sl;
}

int main(int argc, char *argv[])
{
	/*
	vector<string> scenarios = GetScenarios(argv[1]);
	cout << scenarios.size() << endl;
	string path = scenarios[stoi(argv[2])];
	Test("/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2", path.c_str());
	*/
	
	/*
	for (int i = stoi(argv[4]); i < stoi(argv[4]) + stoi(argv[5]); i++)
	{
		if (stoi(argv[3]) == 0)
			BGBFS(argv[1], i, 0, 100000, stoi(argv[2]), argv[6]);
		else if (stoi(argv[3]) == 1)
			GBFS(argv[1], i, 0, 100000, stoi(argv[2]), argv[6]);
		else
			BackGBFS(argv[1], i, 0, 100000, stoi(argv[2]), argv[6]);
	}
	*/
	
	//TASATest(argv[1], stoi(argv[2]), 0, 100000);
	//ExstensiveTest(argv[1], stoi(argv[2]), 0, 100000, argv[3]);
    //DNodeTest(argv[1], stoi(argv[2]));


    //QuickTest2(argv[1], argv[2]);
	QuickTest2(argv[1], stoi(argv[2]), stoi(argv[3]), stoi(argv[4]), argv[5], argv[6]);
	//DrawTest(argv[1], stoi(argv[2]), stoi(argv[3]));

	//Timer t;
	//t.StartTimer();
	//GASTest(argv[1], stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
	//t.EndTimer();
	//cout << t.GetElapsedTime() << endl;
	
	//BGBFS("wc3maps512", 0, 5000, 1, 1, "test");
	/*
    std::string scenaro(argv[1]);
    string base_path("/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2/scenarios/");
    base_path += scenaro;
    Test("/home/sepehr3/projects/def-nathanst-ab/sepehr3/hog2/hog2", base_path.c_str());
	*/
    return 0;
}