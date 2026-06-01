#include <iostream>
#include <fstream>
#include <vector>
#include "BidirectionalGreedyBestFirst.h"
#include <cmath>
#include "TOH.h"
#include "MR1PermutationPDB.h"
#include "TemplateAStar.h"
#include <stdio.h>
#include <cstdint>
#include <math.h>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"
#include "IDAStar.h"
#include "GreedyAnchorSearch.h"
#include "MNPuzzle.h"
#include "STPInstances.h"
#include "RubiksCube.h"
#include "FastMap.h"
#include "TASA.h"
#include "TASS4.h"
#include "RASS.h"
#include "TTBS.h"
#include "DNode.h"
#include "PancakePuzzle.h"
//#include "RASFast.h"

using namespace std;

double phi(double h, double g)
{
    return h;
}



//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "TopSpin.h"

const int N = 60;
const int k = 4;

void TestTSRandom();

template<class state>
class HeuristicsGroup
{
public:
	mutable vector<Heuristic<state>*> heuristics;
	mutable int count;
	HeuristicsGroup(vector<Heuristic<state>*> heu, int _count)
	{
		count = _count;
		heuristics = heu;
	}
	HeuristicsGroup()
	{
		heuristics.clear();
		count = 0;
	}
	~HeuristicsGroup()
	{

	}
};


template<class state>
class GroupHeuristic : public Heuristic<state>
{
    public:
	vector<HeuristicsGroup<state>*> groups;
	double weight;
    double HCost(const state &state1, const state &state2) const
    {
        double res = 0;
		int bestIndex = 0;
        for (int j = 0; j < groups.size(); j++)
        {
            double partialRes = 0;
			for (int i = 0; i < groups[j]->heuristics.size(); i++)
			{
				partialRes += fabs(groups[j]->heuristics[i]->HCost(state1, state1) - groups[j]->heuristics[i]->HCost(state2, state2)) * pow(weight, groups[j]->heuristics.size() - i - 1);
			}
            res = max(res, partialRes);
        }
	    return res;
    }

    void AddGroup(vector<Heuristic<state>*> _heuristics)
    {
		HeuristicsGroup<state> *hg = new HeuristicsGroup<state>(_heuristics, 0);
        groups.push_back(hg);
    }

	void RemoveGroup(int index)
	{
		groups.erase(groups.begin() + index);
	}

	void ClearCounts()
	{
		for (int i = 0; i < groups.size(); i++)
		{
			groups[i]->count = 0;
		}
	}

	int GetLeastAccessed()
	{
		int min = 1000000000;
		int ind = -1;
		for (int i = 0; i < groups.size(); i++)
		{
			//cout << "count " << i << ": " << groups[i]->count << endl;
			if (groups[i]->count < min)
			{
				min = groups[i]->count;
				ind = i;
			}
		}
		//cout << "min: " << ind << endl;
		return ind;
	}
};





typedef MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> TSPDB;
void _MakePDBs(const TopSpinState<N> g, Heuristic<TopSpinState<N>> &h, TopSpin<N, k> &ts, bool small = false)
{
//	std::vector<int> p1 = {0,1,2,3,4};
//	std::vector<int> p2 = {5,6,7,8,9};
//	std::vector<int> p3 = {10,11,12,13,14};
	std::vector<int> p1 = {0,1,2,3,4};
	std::vector<int> p2 = {3,4,5,6,7};
	std::vector<int> p3 = {5,6,7,8,9};
	//	std::vector<int> p4 = {0,12,13,14,15};
	//	mnp.StoreGoal(g);
	if (small)
	{
		p1.pop_back();
		p2.pop_back();
		p3.pop_back();
	}
	TSPDB *pdb1 = new TSPDB(&ts, g, p1);
	TSPDB *pdb2 = new TSPDB(&ts, g, p2);
	TSPDB *pdb3 = new TSPDB(&ts, g, p3);
	//	STPPDB *pdb4 = new STPPDB(&mnp, g, p4);
	pdb1->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb2->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb3->BuildPDB(g, std::thread::hardware_concurrency(), false);
	//	pdb4->BuildPDB(g, std::thread::hardware_concurrency());
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	h.heuristics.push_back(pdb3);
}


void MakePDBs(const TopSpinState<N> g, Heuristic<TopSpinState<N>> &h, TopSpin<N, k> &ts, bool small = false)
{
//	std::vector<int> p1 = {0,1,2,3,4};
//	std::vector<int> p2 = {5,6,7,8,9};
//	std::vector<int> p3 = {10,11,12,13,14};
	std::vector<int> p1 = {0,1,2,3,4,5,6};

	TSPDB *pdb1 = new TSPDB(&ts, g, p1);
	//	STPPDB *pdb4 = new STPPDB(&mnp, g, p4);
	pdb1->BuildPDB(g, std::thread::hardware_concurrency(), false);
	//	pdb4->BuildPDB(g, std::thread::hardware_concurrency());
	h.lookups.resize(0);
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb1);
}



void Test(int seed_problem, string alg, int samples, GroupHeuristic<TopSpinState<N>> hm)
{
    srandom(seed_problem);
    TopSpin<N, k> ts;// = new TopSpin<N, k>();
    TopSpinState<N> start, goal, pivot1, pivot2;
    Heuristic<TopSpinState<N>> hf, hb, h1, h2, h3, h4;
    vector<TopSpinState<N>> thePath, path, aPath, fPath, bPath, path1, path2, path3;
    Timer timer;

    start.Reset();
	for (int x = 0; x < N; x++)
		std::swap(start.puzzle[x], start.puzzle[x+random()%(N-x)]);

	goal.Reset();
	for (int x = 0; x < N; x++)
		std::swap(goal.puzzle[x], goal.puzzle[x+random()%(N-x)]);
	

	Heuristic<TopSpinState<N>> spec1, spec2;
	MakePDBs(start, spec1, ts);
	MakePDBs(goal, spec2, ts);
	for (auto h : spec1.heuristics)
		hm.AddGroup({h});
	for (auto h : spec2.heuristics)
		hm.AddGroup({h});
    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;


	if (alg == "tas-t")
    {
        TASS<TopSpin<N, k>, TopSpinState<N>> tas(&ts, start, goal, &hm, &hm, samples);
		tas.SetAnchorSelection(Temporal);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-T: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
    else if (alg == "tas-a")
    {
        TASS<TopSpin<N, k>, TopSpinState<N>> tas(&ts, start, goal, &hm, &hm, samples);
		tas.SetAnchorSelection(Anchor);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-A: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
    else if (alg == "tas-af")
    {
        TASS<TopSpin<N, k>, TopSpinState<N>> tas(&ts, start, goal, &hm, &hm, samples);
		tas.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-AF: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
    else if (alg == "ras-t")
	{
		RASS<TopSpin<N, k>, TopSpinState<N>> ras(&ts, start, goal, &hm, &hm, samples);
		ras.SetAnchorSelection(Temporal);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-T: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "ras-a")
	{
		RASS<TopSpin<N, k>, TopSpinState<N>> ras(&ts, start, goal, &hm, &hm, samples);
		ras.SetAnchorSelection(Anchor);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-A: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "ras-af")
	{
		RASS<TopSpin<N, k>, TopSpinState<N>> ras(&ts, start, goal, &hm, &hm, samples);
		ras.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-AF: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "gbfs")
    {
        TemplateAStar<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> astar;
		astar.SetPhi(phi);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(&ts, start, goal, path);
    	timer.EndTimer();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
    }
    else if (alg == "bgbfs")
    {

    }
    else if (alg == "dnr")
    {

    }
    else if (alg == "ttbs-lifo")
    {

    }
    else if (alg == "ttbs-fifo")
    {

    }


}




void PancakeTest(int seed_problem, string alg, int samples)
{
    srandom(seed_problem);
	PancakePuzzleState<N> start;
	PancakePuzzleState<N> goal;
	PancakePuzzle<N> *pancake = new PancakePuzzle<N>();;
    vector<PancakePuzzleState<N>> thePath, path, aPath, fPath, bPath, path1, path2, path3;
    Timer timer;


	for (int i = 1; i <= 100; i++)
	{
    start.Reset();
	for (int x = 0; x < N; x++)
		std::swap(start.puzzle[x], start.puzzle[x+random()%(N-x)]);

	goal.Reset();
	for (int x = 0; x < N; x++)
		std::swap(goal.puzzle[x], goal.puzzle[x+random()%(N-x)]);
    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;
	if (alg == "tas-t")
    {
        TASS<PancakePuzzle<N>, PancakePuzzleState<N>> tas(pancake, start, goal, pancake, pancake, samples);
		tas.SetAnchorSelection(Temporal);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-T: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
	else if (alg == "tas-a")
    {
        TASS<PancakePuzzle<N>, PancakePuzzleState<N>> tas(pancake, start, goal, pancake, pancake, samples);
		tas.SetAnchorSelection(Anchor);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-A: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
    else if (alg == "tas-af")
    {
        TASS<PancakePuzzle<N>, PancakePuzzleState<N>> tas(pancake, start, goal, pancake, pancake, samples);
		tas.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-AF: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
    }
    else if (alg == "ras-t")
	{
		RASS<PancakePuzzle<N>, PancakePuzzleState<N>> ras(pancake, start, goal, pancake, pancake, samples);
		ras.SetAnchorSelection(Temporal);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-T: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "ras-a")
	{
		RASS<PancakePuzzle<N>, PancakePuzzleState<N>> ras(pancake, start, goal, pancake, pancake, samples);
		ras.SetAnchorSelection(Anchor);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-A: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "ras-af")
	{
		RASS<PancakePuzzle<N>, PancakePuzzleState<N>> ras(pancake, start, goal, pancake, pancake, samples);
		ras.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "RAS-AF: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
    else if (alg == "gbfs")
    {
        TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
		astar.SetPhi(phi);
    	timer.StartTimer();
		astar.GetPath(pancake, start, goal, path);
    	timer.EndTimer();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
    }
    else if (alg == "bgbfs")
    {
		BidirectionalGreedyBestFirst<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bgbfs;
		std::vector<PancakePuzzleState<N>> fPath, bPath;
		fPath.clear();
		bPath.clear();
		bgbfs.SetPhi(phi);
    	timer.StartTimer();
		bgbfs.GetPath(pancake, start, goal, fPath, bPath);
    	timer.EndTimer();
		cout << "BGBFS: " << bgbfs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << fPath.size() + bPath.size() - 1 << endl;
    }
    else if (alg == "dnr")
    {
		DNode<PancakePuzzle<N>, PancakePuzzleState<N>> dnr(pancake, start, goal, pancake, pancake, samples);
    	timer.StartTimer();
		dnr.GetPath(path);
    	timer.EndTimer();
		cout << "DNR: " << dnr.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << dnr.pathRatio << endl;
    }
    else if (alg == "ttbs-lifo")
    {
		TTBS<PancakePuzzle<N>, PancakePuzzleState<N>> ttbs(pancake, start, goal, pancake, pancake, 0);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
    }
    else if (alg == "ttbs-fifo")
    {
		TTBS<PancakePuzzle<N>, PancakePuzzleState<N>> ttbs(pancake, start, goal, pancake, pancake, 1);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
    }
	}
}


int main(int argc, char *argv[])
{
	PancakeTest(stoi(argv[1]), argv[2], stoi(argv[3]));
    return 0;

}
