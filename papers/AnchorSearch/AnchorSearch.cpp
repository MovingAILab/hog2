#include <cstring>
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "MNPuzzle.h"
#include "TOH.h"
#include "Timer.h"
#include "STPInstances.h"
#include "TemplateAStar.h"
#include <iostream>
#include <vector>
#include "TemporalAnchorSearch.h"
#include "TTBS.h"
#include "RandomizedAnchorSearch.h"
#include "DNode.h"
#include "TemporalSearch.h"
#include "BidirectionalGreedyBestFirst.h"
#include <unordered_set>

using namespace std;


double Phi10(double h, double g)
{
    return g + 10.0 * h;
}


template<class state>
class HGroup
{
public:
	mutable vector<Heuristic<state>*> heuristics;
	mutable int count;
	HGroup(vector<Heuristic<state>*> heu, int _count)
	{
		count = _count;
		heuristics = heu;
	}
	HGroup()
	{
		heuristics.clear();
		count = 0;
	}
	~HGroup()
	{

	}
};


template<class state>
class GroupHeuristic : public Heuristic<state>
{
    public:
	vector<HGroup<state>*> groups;
	double weight = 1;
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
				//cout << fabs(groups[j]->heuristics[i]->HCost(state1, state1) - groups[j]->heuristics[i]->HCost(state2, state2)) << "*" << pow(weight, groups[j]->heuristics.size() - i - 1) << "+";
			}
			//cout << endl;
            res = max(res, partialRes);
        }
	    return res;
    }

    void AddGroup(vector<Heuristic<state>*> _heuristics)
    {
		HGroup<state> *hg = new HGroup<state>(_heuristics, 0);
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
			if (groups[i]->count < min)
			{
				min = groups[i]->count;
				ind = i;
			}
		}
		return ind;
	}
};





template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	TOHState<pdb2Disks> absTohState2;
	
	
	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency(), false);
	pdb2->BuildPDB(goal, std::thread::hardware_concurrency(), false);
	
	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
	
	h->lookups.resize(0);
	
	h->lookups.push_back({kAddNode, 1, 2});
	h->lookups.push_back({kLeafNode, 0, 0});
	h->lookups.push_back({kLeafNode, 1, 1});
	h->heuristics.resize(0);
	h->heuristics.push_back(pdb1);
	h->heuristics.push_back(pdb2);
	
	return h;
}

template <int numDisks, int pdb1Disks>
void BuildSinglePDB(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	//TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	//TOHState<pdb2Disks> absTohState2;
	
	
	//TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, pivot1); // top disks
	TOHPDB<pdb1Disks, numDisks> *pdb2 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1); // bottom disks
	pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.lookups.resize(0);
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb2);
}


template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks % pdb1Disks>
void BuildMultiplePDBs(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;

	h.lookups.resize(0);
	h.heuristics.resize(0);

	TOHPDB<pdb1Disks, numDisks> *pdb1 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1);
	pdb1->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.heuristics.push_back(pdb1);

	if (numDisks >= 2 * pdb1Disks)
	{
		TOHPDB<pdb1Disks, numDisks, pdb1Disks> *pdb2 = new TOHPDB<pdb1Disks, numDisks, pdb1Disks>(&absToh1, pivot1);
		pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
		h.heuristics.push_back(pdb2);
		if (numDisks > 2 * pdb1Disks)
		{
			TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb3 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, pivot1);
			pdb3->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
			h.heuristics.push_back(pdb3);
		}
	}
	else
	{
		TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb2 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, pivot1);
		pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
		h.heuristics.push_back(pdb2);
	}
}

template <int numDisks, int pdb1Disks, int pdb2Disks>
void BuildWeakMultiplePDBs(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;

	h.lookups.resize(0);
	h.heuristics.resize(0);

	TOHPDB<pdb1Disks, numDisks> *pdb1 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1);
	pdb1->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.heuristics.push_back(pdb1);

	TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb2 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, pivot1);
	pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.heuristics.push_back(pdb2);
}

template<int numDisks, int pdb1Disks, int maxPivots = 5>
void HUpdate(TOHState<numDisks> s, TOHState<numDisks> g, Heuristic<TOHState<numDisks>>*& h)
{
	GroupHeuristic<TOHState<numDisks>>* mhh;
	mhh = (GroupHeuristic<TOHState<numDisks>>*)h;
	//cout << mhh->groups.size() << endl;
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOHState<pdb1Disks> absTohState1;

	TOHPDB<pdb1Disks, numDisks> *pdb2 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, s); // bottom disks
	TOHPDB<pdb1Disks, numDisks> *pdb4 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, g); // bottom disks
	pdb2->BuildPDB(s, std::thread::hardware_concurrency(), false);
	pdb4->BuildPDB(g, std::thread::hardware_concurrency(), false);

	Heuristic<TOHState<numDisks>> __h;
	/*
	while (mhh->groups.size() + 2 > maxPivots)
	{
		auto la = mhh->GetLeastAccessed();
		mhh->RemoveGroup(la);
	}
	*/
	while(mhh->groups.size() > 2)
	{
		mhh->RemoveGroup(mhh->groups.size() - 1);
	}
	mhh->AddGroup({pdb2});
	mhh->AddGroup({pdb4});
	//mhh->ClearCounts();
	h = mhh;
}




double Phi(double h, double g)
{
    return h;
}



template<int numOfDisks, int pdb1Disks>
void FixedHeuristicTOHTest(int seed_problem, string alg, GroupHeuristic<TOHState<numOfDisks>> &hm)
{
    srandom(seed_problem);
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    TOHState<numOfDisks> start, goal, aux;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path;
    start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
	for (int x = numOfDisks; x > 0; x--)
	{
		int whichPeg = random()%4;
		start.disks[whichPeg][start.counts[whichPeg]] = x;
		start.counts[whichPeg]++;
	}
    goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
	for (int x = numOfDisks; x > 0; x--)
	{
		int whichPeg = random()%4;
		goal.disks[whichPeg][goal.counts[whichPeg]] = x;
		goal.counts[whichPeg]++;
	}

	Heuristic<TOHState<numOfDisks>> spec1, spec2, spec3;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});

	std::cout << "---------------- " << seed_problem << " ----------------"  << std::endl;
    cout << start << endl;
    cout << goal << endl;

	Timer timer;

	if (alg == "tas-t")
	{
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, 10);
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
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, 10);
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
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, 10);
		tas.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "TAS-AF: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
	}
	else if (alg == "gbfs")
	{
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetPhi(Phi);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	else if (alg == "ttbs-fifo")
	{
		TTBS<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, 0);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
	}
	else if (alg == "ttbs-lifo")
	{
		TTBS<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, 1);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "TTBS-LIFO: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
	}
	else if (alg == "dnr")
	{
		DNode<TOH<numOfDisks>, TOHState<numOfDisks>> dnr(toh, start, goal, &hm, &hm, 100);
    	timer.StartTimer();
		dnr.GetPath(path);
    	timer.EndTimer();
		cout << "DNR: " << dnr.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << dnr.pathRatio << endl;
	}
}


template<int numOfDisks, int pdb1Disks, int pdb2Disks = 0>
void BuildHeuristic(TOHState<numOfDisks> start, TOHState<numOfDisks> goal, GroupHeuristic<TOHState<numOfDisks>> &hm, bool multiple = true)
{
    Heuristic<TOHState<numOfDisks>> spec1, spec2;
    if (pdb2Disks <= 0)
    {
        if (multiple)
        {
            BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec1, start);
	        BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec2, goal);
        }
        else
        {
            BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	        BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
        }
    }
    else
    {
        BuildWeakMultiplePDBs<numOfDisks, pdb1Disks, pdb2Disks>(spec1, start);
	    BuildWeakMultiplePDBs<numOfDisks, pdb1Disks, pdb2Disks>(spec2, goal);
    }
	vector<Heuristic<TOHState<numOfDisks>>*> v1;
	vector<Heuristic<TOHState<numOfDisks>>*> v2;
	for (auto item : spec1.heuristics)
	{
		v1.push_back(item);
	}
	for (auto item : spec2.heuristics)
	{
		v2.push_back(item);
	}
	hm.AddGroup(v1);
	hm.AddGroup(v2);
}


template<int numOfDisks>
void GenerateRandomEnds(TOHState<numOfDisks> &start, TOHState<numOfDisks> &goal, int seed)
{
    srandom(seed);
    start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
	for (int x = numOfDisks; x > 0; x--)
	{
		int whichPeg = random()%4;
		start.disks[whichPeg][start.counts[whichPeg]] = x;
		start.counts[whichPeg]++;
	}
    goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
	for (int x = numOfDisks; x > 0; x--)
	{
		int whichPeg = random()%4;
		goal.disks[whichPeg][goal.counts[whichPeg]] = x;
		goal.counts[whichPeg]++;
	}
    std::cout << "---------------- problem seed: " << seed << " ----------------"  << std::endl;
    cout << "start:\t" << start << endl;
    cout << "goal:\t" << goal << endl;
}


template<int numOfDisks>
void IterativeHeuristicTOHTest(TOHState<numOfDisks> start, TOHState<numOfDisks> goal, string alg, int ep, GroupHeuristic<TOHState<numOfDisks>> &hm, int candidates = 10)
{
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    //TOHState<numOfDisks> start, goal, aux;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path;


	Timer timer;
	
    if (alg == "tas-af")
	{
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, candidates);
		tas.SetAnchorSelection(Anchor, Fixed);
        //tas.SetHeuristicUpdate(HUpdate<numOfDisks, pdb1Disks>);
        tas.episode = ep;
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "result: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
	}
	else if (alg == "tas-a")
	{
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, candidates);
		tas.SetAnchorSelection(Anchor);
        //tas.SetHeuristicUpdate(HUpdate<numOfDisks, pdb1Disks>);
        tas.episode = ep;
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "result: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
	}
	else if (alg == "tas-t")
	{
		TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, candidates);
		tas.SetAnchorSelection(Temporal);
        //tas.SetHeuristicUpdate(HUpdate<numOfDisks, pdb1Disks>);
        tas.episode = ep;
		timer.StartTimer();
		tas.GetPath(path);
		timer.EndTimer();
		cout << "result: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << tas.pathRatio << endl;
		delete tas.ff;
		delete tas.bf;
	}
	else if (alg == "ras-t")
	{
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, candidates);
		ras.SetAnchorSelection(Temporal);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "result: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg == "ras-a")
	{
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, candidates);
		ras.SetAnchorSelection(Anchor);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "result: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg == "ras-af")
	{
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, candidates);
		ras.SetAnchorSelection(Anchor, Fixed);
		timer.StartTimer();
		ras.GetPath(path);
		timer.EndTimer();
		cout << "result: " << ras.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ras.pathRatio << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg == "gbfs")
	{
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetPhi(Phi);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "result: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	else if (alg == "bgbfs")
	{
		BidirectionalGreedyBestFirst<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> bgbfs;
		std::vector<TOHState<numOfDisks>> fPath, bPath;
		fPath.clear();
		bPath.clear();
		bgbfs.SetPhi(Phi);
		bgbfs.SetHeuristic(&hm);
    	timer.StartTimer();
		bgbfs.GetPath(toh, start, goal, fPath, bPath);
    	timer.EndTimer();
		cout << "result: " << bgbfs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << fPath.size() + bPath.size() - 1 << " " << (float)min(fPath.size(), bPath.size()) / (float)(fPath.size() + bPath.size()) << endl;
	}
	else if (alg == "wa*")
	{
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetPhi(Phi10);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "result: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	else if (alg == "ttbs-fifo")
	{
		TTBS<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, 0);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "result: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
	}
	else if (alg == "ttbs-lifo")
	{
		TTBS<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, 1);
    	timer.StartTimer();
		ttbs.GetPath(path);
    	timer.EndTimer();
		cout << "result: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << ttbs.pathRatio << endl;
	}
	else if (alg == "dnr")
	{
		DNode<TOH<numOfDisks>, TOHState<numOfDisks>> dnr(toh, start, goal, &hm, &hm, candidates);
    	timer.StartTimer();
		dnr.GetPath(path);
    	timer.EndTimer();
		cout << "result: " << dnr.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << " " << dnr.pathRatio << endl;
	}
	else if (alg == "ts")
	{
		TemporalSearch<TOH<numOfDisks>, TOHState<numOfDisks>> ts(toh, start, goal, &hm, candidates);
		timer.StartTimer();
		ts.GetPath(path);
		timer.EndTimer();
		cout << "result: " << ts.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
}


template<int numOfDisks, int pdb1Disks, int pdb2Disks = 0>
void Dispatch(string alg, int seed, float weight, bool multiple = true, int candidates = 10)
{
    TOHState<numOfDisks> start, goal;
    GenerateRandomEnds<numOfDisks>(start, goal, seed);
    GroupHeuristic<TOHState<numOfDisks>> gh;
	gh.weight = weight;
    BuildHeuristic<numOfDisks, pdb1Disks, pdb2Disks>(start, goal, gh, multiple);
    IterativeHeuristicTOHTest<numOfDisks>(start, goal, alg, -1, gh, candidates);
}

template<int numOfDisks, int pdb1Disks>
void Dispatch(string alg, int seed, float weight, int pdb2Disks = 0, bool multiple = true, int candidates = 10)
{
    switch (pdb2Disks)
    {
        case 0:
            Dispatch<numOfDisks, pdb1Disks, 0>(alg, seed, weight, multiple, candidates);
            break;
        case 5:
            Dispatch<numOfDisks, pdb1Disks, 5>(alg, seed, weight, multiple, candidates);
            break;
        case 6:
            Dispatch<numOfDisks, pdb1Disks, 6>(alg, seed, weight, multiple, candidates);
            break;
        case 7:
            Dispatch<numOfDisks, pdb1Disks, 7>(alg, seed, weight, multiple, candidates);
            break;
        case 8:
            Dispatch<numOfDisks, pdb1Disks, 8>(alg, seed, weight, multiple, candidates);
            break;
        case 9:
            Dispatch<numOfDisks, pdb1Disks, 9>(alg, seed, weight, multiple, candidates);
            break;
        case 10:
            Dispatch<numOfDisks, pdb1Disks, 10>(alg, seed, weight, multiple, candidates);
            break;
        case 11:
            Dispatch<numOfDisks, pdb1Disks, 11>(alg, seed, weight, multiple, candidates);
            break;
        case 12:
            Dispatch<numOfDisks, pdb1Disks, 12>(alg, seed, weight, multiple, candidates);
            break;
		default:
			std::cout << "pdb2 of size " << pdb2Disks << " is not supported!";
    }
}

template<int numOfDisks>
void Dispatch(string alg, int seed, float weight, int pdb1Disks, int pdb2Disks = 0, bool multiple = true, int candidates = 10)
{
    switch (pdb1Disks)
    {
        case 5:
            Dispatch<numOfDisks, 5>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 6:
            Dispatch<numOfDisks, 6>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 7:
            Dispatch<numOfDisks, 7>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 8:
            Dispatch<numOfDisks, 8>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 9:
            Dispatch<numOfDisks, 9>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 10:
            Dispatch<numOfDisks, 10>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 11:
            Dispatch<numOfDisks, 11>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
        case 12:
            Dispatch<numOfDisks, 12>(alg, seed, weight, pdb2Disks, multiple, candidates);
            break;
		default:
			std::cout << "pdb1 of size " << pdb1Disks << " is not supported!";
    }
}


void Dispatch(string alg, int seed, float weight, int size, int pdb1Disks, int pdb2Disks = 0, bool multiple = true, int candidates = 10)
{
    switch (size)
    {
        case 10:
            Dispatch<10>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 22:
            Dispatch<22>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 24:
            Dispatch<24>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 26:
            Dispatch<26>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 28:
            Dispatch<28>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 30:
            Dispatch<30>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
        case 32:
            Dispatch<32>(alg, seed, weight, pdb1Disks, pdb2Disks, multiple, candidates);
            break;
		default:
			std::cout << "size " << size << " is not supported!";
    }
}


int main(int argc, char* argv[])
{


    //const int psize = 16;
	//const int pdbsize = 12;
	//for (int i = 1; i <= 100; i++)
	//{

	int size = -1;
	int pdb1Disks = -1; 
	int pdb2Disks = 0;
	int seed = 1;
	int candidates = 10;
	double weight = 1.;
	bool multiple = false;
	int verbosity = 1;
	std::string alg = "none";

	std::unordered_map<std::string, std::string> args;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg[0] == '-') {
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                args[arg] = argv[++i];
            } else {
                args[arg] = "";
            }
        }
    }

	if (args.find("--help") != args.end() || args.find("-h") != args.end()) {
        std::string message = R"(
Arguments:
	--size: problem size, i.e., number of disks (options: 10, 22, 24, 26, 28, 30, 32)
	--seed: the seed used to generate start and goal
	--pdb1: size of the first PDB capturing bottom disks (options: 5 to 12)
	--pdb2: size of the second PDB (options: 5 to 12; disabled by default)
	--repeat: if true, all the disks will be covered by non-overlapping PDBs of size pdb1. Note that the last PDB would be of size problem_size % pdb1 (default: false).
	--weight: for weighted additive PDBs (default: 1)
	--alg: the search algorithm; options:
		- gbfs: Gready Best First Search
		- bgbfs: bidirectional GBFS
		- wa*: Weighted A* with weight = 10
		- ttbs-lifo: TTBS with LIFO tie-breaking
		- ttbs-fifo: TTBS with FIFO tie-breaking
		- dnr: D-node retargeting
		- ts: Temporal Search (TS); equivalent to temporal anchor search in only one direction.
		- tas-t: temporal anchor search with temporal anchor selection policy, i.e., AS^T_T
		- tas-a: temporal anchor search with closes-to-the-opposite-anchor anchor selection policy, i.e., AS^T_A
		- tas-af: temporal anchor search with hybrid anchor selection policy, i.e., AS^T_AF
		- ras-t: randomized anchor search with temporal anchor selection policy, i.e., AS^R_T
		- ras-a: randomized anchor search with closes-to-the-opposite-anchor anchor selection policy, i.e., AS^R_A
		- ras-af: randomized anchor search with hybrid anchor selection policy, i.e., AS^R_AF
	--candidates/-k: number of candidates for TS, DNR, and anchor search algorithms. (default: 10).
	--verbose: when larger than 0, the resolved arguments will be printed (default: 1)
Examples:
	./AnchorSearch --size 10 --pdb1 5 --seed 1 --alg tas-t -k 10                                ** AS^T(10)_T; PDB(0-4)
	./AnchorSearch --size 22 --pdb1 12 --pdb2 4 --seed 1 --alg tas-af -k 20                     ** AS^T(20)_AF; PDB(0-11) + PDB(12-15)
	./AnchorSearch --size 22 --pdb1 12 --pdb2 4 --seed 1 --alg tas-af -k 20	--weight 100        ** AS^T(20)_AF; PDB(0-11) * 100 + PDB(12-15)
	./AnchorSearch --size 22 --pdb1 12 --repeat --seed 1 --alg tas-a -k 10                      ** AS^T(10)_A; PDB(0-11) + PDB(12-inf)
	./AnchorSearch --size 22 --pdb1 12 --repeat --seed 1 --alg dnr -k 100                       ** DNR with 100 candidates; PDB(0-11) + PDB(12-inf)
Results:
	Results are of format "results: <expansions> <time> <solution_length> <path_ratio*>"		* path ratio is only reported for bidirectional algorithms.
)";
		std::cout << message << std::endl;
		return 0;
    }
	if (args.find("--verbose") != args.end())
	{
		verbosity = stoi(args["--verbose"]);
	}
	if (args.find("--size") != args.end())
	{
		size = stoi(args["--size"]);
		if (size != 10 && (size < 22 || size > 32 || size % 2 == 1))
		{
			std::cout << "Problem size \"" << size <<"\" is not supported!" << std::endl;
			return 0;
		}
	}
	if (args.find("--pdb1") != args.end())
	{
		pdb1Disks = stoi(args["--pdb1"]);
		if (pdb1Disks < 5 || pdb1Disks > 12)
		{
			std::cout << "pdb1 of size \"" << pdb1Disks <<"\" is not supported!" << std::endl;
			return 0;
		}
	}
	if (args.find("--pdb2") != args.end())
	{
		pdb2Disks = stoi(args["--pdb2"]);
		if (pdb2Disks < 5 || pdb2Disks > 12)
		{
			std::cout << "pdb2 of size \"" << pdb1Disks <<"\" is not supported!" << std::endl;
			return 0;
		}
	}
	if (args.find("--seed") != args.end())
	{
		seed = stoi(args["--seed"]);
	}
	if (args.find("-k") != args.end())
	{
		candidates = stoi(args["-k"]);
	}
	else if (args.find("--candidates") != args.end())
	{
		candidates = stoi(args["-candidates"]);
	}
	if (args.find("--repeat") != args.end())
	{
		multiple = true;
	}
	if (args.find("--weight") != args.end())
	{
		weight = stoi(args["--weight"]);
	}
	if (args.find("--alg") != args.end())
	{
		std::unordered_set<std::string> options = {"gbfs", "bgbfs", "dnr", "ttbs-lifo", "ttbs-fifo", "ts", "tas-t", "tas-a", "tas-af", "ras-t", "ras-a", "ras-af"};
		alg = args["--alg"];
		if (options.find(alg) == options.end())
		{
			std::cout << "algorithm \"" << alg << "\" is not supported!" << std::endl;
			return 0;
		}
	}

	if (alg == "none")
	{
		std::cout << "Error: Argument --alg is required to be specified." << std::endl;
		return 0;
	}
	if (size == -1)
	{
		std::cout << "Error: Argument --size is required to be specified." << std::endl;
		return 0;
	}
	if (pdb1Disks == -1)
	{
		std::cout << "Error: Argument --pdb1 is required to be specified." << std::endl;
		return 0;
	}
	if (multiple && pdb2Disks > 0)
	{
		std::cout << "Error: Only one of --repeat or --pdb2Disks can be specified at a time." <<  std::endl;
		return 0;
	}
	if (verbosity > 0)
		std::cout << "problem_size: " << size << ", seed: " << seed << ", pdb1: " << pdb1Disks << ", pdb2: " << pdb2Disks << ", multiple: " <<  multiple << ", weight: " << weight << ", alg: " << alg << ", candidates: " << candidates << std::endl;
	Dispatch(alg, seed, weight, size, pdb1Disks, pdb2Disks, multiple, candidates);
	return 0;
}