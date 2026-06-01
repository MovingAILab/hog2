#include <cstring>
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "MNPuzzle.h"
#include "TOH.h"
#include "Timer.h"
#include "STPInstances.h"
#include "TemplateAStar.h"
#include "TASA.h"
#include "TAS.h"
#include <iostream>
#include <vector>
#include "MultipleAdditiveHeuristic.h"
//#include "TASTest.h"
#include "TASS4.h"
#include "TASH.h"
#include "TASS3.h"
//#include "FastDict.h"
#include "TTBS.h"
//#include "RASFast.h"
#include "RASS.h"
#include "DNode.h"
#include "TOHWrapper.h"
#include "MultiTAS.h"
#include "TemporalSearch.h"
#include "BidirectionalGreedyBestFirst.h"

using namespace std;


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



template<class state>
class WeightedGroupHeuristic : public GroupHeuristic<state>
{
	public:
	double weight;
	vector<HeuristicsGroup<state>*> groups;
	WeightedGroupHeuristic(double _weight)
	{
		weight = _weight;
	}
	double HCost(const state &state1, const state &state2) const
    {
        double res = 0;
		int bestIndex = 0;
		cout << "===========" << endl;
        for (int j = 0; j < groups.size(); j++)
        {
            double partialRes = 0;
			for (int i = 0; i < groups[j]->heuristics.size(); i++)
			{
				cout << fabs(groups[j]->heuristics[i]->HCost(state1, state1) - groups[j]->heuristics[i]->HCost(state2, state2)) << endl;
				partialRes += fabs(groups[j]->heuristics[i]->HCost(state1, state1) - groups[j]->heuristics[i]->HCost(state2, state2)) * pow(weight, groups[j]->heuristics.size() - i - 1);
			}
			cout << ':: ' << partialRes << endl;
			cout << '-----' << endl;
            res = max(res, partialRes);
        }
		cout << "===========" << endl;
	    return res;
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

typedef MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> STPPDB;
void MakeSTPPDBs(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> *h, MNPuzzle<4,4> *mnp)
{
	std::vector<int> p1 = {0,1,2,3};
	std::vector<int> p2 = {0,4,5,6,7};
	std::vector<int> p3 = {0,8,9,10,11};
	std::vector<int> p4 = {0,12,13,14,15};

	std::vector<int> p5 = {0,4,8,12};
	std::vector<int> p6 = {0,1,5,9,13};
	std::vector<int> p7 = {0,2,6,10,14};
	std::vector<int> p8 = {0,3,7,11,15};

	STPPDB *pdb1r = new STPPDB(mnp, g, p1);
	STPPDB *pdb2r = new STPPDB(mnp, g, p2);
	STPPDB *pdb3r = new STPPDB(mnp, g, p3);
	STPPDB *pdb4r = new STPPDB(mnp, g, p4);
	STPPDB *pdb1c = new STPPDB(mnp, g, p5);
	STPPDB *pdb2c = new STPPDB(mnp, g, p6);
	STPPDB *pdb3c = new STPPDB(mnp, g, p7);
	STPPDB *pdb4c = new STPPDB(mnp, g, p8);

	pdb1r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb2r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb3r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb4r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb1c->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb2c->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb3c->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb4c->BuildPDB(g, std::thread::hardware_concurrency(), false);

	h->lookups.resize(0);
	h->heuristics.resize(0);
	//h.heuristics.push_back(&mnp);
	h->heuristics.push_back(pdb1r);
	h->heuristics.push_back(pdb2r);
	h->heuristics.push_back(pdb3r);
	h->heuristics.push_back(pdb4r);
	h->heuristics.push_back(pdb1c);
	h->heuristics.push_back(pdb2c);
	h->heuristics.push_back(pdb3c);
	h->heuristics.push_back(pdb4c);
}

template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
void BuildSinglePDB(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	TOHState<pdb2Disks> absTohState2;
	
	
	//TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, pivot1); // top disks
	TOHPDB<pdb1Disks, numDisks> *pdb2 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1); // bottom disks
	pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.lookups.resize(0);
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb2);
}


template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
void BuildMultiplePDBs(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	
	//TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, pivot1); // top disks
	TOHPDB<pdb1Disks, numDisks> *pdb1 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1); // bottom disks
	TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb2 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, pivot1); // bottom disks
	//TOHPDB<pdb1Disks, numDisks, 2> *pdb3 = new TOHPDB<pdb1Disks, numDisks, 2>(&absToh1, pivot1); // bottom disks
	//TOHPDB<pdb1Disks, numDisks, 3> *pdb4 = new TOHPDB<pdb1Disks, numDisks, 3>(&absToh1, pivot1); // bottom disks
	//TOHPDB<pdb1Disks, numDisks, 4> *pdb5 = new TOHPDB<pdb1Disks, numDisks, 4>(&absToh1, pivot1); // bottom disks
	//TOHPDB<pdb1Disks, numDisks, 5> *pdb6 = new TOHPDB<pdb1Disks, numDisks, 5>(&absToh1, pivot1); // bottom disks
	pdb1->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	pdb2->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	//pdb3->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	//pdb4->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	//pdb5->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	//pdb6->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.lookups.resize(0);
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	//h.heuristics.push_back(pdb3);
	//h.heuristics.push_back(pdb4);
	//h.heuristics.push_back(pdb5);
	//h.heuristics.push_back(pdb6);
}


template <int numDisks, int pdb1Disks>
void BuildMultiLookupPDB(Heuristic<TOHState<numDisks>> &h, const TOHState<numDisks> &pivot1, int pdbCount, int step)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	//TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	//TOHState<pdb2Disks> absTohState2;
	TOHPDB<pdb1Disks, numDisks> *pdb = new TOHPDB<pdb1Disks, numDisks>(&absToh1, pivot1); // bottom disks
	pdb->BuildPDB(pivot1, std::thread::hardware_concurrency(), false);
	h.lookups.resize(0);
	h.heuristics.resize(0);

	for (int i = 0; i < pdbCount; i++)
	{
		cout << i * step << " ";
		TOHPDBWrapper<pdb1Disks, numDisks> *wrapper = new TOHPDBWrapper<pdb1Disks, numDisks>(&absToh1, pivot1);
		wrapper->SetOffset(i * step);
		wrapper->SetBasePDB(pdb);
		//PDBHeuristic<TOHState<16>, TOHMove, TOH<16>, TOHState<numDisks>> *_h = wrapper;
		h.heuristics.push_back(wrapper);
	}

	
	//h.heuristics.push_back(pdb2);
}

template<int numOfDisks, int pdb1Disks>
void BuildTOHPDB(TOHState<numOfDisks> start, TOHState<numOfDisks> goal, TOHState<numOfDisks> aux, GroupHeuristic<TOHState<numOfDisks>> &hm)
{
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
	Heuristic<TOHState<numOfDisks>> spec1, spec2, spec3;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec3, aux);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});
	hm.AddGroup({spec3.heuristics[0]});
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
	while(mhh->groups.size() > 0)
	{
		mhh->RemoveGroup(mhh->groups.size() - 1);
	}
	mhh->AddGroup({pdb2});
	mhh->AddGroup({pdb4});
	//mhh->ClearCounts();
	h = mhh;
}


void STPHUpdate(MNPuzzleState<4, 4> s, MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>>*& h)
{
	GroupHeuristic<MNPuzzleState<4, 4>>* mhh;
	mhh = (GroupHeuristic<MNPuzzleState<4, 4>>*)h;
	//cout << mhh->groups.size() << endl;
	MNPuzzle<4, 4>* mnp = new MNPuzzle<4, 4>();
	Heuristic<MNPuzzleState<4, 4>>* hf = new Heuristic<MNPuzzleState<4, 4>>();
	Heuristic<MNPuzzleState<4, 4>>* hb = new Heuristic<MNPuzzleState<4, 4>>();

	MakeSTPPDBs(s, hf, mnp);
	MakeSTPPDBs(g, hb, mnp);
	
	while(mhh->groups.size() > 2)
	{
		mhh->RemoveGroup(mhh->groups.size() - 1);
	}
	mhh->AddGroup({hf->heuristics[0], hf->heuristics[1], hf->heuristics[2], hf->heuristics[3], hf->heuristics[4], hf->heuristics[5], hf->heuristics[6], hf->heuristics[7]});
	mhh->AddGroup({hb->heuristics[0], hb->heuristics[1], hb->heuristics[2], hb->heuristics[3], hb->heuristics[4], hb->heuristics[5], hb->heuristics[6], hb->heuristics[7]});
	h = mhh;
}


double Phi(double h, double g)
{
    return h;
}

double Phi10(double h, double g)
{
    return g + 10.0 * h;
}


template<int numOfDisks, int pdb1Disks>
void BuildPDBRandomPivots(int seed, int count, GroupHeuristic<TOHState<numOfDisks>> &hm)
{
	srandom(seed);
	for (int i = 0; i < count; i++)
	{
		Heuristic<TOHState<numOfDisks>> h;
		TOHState<numOfDisks> piv;
		piv.counts[0] = piv.counts[1] = piv.counts[2] = piv.counts[3] = 0;
		for (int x = numOfDisks; x > 0; x--)
		{
			int whichPeg = random()%4;
			piv.disks[whichPeg][piv.counts[whichPeg]] = x;
			piv.counts[whichPeg]++;
		}
		BuildSinglePDB<numOfDisks, pdb1Disks>(h, piv);
		hm.AddGroup({h.heuristics[0]});
	}
}

template<int numOfDisks, int pdb1Disks>
void TOHPivotAnalysis(int seed_problem, string alg, GroupHeuristic<TOHState<numOfDisks>> &hm, double &acc, int withGoals = 1)
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
	if (withGoals == 1){
		BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
		BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
		hm.AddGroup({spec1.heuristics[0]});
		hm.AddGroup({spec2.heuristics[0]});
	}

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
		acc += tas.GetNodesExpanded();
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
		acc += tas.GetNodesExpanded();
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
		acc += tas.GetNodesExpanded();
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
		acc += astar.GetNodesExpanded();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	if (withGoals == 1)
	{
		hm.RemoveGroup(hm.groups.size() - 1);
		hm.RemoveGroup(hm.groups.size() - 1);
	}
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
	Timer pdbTime;
	pdbTime.StartTimer();
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	pdbTime.EndTimer();
	cout << "PDB time: " <<  pdbTime.GetElapsedTime() << endl;
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});
	//BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec1, start);
	//BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec2, goal);
	//hm.AddGroup({spec1.heuristics[0], spec1.heuristics[1]});
	//hm.AddGroup({spec2.heuristics[0], spec2.heuristics[1]});
	/*
	int pdbCount = numOfDisks - pdb1Disks + 1;
	int step = 1;
	BuildMultiLookupPDB<numOfDisks, pdb1Disks>(spec1, start, pdbCount, step);
	BuildMultiLookupPDB<numOfDisks, pdb1Disks>(spec2, goal, pdbCount, step);
	//for (int i = 0; i < spec1.heuristics.size(); i++)
	//{
	//	hm.AddGroup({spec1.heuristics[i]});
	//	hm.AddGroup({spec2.heuristics[i]});
	//}
	hm.AddGroup(spec1.heuristics);
	hm.AddGroup(spec2.heuristics);
	
	BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec1, start);
	BuildMultiplePDBs<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics});
	hm.AddGroup({spec2.heuristics});
	*/

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
	else if (alg == "ras-t")
	{
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, 10);
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
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, 10);
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
		RASS<TOH<numOfDisks>, TOHState<numOfDisks>> ras(toh, start, goal, &hm, &hm, 10);
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
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetPhi(Phi);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	else if (alg == "astar")
	{
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "A*: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
	}
	else if (alg == "wa*")
	{
		TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
		astar.SetPhi(Phi10);
		astar.SetHeuristic(&hm);
    	timer.StartTimer();
		astar.GetPath(toh, start, goal, path);
    	timer.EndTimer();
		cout << "WA*: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
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
	else if (alg == "ts")
	{
		TemporalSearch<TOH<numOfDisks>, TOHState<numOfDisks>> ts(toh, start, goal, &hm, 10);
		timer.StartTimer();
		ts.GetPath(path);
		timer.EndTimer();
		cout << "TS: " << ts.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
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
		cout << "BGBFS: " << bgbfs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << fPath.size() + bPath.size() - 1<< endl;
	}
}



int main(int argc, char* argv[])
{
	
    //const int psize = 16;
	//const int pdbsize = 10;
	string setting = argv[1];
	for (int i = 1; i <= 100; i++)
	{
	int seed = i;// = stoi(argv[2]);
	if (setting == "14-7")
	{
		GroupHeuristic<TOHState<14>> gh;
		FixedHeuristicTOHTest<14, 7>(seed, argv[3], gh);
	}
	else if (setting == "14-10")
	{
		GroupHeuristic<TOHState<14>> gh;
		FixedHeuristicTOHTest<14, 10>(seed, argv[3], gh);
	}
	else if (setting == "16-8")
	{
		GroupHeuristic<TOHState<16>> gh;
		FixedHeuristicTOHTest<16, 8>(seed, argv[3], gh);
	}
	else if (setting == "16-10")
	{
		GroupHeuristic<TOHState<16>> gh;
		FixedHeuristicTOHTest<16, 10>(seed, argv[3], gh);
	}
	else if (setting == "12-8")
	{
		GroupHeuristic<TOHState<12>> gh;
		FixedHeuristicTOHTest<12, 8>(seed, argv[3], gh);
	}
	else if (setting == "12-10")
	{
		GroupHeuristic<TOHState<12>> gh;
		FixedHeuristicTOHTest<12, 10>(seed, argv[3], gh);
	}
	else if (setting == "12-6")
	{
		GroupHeuristic<TOHState<12>> gh;
		FixedHeuristicTOHTest<12, 6>(seed, argv[3], gh);
	}
	else if (setting == "10-5")
	{
		GroupHeuristic<TOHState<10>> gh;
		FixedHeuristicTOHTest<10, 5>(seed, argv[3], gh);
	}
	else if (setting == "10-6")
	{
		GroupHeuristic<TOHState<10>> gh;
		FixedHeuristicTOHTest<10, 6>(seed, argv[3], gh);
	}
	else if (setting == "12-7")
	{
		GroupHeuristic<TOHState<12>> gh;
		FixedHeuristicTOHTest<12, 7>(seed, argv[3], gh);
	}
	}
	
	/*
	double res = 0;
	for (int i = 1; i <= 100; i++)
	{
		GroupHeuristic<TOHState<12>> gh;
		BuildPDBRandomPivots<12, 6>(1, stoi(argv[1]), gh);
		TOHPivotAnalysis<12, 6>(i, argv[3], gh, res, stoi(argv[2]));
	}
	cout << "--------------------" << endl;
	cout << res / 100.0 << endl;
	
	for (int i = 1; i <= 100; i++)
	{
		int seed = i;
		GroupHeuristic<TOHState<14>> gh;
		gh.weight = stod(argv[2]);
		FixedHeuristicTOHTest<14, 7>(seed, argv[1], gh);
	}

	
	int seed = stoi(argv[1]);
	GroupHeuristic<TOHState<24>> gh;
	gh.weight = stod(argv[3]);
	FixedHeuristicTOHTest<24, 12>(seed, argv[2], gh);
	*/
	
	
	return 0;
}