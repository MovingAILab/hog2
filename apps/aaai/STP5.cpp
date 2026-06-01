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
#include "TASS3.h"
//#include "FastDict.h"
#include "TTBS.h"
//#include "RASFast.h"
#include "RASS.h"
#include "DNode.h"

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
    double HCost(const state &state1, const state &state2) const
    {
        double res = 0;
		int bestIndex = 0;
        for (int j = 0; j < groups.size(); j++)
        {
			auto group = groups[j];
            double partialRes = 0;
            for (auto h : group->heuristics)
            {
                partialRes += fabs(h->HCost(state1, state1) - h->HCost(state2, state2));
            }
			//res += partialRes;
            res = max(res, partialRes);
			if (res == partialRes)
			{
				bestIndex = j;
				//groups[j].second++;
			}
        }
		//groups[bestIndex]->count++;// = HeuristicGroup(groups[bestIndex].indices, groups[bestIndex].count);
	    return res;
    	//return abs(baseH.HCost(state1, state1) - baseH.HCost(state2, state2));
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
	while(mhh->groups.size() > 2)
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

void Test5(int problemSeed)
{
    srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<5,5> mnp;
	MNPuzzleState<5, 5> start, goal;
    vector<MNPuzzleState<5, 5>> path1, path2;
    start = mnp.Generate_Random_Puzzle();
    goal = mnp.Generate_Random_Puzzle();
    cout << mnp.HCost(start, goal) << endl;
    cout << start << endl;
    cout << goal << endl;

    TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
    astar.SetPhi(Phi);
    TASA<MNPuzzle<5,5>, MNPuzzleState<5,5>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
    Timer timer1, timer2;
    
    timer1.StartTimer();
    astar.GetPath(&mnp, start, goal, path1);
    timer1.EndTimer();
    cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;

    timer2.StartTimer();
    tasa.GetPath(path2);
    timer2.EndTimer();
    cout << "TASA: " << tasa.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
}


template<int disks>
void TOHTest(int problemSeed)
{
    srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	TOH<disks> toh;
	TOHState<disks> start, goal;
    vector<TOHState<disks>> path1, path2;
    MultipleAdditiveHeuristic<TOHState<disks>> h;

    start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
	for (int x = disks; x > 0; x--)
	{
		int whichPeg = random()%4;
		start.disks[whichPeg][start.counts[whichPeg]] = x;
		start.counts[whichPeg]++;
	}
	goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
	for (int x = disks; x > 0; x--)
	{
		int whichPeg = random()%4;
		goal.disks[whichPeg][goal.counts[whichPeg]] = x;
		goal.counts[whichPeg]++;
	}

    cout << start << endl;
    cout << goal << endl;

    auto fh = BuildPDB<disks, disks / 2>(goal);
    auto bh = BuildPDB<disks, disks / 2>(start);
    h.AddHeuristic(fh->heuristics[0]);
    h.AddHeuristic(fh->heuristics[1]);
    h.AddHeuristic(bh->heuristics[0]);
    h.AddHeuristic(bh->heuristics[1]);
    h.AddGroup({0, 1});
    h.AddGroup({2, 3});


    TemplateAStar<TOHState<disks>, TOHMove, TOH<disks>> astar;
    astar.SetPhi(Phi);
    astar.SetHeuristic(&h);
    TASA<TOH<disks>, TOHState<disks>> tasa(&toh, start, goal, &h, &h, 10);
    Timer timer1, timer2;
    
    timer1.StartTimer();
    astar.GetPath(&toh, start, goal, path1);
    timer1.EndTimer();
    cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;

    timer2.StartTimer();
    tasa.GetPath(path2);
    timer2.EndTimer();
    cout << "TASA: " << tasa.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
}

/*
template<int disks>
void TASOptTest(int problemSeed)
{
    srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	TOH<disks> toh;
	TOHState<disks> start, goal;
    vector<TOHState<disks>> path1, path2;
    MultipleAdditiveHeuristic<TOHState<disks>> h;

    start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
	for (int x = disks; x > 0; x--)
	{
		int whichPeg = random()%4;
		start.disks[whichPeg][start.counts[whichPeg]] = x;
		start.counts[whichPeg]++;
	}
	goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
	for (int x = disks; x > 0; x--)
	{
		int whichPeg = random()%4;
		goal.disks[whichPeg][goal.counts[whichPeg]] = x;
		goal.counts[whichPeg]++;
	}

    cout << start << endl;
    cout << goal << endl;

    auto fh = BuildPDB<disks, disks / 2>(goal);
    auto bh = BuildPDB<disks, disks / 2>(start);
    h.AddHeuristic(fh->heuristics[0]);
    h.AddHeuristic(fh->heuristics[1]);
    h.AddHeuristic(bh->heuristics[0]);
    h.AddHeuristic(bh->heuristics[1]);
    h.AddGroup({0, 1});
    h.AddGroup({2, 3});


    TemplateAStar<TOHState<disks>, TOHMove, TOH<disks>> astar;
    astar.SetPhi(Phi);
    astar.SetHeuristic(&h);
    //TASA<TOH<disks>, TOHState<disks>> tasa(&toh, start, goal, &h, &h, 10);
    TASOpt<TOH<disks>, TOHState<disks>> tasOpt(&toh, start, goal, &h, &h, 10);
    Timer timer1, timer2;

    //timer2.StartTimer();
    //tasa.GetPath(path2);
    //timer2.EndTimer();
    //cout << "TASA: " << tasa.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
    

    timer1.StartTimer();
    astar.GetPath(&toh, start, goal, path1);
    timer1.EndTimer();
    cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;

    timer2.StartTimer();
    tasOpt.GetPath(path2);
    timer2.EndTimer();
    cout << "TASO: " << tasOpt.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
    

    delete fh;
    delete bh;
}
*/

template<int numOfDisks, int pdb1Disks>
void PlanningTest(int seed_problem, int samples, kAnchorSelection selection, int episode, GroupHeuristic<TOHState<numOfDisks>> &hm)
{
	std::cout << "TAS(" << samples << ")" << std::endl;
    srandom(seed_problem);
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    TOHState<numOfDisks> start, goal, aux;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path, aPath, fPath, bPath;
	//aux.Reset();

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
	/*
	aux.counts[0] = aux.counts[1] = aux.counts[2] = aux.counts[3] = 0;
	for (int x = numOfDisks; x > 0; x--)
	{
		int whichPeg = random()%4;
		aux.disks[whichPeg][aux.counts[whichPeg]] = x;
		aux.counts[whichPeg]++;
	}
	*/
	
	Heuristic<TOHState<numOfDisks>> spec1, spec2, spec3;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});
	

	//BuildTOHPDB<numOfDisks, pdb1Disks>(start, goal, aux, hm);

    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;
	TASS<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, samples);
	tas.episode = episode;
	tas.SetHeuristicUpdate(HUpdate<numOfDisks, pdb1Disks, 2>);
    tas.SetAnchorSelection(selection, Fixed);
    Timer timer, timer2, timer3;
    timer.StartTimer();
	tas.GetPath(path);
    timer.EndTimer();
	cout << "Info:" << endl;
	cout << "Anchor Selection: " << selection << endl;
	cout << "Heuristic: " << endl;
	cout << "PDB size: " << pdb1Disks << endl;
	cout << "episode length: " << episode << endl;
	cout << "TAS: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
}


template<int numOfDisks, int pdb1Disks>
void PlanningTest2(int seed_problem, int samples, kAnchorSelection2 selection, int episode, GroupHeuristic<TOHState<numOfDisks>> hm)
{
	std::cout << "TAS(" << samples << ")" << std::endl;
    srandom(seed_problem);
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    TOHState<numOfDisks> start, goal, pivot1, pivot2;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path, aPath, fPath, bPath;

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
	Heuristic<TOHState<numOfDisks>> spec1, spec2;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});

    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;
	TASS2<TOH<numOfDisks>, TOHState<numOfDisks>> tas(toh, start, goal, &hm, &hm, samples);
	tas.episode = episode;
	tas.SetHeuristicUpdate(HUpdate<numOfDisks, pdb1Disks, 2>);
    tas.SetAnchorSelection(selection);
	//astar.SetPhi(phi);
	//astar.SetHeuristic(&hm);
    Timer timer, timer2, timer3;
    timer.StartTimer();
	tas.GetPath(path);
    timer.EndTimer();
	cout << "Info:" << endl;
	cout << "Anchor Selection: " << selection << endl;
	cout << "Heuristic: " << endl;
	cout << "PDB size: " << pdb1Disks << endl;
	cout << "episode length: " << episode << endl;
	cout << "TAS: " << tas.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
}


template<int numOfDisks, int pdb1Disks>
void PlanningTest3(int seed_problem, int lifo, GroupHeuristic<TOHState<numOfDisks>> hm)
{
    srandom(seed_problem);
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    TOHState<numOfDisks> start, goal, pivot1, pivot2;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path, aPath, fPath, bPath;

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
	Heuristic<TOHState<numOfDisks>> spec1, spec2;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});

    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;
	//DNode<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, 100);
	TTBS<TOH<numOfDisks>, TOHState<numOfDisks>> ttbs(toh, start, goal, &hm, &hm, lifo);
    Timer timer, timer2, timer3;
    timer.StartTimer();
	ttbs.GetPath(path);
    timer.EndTimer();
	cout << "Info:" << endl;
	cout << "Heuristic: " << endl;
	cout << "PDB size: " << pdb1Disks << endl;
	cout << "TTBS: " << ttbs.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
}


template<int numOfDisks, int pdb1Disks>
void GBFSTest(int seed_problem, GroupHeuristic<TOHState<numOfDisks>> hm)
{
    srandom(seed_problem);
    TOH<numOfDisks> *toh = new TOH<numOfDisks>();
    TOHState<numOfDisks> start, goal, pivot1, pivot2;
    Heuristic<TOHState<numOfDisks>> hf, hb, h1, h2, h3, h4;
    vector<TOHState<numOfDisks>> path, aPath, fPath, bPath;

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
	Heuristic<TOHState<numOfDisks>> spec1, spec2;
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec1, start);
	BuildSinglePDB<numOfDisks, pdb1Disks>(spec2, goal);
	hm.AddGroup({spec1.heuristics[0]});
	hm.AddGroup({spec2.heuristics[0]});

    cout << start << endl;
    cout << goal << endl;
    std::cout << "------------------------------------" << std::endl;
	TemplateAStar<TOHState<numOfDisks>, TOHMove, TOH<numOfDisks>> astar;
	astar.SetPhi(Phi);
	astar.SetHeuristic(&hm);
    Timer timer, timer2, timer3;
    timer.StartTimer();
	astar.GetPath(toh, start, goal, path);
    timer.EndTimer();
	cout << "Heuristic: " << endl;
	cout << "PDB size: " << pdb1Disks << endl;
	cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer.GetElapsedTime() << " " << path.size() << endl;
}


void STPTest(int problemIndex, double &avg1, double &avg2)
{
    //srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> start, goal;
    vector<MNPuzzleState<4, 4>> path1, path2, path3;
    start = STP::GetKorfInstance(problemIndex);
    goal = STP::GetKorfInstance((problemIndex + 10) % 100);
    cout << mnp.HCost(start, goal) << endl;
    cout << start << endl;
    cout << goal << endl;

    TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    astar.SetPhi(Phi);
    TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 8);
	tasa.SetAnchorSelection(Closest);
	RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(&mnp, start, goal, &mnp, &mnp, 20);
	ras.SetAnchorSelection(Closest);
	TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp);
    Timer timer1, timer2, timer3;
    
    timer1.StartTimer();
    astar.GetPath(&mnp, start, goal, path1);
    timer1.EndTimer();
    cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;

    timer2.StartTimer();
    ras.GetPath(path2);
    timer2.EndTimer();
    cout << "RASS: " << ras.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << " " << ras.pathRatio << endl;

	avg1 += astar.GetNodesExpanded();
	avg2 += ras.GetNodesExpanded();
	//timer3.StartTimer();
    //ttbs.GetPath(path3);
    //timer3.EndTimer();
    //cout << "TTBS: " << ttbs.GetNodesExpanded() << " " << timer3.GetElapsedTime() << " " << path3.size() << endl;
}


void STPTest2(int problemIndex, double &avg1, double &avg2, double &avg3, double &avg4, string alg, int variant, int parameter)
{
    //srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> start, goal;
    vector<MNPuzzleState<4, 4>> path1, path2, path3;
    start = STP::GetKorfInstance(problemIndex);
    goal = STP::GetKorfInstance((problemIndex + 10) % 100);
    //cout << mnp.HCost(start, goal) << endl;
    //cout << start << endl;
    //cout << goal << endl;

    TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    astar.SetPhi(Phi);
    TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, parameter);
	tasa.SetAnchorSelection((kAnchorSelection)variant);
	RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(&mnp, start, goal, &mnp, &mnp, parameter);
	ras.SetAnchorSelection((kAnchorSelection)variant);
	TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp);
	DNode<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> dnode(&mnp, start, goal, &mnp, &mnp, parameter);
    Timer timer1, timer2, timer3;


	if (alg == "tas")
	{
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg =="ras")
	{
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "ttbs")
	{
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "dnode")
	{
		timer1.StartTimer();
    	dnode.GetPath(path1);
    	timer1.EndTimer();
		avg1 += dnode.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += dnode.pathRatio;
		cout << dnode.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "gbfs")
	{
		timer1.StartTimer();
    	astar.GetPath(&mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
}




void STPPDBTest(int problemIndex, int episode)
{
    //srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<4, 4> *mnp = new MNPuzzle<4, 4>();
	MNPuzzleState<4, 4> start, goal;
    vector<MNPuzzleState<4, 4>> path1, path2, path3;
    start = STP::GetKorfInstance(problemIndex);
    goal = STP::GetKorfInstance((problemIndex + 10) % 100);

	Heuristic<MNPuzzleState<4, 4>>* hf = new Heuristic<MNPuzzleState<4, 4>>();
	Heuristic<MNPuzzleState<4, 4>>* hb = new Heuristic<MNPuzzleState<4, 4>>();


	GroupHeuristic<MNPuzzleState<4, 4>> hm;
	MakeSTPPDBs(start, hf, mnp);
	MakeSTPPDBs(goal, hb, mnp);
	hm.AddGroup({hf->heuristics[0], hf->heuristics[1], hf->heuristics[2], hf->heuristics[3], hf->heuristics[4], hf->heuristics[5], hf->heuristics[6], hf->heuristics[7]});
	hm.AddGroup({hb->heuristics[0], hb->heuristics[1], hb->heuristics[2], hb->heuristics[3], hb->heuristics[4], hb->heuristics[5], hb->heuristics[6], hb->heuristics[7]});

    TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    astar.SetPhi(Phi);
	astar.SetHeuristic(&hm);
    TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tas(mnp, start, goal, &hm, &hm, 10);
	tas.SetAnchorSelection(Anchor);
	tas.SetHeuristicUpdate(STPHUpdate);
	tas.episode = episode;

    Timer timer1, timer2, timer3;


	timer1.StartTimer();
    tas.GetPath(path1);
    timer1.EndTimer();
	cout << tas.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	timer2.StartTimer();
    astar.GetPath(mnp, start, goal, path2);
    timer2.EndTimer();
	cout << astar.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
}


int main(int argc, char* argv[])
{
	//TOHTest<18>(stoi(argv[1]));
    /*
    int size = stoi(argv[1]);
    if (size == 14)
    {
        for (int i = 1; i <= 100; i++)
            TASOptTest<14>(i);
    }
    else if (size == 16)
    {
        for (int i = 1; i <= 100; i++)
            TASOptTest<16>(i);
    }
    else if (size == 18)
    {
        TASOptTest<18>(stoi(argv[2]));
    }
    else if (size == 20)
    {
        TASOptTest<20>(stoi(argv[2]));
    }
    else if (size == 22)
    {
        TASOptTest<22>(stoi(argv[2]));
    }
    */
   	
	/*
   	int problemSize = stoi(argv[1]);
	if (problemSize == 16)
	{
		GroupHeuristic<TOHState<16>> gh;
    	PlanningTest<16, 10>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 20)
	{
		GroupHeuristic<TOHState<20>> gh;
    	PlanningTest<20, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 22)
	{
		GroupHeuristic<TOHState<22>> gh;
    	PlanningTest<22, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 24)
	{
		GroupHeuristic<TOHState<24>> gh;
    	PlanningTest<24, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 26)
	{
		GroupHeuristic<TOHState<26>> gh;
    	PlanningTest<26, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 28)
	{
		GroupHeuristic<TOHState<28>> gh;
    	PlanningTest<28, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 30)
	{
		GroupHeuristic<TOHState<30>> gh;
    	PlanningTest<30, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else if (problemSize == 32)
	{
		GroupHeuristic<TOHState<32>> gh;
    	PlanningTest<32, 12>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	
	*/

	
    const int psize = 20;
	const int pdbsize = 10;

	//GroupHeuristic<TOHState<psize>> gh3;
    //PlanningTest3<psize, pdbsize>(stoi(argv[1]), gh3);

	if (stoi(argv[1]) == 1)
	{
		GroupHeuristic<TOHState<psize>> gh2;
   		GBFSTest<psize, pdbsize>(stoi(argv[2]), gh2);
	}
	else if (stoi(argv[1]) == 0)
	{
		GroupHeuristic<TOHState<psize>> gh;
    	PlanningTest<psize, pdbsize>(stoi(argv[2]), 10, (kAnchorSelection)stoi(argv[3]), stoi(argv[4]), gh);
	}
	else 
	{
		GroupHeuristic<TOHState<psize>> gh;
    	PlanningTest3<psize, pdbsize>(stoi(argv[2]), stoi(argv[4]), gh);
	}
	
	
	//GroupHeuristic<TOHState<psize>> gh1;
    //PlanningTest2<psize, pdbsize>(stoi(argv[1]), 10, (kAnchorSelection2)stoi(argv[2]), stoi(argv[3]), gh1);
	
	
	/*
	double avg1 = 0;
	double avg2 = 0;
	double avg3 = 0;
	double avg4 = 0;
	//STPTest(stoi(argv[1]), avg1, avg2);
	
	for (int i = 0; i < 100; i++)
	{
		STPTest2(i, avg1, avg2, avg3, avg4, argv[1], stoi(argv[2]), stoi(argv[3]));
	}
	std::cout << avg1 / 100.0 << " " << avg2 / 100.0 << " " << avg3 / 100.0 << std::endl;
	*/

	//STPPDBTest(stoi(argv[1]), stoi(argv[2]));
	return 0;
}