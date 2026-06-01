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
#include "MultiTAS.h"
#include "TASS3.h"
//#include "FastDict.h"
#include "TTBS.h"
//#include "RASFast.h"
#include "RASS.h"
#include "DNode.h"
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
            res = max(res, partialRes);
			if (res == partialRes)
			{
				bestIndex = j;
			}
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
};



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

typedef MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> STPPDB;
void MakeSTPPDBsWeak(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> *h, MNPuzzle<4,4> *mnp)
{
	std::vector<int> p1 = {0,1,4,5};
	std::vector<int> p2 = {0,2,3,6,7};
	std::vector<int> p3 = {0,8,9,12,13};
	std::vector<int> p4 = {0,10,11,14,15};

	STPPDB *pdb1r = new STPPDB(mnp, g, p1);
	STPPDB *pdb2r = new STPPDB(mnp, g, p2);
	STPPDB *pdb3r = new STPPDB(mnp, g, p3);
	STPPDB *pdb4r = new STPPDB(mnp, g, p4);

	pdb1r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb2r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb3r->BuildPDB(g, std::thread::hardware_concurrency(), false);
	pdb4r->BuildPDB(g, std::thread::hardware_concurrency(), false);

	h->lookups.resize(0);
	h->heuristics.resize(0);
	//h.heuristics.push_back(&mnp);
	h->heuristics.push_back(pdb1r);
	h->heuristics.push_back(pdb2r);
	h->heuristics.push_back(pdb3r);
	h->heuristics.push_back(pdb4r);
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


void STPTestHybrid(int problemIndex, double &avg1, double &avg2, double &avg3, double &avg4, string alg)
{
    //srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> start, goal;
    vector<MNPuzzleState<4, 4>> path1, path2, path3;
    start = STP::GetKorfInstance(problemIndex);
    goal = STP::GetKorfInstance((problemIndex + 10) % 100);

    Timer timer1, timer2, timer3;


	if (alg == "tas-t")
	{
		MultiTAS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Temporal);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-T: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-a")
	{
		MultiTAS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Anchor);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-A: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-af")
	{
		MultiTAS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Anchor, Fixed);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-AF: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg == "ttbs-lifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp, 1);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-LIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "ttbs-fifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp, 0);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "dnr")
	{
		DNode<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> dnode(&mnp, start, goal, &mnp, &mnp, 100);
		timer1.StartTimer();
    	dnode.GetPath(path1);
    	timer1.EndTimer();
		avg1 += dnode.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += dnode.pathRatio;
		cout << "DNR: " << dnode.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "gbfs")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi);
		timer1.StartTimer();
    	astar.GetPath(&mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "wa*")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi10);
		timer1.StartTimer();
    	astar.GetPath(&mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "WA*: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
}


void STPTest2(int problemIndex, double &avg1, double &avg2, double &avg3, double &avg4, string alg)
{
    //srandom(problemSeed);
	//TemplateAStar<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5,5>> astar;
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> start, goal;
    vector<MNPuzzleState<4, 4>> path1, path2, path3;
    start = STP::GetKorfInstance(problemIndex);
    goal = STP::GetKorfInstance((problemIndex + 10) % 100);

    Timer timer1, timer2, timer3;


	if (alg == "tas-t")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Temporal);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-T: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-a")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Anchor);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-A: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-af")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(&mnp, start, goal, &mnp, &mnp, 10);
		tasa.SetAnchorSelection(Anchor, Fixed);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-AF: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg == "ras-t")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(&mnp, start, goal, &mnp, &mnp, 10);
		ras.SetAnchorSelection(Temporal);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-T: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg =="ras-a")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(&mnp, start, goal, &mnp, &mnp, 10);
		ras.SetAnchorSelection(Anchor);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-A: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg =="ras-af")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(&mnp, start, goal, &mnp, &mnp, 10);
		ras.SetAnchorSelection(Anchor, Fixed);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-AF: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg == "ttbs-lifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp, 1);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-LIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "ttbs-fifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(&mnp, start, goal, &mnp, &mnp, 0);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "dnr")
	{
		DNode<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> dnode(&mnp, start, goal, &mnp, &mnp, 100);
		timer1.StartTimer();
    	dnode.GetPath(path1);
    	timer1.EndTimer();
		avg1 += dnode.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += dnode.pathRatio;
		cout << "DNR: " << dnode.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "gbfs")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi);
		timer1.StartTimer();
    	astar.GetPath(&mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "wa*")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi10);
		timer1.StartTimer();
    	astar.GetPath(&mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "WA*: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "bgbfs")
	{
		vector<MNPuzzleState<4, 4>> fPath, bPath;
		fPath.clear();
		bPath.clear();
		BidirectionalGreedyBestFirst<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bgbfs;
    	bgbfs.SetPhi(Phi);
		timer1.StartTimer();
    	bgbfs.GetPath(&mnp, start, goal, fPath, bPath);
    	timer1.EndTimer();
		avg1 += bgbfs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += (fPath.size() + bPath.size() - 1);
		cout << "BGBFS: " << bgbfs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << fPath.size() + bPath.size() - 1 << endl;
	}
}




void STPPDBTest(int problemIndex, double &avg1, double &avg2, double &avg3, double &avg4, string alg, int episode)
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
	MakeSTPPDBsWeak(start, hf, mnp);
	MakeSTPPDBsWeak(goal, hb, mnp);
	hm.AddGroup({hf->heuristics[0], hf->heuristics[1], hf->heuristics[2], hf->heuristics[3]});//, hf->heuristics[4], hf->heuristics[5], hf->heuristics[6], hf->heuristics[7]});
	hm.AddGroup({hb->heuristics[0], hb->heuristics[1], hb->heuristics[2], hb->heuristics[3]});//, hb->heuristics[4], hb->heuristics[5], hb->heuristics[6], hb->heuristics[7]});

    //TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    //astar.SetPhi(Phi);
	//astar.SetHeuristic(&hm);
    //TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tas(mnp, start, goal, &hm, &hm, 10);
	//tas.SetAnchorSelection(Anchor, Fixed);
	//tas.SetHeuristicUpdate(STPHUpdate);
	//tas.episode = episode;

    Timer timer1, timer2, timer3;


	//timer1.StartTimer();
    //tas.GetPath(path1);
    //timer1.EndTimer();
	//cout << problemIndex << ": " << tas.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	//
	//timer2.StartTimer();
    //astar.GetPath(mnp, start, goal, path2);
    //timer2.EndTimer();

	if (alg == "tas-t")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(mnp, start, goal, &hm, &hm, 10);
		tasa.SetAnchorSelection(Temporal);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-T: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-a")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(mnp, start, goal, &hm, &hm, 10);
		tasa.SetAnchorSelection(Anchor);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-A: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg =="tas-af")
	{
		TASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> tasa(mnp, start, goal, &hm, &hm, 10);
		tasa.SetAnchorSelection(Anchor, Fixed);
		timer1.StartTimer();
    	tasa.GetPath(path1);
    	timer1.EndTimer();
		avg1 += tasa.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += tasa.pathRatio;
		cout << "TAS-AF: " << tasa.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete tasa.ff;
		delete tasa.bf;
	}
	else if (alg == "ras-t")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(mnp, start, goal, &hm, &hm, 10);
		ras.SetAnchorSelection(Temporal);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-T: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg =="ras-a")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(mnp, start, goal, &hm, &hm, 10);
		ras.SetAnchorSelection(Anchor);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-A: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg =="ras-af")
	{
		RASS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ras(mnp, start, goal, &hm, &hm, 10);
		ras.SetAnchorSelection(Anchor, Fixed);
		timer1.StartTimer();
    	ras.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ras.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ras.pathRatio;
		cout << "RAS-AF: " << ras.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
		delete ras.ff;
		delete ras.bf;
	}
	else if (alg == "ttbs-lifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(mnp, start, goal, &hm, &hm, 1);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-LIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "ttbs-fifo")
	{
		TTBS<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> ttbs(mnp, start, goal, &hm, &hm, 0);
		timer1.StartTimer();
    	ttbs.GetPath(path1);
    	timer1.EndTimer();
		avg1 += ttbs.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += ttbs.pathRatio;
		cout << "TTBS-FIFO: " << ttbs.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "dnr")
	{
		DNode<MNPuzzle<4, 4>, MNPuzzleState<4, 4>> dnode(mnp, start, goal, &hm, &hm, 100);
		timer1.StartTimer();
    	dnode.GetPath(path1);
    	timer1.EndTimer();
		avg1 += dnode.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		avg4 += dnode.pathRatio;
		cout << "DNR: " << dnode.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "gbfs")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi);
		astar.SetHeuristic(&hm);
		timer1.StartTimer();
    	astar.GetPath(mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "GBFS: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}
	else if (alg == "wastar")
	{
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    	astar.SetPhi(Phi10);
		astar.SetHeuristic(&hm);
		timer1.StartTimer();
    	astar.GetPath(mnp, start, goal, path1);
    	timer1.EndTimer();
		avg1 += astar.GetNodesExpanded();
		avg2 += timer1.GetElapsedTime();
		avg3 += path1.size();
		cout << "WA*: " << astar.GetNodesExpanded() << " " << timer1.GetElapsedTime() << " " << path1.size() << endl;
	}

	//cout << astar.GetNodesExpanded() << " " << timer2.GetElapsedTime() << " " << path2.size() << endl;
	//avg1 += tas.GetNodesExpanded();
	//avg2 += timer1.GetElapsedTime();
	//avg3 += path1.size();
	//avg4 += tas.pathRatio;
}


int main(int argc, char* argv[])
{
	
	
	double avg1 = 0;
	double avg2 = 0;
	double avg3 = 0;
	double avg4 = 0;
	//STPTest(stoi(argv[1]), avg1, avg2);
	
	
	for (int i = 0; i < 100; i++)
	{
		STPTest2(i, avg1, avg2, avg3, avg4, argv[1]);
		//if (stoi(argv[2]) == 1)
		//	STPTestHybrid(i, avg1, avg2, avg3, avg4, argv[1]);
		//else
		//	STPTest2(i, avg1, avg2, avg3, avg4, argv[1]);
	}
	std::cout << avg1 / 100.0 << " " << avg2 / 100.0 << " " << avg3 / 100.0 << " " << avg4 / 100.0 << std::endl;
	
	//for (int i = 0; i < 100; i++)
	//{
	//	STPPDBTest(i, avg1, avg2, avg3, avg4, argv[1], -1);
	//}
	//std::cout << avg1 / 100.0 << " " << avg2 / 100.0 << " " << avg3 / 100.0 << " " << avg4 / 100.0 << std::endl;
	return 0;
}