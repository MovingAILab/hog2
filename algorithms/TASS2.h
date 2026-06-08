#include <set>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include "Map.h"
#include <cmath>
#include <algorithm>
#include "MR1PermutationPDB.h"
#include <math.h>

enum kAnchorSelection2
{
    Temporal2,
    Closest2,
    Random2
};

template<class State>
struct OpenClosedData2
{
public:

    double g = 0;
    State parent;
    int loc = -1;
    int dir;
};


template<class Env, class State>
class TASS2;

template <class Env, class State>
class TASS2Frontier
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	std::vector<State> open;
	std::set<uint64_t> closed;
	//std::unordered_map<uint64_t, double> gValues;
	//std::unordered_map<uint64_t, State> parents;
	//std::unordered_map<uint64_t, int> loc;
    TASS2<Env, State>* tss;
    //std::unordered_map<uint64_t, OpenClosedData2<State>>* openClosed;
	std::vector<int> occurrences;
	std::vector<int> _occurrences;
	int numOfExp = 0;
	State rendezvous;
	TASS2Frontier *other;
    int sampleCount;
	State anchor;
	State start;
    Heuristic<State> *h;
	TASS2Frontier(Env *_env, State _start, Heuristic<State> *h, int _sampleCount, int _dir, TASS2<Env, State>* _tss);
	~TASS2Frontier(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
    double anchorH, anchorG;

    kAnchorSelection2 anchorSelection;
    int dir;

	//double comps = 0;



	double AdditiveDH(Heuristic<State> *heu, State s1, State s2)
	{
		double res = 0;
	    for (int i = 0; i < heu->heuristics.size(); i++)
	    {
	    	res += fabs(heu->heuristics[i]->HCost(s1, s1) - heu->heuristics[i]->HCost(s2, s2));
	    }
		return res;
	}

	double MaxDH(State s1, State s2)
	{
		return max(AdditiveDH(h, s1, s2), AdditiveDH(other->h, s1, s2));
	}

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));

		return h->HCost(s1, s2); 
		//return MaxDH(s1, s2);
	}

};


template <class Env,  class State>
TASS2Frontier<Env, State>::TASS2Frontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount, int _dir, TASS2<Env, State>* _tss)
{
    tss = _tss;
    dir = _dir;
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	h = _h;
	start = _start;
    //openClosed = _openClosed;
	open.push_back(start);
    OpenClosedData2<State> data;
    data.g = 0;
    data.loc = 0;
    data.parent = start;
    data.dir = dir;
    tss->openClosed1.insert({env->GetStateHash(start), data});
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void TASS2Frontier<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = rendezvous;
	while (true)
	{
		path.push_back(current);
		if (tss->openClosed1.find(env->GetStateHash(current)) != tss->openClosed1.end())
			current = tss->openClosed1[env->GetStateHash(current)].parent;
		else
			break;
	}
	path.push_back(start);
}




template <class Env,  class State>
bool TASS2Frontier<Env, State>::DoSingleSearchStep()
{
	if (open.size() == 0)
	{
		//std::cout << "******* No Solution Found! ********" << std::endl;
		validSolution = false;
		return true;
	}
		
	double minDist = 99999999;
	State bestCandidate = open.back();
	uint64_t bestCandidateHash = env->GetStateHash(bestCandidate);
	int _samples = sampleCount;
	int index = open.size() - 1;
    while (index >= 0 && _samples > 0)
    {
        State c = open[index];
        auto hash = env->GetStateHash(c);
		auto dist = HCost(c, other->anchor);
		if (dist < minDist || dist == minDist && tss->openClosed1[hash].g > tss->openClosed1[bestCandidateHash].g)
		{
			minDist = dist;
			bestCandidate = c;
			bestCandidateHash = env->GetStateHash(bestCandidate);
		}
        index--;
		_samples--;
    }
	open[tss->openClosed1[bestCandidateHash].loc] = open.back();
	tss->openClosed1[env->GetStateHash(open.back())].loc = tss->openClosed1[bestCandidateHash].loc;
	open.pop_back();
	tss->openClosed1[bestCandidateHash].loc = -1;
	//closed.insert(bestCandidateHash);
	//_occurrences[int(HCost(bestCandidate, other->anchor))]++;

	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		//occurrences[int(HCost(neighbor, other->anchor))]++;
		auto nhash = env->GetStateHash(neighbor);
		double g = tss->openClosed1[bestCandidateHash].g + env->GCost(bestCandidate, neighbor);
        auto ent = tss->openClosed1.find(nhash);
		if (ent != tss->openClosed1.end())
		{
            if (ent->second.dir == dir)
            {
                if (g < ent->second.g)
			    {
			    	ent->second.g = g;
			    	ent->second.parent = bestCandidate;
			    }
			    else
			    {
			    	open[ent->second.loc] = open.back();
			        tss->openClosed1[env->GetStateHash(open.back())].loc = ent->second.loc;
			        open[open.size() - 1] = neighbor;
			        ent->second.loc = open.size() - 1;
			    }
            }
            else
            {
                rendezvous = bestCandidate;
                other->rendezvous = neighbor;
                return true;
            }
		}
		else
		{
			//gValues[nhash] = g;
            std::cout << open.size() << " " << nhash << std::endl;
			open.push_back(neighbor);
            OpenClosedData2<State> data;
            data.g = g;
            data.loc = open.size() - 1;
            data.parent = bestCandidate;
            data.dir = dir;
            tss->openClosed1[nhash] = data;
			//loc.insert({nhash, open.size() - 1});
			//parents[nhash] = bestCandidate;
		}
	}

    switch (anchorSelection)
    {
        case Temporal2:
            anchor = bestCandidate;
            break;
        case Closest2:
		{
			anchorH = HCost(anchor, other->start);
			auto hh = HCost(bestCandidate, other->start);
            if (hh < anchorH)
            {
                anchor = bestCandidate;
                anchorH = hh;
                anchorG = tss->openClosed1[bestCandidateHash].g;
            }
            else if (hh == anchorH)
            {
                if (tss->openClosed1[bestCandidateHash].g < anchorG)
                {
                    anchor = bestCandidate;
                    anchorG = tss->openClosed1[bestCandidateHash].g;
                }
            }
            break;
		}
        case Random2:
		{
			int Random2_index = rand_r(&seed) % open.size();
            anchor = open[Random2_index];
            break;
		}
		default:
			break;
    }

    
	//anchor = bestCandidate;
    
    /*
	if ((*(other->openClosed)).find(bestCandidateHash) != (*(other->openClosed)).end())// || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
    */
	return false;
}


template <class Env,  class State>
void TASS2Frontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}

template <class Env, class State>
class TASS2
{
private:
	int turn = 0;
public:
	int episode = -1;

    std::unordered_map<uint64_t, OpenClosedData2<State>> openClosed1, openClosed2;
	TASS2Frontier<Env, State>* ff;
	TASS2Frontier<Env, State>* bf;
	TASS2();
	TASS2(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount);
	~TASS2(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
	{
		ff = new TASS2Frontier<Env, State>(_env, _start, hf, _sampleCount);
		bf = new TASS2Frontier<Env, State>(_env, _goal, hb, _sampleCount);
		ff->other = bf;
		bf->other = ff;
		ff.anchorH = ff.HCost(_start, _goal);
		bf.anchorH = bf.HCost(_goal, _start);
	}
	std::function<void(State, State, Heuristic<State>*&)> heuristicUpdate;
	void SetHeuristicUpdate(std::function<void(State, State, Heuristic<State>*&)> p)
	{
		heuristicUpdate = p;
	}
	int GetPathLength()
	{

	}
	int GetNodesExpanded()
	{
		return ff->numOfExp + bf->numOfExp;
	}
	bool DoSingleSearchStep()
	{
		if (turn == 0)
		{
			turn = 1;
			return ff->DoSingleSearchStep();
		}
		else
		{
			turn = 0;
			return bf->DoSingleSearchStep();
		}
	}
	void ExtractPath(std::vector<State> &path)
	{
		auto current = bf->rendezvous;
		std::vector<State> front, back;
		while (true)
		{
			back.push_back(current);
			if (openClosed1.find(bf->env->GetStateHash(current))->second.parent != current)
				current = openClosed1[bf->env->GetStateHash(current)].parent;
			else
				break;
		}
		back.push_back(bf->start);
		current = ff->rendezvous;
		while (true)
		{
			front.push_back(current);
			if (openClosed1.find(ff->env->GetStateHash(current))->second.parent != current)
				current = openClosed1[ff->env->GetStateHash(current)].parent;
			else
				break;
		}
		front.push_back(ff->start);
		path.resize(0);
		for (int i = front.size() - 1; i >= 0; i--)
			path.push_back(front[i]);
		for (int i = 0; i < back.size(); i++)
			path.push_back(back[i]);
	}
	void GetPath(std::vector<State> &path)
	{
		path.resize(0);
		int count = 0;
		while (true)
		{
			if (episode > 0 && count % episode == 0)
			{
				//Heuristic<State> *h1 = new Heuristic<State>();
				heuristicUpdate(ff->anchor, bf->anchor, bf->h);
				//bf->h = h1;
				//std::cout << "TEST: " << bf->h->HCost(ff->anchor, bf->anchor) << std::endl;

				//Heuristic<State> *h2 = new Heuristic<State>();
				//heuristicUpdate(bf->anchor, ff->h);
				ff->h = bf->h;
				//std::cout << "TEST: " << bf->h->HCost(ff->anchor, bf->anchor) << std::endl;
				std::cout << "episode(" << episode << "): " << count / episode << std::endl;
				std::cout << ff->HCost(ff->anchor, bf->start) << " " << ff->anchorH << std::endl;
			}
			count++;
			if (DoSingleSearchStep())
			{
				break;
			}
		}
		if (!ff->validSolution || !bf->validSolution)
			return;
        //std::cout << "Found" << std::endl;
		ExtractPath(path);
		//std::cout << "-------------" << std::endl;
		//for (int i = 0; i < 100; i++)
		//{
		//	std::cout << i << ": " << ff->occurrences[i] + bf->occurrences[i] << std::endl;
		//}
		//std::cout << "-------------" << std::endl;
		//for (int i = 0; i < 100; i++)
		//{
		//	std::cout << i << ": " << ff->_occurrences[i] + bf->_occurrences[i] << std::endl;
		//}
		//std::cout << "-------------" << std::endl;
	}
	void SetSeed(unsigned int seed)
	{
		this.seed = seed;
	}

    void SetAnchorSelection(kAnchorSelection2 selection)
    {
        ff->anchorSelection = selection;
        bf->anchorSelection = selection;
    }
};


template <class Env, class State>
TASS2<Env, State>::TASS2(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
    openClosed1.clear();
	ff = new TASS2Frontier<Env, State>(_env, _start, hf, _sampleCount, 1, this);
	bf = new TASS2Frontier<Env, State>(_env, _goal, hb, _sampleCount, 2, this);
	ff->other = bf;
	bf->other = ff;
}
