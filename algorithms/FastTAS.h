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
#include "AnchorSearchUtils.h"

template<class State>
class FastIndexData
{
public:
    FastIndexData()
    {
        index = -1;
        //parent = NULL;
        g = 0;
    }
    FastIndexData(int _index, State _parent, double _g)
    {
        index = _index;
        parent = _parent;
        g = _g;
        //hash = _hash;
    }
    int index;
    State parent;
    double g;
    //uint64_t hash;
};


template <class Env, class State>
class FastTASFrontier
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	std::vector<std::pair<State, uint64_t>> open;
	std::vector<FastIndexData<State>> loc;
	int numOfExp = 0;
	State rendezvous;
	FastTASFrontier *other;
    int sampleCount;
	State anchor;
	State start;
    Heuristic<State> *h;
	FastTASFrontier(Env *_env, State _start, Heuristic<State> *h, int _sampleCount);
	~FastTASFrontier(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
    kAnchorSelection anchorSelection;
    double anchorH = -1;
    double anchorG = 0;
	//double comps = 0;

	

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
FastTASFrontier<Env, State>::FastTASFrontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	//open.reserve(env->GetMaxHash());
	h = _h;
	start = _start;
    auto hash = env->GetStateHash(start);
	open.push_back(std::make_pair(start, hash));
	//gValues[env->GetStateHash(start)] = 0;
	loc.resize(env->GetMaxHash());
	loc[env->GetStateHash(start)] = FastIndexData<State>(0, start, 0);
	anchor = start;
    anchorG = 0;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void FastTASFrontier<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = rendezvous;
	while (true)
	{
		path.push_back(current);
		if (loc[env->GetStateHash(current)].index != -1)
			current = loc[env->GetStateHash(current)].parent;
		else
			break;
	}
	path.push_back(start);
}




template <class Env,  class State>
bool FastTASFrontier<Env, State>::DoSingleSearchStep()
{
	if (open.size() == 0)
	{
		//std::cout << "******* No Solution Found! ********" << std::endl;
		validSolution = false;
		return true;
	}
		
	double minDist = 99999999;
    auto bestCandidateData = open.back();
	State bestCandidate = bestCandidateData.first;
	uint64_t bestCandidateHash = bestCandidateData.second;//env->GetStateHash(bestCandidate);
	int _samples = sampleCount;
	int index = open.size() - 1;
    while (index >= 0 && _samples > 0)
    {
        State c = open[index].first;
        auto hash = open[index].second;//env->GetStateHash(c);
		auto dist = HCost(c, other->anchor);
		if (dist < minDist || dist == minDist && loc[hash].g > loc[bestCandidateHash].g)
		{
			minDist = dist;
			bestCandidate = c;
			bestCandidateHash = hash;
		}
        index--;
		_samples--;
    }
	open[loc[bestCandidateHash].index] = open.back();
	loc[open.back().second].index = loc[bestCandidateHash].index;
	open.pop_back();
	loc[bestCandidateHash].index = -2;
	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		auto nhash = env->GetStateHash(neighbor);
		double g = loc[bestCandidateHash].g + env->GCost(bestCandidate, neighbor);
        if (other->loc[nhash].index != -1)// || bestCandidate == other->start)
		{
			rendezvous = bestCandidate;
			other->rendezvous = neighbor;
			return true;
		}
		if (loc[nhash].index != -1)
		{
			if (g < loc[nhash].g)
			{
				loc[nhash].g = g;
				loc[nhash].parent = bestCandidate;
			}
			else
			{
				if (loc[nhash].index != -2)
				{
					open[loc[nhash].index] = open.back();
					loc[open.back().second].index = loc[nhash].index;
					open[open.size() - 1] = std::make_pair(neighbor, nhash);//neighbor;
					loc[nhash].index = open.size() - 1;
				}
			}
		}
		else
		{
			loc[nhash].g = g;
			open.push_back(std::make_pair(neighbor, nhash));
			loc[nhash] = FastIndexData<State>(open.size() - 1, bestCandidate, g);
			//parents[nhash] = bestCandidate;
		}
	}
	//anchor = bestCandidate;
    switch (anchorSelection)
    {
        case Temporal:
            anchor = bestCandidate;
            break;
        case Closest:
		{
			auto hh = HCost(bestCandidate, other->start);
            if (hh < anchorH || anchorH < 0)
            {
                anchor = bestCandidate;
                anchorH = hh;
                anchorG = loc[bestCandidateHash].g;
				//std::cout << numOfExp << ": " << HCost(anchor, other->anchor) << " " << HCost(anchor, start) << " " << HCost(other->anchor, start) << " " << HCost(anchor, other->start) << " " << HCost(other->anchor, other->start) << std::endl;
            }
            else if (hh == anchorH)
            {
                if (loc[bestCandidateHash].g > anchorG)
                {
                    anchor = bestCandidate;
                    anchorG = loc[bestCandidateHash].g;
					//std::cout << numOfExp << ": " << HCost(anchor, other->anchor) << " " << HCost(anchor, start) << " " << HCost(other->anchor, start) << " " << HCost(anchor, other->start) << " " << HCost(other->anchor, other->start) << std::endl;
                }
            }
			break;
		}
		case Fixed:
		{
			break;
		}
		case Anchor:
		{
			if (HCost(bestCandidate, other->anchor) < HCost(anchor, other->anchor))
            {
                anchor = bestCandidate;
                anchorG = loc[bestCandidateHash].g;
				//std::cout << "New anchor: " << anchorH << " " << anchorG << endl;
            }
			break;
		}
		default:
			break;
    }


	if (other->loc[bestCandidateHash].index != -1)// || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
	return false;
}


template <class Env,  class State>
void FastTASFrontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}

template <class Env, class State>
class FastTAS
{
private:
	int turn = 0;
public:
    double pathRatio;
	FastTASFrontier<Env, State>* ff;
	FastTASFrontier<Env, State>* bf;
	FastTAS();
	FastTAS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount);
	~FastTAS(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
	{
		ff = new FastTASFrontier<Env, State>(_env, _start, hf, _sampleCount);
		bf = new FastTASFrontier<Env, State>(_env, _goal, hb, _sampleCount);
		ff->other = bf;
		bf->other = ff;
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
			if (bf->loc[bf->env->GetStateHash(current)].parent != bf->start)
				current = bf->loc[bf->env->GetStateHash(current)].parent;
			else
				break;
		}
		back.push_back(bf->start);
		current = ff->rendezvous;
		while (true)
		{
			front.push_back(current);
			if (ff->loc[ff->env->GetStateHash(current)].parent != ff->start)
				current = ff->loc[ff->env->GetStateHash(current)].parent;
			else
				break;
		}
		front.push_back(ff->start);
		path.resize(0);
		for (int i = front.size() - 1; i >= 0; i--)
			path.push_back(front[i]);
		for (int i = 0; i < back.size(); i++)
			path.push_back(back[i]);
        pathRatio = min((double)front.size(), (double)back.size())/(double)path.size();
	}
	void GetPath(std::vector<State> &path)
	{
		path.resize(0);
		while (true)
		{
			if (DoSingleSearchStep())
			{
				break;
			}
		}
        //std::cout << "Found" << std:: endl;
		if (!ff->validSolution || !bf->validSolution)
			return;
		ExtractPath(path);
	}
	void SetSeed(unsigned int seed)
	{
		this.seed = seed;
	}

    void SetAnchorSelection(kAnchorSelection selection, kAnchorSelection selection2)
    {
        ff->anchorSelection = selection;
        bf->anchorSelection = selection2;
    }

	void SetAnchorSelection(kAnchorSelection selection)
    {
        ff->anchorSelection = selection;
        bf->anchorSelection = selection;
    }
};


template <class Env, class State>
FastTAS<Env, State>::FastTAS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
	ff = new FastTASFrontier<Env, State>(_env, _start, hf, _sampleCount);
	bf = new FastTASFrontier<Env, State>(_env, _goal, hb, _sampleCount);
	ff->other = bf;
	bf->other = ff;
}




