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

template<class State>
class IndexDataA
{
public:
    IndexDataA()
    {
        index = -1;
        //parent = NULL;
        g = 0;
    }
    IndexDataA(int _index, State _parent, double _g)
    {
        index = _index;
        parent = _parent;
        g = _g;
    }
    int index;
    State parent;
    double g;
};


template <class Env, class State>
class TASAFrontier
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	std::vector<State> open;
	std::vector<IndexDataA<State>> loc;
	int numOfExp = 0;
	State rendezvous;
	TASAFrontier *other;
    int sampleCount;
	State anchor;
	State start;
    Heuristic<State> *h;
	TASAFrontier(Env *_env, State _start, Heuristic<State> *h, int _sampleCount);
	~TASAFrontier(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
    double anchorH, anchorG;

	//double comps = 0;

	

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
TASAFrontier<Env, State>::TASAFrontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	//open.reserve(env->GetMaxHash());
	h = _h;
	start = _start;
	open.push_back(start);
	//gValues[env->GetStateHash(start)] = 0;
	loc.resize(env->GetMaxHash());
	loc[env->GetStateHash(start)] = IndexDataA<State>(0, start, 0);
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void TASAFrontier<Env, State>::ExtractPath(std::vector<State> &path)
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
bool TASAFrontier<Env, State>::DoSingleSearchStep()
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
		if (dist < minDist || dist == minDist && loc[hash].g > loc[bestCandidateHash].g)
		{
			minDist = dist;
			bestCandidate = c;
			bestCandidateHash = env->GetStateHash(bestCandidate);
		}
        index--;
		_samples--;
    }
	open[loc[bestCandidateHash].index] = open.back();
	loc[env->GetStateHash(open.back())].index = loc[bestCandidateHash].index;
	open.pop_back();
	loc[bestCandidateHash].index = -2;
	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		auto nhash = env->GetStateHash(neighbor);
		double g = loc[bestCandidateHash].g + env->GCost(bestCandidate, neighbor);
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
					loc[env->GetStateHash(open.back())].index = loc[nhash].index;
					open[open.size() - 1] = neighbor;
					loc[nhash].index = open.size() - 1;
				}
			}
		}
		else
		{
			loc[nhash].g = g;
			open.push_back(neighbor);
			loc[nhash] = IndexDataA<State>(open.size() - 1, bestCandidate, g);
			//parents[nhash] = bestCandidate;
		}
	}
    auto hh = HCost(bestCandidate, other->start);
    if (hh < anchorH)
    {
        anchor = bestCandidate;
        anchorH = hh;
        anchorG = loc[bestCandidateHash].g;
    }
    else if (hh == anchorH)
    {
        if (loc[bestCandidateHash].g < anchorG)
        {
            anchor = bestCandidate;
            anchorG = loc[bestCandidateHash].g;
        }
    }
	//anchor = bestCandidate;
	if (other->loc[bestCandidateHash].index != -1)// || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
	return false;
}


template <class Env,  class State>
void TASAFrontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}

template <class Env, class State>
class TASA
{
private:
	int turn = 0;
public:
	TASAFrontier<Env, State>* ff;
	TASAFrontier<Env, State>* bf;
	TASA();
	TASA(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount);
	~TASA(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
	{
		ff = new TASAFrontier<Env, State>(_env, _start, hf, _sampleCount);
		bf = new TASAFrontier<Env, State>(_env, _goal, hb, _sampleCount);
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
		for (int i = front.size() - 1; i >= 1; i--)
			path.push_back(front[i]);
		for (int i = 0; i < back.size(); i++)
			path.push_back(back[i]);
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
};


template <class Env, class State>
TASA<Env, State>::TASA(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
	ff = new TASAFrontier<Env, State>(_env, _start, hf, _sampleCount);
	bf = new TASAFrontier<Env, State>(_env, _goal, hb, _sampleCount);
	ff->other = bf;
	bf->other = ff;
    ff->anchorH = ff->HCost(_start, _goal);
    ff->anchorG = 0;
    bf->anchorH = bf->HCost(_start, _goal);
    bf->anchorG = 0;
}




