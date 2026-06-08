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
#include "UnsortedAStarOpenClosed.h"



template <typename State>
class RASData
{
public:
    RASData(){}
    ~RASData(){}
    RASData(State _state, double _g, State _parent)
    {
        g = _g;
        parent = _parent;
        state = _state;
    }
    State state;
    double g;
    State parent;
};


template <class Env, class State>
class RASFastFrontier
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	UnsortedOpenClosed<State> open;
	int numOfExp = 0;
	State rendezvous;
	RASFastFrontier *other;
    int sampleCount;
	State anchor;
	State start;
    Heuristic<State> *h;
	RASFastFrontier(Env *_env, State _start, Heuristic<State> *h, int _sampleCount);
	~RASFastFrontier(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
	bool recent_valid = false;
	//double comps = 0;

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
RASFastFrontier<Env, State>::RASFastFrontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.reset();
	h = _h;
	start = _start;
	open.Add(start, env->GetStateHash(start), 0, env->GetStateHash(start));
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void RASFastFrontier<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = rendezvous;
    path.resize(0);
	if (!validSolution)
		return;
	while (true)
	{
		//std::cout << "Path: " << current << " " << start << std::endl;
		auto hash = env->GetStateHash(current);
		path.push_back(current);
		uint64_t index;
		auto b = open.Lookup(hash, index);
		auto parentHash = open.Lookup(index, b).parent;
		uint64_t parentIndex;
		auto b2 = open.Lookup(parentHash, parentIndex);
		auto parent = open.Lookup(parentIndex, b2).data;
		if (hash != parentHash)
			current = parent;
		else
			break;
	}
	path.push_back(start);
	
}




template <class Env,  class State>
bool RASFastFrontier<Env, State>::DoSingleSearchStep()
{
    //std::cout << "---------- " << numOfExp << std:: endl;
	//std::cout << "size: " << open.size() << std::endl;
	if (open.size() == 0)
	{
		validSolution = false;
		std::cout << "******* No Solution Found! ********" << std::endl;
		return true;
	}
		
	double minDist = 99999999;
	State bestCandidate = open.Lookup(0).data;
	//State bestCandidate = open.elements[0].data.state;
	uint64_t bestInd = 0;
	uint64_t bestCandidateHash = env->GetStateHash(bestCandidate);
	int _samples = min(sampleCount - 1, open.size() - 1);
	int _size = open.size();
	for (size_t i = 0; i < _samples; i++)
	{
        int index = rand() % (_size - 1);
		index++;
        auto data = open.Lookup(index);//open.elements[index].data;
		State c = data.data;
		auto hash = env->GetStateHash(c);
        open.Move(hash, _size - 1);

		auto dist = HCost(c, other->anchor);
		if (dist < minDist || dist == minDist && data.g > open.Lookup(bestInd).g)
		{
			minDist = dist;
			bestCandidate = c;
			bestInd = _size - 1;
			bestCandidateHash = env->GetStateHash(bestCandidate);
		}
		_size--;
	}

	if (recent_valid && HCost(open.Lookup(0).data, other->anchor) <= minDist)
	{
		bestCandidate = open.Lookup(0).data;
		bestInd = 0;
		bestCandidateHash = env->GetStateHash(bestCandidate);
	}
    open.Close(bestCandidateHash);
	open.Lookup(bestCandidateHash, bestInd);
	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	recent_valid = false;
	for (State neighbor : children)
	{
		auto nhash = env->GetStateHash(neighbor);
		double g = open.Lookup(bestInd, false).g + env->GCost(bestCandidate, neighbor);
		if (open.Contains(nhash))
		{
			uint64_t ind;
        	bool isOpen = open.Lookup(nhash, ind);
			auto &nData = open.Lookup(ind, isOpen);
			if (g < nData.g)
			{
                nData.g = g;
                nData.parent = bestCandidateHash;
			}
		}
		else
		{
            open.Add(neighbor, nhash, g, bestCandidateHash);
			if (!recent_valid || recent_valid && HCost(neighbor, other->anchor) < HCost(open.Lookup(0).data, other->anchor))
			{
				recent_valid = true;
                open.Move(nhash, 0);
			}
				
		}
	}
	anchor = bestCandidate;
	if (other->open.Contains(bestCandidateHash))// || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
	return false;
}


template <class Env,  class State>
void RASFastFrontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}

template <class Env, class State>
class RASFast
{
private:
	int turn = 0;
public:
	RASFastFrontier<Env, State>* ff;
	RASFastFrontier<Env, State>* bf;
	RASFast();
	RASFast(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount);
	~RASFast(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
	{
		ff = new RASFastFrontier<Env, State>(_env, _start, hf, _sampleCount);
		bf = new RASFastFrontier<Env, State>(_env, _goal, hb, _sampleCount);
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
		std::vector<State> front, back;
        ff->ExtractPath(front);
        bf->ExtractPath(back);
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
RASFast<Env, State>::RASFast(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
	ff = new RASFastFrontier<Env, State>(_env, _start, hf, _sampleCount);
	bf = new RASFastFrontier<Env, State>(_env, _goal, hb, _sampleCount);
	ff->other = bf;
	bf->other = ff;
}




