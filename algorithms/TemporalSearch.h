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
struct TemporalOpenClosed
{
public:

    double g = 0;
    State parent;
    int loc = -1;
};

template <class Env, class State>
class TemporalSearch
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	std::vector<std::pair<State, uint64_t>> open;
	std::set<uint64_t> closed;
	//std::unordered_map<uint64_t, double> gValues;
	//std::unordered_map<uint64_t, State> parents;
	//std::unordered_map<uint64_t, int> loc;
    std::unordered_map<uint64_t, TemporalOpenClosed<State>> openClosed;
	std::vector<int> occurrences;
	std::vector<int> _occurrences;
	int numOfExp = 0;
	State rendezvous;
	TemporalSearch *other;
    int sampleCount;
	State anchor;
	State start, goal;
    Heuristic<State> *h;
	TemporalSearch(Env *_env, State _start, State _goal, Heuristic<State> *_h, int _sampleCount);
	~TemporalSearch(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
    double anchorH = -1;
	double anchorG = 0;
	int anchorUpdateCounter;

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));

		return h->HCost(s1, s2); 
		//return MaxDH(s1, s2);
	}
    int GetNodesExpanded()
	{
		return numOfExp;
	}
};


template <class Env,  class State>
TemporalSearch<Env, State>::TemporalSearch(Env *_env, State _start, State _goal, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	open.reserve(100000);
	h = _h;
	start = _start;
    goal = _goal;
    openClosed.clear();
    openClosed.reserve(100000);
    auto shash = env->GetStateHash(start);
	open.push_back(std::make_pair(start, shash));
    TemporalOpenClosed<State> data;
    data.g = 0;
    data.loc = 0;
    data.parent = start;
    openClosed.insert({shash, data});
	anchor = start;
	numOfExp = 0;
	validSolution = true;
	anchorUpdateCounter = 100;
}


template <class Env,  class State>
void TemporalSearch<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = goal;
	while (true)
	{
		path.push_back(current);
		if (openClosed.find(env->GetStateHash(current)) != openClosed.end())
			current = openClosed[env->GetStateHash(current)].parent;
		else
			break;
	}
	path.push_back(start);
}


template <class Env,  class State>
void TemporalSearch<Env, State>::GetPath(std::vector<State> &path)
{
	path.resize(0);
	while (true)
	{
		if (DoSingleSearchStep())
		{
			break;
		}
	}
	ExtractPath(path);
}


template <class Env,  class State>
bool TemporalSearch<Env, State>::DoSingleSearchStep()
{
	if (open.size() == 0)
	{
		//std::cout << "******* No Solution Found! ********" << std::endl;
		validSolution = false;
		return true;
	}
		
	double minDist = 99999999;
    auto back = open.back();
	State bestCandidate = back.first;
	uint64_t bestCandidateHash = back.second;
	int _samples = sampleCount;
	int index = open.size() - 1;
    while (index >= 0 && _samples > 0)
    {
        auto pair = open[index];
        State c = pair.first;
        auto hash = pair.second;
		auto dist = HCost(c, goal);
		if (dist < minDist || dist == minDist && openClosed[hash].g > openClosed[bestCandidateHash].g)
		{
			minDist = dist;
			bestCandidate = c;
			bestCandidateHash = hash;
		}
        index--;
		_samples--;
    }
	open[openClosed[bestCandidateHash].loc] = open.back();
	openClosed[open.back().second].loc = openClosed[bestCandidateHash].loc;
	open.pop_back();
	openClosed[bestCandidateHash].loc = -1;
	//closed.insert(bestCandidateHash);
	//_occurrences[int(HCost(bestCandidate, other->anchor))]++;

	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		//occurrences[int(HCost(neighbor, other->anchor))]++;
		auto nhash = env->GetStateHash(neighbor);
		double g = openClosed[bestCandidateHash].g + env->GCost(bestCandidate, neighbor);
		if (neighbor == goal)
		{
			return true;
		}
        auto ent = openClosed.find(nhash);
		if (ent != openClosed.end())
		{
			if (g < ent->second.g)
			{
				ent->second.g = g;
				ent->second.parent = bestCandidate;
			}
			else if (ent->second.loc != -1)
			{
				open[ent->second.loc] = open.back();
			    openClosed[open.back().second].loc = ent->second.loc;
			    open[open.size() - 1] = std::make_pair(neighbor, nhash);
			    ent->second.loc = open.size() - 1;
			}
		}
		else
		{
			open.push_back(std::make_pair(neighbor, nhash));
            TemporalOpenClosed<State> data;
            data.g = g;
            data.loc = open.size() - 1;
            data.parent = bestCandidate;
            openClosed[nhash] = data;
		}
    }
	return false;
}