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
	std::set<uint64_t> closed;
	std::map<uint64_t, double> gValues;
	std::map<uint64_t, State> parents;
	std::map<uint64_t, int> loc;
	std::vector<int> occurrences;
	std::vector<int> _occurrences;
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
    double anchorH = 1000000;
	double anchorG = 0;

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
TASAFrontier<Env, State>::TASAFrontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	//open.reserve(env->GetMaxHash());
	h = _h;
	start = _start;
	open.push_back(start);
	gValues[env->GetStateHash(start)] = 0;
	loc.clear();
	//loc[env->GetStateHash(start)] = 0;
	loc.insert({env->GetStateHash(start), 0});
	anchor = start;
	numOfExp = 0;
	validSolution = true;
	occurrences.resize(100);
	_occurrences.resize(100);
}


template <class Env,  class State>
void TASAFrontier<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = rendezvous;
	while (true)
	{
		path.push_back(current);
		if (parents.find(env->GetStateHash(current)) != parents.end())
			current = parents[env->GetStateHash(current)];
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
		if (dist < minDist || dist == minDist && gValues[hash] > gValues[bestCandidateHash])
		{
			minDist = dist;
			bestCandidate = c;
			bestCandidateHash = env->GetStateHash(bestCandidate);
		}
        index--;
		_samples--;
    }
	open[loc[bestCandidateHash]] = open.back();
	loc[env->GetStateHash(open.back())] = loc[bestCandidateHash];
	open.pop_back();
	loc.erase(bestCandidateHash);
	closed.insert(bestCandidateHash);
	_occurrences[int(HCost(bestCandidate, other->anchor))]++;

	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		occurrences[int(HCost(neighbor, other->anchor))]++;
		auto nhash = env->GetStateHash(neighbor);
		double g = gValues[bestCandidateHash] + env->GCost(bestCandidate, neighbor);
		if (gValues.find(nhash) != gValues.end())
		{
			if (g < gValues[nhash])
			{
				gValues[nhash] = g;
				parents[nhash] = bestCandidate;
			}
			else
			{
				if (closed.find(nhash) == closed.end())
				{
					open[loc[nhash]] = open.back();
					loc[env->GetStateHash(open.back())] = loc[nhash];
					open[open.size() - 1] = neighbor;
					loc[nhash] = open.size() - 1;
				}
			}
		}
		else
		{
			gValues[nhash] = g;
			open.push_back(neighbor);
			loc.insert({nhash, open.size() - 1});
			parents[nhash] = bestCandidate;
		}
	}
    auto hh = HCost(bestCandidate, other->start);
    if (hh < anchorH)
    {
        anchor = bestCandidate;
        anchorH = hh;
        anchorG = gValues[bestCandidateHash];
		//std::cout << "CHANGED" << std::endl;
    }
    else if (hh == anchorH)
    {
        if (gValues[bestCandidateHash] < anchorG)
        {
            anchor = bestCandidate;
            anchorG = gValues[bestCandidateHash];
			//std::cout << "CHANGED" << std::endl;
        }
    }
	//anchor = bestCandidate;
    
	if (other->gValues.find(bestCandidateHash) != other->gValues.end())// || bestCandidate == other->start)
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
		ff.anchorH = ff.HCost(_start, _goal);
		bf.anchorH = bf.HCost(_goal, _start);
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
			if (bf->parents.find(bf->env->GetStateHash(current)) != bf->parents.end())
				current = bf->parents[bf->env->GetStateHash(current)];
			else
				break;
		}
		back.push_back(bf->start);
		current = ff->rendezvous;
		while (true)
		{
			front.push_back(current);
			if (ff->parents.find(ff->env->GetStateHash(current)) != ff->parents.end())
				current = ff->parents[ff->env->GetStateHash(current)];
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
		if (!ff->validSolution || !bf->validSolution)
			return;
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
};


template <class Env, class State>
TASA<Env, State>::TASA(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
	ff = new TASAFrontier<Env, State>(_env, _start, hf, _sampleCount);
	bf = new TASAFrontier<Env, State>(_env, _goal, hb, _sampleCount);
	ff->other = bf;
	bf->other = ff;
}
