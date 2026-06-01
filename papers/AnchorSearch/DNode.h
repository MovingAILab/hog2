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
class DNRData
{
public:
    State state;
	double g;
	uint64_t hash;
    DNRData(State _state, double _g, uint64_t _hash)
    {
        state = _state;
		g = _g;
		hash = _hash;
    }
};



template<class State>
struct _DNodeCompare {
public:
    _DNodeCompare<State>(Heuristic<State> *_h, State _dnode)
    {
        dnode = _dnode;
        h = _h;
    }
    State dnode;
    Heuristic<State> *h;
    bool operator()(const State& lhs, const State& rhs) {
        return h->HCost(lhs, dnode) > h->HCost(rhs, dnode);
    }
};


template<class Data, class State>
struct DNodeCompare {
public:
    DNodeCompare<Data, State>(Heuristic<State> *_h, State _dnode)
    {
        dnode = _dnode;
        h = _h;
    }
    State dnode;
    Heuristic<State> *h;
    bool operator()(const Data& lhs, const Data& rhs) {
		auto h1 = h->HCost(lhs.state, dnode);
		auto h2 = h->HCost(rhs.state, dnode);
		if (h1 == h2)
			return lhs.g < rhs.g;
        return h1 > h2;
    }
};




template <class Env, class State>
class DNodeFrontier
{
public:
    Env *env;
	std::vector<State> children;
	std::priority_queue<DNRData<State>, std::vector<DNRData<State>>, DNodeCompare<DNRData<State>, State>> *open;
	std::set<uint64_t> closed;
	std::map<uint64_t, double> gValues;
	std::map<uint64_t, State> parents;
	int numOfExp = 0;
	State rendezvous;
	DNodeFrontier *other;
	State anchor;
    double anchorG;
	State start;
    Heuristic<State> *h;
	DNodeFrontier(Env *_env, State _start, Heuristic<State> *h);
	~DNodeFrontier(){
		delete open;
	}
	bool DoSingleSearchStep();
    void InitializeSearch();
	void GetPath(std::vector<State> &path);
	std::vector<State> GetPath(State node, bool forward);
    void Retarget();
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
	bool dnodeChanged = false;

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
DNodeFrontier<Env, State>::DNodeFrontier(Env *_env, State _start, Heuristic<State> *_h)
{
	env = _env;
	//open->resize(0);
	h = _h;
	start = _start;
	//open->push(start);
	gValues[env->GetStateHash(start)] = 0;
    anchorG = 0;
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void DNodeFrontier<Env, State>::ExtractPath(std::vector<State> &path)
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
void DNodeFrontier<Env, State>::Retarget()
{
	if (!other->dnodeChanged)
		return;
    std::vector<DNRData<State>> tmp;
    while (!open->empty())
    {
        tmp.push_back(open->top());
        open->pop();
    }
    open = new std::priority_queue<DNRData<State>, std::vector<DNRData<State>>, DNodeCompare<DNRData<State>, State>>(DNodeCompare<DNRData<State>, State>(h, other->anchor));
    for (auto s : tmp)
    {
        open->push(s);
    }
	other->dnodeChanged = false;
}

template <class Env,  class State>
void DNodeFrontier<Env, State>::InitializeSearch()
{
    open = new std::priority_queue<DNRData<State>, std::vector<DNRData<State>>, DNodeCompare<DNRData<State>, State>>(DNodeCompare<DNRData<State>, State>(h, other->anchor));
    open->push(DNRData<State>(start, 0, env->GetStateHash(start)));
}

template <class Env,  class State>
bool DNodeFrontier<Env, State>::DoSingleSearchStep()
{
	if (open->size() == 0)
	{
		std::cout << "******* No Solution Found! ********" << std::endl;
		return true;
	}

	auto data = open->top();
	State bestCandidate = data.state;
    open->pop();
    auto bestCandidateHash = data.hash;
	closed.insert(bestCandidateHash);
	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		auto nhash = env->GetStateHash(neighbor);
		double g = gValues[bestCandidateHash] + env->GCost(bestCandidate, neighbor);
		if (other->gValues.find(nhash) != other->gValues.end())// || bestCandidate == other->start)
		{
			rendezvous = bestCandidate;
			other->rendezvous = neighbor;
			return true;
		}
		if (gValues.find(nhash) != gValues.end())
		{
			if (g < gValues[nhash])
			{
				gValues[nhash] = g;
				parents[nhash] = bestCandidate;
                if (g > anchorG)
                {
                    anchor = neighbor;
                    anchorG = g;
					dnodeChanged = true;
                }
			}
		}
		else
		{
			gValues[nhash] = g;
			open->push(DNRData<State>(neighbor, g, nhash));
			parents[nhash] = bestCandidate;
            if (g > anchorG)
            {
                anchor = neighbor;
                anchorG = g;
				dnodeChanged = true;
            }
		}
	}
    //std::cout << "new size: " << open->size() << std::endl;
	//anchor = bestCandidate;
	//if (other->gValues.find(bestCandidateHash) != other->gValues.end())// || bestCandidate == other->start)
	//{
	//	rendezvous = bestCandidate;
	//	other->rendezvous = rendezvous;
	//	return true;
	//}
	return false;
}


template <class Env, class State>
class DNode
{
private:
	int turn = 0;
    int steps = 0;
    int k;
public:
	DNodeFrontier<Env, State>* ff;
	DNodeFrontier<Env, State>* bf;
	double pathRatio;
	DNode();
	DNode(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _k);
	~DNode(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _k)
	{
		ff = new DNodeFrontier<Env, State>(_env, _start, hf, _k);
		bf = new DNodeFrontier<Env, State>(_env, _goal, hb, _k);
		ff->other = bf;
		bf->other = ff;
        ff->InitializeSearch();
        bf->InitializeSearch();
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
        bool res;
		if (turn == 0)
		{
			res = ff->DoSingleSearchStep();
		}
		else
		{
			res = bf->DoSingleSearchStep();
		}
        if (++steps >= k)
        {
            steps = 0;
            turn = (turn + 1) % 2;
            if (turn == 0)
            {
                ff->Retarget();
            }
            else
            {
                bf->Retarget();
            }
        }
        return res;
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
		if (!ff->validSolution || !bf->validSolution)
			return;
		ExtractPath(path);
		//delete ff->open;
		//delete bf->open;
	}
	void Clear()
	{
		delete ff;
		delete bf;
	}
	void SetSeed(unsigned int seed)
	{
		this.seed = seed;
	}
};


template <class Env, class State>
DNode<Env, State>::DNode(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _k)
{
    k = _k;
	ff = new DNodeFrontier<Env, State>(_env, _start, hf);
	bf = new DNodeFrontier<Env, State>(_env, _goal, hb);
	ff->other = bf;
	bf->other = ff;
    ff->InitializeSearch();
    bf->InitializeSearch();
}




