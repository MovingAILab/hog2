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
class FSData
{
public:
    State state;
    int iter;
    double h;
    double g;
    bool reprioritized;
    FSData(State _state, double _h, double _g)
    {
        state = _state;
        h = _h;
        g = _g;
    }

};

template<class Data>
struct BiFSCompare {
public:
	BiFSCompare()
	{
		
	}
    bool operator()(const Data& a, const Data& b) {
        if (a.h == b.h)
        {
            if (a.reprioritized == b.reprioritized)
				if (lifo == 1)		//lifo
					return a.iter < b.iter;
				else if (lifo == 0) //fifo
                	return a.iter > b.iter;
            return b.reprioritized;
        }
        return a.h > b.h;
    }
};

template <class Env, class State>
class TTBSFrontier
{
public:
    Env *env;
	std::vector<State> children;
	std::priority_queue<FSData<State>, std::vector<FSData<State>>, BiFSCompare<FSData<State>>> *open;
	std::set<uint64_t> closed;
	std::map<uint64_t, double> gValues;
	std::map<uint64_t, State> parents;
	int numOfExp = 0;
	State rendezvous;
	TTBSFrontier *other;
	State anchor;
	State start;
    Heuristic<State> *h;
	TTBSFrontier(Env *_env, State _start, Heuristic<State> *h);
	~TTBSFrontier(){
		delete open;
	}
	bool DoSingleSearchStep();
    void InitializeSearch(int lifo);
	void GetPath(std::vector<State> &path);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
TTBSFrontier<Env, State>::TTBSFrontier(Env *_env, State _start, Heuristic<State> *_h)
{
	env = _env;
	//open->resize(0);
	h = _h;
	start = _start;
	//open->push(start);
	gValues[env->GetStateHash(start)] = 0;
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void TTBSFrontier<Env, State>::ExtractPath(std::vector<State> &path)
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
void TTBSFrontier<Env, State>::InitializeSearch(int lifo)
{
    open = new std::priority_queue<FSData<State>, std::vector<FSData<State>>, BiFSCompare<FSData<State>>>(BiFSCompare<FSData<State>>(lifo));
    open->push(FSData<State>(start, other->start, HCost(start, other->start), 0, false));
}

template <class Env,  class State>
bool TTBSFrontier<Env, State>::DoSingleSearchStep()
{
    //std::cin;
	if (open->size() == 0)
	{
		std::cout << "******* No Solution Found! ********" << std::endl;
		return true;
	}
    /*
    std::vector<FSData<State>> temp;
    temp.clear();
    while (!open->empty())
    {
        std::cout << "(" << open->top().h << " " << open->top().iter << " " << open->top().reprioritized << ") ";
        temp.push_back(open->top());
        open->pop();
    }
    std::cout << endl;
    while (!temp.empty())
    {
        open->push(temp.back());
        temp.pop_back();
    }
	*/
    State s;
    uint64_t hash;
	int counter = 0;
    while(true)
    {
		counter++;
        auto data = open->top();
	    s = data.state;
        hash = env->GetStateHash(s);
        open->pop();
        std::vector<State> similars;
        env->GetSuccessors(data.d, similars);
        similars.push_back(data.d);
        bool expanded = false;
        for (auto item : similars)
        {
            if (item == other->open->top().state)
            {
				//std::cout << counter << std::endl;
				//closed.insert(hash);
                expanded = true;
                std::vector<State> children;
                env->GetSuccessors(data.state, children);
                for (auto neighbor : children)
                {
                    auto nhash = env->GetStateHash(neighbor);
                    double g = gValues[hash] + env->GCost(s, neighbor);
                    if (gValues.find(nhash) != gValues.end())
                    {
                        if (g < gValues[nhash])
			            {
			            	gValues[nhash] = g;
			            	parents[nhash] = s;
			            }
                    }
                    else
                    {
                        gValues[nhash] = g;
			            open->push(FSData<State>(neighbor, other->open->top().state, HCost(neighbor, other->open->top().state), numOfExp, false));
			            parents[nhash] = s;
                    }
                }
                break;
            }
        }
        if (expanded)
            break;
        else
        {
            open->push(FSData<State>(s, other->open->top().state, HCost(s, other->open->top().state), numOfExp, true));
        }
    }
    numOfExp++;
    //std::cout << numOfExp << std::endl;

	if (other->gValues.find(hash) != other->gValues.end())// || bestCandidate == other->start)
	{
		rendezvous = s;
		other->rendezvous = rendezvous;
        //std::cout << "FOUND" << std::endl;
		return true;
	}
	return false;
}


template <class Env, class State>
class TTBS
{
public:
	double pathRatio;
    int turn = 0;
	int lifo;
	TTBSFrontier<Env, State>* ff;
	TTBSFrontier<Env, State>* bf;
	TTBS();
	TTBS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int lifo = 0);
	~TTBS(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb)
	{
		ff = new TTBSFrontier<Env, State>(_env, _start, hf);
		bf = new TTBSFrontier<Env, State>(_env, _goal, hb);
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
        turn = (turn + 1) % 2;
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
		for (int i = front.size() - 1; i >= 1; i--)
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
TTBS<Env, State>::TTBS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _lifo)
{
	lifo = _lifo;
	ff = new TTBSFrontier<Env, State>(_env, _start, hf);
	bf = new TTBSFrontier<Env, State>(_env, _goal, hb);
	ff->other = bf;
	bf->other = ff;
    ff->InitializeSearch(lifo);
    bf->InitializeSearch(lifo);
}




