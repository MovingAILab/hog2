#include <set>
#include <vector>
#include <map>
#include "Map.h"
#include <cmath>
#include <algorithm>
#include "MR1PermutationPDB.h"

template <class Env, class State>
class RandomBFSFrontier
{
private:
	Env *env;
	unsigned int seed;
	std::vector<State> children;
public:
	std::vector<uint64_t> open;
	int numOfExp = 0;
	State rendezvous;
	std::set<uint64_t> closed;
	std::map<uint64_t, double> gValues;
	std::map<uint64_t, State> parents;
	RandomBFSFrontier *other;
	State start;
	RandomBFSFrontier(Env *_env, State _start);
	~RandomBFSFrontier();
	bool DoSingleSearchStep();
	void SetSeed(unsigned int seed);
	std::map<uint64_t, uint64_t> GetPath(State node, bool forward);
};


template <class Env,  class State>
RandomBFSFrontier<Env, State>::RandomBFSFrontier(Env *_env, State _start)
{
	open.clear();
	env = _env;
	start = _start;
	auto hash = env->GetStateHash(start);
	open.push_back(hash);
	gValues[env->GetStateHash(start)] = 0;
	numOfExp = 0;
}

template <class Env,  class State>
RandomBFSFrontier<Env, State>::~RandomBFSFrontier()
{
}

template <class Env,  class State>
bool RandomBFSFrontier<Env, State>::DoSingleSearchStep()
{
	if (open.size() == 0)
	{
		std::cout << "******* BUG ********" << std::endl;
		return true;
	}
	numOfExp++;
	int random_index = rand_r(&seed) % open.size();
	auto bestCandidateHash = open[random_index];
	State bestCandidate;
	env->GetStateFromHash(bestCandidateHash, bestCandidate);
	open[random_index] = open[open.size()-1];
	open.pop_back();
	children.clear();
	env->GetSuccessors(bestCandidate, children);
	for (State neighbor : children)
	{
		uint64_t neighborHash = env->GetStateHash(neighbor);
		double g = gValues[bestCandidateHash] + env->GCost(bestCandidate, neighbor);
		if (gValues.find(neighborHash) != gValues.end())
		{
			if (g < gValues[neighborHash])
			{
				parents.at(neighborHash) = bestCandidate;
				gValues[neighborHash] = g;
			}
		}
		else
		{
			parents.insert(std::make_pair(neighborHash, bestCandidate));
			gValues[neighborHash] = g;
			open.push_back(neighborHash);
		}
	}

	if (other->gValues.find(bestCandidateHash) != other->gValues.end() || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
	return false;
}




template <class Env,  class State>
void RandomBFSFrontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}