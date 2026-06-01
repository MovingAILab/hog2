/*
 *  $Id: GenericAStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/22/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "GenericAStar.h"
#include "float.h"
#include "FPUtil.h"

using namespace GenericAStarUtil;
static const bool verbose = false;
using namespace OldSearchCode;

const char *GenericAStar::GetName()
{
	static char name[32];
	sprintf(name, "GenericAStar[]");
	return name;
}


void GenericAStar::GetPath(SearchEnvironment *_env, uint32_t from, uint32_t to,
													 std::vector<uint32_t> &thePath)
{
	thePath.resize(0);
	if (!InitializeSearch(_env, from, to, thePath))
		return;
	while (!DoSingleSearchStep(thePath)) {}
}

bool GenericAStar::InitializeSearch(SearchEnvironment *_env, uint32_t from, uint32_t to,
																		std::vector<uint32_t> &thePath)
{
	env = _env;
	assert(openQueue.size() == 0);
	assert(closedList.size() == 0);
	nodesTouched = nodesExpanded = 0;
	start = from;
	goal = to;
	
	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}

	SearchNode first(env->heuristic(goal, start), 0, start, start);
	openQueue.Add(first);

	return true;
}

bool GenericAStar::DoSingleSearchStep(std::vector<uint32_t> &thePath)
{
	uint32_t currentOpenNode = UINT32_MAX;

	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		return true;
	}
			
	// get top of queue
	currentOpenNode = GetNextNode();
	
	if (env->goalTest(currentOpenNode, goal))
	{
		ExtractPathToStart(currentOpenNode, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}
	
	if (verbose) { printf("Opening %d\n", currentOpenNode); }
	
	if (currentOpenNode == UINT32_MAX)
		printf("Oh no! The current open node is NULL\n");
	
	neighbors.resize(0);
	env->getNeighbors(currentOpenNode, neighbors);

	// iterate over all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		uint32_t neighbor = neighbors[x];
		assert(neighbor != UINT32_MAX);
		
		if (closedList.find(neighbor) != closedList.end())
		{
			if (verbose) { printf("skipping node %d\n", neighbor); }
			continue;
		}
		else if (openQueue.IsIn(SearchNode(neighbor)))
		{
			if (verbose) { printf("updating node %d\n", neighbor); }
			UpdateWeight(currentOpenNode, neighbor);
		}
		else {
			if (verbose) { printf("addinging node %d\n", neighbor); }
			AddToOpenList(currentOpenNode, neighbor);
		}
	}
	return false;
}

uint32_t GenericAStar::CheckNextNode()
{
	return openQueue.top().currNode;
}

uint32_t GenericAStar::GetNextNode()
{
	nodesExpanded++;
	uint32_t next;
	SearchNode it = openQueue.Remove();
	next = it.currNode;
	closedList[next] = it;
	return next;
}

void GenericAStar::UpdateWeight(uint32_t currOpenNode, uint32_t neighbor)
{
	SearchNode prev = openQueue.find(SearchNode(neighbor));
	SearchNode alt = closedList[currOpenNode];
	double edgeWeight = env->gcost(currOpenNode, neighbor);
	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		prev.fCost = altCost;
		prev.gCost = alt.gCost+edgeWeight;
		prev.prevNode = currOpenNode;
		openQueue.DecreaseKey(prev);
	}
}

void GenericAStar::AddToOpenList(uint32_t currOpenNode, uint32_t neighbor)
{
	double edgeWeight = env->gcost(currOpenNode, neighbor);
	SearchNode n(closedList[currOpenNode].gCost+edgeWeight+env->heuristic(neighbor, goal),
							 closedList[currOpenNode].gCost+edgeWeight,
							 neighbor, currOpenNode);
	if (verbose) 
	{ printf("Adding %u to openQueue, old size %u\n", neighbor, openQueue.size()); }
	openQueue.Add(n);
}

void GenericAStar::ExtractPathToStart(uint32_t goalNode,
																			std::vector<uint32_t> &thePath)
{
	SearchNode n;
	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else n = openQueue.find(SearchNode(goalNode));

	do {
		thePath.push_back(n.currNode);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	thePath.push_back(n.currNode);
}

void GenericAStar::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)closedList.size());
	printf("%u items in open queue\n", (unsigned int)openQueue.size());
}

int GenericAStar::GetMemoryUsage()
{
	return closedList.size()+openQueue.size();
}

closedList_iterator GenericAStar::GetClosedListIter() const
{
	return closedList.begin();
}

uint32_t GenericAStar::ClosedListIterNext(closedList_iterator& it) const
{
	if (it == closedList.end())
		return UINT32_MAX;
	uint32_t val = (*it).first;
	it++;
	return val;
}
