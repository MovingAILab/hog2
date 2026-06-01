/*
 *  UnitCostBidirectionalBFS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/6/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#ifndef UNITCOSTBIDIRECTIONALBFS
#define UNITCOSTBIDIRECTIONALBFS

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"

template <class state, class action>
class UnitCostBidirectionalBFS {
public:
	UnitCostBidirectionalBFS() { }
	virtual ~UnitCostBidirectionalBFS() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
private:
	typedef std::unordered_map<uint64_t, state, Hash64> BFSClosedList;

	bool ExtractPath(SearchEnvironment<state, action> *env,
					 std::vector<state> &thePath);
	bool ExpandLayer(SearchEnvironment<state, action> *env,
					 std::vector<state> &nodesToExpand,
					 std::vector<state> &expansionLocation,
					 BFSClosedList &duplicateHash,
					 BFSClosedList &completionHash);
	
	unsigned long nodesExpanded, nodesTouched;

	std::vector<state> startOpenA;
	std::vector<state> startOpenB;
	std::vector<state> goalOpenA;
	std::vector<state> goalOpenB;
//	std::vector<state> closed;
	
	BFSClosedList startNodeTable; // store parent id!
	BFSClosedList goalNodeTable;
	state middle;
};	

template <class state, class action>
void UnitCostBidirectionalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<state> &thePath)
{
	nodesExpanded = nodesTouched = 0;

	startNodeTable.clear();
	goalNodeTable.clear();
	startOpenA.clear();
	startOpenB.clear();
	goalOpenA.clear();
	goalOpenB.clear();
	
	startOpenA.push_back(from);	
	goalOpenA.push_back(to);
	startNodeTable[env->GetStateHash(from)] = from;
	goalNodeTable[env->GetStateHash(to)] = to;

	// cheating and not return path
//	thePath.push_back(from);
//	thePath.push_back(to);
	int cost = 0;
	while (1)
	{
		cost++;
		//printf("Expanding from start. List size %d\n", (int)startOpenA.size());
		if (ExpandLayer(env, startOpenA, startOpenB, startNodeTable, goalNodeTable))
		{
			//printf("Found goal with cost %d\n", cost);
			break;
		}
		if (startOpenB.size() == 0) break;
		//printf("%d nodes generated in search\n", (int)startOpenB.size());
		cost++;
		//printf("Expanding from goal. List size %d\n", (int)goalOpenA.size());
		if (ExpandLayer(env, goalOpenA, goalOpenB, goalNodeTable, startNodeTable))
		{
			//printf("Found goal with cost %d\n", cost);
			break;
		}
		if (goalOpenB.size() == 0) break;
		//printf("%d nodes generated in search\n", (int)goalOpenB.size());
		cost++;
		//printf("Expanding from start. List size %d\n", (int)startOpenB.size());
		if (ExpandLayer(env, startOpenB, startOpenA, startNodeTable, goalNodeTable))
		{
			//printf("Found goal with cost %d\n", cost);
			break;
		}
		if (startOpenA.size() == 0) break;
		//printf("%d nodes generated in search\n", (int)startOpenA.size());
		cost++;
		//printf("Expanding from goal. List size %d\n", (int)goalOpenB.size());
		if (ExpandLayer(env, goalOpenB, goalOpenA, goalNodeTable, startNodeTable))
		{
			//printf("Found goal with cost %d\n", cost);
			break;
		}
		if (goalOpenA.size() == 0) break;
		//printf("%d nodes generated in search\n", (int)goalOpenA.size());
	}
	ExtractPath(env, thePath);
}

template <class state, class action>
void UnitCostBidirectionalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	assert(!"not defined");
}

template <class state, class action>
bool UnitCostBidirectionalBFS<state, action>::ExtractPath(SearchEnvironment<state, action> *env,
														  std::vector<state> &thePath)
{
	thePath.clear();
	std::vector<state> forwardPart;
	std::vector<state> backwardPart;

	forwardPart.push_back(middle);
	backwardPart.push_back(middle);
	while (1)
	{
		if (startNodeTable.find(env->GetStateHash(forwardPart.back())) != startNodeTable.end())
		{
			state s = startNodeTable[env->GetStateHash(forwardPart.back())];
			if (forwardPart.back() == s) // reached the start state
			{
				break;
			}
			else {
				forwardPart.push_back(s);
			}

		}
		else {
			return false;
		}
	}
	while (1)
	{
		if (goalNodeTable.find(env->GetStateHash(backwardPart.back())) != goalNodeTable.end())
		{
			state s = goalNodeTable[env->GetStateHash(backwardPart.back())];
			if (backwardPart.back() == s) // reached the goal state
			{
				break;
			}
			else {
				backwardPart.push_back(s);
			}
			
		}
		else {
			return false;
		}
	}

	while (forwardPart.size() > 0)
	{
		thePath.push_back(forwardPart.back());
		forwardPart.pop_back();
	}
	for (unsigned int x = 1; x < backwardPart.size(); x++)
	{
		thePath.push_back(backwardPart[x]);
	}
	//	BFSClosedList startNodeTable; // store parent id!
//	BFSClosedList goalNodeTable;
	return true;
}

template <class state, class action>
bool UnitCostBidirectionalBFS<state, action>::ExpandLayer(SearchEnvironment<state, action> *env,
														  std::vector<state> &nodesToExpand,
														  std::vector<state> &expansionLocation,
														  BFSClosedList &duplicateHash,
														  BFSClosedList &completionHash)
{
	while (nodesToExpand.size() > 0)
	{
		state s = nodesToExpand.back();
		nodesToExpand.pop_back();
		
		//std::cout << "Expanding " << s << std::endl;
		
		// this doesn't belong here. Just pretending we aren't uniform cost
		// check state against duplicateHash and completionHash
		if (completionHash.find(env->GetStateHash(s)) != completionHash.end())
		{
			middle = s;
			assert(startNodeTable.find(env->GetStateHash(s)) != startNodeTable.end());
			assert(goalNodeTable.find(env->GetStateHash(s)) != goalNodeTable.end());
			return true;
		}
		
		// expand s
		std::vector<state> succ;
		env->GetSuccessors(s, succ);
		nodesExpanded++;
		for (unsigned int x = 0; x < succ.size(); x++)
		{
			nodesTouched++;
//			// check successors against duplicateHash and completionHash
			if (completionHash.find(env->GetStateHash(succ[x])) != completionHash.end())
			{
				//std::cout << "Successor: " << succ[x] << " [other frontier]" << std::endl;
				duplicateHash[env->GetStateHash(succ[x])] = s;
				middle = succ[x];
				assert(startNodeTable.find(env->GetStateHash(middle)) != startNodeTable.end());
				assert(goalNodeTable.find(env->GetStateHash(middle)) != goalNodeTable.end());
				return true;
			}
			if (duplicateHash.find(env->GetStateHash(succ[x])) != duplicateHash.end())
			{
				//std::cout << "Successor: " << succ[x] << " [this frontier]" << std::endl;
				continue;
			}

			//std::cout << "Successor: " << succ[x] << " [new node]" << std::endl;
			// then add to expansionLocation
			duplicateHash[env->GetStateHash(succ[x])] = s;
			expansionLocation.push_back(succ[x]);
		}
	}
	return false;
}

#endif
