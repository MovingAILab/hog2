/*
 *  FrontierBFS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/29/11.
 *  Copyright 2011. All rights reserved.
 *
 */

#ifndef FRONTIERBFS_H
#define FRONTIERBFS_H

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"

typedef std::unordered_map<uint64_t, bool, Hash64> FrontierBFSClosedList;

template <class state, class action>
class FrontierBFS {
public:
	FrontierBFS() { }
	virtual ~FrontierBFS() {}
	void GetPath(SearchEnvironment<state, action> *env, state &from, state &to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state &from, state &to,
				 std::vector<action> &thePath);
	
	void InitializeSearch(SearchEnvironment<state, action> *env, state &from);
	void InitializeSearch(SearchEnvironment<state, action> *env, std::vector<state> &from);
	bool DoOneIteration(SearchEnvironment<state, action> *env);
	
	const FrontierBFSClosedList &GetCurrentClosedList()
	{
		if (mOpen1.size() > 0) return mClosed2;
		if (mOpen2.size() > 0) return mClosed1;
		//assert(!"Open and closed are both null");
		return mClosed1;
	}
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
private:

	void ExpandLevel(SearchEnvironment<state, action> *env,
					 std::deque<state> &currentOpenList,
					 FrontierBFSClosedList &currentClosedList,
					 std::deque<state> &nextOpenList,
					 FrontierBFSClosedList &lastClosedList);
	
	
	uint64_t nodesExpanded, nodesTouched;
	int depth;
	
	std::deque<state> mOpen1;
	std::deque<state> mOpen2;
	FrontierBFSClosedList mClosed1; // store parent id!
	FrontierBFSClosedList mClosed2; // store parent id!
};


template <class state, class action>
void FrontierBFS<state, action>::InitializeSearch(SearchEnvironment<state, action> *env,
												  state &from)
{
	nodesExpanded = nodesTouched = 0;
	
	mOpen1.clear();
	mOpen2.clear();
	mClosed1.clear();
	mClosed2.clear();
	
	mOpen1.push_back(from);	
	depth = 0;
}

template <class state, class action>
void FrontierBFS<state, action>::InitializeSearch(SearchEnvironment<state, action> *env,
												  std::vector<state> &from)
{
	nodesExpanded = nodesTouched = 0;
	
	mOpen1.clear();
	mOpen2.clear();
	mClosed1.clear();
	mClosed2.clear();
	
	for (unsigned int x = 0; x < from.size(); x++)
		mOpen1.push_back(from[x]);
	depth = 0;
}

template <class state, class action>
bool FrontierBFS<state, action>::DoOneIteration(SearchEnvironment<state, action> *env)
{
	uint64_t n = nodesExpanded;
	if ((mOpen1.size() != 0) || (mOpen2.size() != 0))
	{
		n = nodesExpanded;
		if (mOpen1.size() == 0)
		{
			std::cout << mOpen2.front() << std::endl << mOpen2.back() << std::endl;
			ExpandLevel(env, mOpen2, mClosed2, mOpen1, mClosed1);
			mClosed1.clear();
		}
		else {
			std::cout << mOpen1.front() << std::endl << mOpen1.back() << std::endl;
			ExpandLevel(env, mOpen1, mClosed1, mOpen2, mClosed2);
			mClosed2.clear();
		}
		depth++;		
	}
	else {
		return true;
	}
	printf("Depth %d complete; nodes expanded %lld (%lld new); %lu in memory\n", depth, nodesExpanded, nodesExpanded - n,
		   mOpen1.size()+mOpen2.size()+mClosed1.size()+mClosed2.size());
	if ((mOpen1.size() == 0) && (mOpen2.size() == 0))
		return true;
	return false;
}


template <class state, class action>
void FrontierBFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
										 state &from, state &to,
										 std::vector<state> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	
	mOpen1.clear();
	mOpen2.clear();
	mClosed1.clear();
	mClosed2.clear();
	
	mOpen1.push_back(from);
	
	depth = 0;
	uint64_t n = nodesExpanded;
	while ((mOpen1.size() != 0) || (mOpen2.size() != 0))
	{
		printf("Depth %d; nodes expanded %lld (%lld new); %d in memory\n", depth, nodesExpanded, nodesExpanded - n,
			   mOpen1.size()+mOpen2.size()+mClosed1.size()+mClosed2.size());
		n = nodesExpanded;
		if (mOpen1.size() == 0)
		{
			ExpandLevel(env, mOpen2, mClosed2, mOpen1, mClosed1);
			mClosed1.clear();
		}
		else {
			ExpandLevel(env, mOpen1, mClosed1, mOpen2, mClosed2);
			mClosed2.clear();
		}
		depth++;		
	}
}

template <class state, class action>
void FrontierBFS<state, action>::ExpandLevel(SearchEnvironment<state, action> *env,
											 std::deque<state> &currentOpenList,
											 FrontierBFSClosedList &currentClosedList,
											 std::deque<state> &nextOpenList,
											 FrontierBFSClosedList &lastClosedList)
{
	static std::vector<state> neighbors;
	neighbors.resize(0);
	while (currentOpenList.size() > 0)
	{
		state s = currentOpenList.front();
		currentOpenList.pop_front();
		
		if (currentClosedList.find(env->GetStateHash(s)) != currentClosedList.end())
		{
//			printf("Needed to check against current\n");
			continue;
		}
//		if (lastClosedList.find(env->GetStateHash(s)) != lastClosedList.end())
//		{
//			printf("Needed to check against last\n");
//			continue;
//		}
		
		currentClosedList[env->GetStateHash(s)] = true;
		
		nodesExpanded++;
		env->GetSuccessors(s, neighbors);
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			if (currentClosedList.find(env->GetStateHash(neighbors[x])) != currentClosedList.end())
				continue;
			if (lastClosedList.find(env->GetStateHash(neighbors[x])) != lastClosedList.end())
				continue;

			nextOpenList.push_back(neighbors[x]);
		}
	}
}

template <class state, class action>
void FrontierBFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
										 state &from, state &to,
										 std::vector<action> &thePath)
{
	assert(!"not defined");
}


#endif
