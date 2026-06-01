/*
 *  AStarDelay.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/27/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ASTARDELAY_H
#define ASTARDELAY_H

#include <math.h>
#include "GraphAlgorithm.h"
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"
#include "OpenListB.h"
#include <deque>

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif


#define D_ONE 1
#define D_TWO 2
#define D_THREE 3
#define D_FOUR 4
#define D_LOG2 5
#define D_SQRT 6
//const double pi = 3.141592654;

//typedef std::unordered_map<uint64_t, double> NodeHashTable;

namespace AStarDelayUtil
{
	class SearchNode {
	public:
		SearchNode(double _fCost=0, double _gCost=0, graphState curr=UINT32_MAX, graphState prev=UINT32_MAX)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev),isGoal(false) {}

		SearchNode(graphState curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(curr),isGoal(false) {}

		void copy(double f, double g, graphState curr, graphState prev)
		{
			fCost = f;
			gCost = g;
			currNode = curr;
			prevNode = prev;
		}

		double fCost;
		double gCost;
		graphState currNode;
		graphState prevNode;
		bool isGoal;

		//bool rxp;
	};

	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{ return (i1.currNode == i2.currNode); } 
	};

	struct SearchNodeCompare { // true means i2 is preferable over i1
		// prefering larger g, i.e. smaller h is also in favor of goal nodes
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				if (i2.isGoal) // always prefer a goal node in tie
					return true;
				if (i1.isGoal)
					return false;
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} 
	};

	struct GGreater {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
	  {
			if (fequal(i1.gCost,i2.gCost)) {
				//if (i2.isGoal) // always prefer a goal node in tie
				//	return true;
				//if (i1.isGoal)
				//	return false;

				return fgreater(i1.fCost,i2.fCost);
			}

			return fgreater(i1.gCost,i2.gCost);
		}
	};

	struct FExtract {
		double operator()(const SearchNode &i) const {
			return i.fCost;	
		}
	};

	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};

	typedef OpenListB<AStarDelayUtil::SearchNode, AStarDelayUtil::SearchNodeHash,
		AStarDelayUtil::SearchNodeEqual, AStarDelayUtil::SearchNodeCompare, AStarDelayUtil::GGreater, AStarDelayUtil::FExtract> PQueue;

	typedef std::unordered_map<graphState, AStarDelayUtil::SearchNode > NodeLookupTable;

	typedef OpenListB<AStarDelayUtil::SearchNode, AStarDelayUtil::SearchNodeHash,
		AStarDelayUtil::SearchNodeEqual, AStarDelayUtil::GGreater, AStarDelayUtil::GGreater, AStarDelayUtil::FExtract> GQueue;
}

class AStarDelay : public GraphAlgorithm {
public:
	AStarDelay() { verID=6; bpmxLevel = 0; fD=2;}
	AStarDelay(int lev) { verID=6; bpmxLevel = lev; fD=2;}
	virtual ~AStarDelay() {}
	void GetPath(GraphEnvironment *env, Graph* _g, graphState from, graphState to, std::vector<graphState> &thePath);
	
	uint64_t GetNodesExpanded() { return (uint64_t)nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }

	uint64_t GetNodesFirstExpanded() {return closedSize;}
	uint64_t GetNodesMetaExpanded() {return metaexpanded;}
	uint64_t GetNodesReopened() { return nodesReopened; }

	bool InitializeSearch(GraphEnvironment *env, Graph* g, graphState from, graphState to, std::vector<graphState> &thePath);
	bool DoSingleSearchStep(std::vector<graphState> &thePath);

	void ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath);
	//void OpenGLDraw() const;
//	void OpenGLDraw() const;
//	void DrawText(double x, double y, double z, float r, float g, float b, char* str);
//	void DrawEdge(unsigned int from, unsigned int to, double weight);

	double GetSolutionCost() {return solutionCost;}
	const char* GetName() {return "AStarDelay";}
	int GetSolutionEdges() {return pathSize;}

	void ReversePropX1(AStarDelayUtil::SearchNode& topNode);
	void Broadcast(int level, int levelcount);

	double solutionCost;
	int verID;
	int fD;


	//unsigned long tickNewExp;
	//unsigned long tickReExp;
	//unsigned long tickBPMX;

	//unsigned long nNewExp;
	//unsigned long nReExp;
	//unsigned long nBPMX;

	long metaexpanded;

	AStarDelayUtil::PQueue openQueue;
	AStarDelayUtil::NodeLookupTable closedList; 

	uint64_t closedSize;

private:
	bool DoSingleStep(AStarDelayUtil::SearchNode &topNode,
										std::vector<graphState> &thePath);
	double HandleNeighbor(graphState neighbor,
												AStarDelayUtil::SearchNode &topNode);
	double HandleNeighborX(graphState neighbor,
												AStarDelayUtil::SearchNode &topNode);
//	double UpdateLowGNode(graphState neighbor,
//												AStarDelayUtil::SearchNode &topNode);
	double UpdateDelayedNode(graphState neighbor,
													 AStarDelayUtil::SearchNode &topNode);
	double UpdateClosedNode(graphState neighbor,
													AStarDelayUtil::SearchNode &topNode);
	double UpdateOpenNode(graphState neighbor,
												AStarDelayUtil::SearchNode &topNode);
	double AddNewNode(graphState neighbor,
										AStarDelayUtil::SearchNode &topNode);

	double F;

	double fDelay(double N) 
	{
		switch (fD)
		{
		case D_ONE: return 1;
		case D_TWO: return 2;
		case D_THREE: return 3;
		case D_FOUR:  return 4;
		case D_LOG2:  return log2(N);
		case D_SQRT:  return sqrt(N);
		default: return 2;
		}
	}

	double nodesExpanded;
	uint64_t nodesTouched, nodesReopened;
	std::vector<graphState> neighbors;
	graphState goal, start;
	GraphEnvironment *env;

	AStarDelayUtil::GQueue delayQueue, fQueue;

	Graph *g; // for OpenGL drawing only

	
	int pathSize;

	int bpmxLevel;

	std::deque<graphState> fifo;
	std::vector<graphState> myneighbors;

};	

#endif
