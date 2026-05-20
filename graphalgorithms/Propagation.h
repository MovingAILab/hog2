/*
 *  Propagation.h
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code emphasizes on correctness.
 */

#ifndef PROPAGATION_H
#define PROPAGATION_H

#include <math.h>
#include <cstdlib>
#include "GraphAlgorithm.h"
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include <deque>
#include <unordered_map>
#include "FPUtil.h"
#include "OpenListB.h"

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#define PROP_A        0
#define PROP_B        1
#define PROP_BP       2
#define PROP_APPROX   3
#define PROP_BFS      4
#define PROP_DELAY    5
#define PROP_DP       7  // 6 is AStarDelay
#define PROP_BPMX     8
#define PROP_DPMX     9
#define PROP_BPMXE    10

#define PROP_DPDLMX   11  // DP + Delay + MX

namespace PropUtil
{
	class SearchNode {
	public:
		/* the no parameter constructor will be called by OpenListB, and should construct invalid objects */
		//SearchNode()
		//:fCost(0),gCost(0),currNode((graphState)-1),prevNode((graphState)-1),lastExpanded(0),expansions(0),threshold(1) {}

		SearchNode(double _fCost=0, double _gCost=0, graphState curr=UINT32_MAX, graphState prev=UINT32_MAX)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev), isGoal(false) {}

		SearchNode(graphState curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(curr), isGoal(false) {}


		void copy(double f, double g, graphState curr, graphState prev) {
			fCost = f;
			gCost = g;
			currNode = curr;
			prevNode = prev;
		}

		double fCost;
		double gCost;
		graphState currNode;
		graphState prevNode;

		bool isGoal; // is it goal?

		//bool rxp;
	};

	struct SearchNodeEX {
		SearchNode sn;
		double weight;

		SearchNodeEX(SearchNode& n, double w) {
			sn = n;
			weight = w;
		}
	};

	struct StateEX {
		graphState gs;
		double weight;

		StateEX(graphState s, double w) {
			gs = s;
			weight = w;
		}
	};

	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{ return (i1.currNode == i2.currNode); } 
	};

	// comparing f value
	struct SearchNodeCompare { // true means i2 is preferable over i1
		// in favor of goal when f ties
		// prefering larger g, i.e. smaller h is also in favor of goal nodes
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				if (i2.isGoal) // always prefer a goal node in tie
					return true;
				if (i1.isGoal)
					return false;
				return fless(i1.gCost, i2.gCost); // favor larger g
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

	struct TGreater {
		bool operator()(const SearchNode &, const SearchNode &) const
		{
			assert(false); // parameters ignored, this code must be wrong
			//if (i1.threshold == i2.threshold) // threshold is integer
			//	return fgreater(i1.gCost,i2.gCost);
			//return i1.threshold > i2.threshold;
			return true;
		}
	};

	struct FExtract {
		double operator()(const SearchNode &i) const
		{
			return i.fCost;	
		}
	};

	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};

	struct graphGenerator {
		static void SetLoc(node *n, double x, double y, double z)
		{
			n->SetLabelF(GraphSearchConstants::kXCoordinate, x);
			n->SetLabelF(GraphSearchConstants::kYCoordinate, y);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, z);
//			std::cout << *n << std::endl;
//			printf("(%f, %f, %f)\n", x, y, z);
		}

		static void GetLoc(node *n, double &x, double &y, double &z)
		{
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		}

		static Graph* genFig1(unsigned int N)
	  {
			// h(0) = h(1) = 0; 
			// h(i) = 2^(i-1) + 2i - 3, for 1<i<=N
			// c(i,j) = 2^(i-2) + i - 2^(j-1) - j, for 1<=j<i<=N
			// c(1,0) = 2^(N-1) + N - 2
			assert(N >= 3);

			Graph *g = new Graph();

			// add nodes
			
			for (unsigned int nodeID = 0; nodeID <= N; nodeID++)
			{
				node *n = new node("");
				if (nodeID == 0 || nodeID == 1)
				{
					n->SetLabelF(GraphSearchConstants::kHCost, 0);
				}
				else {
					double h = pow(2,nodeID-1) + 2*nodeID - 3;
					n->SetLabelF(GraphSearchConstants::kHCost,h);
				}
				g->AddNode(n); // the real nodeID is assigned here

				if (nodeID == 0)
				{
					SetLoc(n,-1,-1,0);
				}
				else // nodes 1 - N will be drawn along 3/4 circle
				{
					double alpha = ((double)N - nodeID) * 1.5*PI /(N - 1.0);
					double beta = alpha - 0.5*PI;
					SetLoc(n,cos(beta),sin(beta),0);
				}
			}

			// add edges
			double c = pow(2,N-1) + N - 2;
			edge *e = new edge(1,0,c);
			g->AddEdge(e);
			for (unsigned int j = 1; j < N; j++)
			{
				for (unsigned int i = j+1; i <= N; i++)
				{
					c = pow(2,i-2) + i - pow(2,j-1) - j;
					e = new edge(i,j,c);
					g->AddEdge(e);
				}
			}

			return g;
		}

		/* fig 2 is a variant of fig 1 by changing h(N) from 23 to 0 */
		static Graph* genFig2(unsigned int N)
	  {
			// h(0) = h(1) = 0; 
			// h(i) = 2^(i-1) + 2i - 3, for 1<i<=N
			// c(i,j) = 2^(i-2) + i - 2^(j-1) - j, for 1<=j<i<=N
			// c(1,0) = 2^(N-1) + N - 2
			assert(N >= 3);

			Graph *g = new Graph();

			// add nodes
			
			for (unsigned int nodeID = 0; nodeID <= N; nodeID++)
			{
				node *n = new node("");
				if (nodeID == 0 || nodeID == 1 || nodeID==N)
				{
					n->SetLabelF(GraphSearchConstants::kHCost, 0);
				}
				else {
					double h = pow(2,nodeID-1) + 2*nodeID - 3;
					n->SetLabelF(GraphSearchConstants::kHCost,h);
				}
				g->AddNode(n); // the real nodeID is assigned here

				if (nodeID == 0)
				{
					SetLoc(n,-1,-1,0);
				}
				else // nodes 1 - N will be drawn along 3/4 circle
				{
					double alpha = ((double)N - nodeID) * 1.5*PI /(N - 1.0);
					double beta = alpha - 0.5*PI;
					SetLoc(n,cos(beta),sin(beta),0);
				}
			}

			// add edges
			double c = pow(2,N-1) + N - 2;
			edge *e = new edge(1,0,c);
			g->AddEdge(e);
			for (unsigned int j = 1; j < N; j++)
			{
				for (unsigned int i = j+1; i <= N; i++)
				{
					c = pow(2,i-2) + i - pow(2,j-1) - j;
					e = new edge(i,j,c);
					g->AddEdge(e);
				}
			}

			return g;
		}

		static Graph* genFig3(unsigned int N)
	  {
			// h(0) = h(2N-1) = 0
			// h(i) = 2(N-1)^2 - N - i + 2, for i = 1,...,N-1
			// h(N+j) = 2(N-1)(N-2-j) + 1,  for j = 0,...,N-2
			// c(0,i) = 1,  for i = 1,...,N-1
			// c(i,N) = 2(N-1)(i-1) + 1,  for i = 1,...,N-1
			// c(i,i+1) = 2N - 2,  for i = N,...,2N-1 (should it be 2N-2 ?)
			assert(N >= 2);

			Graph* g = new Graph();

			// add nodes
			node* n = new node("");
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, 0, -0.9, 0);
			g->AddNode(n);

			for (unsigned int i=1;i<=N-1;i++)
			{
				n = new node("");
				double h = 2*(N-1)*(N-1) - N - i + 2;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -1+(double)(i-1)*2.0/((double)N-2.0), 0.0, 0);
			}

			for (unsigned int j=0; j<=N-2; j++)
			{
				n = new node("");
				double h = 0; // 2*(N-1)*(N-2-j) + 1;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -(double)j/((double)N-1.0), 0.9+((j%2)?0.1:0.0), 0);
			}

			n = new node(""); // the last node (2N-1)
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, -1, 1, 0);
			g->AddNode(n);

			// add edges

			// type 1
			for (unsigned int i=1;i<=N-1;i++)
			{
				edge* e = new edge(0,i,1);
				g->AddEdge(e);
			}

			// type 2
			for (unsigned int i=1; i<=N-1; i++)
			{
				double c = 2*(N-1)*(i-1) + 1;
				edge* e = new edge(i,N,c);
				g->AddEdge(e);
			}

			// type 3
			for (unsigned int i=N; i<=2*N-2; i++)
			{
				double c = 2*N - 2;
				edge* e = new edge(i,i+1,c);
				g->AddEdge(e);
			}

			return g;
		}

		static Graph* genFig4(unsigned int N)
		{
			// h(0) = h(2N-1) = 0
			// h(i) = N+i-1, for i = 1,...,N-1
			// h(N+j) = 2(N-1)(N-2-j) + 1,  for j = 0,...,N-2
			// c(0,i) = 1,  for i = 1,...,N-1
			// c(i,N) = 2(N-1)(i-1) + 1,  for i = 1,...,N-1
			// c(i,i+1) = 2N - 2,  for i = N,...,2N-1 (should it be 2N-2 ?)
			assert(N >= 2);
			
			Graph* g = new Graph();
			
			// add nodes
			node* n = new node("");
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, 0, -0.9, 0);
			g->AddNode(n);
			
			for (unsigned int i=1;i<=N-1;i++)
			{
				n = new node("");
				double h = 2*N-i;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -1+(double)(i-1)*2.0/((double)N-2.0), 0.0, 0);
			}
			
			for (unsigned int j=0; j<=N+1; j++)
			{
				n = new node("");
				double h = 0; // 2*(N-1)*(N-2-j) + 1;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -(double)j/((double)N-1.0), 0.9+((j%2)?0.1:0.0), 0);
			}
			
			n = new node(""); // the last node (2N-1)
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, -1, 1, 0);
			g->AddNode(n);
			
			// add edges
			
			// type 1
			for (unsigned int i=1;i<=N-1;i++)
			{
				edge* e = new edge(0,i,1);
				g->AddEdge(e);
			}
			
			// type 2
			for (unsigned int i=1; i<=N-1; i++)
			{
				double c = i;
				edge* e = new edge(i,N,c);
				g->AddEdge(e);
			}
			
			// type 3
			for (unsigned int i=N; i<=2*N-1; i++)
			{
				double c = 1;
				edge* e = new edge(i,i+1,c);
				g->AddEdge(e);
			}
			g->AddEdge(new edge(2*N, 2*N+1, N-1));
			return g;
		}
	};


	typedef OpenListB<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::SearchNodeCompare, PropUtil::GGreater, PropUtil::FExtract> PQueue;

	typedef std::unordered_map<graphState, PropUtil::SearchNode > NodeLookupTable;

	typedef OpenListB<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::GGreater, PropUtil::GGreater, PropUtil::FExtract> GQueue;

	typedef OpenListB<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::TGreater, PropUtil::GGreater, PropUtil::FExtract> TQueue;
}


class Prop  : public GraphAlgorithm {
public:
	Prop() { verID = PROP_BP; delta=0; bpmxLevel=1;}  // bpmxLevel is not used if not explicitely calling bpmx root prop
	Prop(unsigned int v) { 
		verID = v; delta=0;  
		if (verID == PROP_BPMX || verID == PROP_DPMX || verID == PROP_BPMXE || verID == PROP_DPDLMX)
			bpmxLevel=1;
		else
			bpmxLevel=0;
	} // [changed needed, regarding bpmxLevel]
	Prop(unsigned int v, double del) {
		verID=v; delta=del;  
		if (verID == PROP_BPMX || verID == PROP_DPMX || verID == PROP_BPMXE || verID == PROP_DPDLMX)
			bpmxLevel=1;
		else
			bpmxLevel=0;
	} //[changed needed, regarding bpmxLevel]
	virtual ~Prop() {}
	void GetPath(GraphEnvironment *env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath);
	
	uint64_t GetNodesExpanded() { return (uint64_t)nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }

	uint64_t GetNodesFirstExpanded() {return closedSize;}
	uint64_t GetNodesMetaExpanded() {return metaexpanded;}
	uint64_t GetNodesReopened() {return NodesReopened;}

	bool InitializeSearch(GraphEnvironment *env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath);
	bool DoSingleSearchStep(std::vector<graphState> &thePath);
	bool DoSingleStepA(std::vector<graphState> &thePath);
	bool DoSingleStepB(std::vector<graphState> &thePath);
	bool DoSingleStepBP(std::vector<graphState> &thePath);
	bool DoSingleStepApprox(std::vector<graphState> &thePath);
	bool DoSingleStepBFS(std::vector<graphState> &thePath);
	//void ClosedListRepair();
	bool DoSingleStepDelay(std::vector<graphState> &thePath);
	bool DoSingleStepDP(std::vector<graphState> &thePath);
	bool DoSingleStepBPMX(std::vector<graphState> &thePath);
	bool DoSingleStepDPMX(std::vector<graphState> &thePath);

	bool DoSingleStepDPDLMX(std::vector<graphState> &thePath);
	//void CleanUpOpen(double solCost);
	//void Categorize(std::vector<graphState>& neighbors);
	//void Categorize2(std::vector<graphState>& neighbors, std::vector<PropUtil::SearchNode>& openN, std::vector<PropUtil::SearchNode>& closedN, std::vector<graphState>& newN);
	void ReverseProp(PropUtil::SearchNode& topNode);
	void ReversePropX1(PropUtil::SearchNode& topNode);
	void ReversePropX2(PropUtil::SearchNode& topNode);
	//bool NextNeighbor(PropUtil::SearchNode& neighborNode, graphState& neighbor, int& mode);
	void ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath);

	void GetLowestG(PropUtil::SearchNode &gNode);
	void GetLowestGF(PropUtil::SearchNode &gNode);
	//bool GetLowestG(PropUtil::TQueue &wList, PropUtil::SearchNode &gNode, double fBound, long TBound);
	bool UpdateHOnly(PropUtil::SearchNode &node, double h);
	void ComputeNewHMero3a(double &h, double &h_tmp, graphState neighbor, PropUtil::SearchNode& neighborNode, double altH, int mode);
	void RelaxOpenNode(double f, double g, graphState neighbor, PropUtil::SearchNode &neighborNode, graphState topNodeID);
	void RelaxDelayNode(double f, double g, graphState neighbor, PropUtil::SearchNode &neighborNode, graphState topNodeID);

	//void OpenGLDraw() const;
	//void OpenGLDraw() const;
	void DrawText(double x, double y, double z, float r, float g, float b, char* str);
	void DrawEdge(unsigned int from, unsigned int to, double weight);

	//int GetReopenings() {return reopenings;}
	double GetSolutionCost() {return solutionCost;}
	const char* GetName() {return algname;}
	int GetSolutionEdges() {return pathSize;}

	bool DoSingleStepBPMXE(std::vector<graphState> &thePath);
	//void ReversePropX1E(PropUtil::SearchNode& topNode);
	//void BroadcastFence();

	char algname[20];
	unsigned int verID;
	double solutionCost;

	int bpmxLevel;

	//unsigned long tickNewExp;
	//unsigned long tickReExp;
	//unsigned long tickBPMX;

	//unsigned long nNewExp;
	//unsigned long nReExp;
	//unsigned long nBPMX;

	long metaexpanded;  // count reverse propagations in bpmx / dp

	PropUtil::PQueue openQueue;
	PropUtil::NodeLookupTable closedList; 

	long closedSize;

private:
	
	double F;
	double  nodesExpanded;
	uint64_t nodesTouched;
	std::vector<graphState> neighbors;
	//std::deque<PropUtil::SearchNode> bfsQueue;
	graphState goal, start;
	GraphEnvironment *env;
	
	PropUtil::GQueue FCache;
	PropUtil::GQueue delayCache; // 
	int reopenings;

	int fDelay(long /*N*/) {
		return 2;
	}

	PropUtil::TQueue WaitList; // for delay alg

	double delta;  // the threshold for approx

	
	int pathSize;

	long NodesReopened;

	Graph *grp;  // for drawing only

	graphState justExpanded; // the node that is expanded in previous round, for drawing only

	// categorize the neighbors
	//std::vector<PropUtil::SearchNode> closedNeighbors;
	//std::vector<PropUtil::SearchNode> openNeighbors;
	//std::vector<PropUtil::SearchNode> waitNeighbors;
	//std::vector<graphState> newNeighbors;

	//PropUtil::SearchNode* newParent; // new parent for topNode, default is null

	//double ET;
	//std::deque<graphState> fifo;

	void Broadcast(int level, int levelcount);

	std::deque<graphState> fifo;
	std::vector<graphState> myneighbors;
};




#endif
