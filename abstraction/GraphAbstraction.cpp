/*
 *  $Id: GraphAbstraction.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/11/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#include "GraphAbstraction.h"
#include <math.h>
#include <cassert>

using namespace GraphAbstractionConstants;
using namespace std;

enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const static int verbose = kQuiet;//kRepairGraph;

GraphAbstraction::~GraphAbstraction()
{ 

  while (abstractions.size() > 0)
	{
    delete abstractions.back();
    abstractions.pop_back();
  }
}

void GraphAbstraction::GetNumAbstractGraphs(node *from, node *to,
											std::vector<node *> &fromChain,
											std::vector<node *> &toChain)
{
  //	while (from->GetLabelL(kParent) != to->GetLabelL(kParent))
  while (!abstractions[from->GetLabelL(kAbstractionLevel)]->
				 FindEdge(from->GetNum(), to->GetNum()) &&
				 //(from->GetLabelL(kParent) != to->GetLabelL(kParent))
				 (from != to))
	{
    fromChain.push_back(from);
    toChain.push_back(to);
		
    from = abstractions[from->GetLabelL(kAbstractionLevel)+1]->
      GetNode(from->GetLabelL(kParent));
		
    to = abstractions[to->GetLabelL(kAbstractionLevel)+1]->
      GetNode(to->GetLabelL(kParent));
		
    if (verbose&kBuildGraph)
			printf("GetNumAbstractGraphs: Moving to nodes #%d and %d\n",
						 from->GetNum(), to->GetNum());
  }
  fromChain.push_back(from);
  toChain.push_back(to);
}

// get nth-level parent of which node
node *GraphAbstraction::GetNthParent(node *which, int n)
{
	while ((which != NULL) && (which->GetLabelL(kAbstractionLevel) < n))
	{
		int nextLevel = which->GetLabelL(kAbstractionLevel)+1;
		Graph *g = abstractions[nextLevel];
		which = g->GetNode(which->GetLabelL(kParent));
	}
	return which;
}

bool GraphAbstraction::IsParentOf(node *parent, node *child)
{
	if (GetAbstractionLevel(child) > GetAbstractionLevel(parent))
		return false;
	while (GetAbstractionLevel(child) < GetAbstractionLevel(parent))
	{
		child = GetParent(child);
	}
	return (parent == child);
}

void GraphAbstraction::ClearMarkedNodes()
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		Graph *g = abstractions[x];
	 	edge_iterator ei = g->getEdgeIter();
		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
		{
			e->setMarked(false);
		}
	}
}

double GraphAbstraction::distance(path *p)
{
	if ((p == 0) || (p->next == 0) || (p->n == 0) || (p->next->n == 0))
		return 0.0;
	return h(p->n, p->next->n)+distance(p->next);
}

void GraphAbstraction::MeasureAbstractionValues(int level, double &n, double &n_dev, double &c, double &c_dev)
{
	assert((level > 0) && (level < (int)abstractions.size()));
	Graph *g = abstractions[level];
	
	std::vector<int> ns;
	std::vector<int> cs;
	n = 0; c = 0;
	node_iterator ni = g->getNodeIter();
	for (node *nd = g->nodeIterNext(ni); nd; nd = g->nodeIterNext(ni))
	{
		ns.push_back(nd->GetLabelL(kNumAbstractedNodes));
		n += ns.back();
		cs.push_back(ComputeWidth(nd));
		c += cs.back();
		//printf("node %d: #%d width:%d\n", nd->GetNum(), ns.back(), cs.back());
	}
	n/= ns.size();
	c/= cs.size();
	n_dev = 0;
	c_dev = 0;
	for (unsigned int x = 0; x < ns.size(); x++)
	{
		n_dev += (ns[x]-n)*(ns[x]-n);
		c_dev += (cs[x]-c)*(cs[x]-c);
	}
	n_dev /= ns.size();
	c_dev /= cs.size();
	n_dev = sqrtf(n_dev);
	c_dev = sqrtf(c_dev);
}

int GraphAbstraction::ComputeWidth(node *n)
{
//	printf("New WIDTH (%d)\n", n->GetNum());
	int width = 0;
	for (int child = 0; child < n->GetLabelL(kNumAbstractedNodes); child++)
	{
		width = std::max(width, WidthBFS(GetNthChild(n, child), n));
	}
	return width;
}

int GraphAbstraction::WidthBFS(node *child, node *parent)
{
	std::vector<int> depth;
	std::vector<node *> q;
	Graph *g = GetAbstractGraph(child);

	q.push_back(child);
	child->key = 0;
	depth.push_back(0);

	for (unsigned int x = 0; x < q.size(); x++)
	{
//		printf("Handling node: %d\n", q[x]->GetNum());
		neighbor_iterator ni = q[x]->getNeighborIter();
		for (int next = q[x]->nodeNeighborNext(ni); next != -1; next = q[x]->nodeNeighborNext(ni))
		{
			node *neighbor = g->GetNode(next);
			if ((neighbor->key < q.size()) && (q[neighbor->key] == neighbor))
			{
//				printf("         neighbor %d in Q\n", neighbor->GetNum());
				continue;
			}
			if (neighbor->GetLabelL(kParent) == (int)parent->GetNum())
			{
//				printf("         neighbor %d onto Q(%d) depth (%d)\n", neighbor->GetNum(), q.size(), depth[x]+1);
				neighbor->key = q.size();
				q.push_back(neighbor);
				depth.push_back(depth[x]+1);
			}
		}
	}
	return depth.back();
}

double GraphAbstraction::MeasureAverageNodeWidth(int level)
{
	if ((level > (int)abstractions.size()) || (level <= 0))
		return 0.0f;
	Graph *g = abstractions[level];

	std::vector<double> widths;
	
	double sum = 0;
	node_iterator ni = g->getNodeIter();
	for (node *nd = g->nodeIterNext(ni); nd; nd = g->nodeIterNext(ni))
	{
		double val = MeasureExpectedNodeWidth(nd);
		// ignore dead-end nodes
		if (val != 0)
		{
			widths.push_back(val);
			sum += val;
			//printf("node %d: expected width:%f\n", nd->GetNum(), val);
		}
	}
	if (widths.size() > 0)
		return sum/widths.size();
	return 0;
}

double GraphAbstraction::MeasureExpectedNodeWidth(node *n)
{
	// for each edge, the number of edges at that depth
	std::vector<std::vector<int> > edgeInfo;
	// the total number of edges for this child
	std::vector<int> sums;
	std::vector<double> value;

	edgeInfo.resize(n->GetLabelL(kNumAbstractedNodes));
	value.resize(n->GetLabelL(kNumAbstractedNodes));
	
	double expectedWidth = 0;
	int totalExternalEdges = 0;
	for (int child = 0; child < n->GetLabelL(kNumAbstractedNodes); child++)
	{
		node *c = GetNthChild(n, child);
		CountEdgesAtDistance(c, n, edgeInfo[child]);
		sums.push_back(0);
		for (unsigned int x = 0; x < edgeInfo[child].size(); x++)
		{
			sums.back() += edgeInfo[child][x];
//			printf("Node %d child %d dist %d has %d edges\n",
//						 n->GetNum(), child, x, edgeInfo[child][x]);
		}
//		printf("%d total edges [%d]\n", sums[child], child);
	}
	
	for (int child = 0; child < n->GetLabelL(kNumAbstractedNodes); child++)
	{
		double dist = 0;
		// this node has at least one external edge
		// and two external edges total
		if ((edgeInfo[child][0] != 0) && sums[child] > 1)
		{
			totalExternalEdges += edgeInfo[child][0];
			for (unsigned int x = 0; x < edgeInfo[child].size(); x++)
			{
				if (edgeInfo[child][x] > 0)
				{
					if (x == 0)
						dist += (edgeInfo[child][x]-1.0)*(x+1);
					else
						dist += (edgeInfo[child][x])*(x+1);
				}
			}
			dist /= (sums[child]-1);
		}
//		printf("Average width = %f\n", dist);
		value[child] = dist;
	}
	
	if (totalExternalEdges != 0)
	{
		for (int child = 0; child < n->GetLabelL(kNumAbstractedNodes); child++)
		{
			expectedWidth += (value[child]*edgeInfo[child][0])/(double)totalExternalEdges;
		}
	}
//	printf("Expected width = %f\n", expectedWidth);
	return expectedWidth;
}

// get the number of edges that n has outside of p
int GraphAbstraction::GetNumExternalEdges(node *n, node *p)
{
	int count = 0;
	neighbor_iterator ni = n->getNeighborIter();
	Graph *g = GetAbstractGraph(n);
	for (int next = n->nodeNeighborNext(ni); next != -1; next = n->nodeNeighborNext(ni))
	{
		node *nb = g->GetNode(next);
		if (GetParent(nb) != p)
			count++;
	}
	return count;
}

// count the number of external edges the child has from the parent at
// each distance
int GraphAbstraction::CountEdgesAtDistance(node *child, node *parent, std::vector<int> &dists)
{
	dists.resize(0);
	
	std::vector<unsigned int> depth;
	std::vector<node *> q;
	Graph *g = GetAbstractGraph(child);
	
	q.push_back(child);
	child->key = 0;
	depth.push_back(0);
	
	for (unsigned int x = 0; x < q.size(); x++)
	{
		//		printf("Handling node: %d\n", q[x]->GetNum());
		neighbor_iterator ni = q[x]->getNeighborIter();
		if (depth[x]+1 > dists.size())
			dists.resize(depth[x]+1);
		dists[depth[x]] += GetNumExternalEdges(q[x], parent);

		for (int next = q[x]->nodeNeighborNext(ni); next != -1; next = q[x]->nodeNeighborNext(ni))
		{
			node *neighbor = g->GetNode(next);
			if ((neighbor->key < q.size()) && (q[neighbor->key] == neighbor))
			{
				//				printf("         neighbor %d in Q\n", neighbor->GetNum());
				continue;
			}
			if (neighbor->GetLabelL(kParent) == (int)parent->GetNum())
			{
				//				printf("         neighbor %d onto Q(%d) depth (%d)\n", neighbor->GetNum(), q.size(), depth[x]+1);
				neighbor->key = q.size();
				q.push_back(neighbor);
				depth.push_back(depth[x]+1);
			}
		}
	}
	return depth.back();
}

node* GraphAbstraction::GetRandomGroundNodeFromNode(node *n)
{
	while (GetAbstractionLevel(n) != 0)
		n = GetNthChild(n, random()%GetNumChildren(n));
	return n;
}
