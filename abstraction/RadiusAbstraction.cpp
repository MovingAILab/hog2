/*
 *  $Id: RadiusAbstraction.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "RadiusAbstraction.h"
#include "Graph.h"

using namespace GraphAbstractionConstants;

RadiusAbstraction::RadiusAbstraction(Map *_m, int _radius)
:MapAbstraction(_m), radius(_radius)
{
  assert(_m!=NULL);

  buildAbstraction();
}

RadiusAbstraction::~RadiusAbstraction()
{ 
}

bool RadiusAbstraction::Pathable(node *from, node *to)
{
  while (from != to)
	{
    if ((!from) || (!to) ||
				(abstractions[from->GetLabelL(kAbstractionLevel)]->GetNumEdges() == 0))
      return false;
		
    from = abstractions[from->GetLabelL(kAbstractionLevel)+1]->
      GetNode(from->GetLabelL(kParent));
    to = abstractions[to->GetLabelL(kAbstractionLevel)+1]->
      GetNode(to->GetLabelL(kParent));
  }
	if ((from == 0) || (to == 0))
		return false;
	return true;	
}

// utility functions
/** verify that the hierarchy is consistent */
void RadiusAbstraction::VerifyHierarchy()
{
}

// hierarchical modifications
/** remove node from abstraction */
void RadiusAbstraction::RemoveNode(node *)
{
}

/** remove edge from abstraction */
void RadiusAbstraction::RemoveEdge(edge *, unsigned int)
{
}

/** add node to abstraction */
void RadiusAbstraction::AddNode(node *)
{
	// add it to the 
	assert(false);
}

/** add edge to abstraction */
void RadiusAbstraction::AddEdge(edge *, unsigned int)
{
	assert(false);
	// if each end is already connected who cares -- doesn't mess anything up -- just add it throughout
}

/** This must be called after any of the above add/remove operations. But the
operations can be stacked followed by a single RepairAbstraction call. */
void RadiusAbstraction::RepairAbstraction()
{
}

void RadiusAbstraction::buildAbstraction()
{
	//inefficient for the moment
	abstractions.push_back(GetMapGraph(GetMap()));
	while (abstractions.back()->GetNumEdges() > 0)
	{
		Graph *g = new Graph();
		addNodes(g);
		addEdges(g);
		abstractions.push_back(g);
	}
}

void RadiusAbstraction::addNodes(Graph *g)
{
	int count = abstractions.back()->GetNumNodes();
	while (count > 0)
	{
		// select a random node
		node *next = abstractions.back()->GetRandomNode();
		assert(next!=NULL);
		// if it isn't abstracted, do a bfs according to the depth and abstract these nodes together
		if (next->GetLabelL(kParent) == -1)
		{
			node *parent;
			g->AddNode(parent = new node("??"));
			assert(parent!=NULL);
			parent->SetLabelL(kAbstractionLevel, next->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->SetLabelF(kXCoordinate, kUnknownPosition);
			parent->SetLabelL(kNodeBlocked, 0);
			abstractionBFS(next, parent, radius);
			count-=parent->GetLabelL(kNumAbstractedNodes);
		}
	}
}

void RadiusAbstraction::addEdges(Graph *aGraph)
{
	Graph *g = abstractions.back();
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
		int to = g->GetNode(e->getTo())->GetLabelL(kParent);
		edge *f=0;
		
		if ((from != to) && (!(f = aGraph->FindEdge(to, from))))
		{
			double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
			f = new edge(from, to, weight);
			f->SetLabelL(kEdgeCapacity, 1);
			aGraph->AddEdge(f);
		}
		else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
	}	
}

void RadiusAbstraction::abstractionBFS(node *which, node *parent, int depth) // depth in edges...should we try literal distance?
{
	if ((which == 0) || (which->GetLabelL(kParent) != -1))
		return;
	buildNodeIntoParent(which, parent);
	if (depth < 0)
		return;
	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		abstractionBFS(abstractions.back()->GetNode(tmp), parent, depth-1);
	}
}

void RadiusAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(GetAbstractionLevel(n)+1 == GetAbstractionLevel(parent));
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
}
