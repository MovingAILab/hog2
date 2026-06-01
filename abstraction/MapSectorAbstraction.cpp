/*
 *  $Id: MapSectorAbstraction.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 8/8/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapSectorAbstraction.h"
#include "Graph.h"

using namespace GraphAbstractionConstants;



MapSectorAbstraction::MapSectorAbstraction(Map *_m, int _sectorSize, int _sectorMultiplier)
:MapAbstraction(_m), sectorSize(_sectorSize), sectorMultiplier(_sectorMultiplier)
{
	assert(_sectorSize>1);
	assert(_sectorMultiplier>1);
	buildAbstraction();
}

MapSectorAbstraction::MapSectorAbstraction(Map *_m, int _sectorSize)
:MapAbstraction(_m), sectorSize(_sectorSize), sectorMultiplier(_sectorSize)
{
	assert(_sectorSize>1);
	buildAbstraction();
}

MapSectorAbstraction::~MapSectorAbstraction()
{
}

bool MapSectorAbstraction::Pathable(node *from, node *to)
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
void MapSectorAbstraction::VerifyHierarchy()
{
}

// hierarchical modifications
/** remove node from abstraction */
void MapSectorAbstraction::RemoveNode(node *)
{
	assert(false);
}

/** remove edge from abstraction */
void MapSectorAbstraction::RemoveEdge(edge *, unsigned int)
{
	assert(false);
}

/** add node to abstraction */
void MapSectorAbstraction::AddNode(node *)
{
	// add it to the 
	assert(false);
}

/** add edge to abstraction */
void MapSectorAbstraction::AddEdge(edge *, unsigned int)
{
	// if each end is already connected who cares -- doesn't mess anything up -- just add it throughout
	assert(false);
}

/** This must be called after any of the above add/remove operations. But the
operations can be stacked followed by a single RepairAbstraction call. */
void MapSectorAbstraction::RepairAbstraction()
{
}

void MapSectorAbstraction::buildAbstraction()
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

void MapSectorAbstraction::addNodes(Graph *g)
{
	node_iterator ni = abstractions.back()->getNodeIter();
	for (node *next = abstractions.back()->nodeIterNext(ni); next;
			 next = abstractions.back()->nodeIterNext(ni))
	{
		// if it isn't abstracted, do a bfs according to the quadrant and abstract these nodes together
		if (next->GetLabelL(kParent) == -1)
		{
			node *parent;
			g->AddNode(parent = new node("??"));
			parent->SetLabelL(kAbstractionLevel, next->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->SetLabelF(kXCoordinate, kUnknownPosition);
			parent->SetLabelL(kNodeBlocked, 0);
			abstractionBFS(next, parent, getQuadrant(next));
		}
	}
}

void MapSectorAbstraction::addEdges(Graph *aGraph)
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

void MapSectorAbstraction::abstractionBFS(node *which, node *parent, int quadrant) // depth in edges...should we try literal distance?
{
	if ((which == 0) || (which->GetLabelL(kParent) != -1) || (getQuadrant(which) != quadrant))
		return;
	buildNodeIntoParent(which, parent);

	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		abstractionBFS(abstractions.back()->GetNode(tmp), parent, quadrant);
	}
}

void MapSectorAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(GetAbstractionLevel(n)+1 == GetAbstractionLevel(parent));
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
}

int MapSectorAbstraction::getQuadrant(node *which)
{ // int sectorSize;
	// 1. get any child
	node *child = which;
	while (child->GetLabelL(kAbstractionLevel) != 0)
		child = GetNthChild(child, 0);
	
	int xloc = child->GetLabelL(kFirstData); // x loc in map
	int yloc = child->GetLabelL(kFirstData+1); // y loc in map

	int level = which->GetLabelL(kAbstractionLevel);
//	int absSectorSize = (int)pow((double)sectorSize, (double)level+1);
	int absSectorSize = (int)sectorSize*pow((double)sectorMultiplier, (double)level);
	
	int sectorNum = (GetMap()->GetMapWidth()/absSectorSize)*(yloc/absSectorSize)+(xloc/absSectorSize);
	
	return sectorNum;
}

//
// 0  1   2   3    ...
// w  w+1 w+2 w+3  ...

// rowSector = (lowerSector%width)/sectorSize
// colSector = (lowerSector/width)/sectorSize

// or

// size at level = sectorSize^level
// low-level-size-x%sizeatlevel
// low-level-size-y%sizeatlevel
