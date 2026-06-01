/*
 *  $Id: MapCliqueAbstraction.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapCliqueAbstraction.h"
#include "FPUtil.h"
#include "Heap.h"
#include <cmath>
#include <memory>

using namespace GraphAbstractionConstants;
using namespace std;

enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const static int verbose = kQuiet;//kMiscMessages;//kRepairGraph;

/**
* Construct a new Graph hierarchy.
 *
 * Constructions a new Graph abstraction hierarchy from the Graph using the
 * designated abstraction method.
 */
MapCliqueAbstraction::MapCliqueAbstraction(Map *_m, bool uniform)
:MapAbstraction(_m), abstractUniformly(uniform)
{
	buildAbstractions();
#ifdef INSTRUMENT_REPAIR
	hit.clear();
	hit.resize(abstractions.size());
#endif
}

MapCliqueAbstraction::~MapCliqueAbstraction()
{
	cleanMemory();
}

void MapCliqueAbstraction::VerifyHierarchy()
{
	cout << "VERIFY START" << endl;
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		// first make sure Graph is ok
		abstractions[x]->verifyGraph();
		// then make sure abstraction is ok
		node_iterator ni = abstractions[x]->getNodeIter();
		for (node *n = abstractions[x]->nodeIterNext(ni); n; n = abstractions[x]->nodeIterNext(ni))
		{
			// verify Graph, because there may be issues...
			if (n->GetLabelL(kParent) != -1)
			{
				Graph *g = abstractions[x+1];
				node *parent = g->GetNode(n->GetLabelL(kParent));
				bool found = false;
				for (int y = 0; y < parent->GetLabelL(kNumAbstractedNodes); y++)
				{
					if (parent->GetLabelL(kFirstData+y) == (long)n->GetNum())
					{ found = true; break; }
				}
				if (!found)
				{
					cout << "VERIFY: Graph doesn't verify; child:" << endl << *n << endl;
					cout << "VERIFY: Graph doesn't verify; parent:" << endl << *parent << endl;
				}
			}
			if (x > 0)
			{
				Graph *g = abstractions[x-1];
				for (int y = 0; y < n->GetLabelL(kNumAbstractedNodes); y++)
				{
					node *child = g->GetNode(n->GetLabelL(kFirstData+y));
					if (!child)
					{
						cout << "VERIFY: Graph doesn't verify; CHILD is null, parent:" << endl << *n << endl;
					}
					else if (child->GetLabelL(kParent) != (long)n->GetNum())
					{
						cout << "VERIFY: Graph doesn't verify; parent:" << endl << *n << endl;
						cout << "VERIFY: Graph doesn't verify; child:" << endl << *child << endl;
					}
				}
			}
			else {
				if (n->GetLabelL(kNumAbstractedNodes) != 1)
					cout << "VERIFY: Level 0 node listed as having more than 1 child!" << endl;
				int x1, y1;
				GetTileFromNode(n, x1, y1);
				if (n != GetNodeFromMap(x1, y1))
					cout << "VERIFY: node doesn't correspond to underlying map" << endl << *n << endl;
			}
		}
		// verify edges
		edge_iterator ei = abstractions[x]->getEdgeIter();
		for (edge *e = abstractions[x]->edgeIterNext(ei); e; e = abstractions[x]->edgeIterNext(ei))
		{
			node *p1, *p2;
			p1 = findNodeParent(abstractions[x]->GetNode(e->getFrom()));
			p2 = findNodeParent(abstractions[x]->GetNode(e->getTo()));
			if (p1 == p2)
				continue;
			if ((p1 == 0) || (p2 == 0))
			{
				cout << "VERIFY: One edge parent is null, and the other isn't " << *e << endl << *p1 << endl << *p2 << endl;
				continue;
			}
			if (!abstractions[x+1]->FindEdge(p1->GetNum(), p2->GetNum()))
			{
				cout << "Didn't find parent edge of " << *e << " at abslevel " << x << endl;
				cout << *p1 << endl << *p2 << endl;
			}
			p1 = abstractions[x]->GetNode(e->getFrom());
			p2 = abstractions[x]->GetNode(e->getTo());
			// VERIFY edge weights
			if (!fequal(e->GetWeight(), h(p1, p2)))
			{
				cout << "VERIFY: Edge weight doesn't match heuristic cost. Level: " << x << endl;
				cout << *p1 << endl << *p2 << endl << *e << endl;
				cout << "P1: (" << p1->GetLabelF(kXCoordinate) << ", " << p1->GetLabelF(kYCoordinate)
					<< ", " << p1->GetLabelF(kZCoordinate) << ")" << endl;
				if (p1->GetLabelL(kAbstractionLevel) == 0)
					cout << "P1: (" << p1->GetLabelL(kFirstData) << ", " << p1->GetLabelL(kFirstData+1) << ")" << endl;
				cout << "P2: (" << p2->GetLabelF(kXCoordinate) << ", " << p2->GetLabelF(kYCoordinate)
					<< ", " << p2->GetLabelF(kZCoordinate) << ")" << endl;
				if (p2->GetLabelL(kAbstractionLevel) == 0)
					cout << "P2: (" << p2->GetLabelL(kFirstData) << ", " << p2->GetLabelL(kFirstData+1) << ")" << endl;
				cout << "weight: " << e->GetWeight() << " heuristic " << h(p1, p2) << endl;
			}
			if (e->GetLabelL(kEdgeCapacity) == 0)
				cout << "VERIFY: Edge capacity is 0?!? " << e << endl;
			if (x > 0) // we can verify the capacity
			{
				int count = 0;
				// we should find kEdgeCapacity edges between the children of p1 and p2
				p1 = abstractions[x]->GetNode(e->getFrom());
				p2 = abstractions[x]->GetNode(e->getTo());
				for (int c1 = 0; c1 < p1->GetLabelL(kNumAbstractedNodes); c1++)
				{
					for (int c2 = 0; c2 < p2->GetLabelL(kNumAbstractedNodes); c2++)
					{
						if (abstractions[x-1]->FindEdge(p1->GetLabelL(kFirstData+c1),
																						p2->GetLabelL(kFirstData+c2)))
						{
							count++;
						}
					}
				}
				if (count != e->GetLabelL(kEdgeCapacity))
				{
					cout << "VERIFY: Edge capactiy of " << *e << " is "
					<< e->GetLabelL(kEdgeCapacity) << " but we only found " << count
					<< " edges below that abstract into it." << endl;
				}
			}
		}
	}
	cout << "VERIFY END" << endl;
}

void MapCliqueAbstraction::cleanMemory()
{
	clearDisplayLists();
	while (displayLists.size() > 0)
		displayLists.pop_back();
}

void MapCliqueAbstraction::clearDisplayLists()
{
	for (unsigned int x = 0; x < displayLists.size(); x++)
	{
		if (displayLists[x] != 0) glDeleteLists(displayLists[x], 1);
		displayLists[x] = 0;
	}
}

void MapCliqueAbstraction::buildAbstractions()
{
	int totalNodes = 0;
	cleanMemory();
	
	abstractions.push_back(GetMapGraph(GetMap()));
	Graph *g = abstractions[0];
	//  abstractions.push_back(g = GetMapGraph(m));
	if (displayLists.size() != 1)
		displayLists.push_back(0);
	if (verbose)
		printf("Base Graph (0) has %d nodes\n", g->GetNumNodes());
	
	for (int x = 1; ; x++)
	{
		if (g->GetNumEdges() == 0) break;
		if (verbose&kMiscMessages) printf("Building abstraction #%2d\n", x);
		abstractions.push_back(g = abstractGraph(g));
		if (verbose&kMiscMessages)
		{
			//printf("Abstract Graph #%2d has %d nodes\n", x, g->GetNumNodes());
			g->printStats();
		}
		totalNodes += g->GetNumNodes();
		displayLists.push_back(0);
	}
	// printf("%d nodes, excluding bottom level", totalNodes);
}

Graph *MapCliqueAbstraction::abstractGraph(Graph *g)
{
	return cliqueAbstractGraph(g);
}

//Graph *MapCliqueAbstraction::neighborAbstractGraph(Graph *g, int width)
//{
//	 std::vector<node *> remainingNodes;
//	 Graph *aGraph = new Graph();
//	 node_iterator ni = g->getNodeIter();
//	 node *newNode;
//	 
//	 // do we want to abstract big nodes first?
//	 ni = g->getNodeIter();
//	 for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni)) {
//		 if (n->GetLabelL(kParent) != -1) continue;
//		 newNode = new node("");
//		 aGraph->AddNode(newNode);
//
//		 newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//		 newNode->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
//		 newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//		 newNode->SetLabelL(kNodeBlocked, 0);
//		 newNode->SetLabelF(kXCoordinate, kUnknownPosition);
//		 
//		 addNodesToParent(g, n, newNode, width);
//	 }
//	 
//	 // now add all the edges
//	 edge_iterator ei = g->getEdgeIter();
//	 for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei)) {
//		 int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
//		 int to = g->GetNode(e->getTo())->GetLabelL(kParent);
//		 edge *f=0;//, *g=0;
//			 if ((from != to) && (!(f = aGraph->FindEdge(to, from)))) {
//				 double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
//				 f = new edge(from, to, weight);
//				 f->SetLabelL(kEdgeCapacity, 1);
//				 aGraph->AddEdge(f);
//			 }
//			 else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
//			 //		else if (g)
//			 //			g->setLabel(kEdgeCapacity, g->GetLabelL(kEdgeCapacity)+1);
//	 }
//	 
//	 return aGraph;
//}
	
void MapCliqueAbstraction::addNodesToParent(Graph *g, node *n, node *parent, int width)
{
	if (n->GetLabelL(kParent) != -1)
		return;
	
	// add this node; add all neighbors
	int oldChildren = parent->GetLabelL(kNumAbstractedNodes);
	parent->SetLabelL(kFirstData+oldChildren, n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, oldChildren+1);
	n->SetLabelL(kParent, parent->GetNum());
	
	if (width <= 0)
		return;
	edge_iterator ei = n->getEdgeIter();
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		if (e->getFrom() == n->GetNum())
			addNodesToParent(g, g->GetNode(e->getTo()), parent, width-1);
		else
			addNodesToParent(g, g->GetNode(e->getFrom()), parent, width-1);
	}
}


Graph *MapCliqueAbstraction::cliqueAbstractGraph(Graph *g)
{
	int abLevel = g->GetNode(0)->GetLabelL(kAbstractionLevel);
	//	if (g != abstractions[0])
	//		return GraphAbstraction::abstractGraph(g);
	
	//  printf("Doing special map abstraction at level %d\n", (int)g->GetNode(0)->GetLabelL(kAbstractionLevel));
	
	// 1) join singly linked paths into single nodes
	// 2) join 4-cliques
	// 3) join 3-cliques
	// 4) join 2-cliques
	std::vector<node *> remainingNodes;
	Graph *aGraph = new Graph();
	node_iterator ni = g->getNodeIter();
	node *newNode;
	
	if (abstractUniformly)
	{
		// going through the map grid in a regular way, trying to abstract
		for (int x = 0; x < GetMap()->GetMapWidth()-1; x+=(2<<abLevel))
		{
			for (int y = 0; y < GetMap()->GetMapHeight()-1; y+=(2<<abLevel))
			{
				// try to abstract (x, y), (x+1, y), (x, y+1) and (x+1, y+1)
				node *a, *b, *c, *d;
				a = GetNthParent(GetNodeFromMap(x, y), abLevel);
				if (!a) continue;
				b = GetNthParent(GetNodeFromMap(x+(1<<abLevel), y), abLevel);
				if (!b) continue;
				c = GetNthParent(GetNodeFromMap(x, y+(1<<abLevel)), abLevel);
				if (!c) continue;
				d = GetNthParent(GetNodeFromMap(x+(1<<abLevel), y+(1<<abLevel)), abLevel);
				if (!d) continue;
				
				if (g->FindEdge(a->GetNum(), b->GetNum()) && g->FindEdge(b->GetNum(), c->GetNum()) &&
						g->FindEdge(c->GetNum(), d->GetNum()) && g->FindEdge(d->GetNum(), a->GetNum()) &&
						g->FindEdge(a->GetNum(), c->GetNum()) && g->FindEdge(b->GetNum(), d->GetNum()))
				{ // we have a 4-clique!
					int nnum = aGraph->AddNode(newNode = new node("4c"));
					a->SetLabelL(kParent, nnum);
					b->SetLabelL(kParent, nnum);
					c->SetLabelL(kParent, nnum);
					d->SetLabelL(kParent, nnum);
					
					newNode->SetLabelL(kAbstractionLevel, a->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
					newNode->SetLabelL(kNumAbstractedNodes, 4); // number of abstracted nodes
					newNode->SetLabelL(kNodeBlocked, 0);
					newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
					newNode->SetLabelF(kXCoordinate, kUnknownPosition);
					newNode->SetLabelL(kFirstData, a->GetNum()); // nodes stored here
					newNode->SetLabelL(kFirstData+1, b->GetNum()); // nodes stored here
					newNode->SetLabelL(kFirstData+2, c->GetNum()); // nodes stored here
					newNode->SetLabelL(kFirstData+3, d->GetNum()); // nodes stored here
				}
			}
		}
	}
	
	// no tunnels for now -- they aren't cliques
	//	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	//	{
	//		if (n->GetLabelL(kParent) != -1)
	//			continue;
	//		if (n->GetNumEdges() == 2)
	//		{
	//			neighbor_iterator nbi = n->getNeighborIter();
	//			int n1 = n->nodeNeighborNext(nbi);
	//			int n2 = n->nodeNeighborNext(nbi);
	//			if ((g->GetNode(n1)->GetNumEdges() == 2) || (g->GetNode(n2)->GetNumEdges() == 2))
	//			{
	//				int nnum = aGraph->AddNode(newNode = new node("tunnel"));
	//				n->setLabel(kParent, nnum);
	//				
	//				newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
	//				newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
	//				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
	//				newNode->setLabel(kNodeBlocked, 0);
	//				newNode->setLabel(kXCoordinate, kUnknownPosition);
	//				newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
	//				
	//				addTunnel(n, g, newNode);
	//			}
	//		}
	//	}
	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		if (n->GetLabelL(kParent) != -1) continue;
		int numEdges = n->GetNumEdges();//getNumOutgoingEdges() + n->getNumIncomingEdges();
			
			// why abstract solo nodes; they just get confusing later?
			//		if (numEdges == 0)
			//		{
			//			n->setLabel(kParent, aGraph->AddNode(newNode = new node("1c")));
			//			newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			//			newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
			//			newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
			//			newNode->setLabel(kNodeBlocked, 0);
			//			newNode->setLabel(kXCoordinate, kUnknownPosition);
			//			newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
			//			continue;
			//		}
			if (numEdges == 1)
			{
				// add to stack; we will merge these after we've finished everything else
				remainingNodes.push_back(n);
				continue;
			}
			
			std::vector<int> neighbor(numEdges);
			edge *e;
			edge_iterator ei = n->getEdgeIter();
			for (int x = 0; x < numEdges; x++)
			{
				e = n->edgeIterNext(ei);
				if (e == 0)
				{
					cout << "That's impossible; we were told we had "
					<< numEdges << ":(" << n->getNumOutgoingEdges()
					<< "+" << n->getNumIncomingEdges()
					<< ") edges, we're on #" << x << " and we got nil!"
					<< endl;
					numEdges = x;
					continue;
				}
				if (e->getFrom() == n->GetNum()) neighbor[x] = e->getTo();
				else                             neighbor[x] = e->getFrom();
			}
			
			// check for all cliques involving 4 nodes
			for (int x = 0; x < numEdges-2; x++)
			{
				if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1) continue;
				for (int y = x+1; y < numEdges-1; y++)
				{
					if (g->GetNode(neighbor[y])->GetLabelL(kParent) != -1) continue;
					for (int z = y+1; z < numEdges; z++)
					{
						if (g->GetNode(neighbor[z])->GetLabelL(kParent) != -1) continue;
						// each node has edge to us; now we also must have
						// x<->y, y<->z, x<->z
						//						if ((g->FindEdge(neighbor[x], neighbor[y]) || g->FindEdge(neighbor[y], neighbor[x])) &&
						//								(g->FindEdge(neighbor[y], neighbor[z]) || g->FindEdge(neighbor[z], neighbor[y])) &&
						//								(g->FindEdge(neighbor[z], neighbor[x]) || g->FindEdge(neighbor[x], neighbor[z])))
						if (g->FindEdge(neighbor[x], neighbor[y]) &&
								g->FindEdge(neighbor[y], neighbor[z]) &&
								g->FindEdge(neighbor[z], neighbor[x]))
						{
							// we have a 4-clique!
							int nnum = aGraph->AddNode(newNode = new node("4c"));
							n->SetLabelL(kParent, nnum);
							g->GetNode(neighbor[x])->SetLabelL(kParent, nnum);
							g->GetNode(neighbor[y])->SetLabelL(kParent, nnum);
							g->GetNode(neighbor[z])->SetLabelL(kParent, nnum);
							
							newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
							newNode->SetLabelL(kNumAbstractedNodes, 4); // number of abstracted nodes
							newNode->SetLabelL(kNodeBlocked, 0);
							newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
							newNode->SetLabelF(kXCoordinate, kUnknownPosition);
							newNode->SetLabelL(kFirstData, n->GetNum()); // nodes stored here
							newNode->SetLabelL(kFirstData+1, neighbor[x]); // nodes stored here
							newNode->SetLabelL(kFirstData+2, neighbor[y]); // nodes stored here
							newNode->SetLabelL(kFirstData+3, neighbor[z]); // nodes stored here
							
							x = y = numEdges;
							break;
						}
					}
				}
			}
			
			// check for all cliques involving 3 nodes
			if (n->GetLabelL(kParent) == -1)
			{
				for (int x = 0; x < numEdges-1; x++)
				{
					if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1) continue;
					for (int y = x+1; y < numEdges; y++)
					{
						if (g->GetNode(neighbor[y])->GetLabelL(kParent) != -1) continue;
						// each node has edge to us; now we also must have
						// x<->y
						//						if (g->FindEdge(neighbor[x], neighbor[y]) || g->FindEdge(neighbor[y], neighbor[x]))
						if (g->FindEdge(neighbor[x], neighbor[y]))
						{
							// we have a 3-clique!
							int nnum = aGraph->AddNode(newNode = new node("3c"));
							n->SetLabelL(kParent, nnum);
							g->GetNode(neighbor[x])->SetLabelL(kParent, nnum);
							g->GetNode(neighbor[y])->SetLabelL(kParent, nnum);
							
							newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
							newNode->SetLabelL(kNumAbstractedNodes, 3); // number of abstracted nodes
							newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
							newNode->SetLabelL(kNodeBlocked, 0);
							newNode->SetLabelF(kXCoordinate, kUnknownPosition);
							newNode->SetLabelL(kFirstData, n->GetNum()); // nodes stored here
							newNode->SetLabelL(kFirstData+1, neighbor[x]); // nodes stored here
							newNode->SetLabelL(kFirstData+2, neighbor[y]); // nodes stored here
							
							x = numEdges;
							break;
						}
					}
				}
			}
			
			//		if (n->GetLabelL(kParent) == -1)
			//		{
			//			// check for all cliques involving 2 nodes
			//			for (int x = 0; x < numEdges; x++)
			//			{
			//				if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1)
			//					continue;
			//				// we have a 2-clique!
			//				int nnum = aGraph->AddNode(newNode = new node("2c"));
			//				n->setLabel(kParent, nnum);
			//				g->GetNode(neighbor[x])->setLabel(kParent, nnum);
			//
			//				newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			//				newNode->setLabel(kNumAbstractedNodes, 2); // number of abstracted nodes
			//				newNode->setLabel(kNodeBlocked, 0);
			//				newNode->setLabel(kXCoordinate, kUnknownPosition);
			//				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
			//				newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
			//				newNode->setLabel(kFirstData+1, neighbor[x]); // nodes stored here
			//				break;
			//			}
			//		}
			
			// we didn't find a 3 or 4 clique
			if (n->GetLabelL(kParent) == -1) remainingNodes.push_back(n);
	}
	
	while (remainingNodes.size() > 0)
	{
		node *orphan = (node*)remainingNodes.back();
		if (!orphan) break;
		remainingNodes.pop_back();
		if (orphan->GetLabelL(kParent) != -1) continue;
		int numEdges = orphan->getNumOutgoingEdges() + orphan->getNumIncomingEdges();
		if (numEdges == 0) continue;
		
		edge_iterator ei = orphan->getEdgeIter();
		for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei))
		{
			int neighbor = (e->getFrom() == orphan->GetNum())?e->getTo():e->getFrom();
			if (g->GetNode(neighbor)->GetLabelL(kParent) == -1)
			{
				unsigned int pNum;
				pNum = aGraph->AddNode(newNode = new node("2c"));
				orphan->SetLabelL(kParent, pNum);
				g->GetNode(neighbor)->SetLabelL(kParent, pNum);
				
				newNode->SetLabelL(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
				newNode->SetLabelL(kNumAbstractedNodes, 2); // number of abstracted nodes
				newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
				newNode->SetLabelL(kNodeBlocked, 0);
				newNode->SetLabelF(kXCoordinate, kUnknownPosition);
				newNode->SetLabelL(kFirstData, orphan->GetNum()); // nodes stored here
				newNode->SetLabelL(kFirstData+1, neighbor); // nodes stored here
				break;
			}
		}
		if ((orphan->GetLabelL(kParent) == -1) && (orphan->GetNumEdges() == 1))
		{
			ei = orphan->getEdgeIter();
			for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei))
			{
				int neighbor = (e->getFrom() == orphan->GetNum())?e->getTo():e->getFrom();
				//printf("merging %d into %d (%d)\n", orphan->GetNum(), neighbor, g->GetNode(neighbor)->GetLabelL(kParent));
				node *adoptee = g->GetNode(neighbor);
				orphan->SetLabelL(kParent, adoptee->GetLabelL(kParent));
				
				node *adopteeParent = aGraph->GetNode(adoptee->GetLabelL(kParent));
				adopteeParent->SetLabelL(kFirstData+adopteeParent->GetLabelL(kNumAbstractedNodes), orphan->GetNum());
				adopteeParent->SetLabelL(kNumAbstractedNodes, adopteeParent->GetLabelL(kNumAbstractedNodes)+1);
				break;
			}
		}
		if (orphan->GetLabelL(kParent) == -1)
		{
			orphan->SetLabelL(kParent, aGraph->AddNode(newNode = new node("stranded*")));
			newNode->SetLabelL(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			newNode->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
			newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			newNode->SetLabelL(kNodeBlocked, 0);
			newNode->SetLabelF(kXCoordinate, kUnknownPosition);
			newNode->SetLabelL(kFirstData, orphan->GetNum()); // nodes stored here
		}
		
		
		//				// we aren't going to push nodes into their neighbors for the moment, because it ruins the
		//				// clique property of nodes.
		////      else {
		////				//printf("merging %d into %d (%d)\n", orphan->GetNum(), neighbor, g->GetNode(neighbor)->GetLabelL(kParent));
		////				node *adoptee = g->GetNode(neighbor);
		////				orphan->setLabel(kParent, adoptee->GetLabelL(kParent));
		////				
		////				node *adopteeParent = aGraph->GetNode((int)adoptee->GetLabelL(kParent));
		////				adopteeParent->setLabel(kFirstData+(int)adopteeParent->GetLabelL(kNumAbstractedNodes), orphan->GetNum());
		////				adopteeParent->setLabel(kNumAbstractedNodes, adopteeParent->GetLabelL(kNumAbstractedNodes)+1);
		//// 			}
		//		}
		// else
		//		if (orphan->GetLabelL(kParent) == -1)
		//		{
		//		orphan->setLabel(kParent, aGraph->AddNode(newNode = new node("stranded*")));
		//		newNode->setLabel(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
		//		newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
		//		newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
		//		newNode->setLabel(kNodeBlocked, 0);
		//		newNode->setLabel(kXCoordinate, kUnknownPosition);
		//		newNode->setLabel(kFirstData, orphan->GetNum()); // nodes stored here
		//		}
}

// now add all the edges
edge_iterator ei = g->getEdgeIter();
for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
{
	int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
	int to = g->GetNode(e->getTo())->GetLabelL(kParent);
	edge *f=0;//, *g=0;
						//if ((from != to) && (!(f = aGraph->FindEdge(to, from))) && (!(g = aGraph->FindEdge(from, to))))
		if ((from != to) && (!(f = aGraph->FindEdge(to, from))))
		{
			//			double weight = (aGraph->GetNode(from)->GetLabelL(kNumAbstractedNodes))+
			//															 (aGraph->GetNode(to)->GetLabelL(kNumAbstractedNodes));
			//			weight /= 2;
			//			weight += e->GetWeight();
			double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
			f = new edge(from, to, weight);
			f->SetLabelL(kEdgeCapacity, 1);
			aGraph->AddEdge(f);
			if (verbose&kBuildGraph)
				printf("Adding edge from %d (%d) to %d (%d) weight %1.2f\n", from, e->getFrom(), to, e->getTo(), weight);
		}
		else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
		//		else if (g)
		//			g->setLabel(kEdgeCapacity, g->GetLabelL(kEdgeCapacity)+1);
}

return aGraph;
}


//Graph *MapCliqueAbstraction::cliqueAbstractGraph(Graph *g)
//{
//  //printf("getting abstract Graph of level %d\n", g->GetNode(0)->GetLabelL(kAbstractionLevel));
//	
//  // 1) join singly linked paths into single nodes
//  // 2) join 4-cliques
//  // 3) join 3-cliques
//  // 4) join 2-cliques
//  std::vector<node *> remainingNodes;
//  Graph *aGraph = new Graph();
//  node_iterator ni = g->getNodeIter();
//  node *newNode;
//	
//  // no tunnels for now -- they aren't cliques
//  //	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
//  //	{
//  //		if (n->GetLabelL(kParent) != -1)
//  //			continue;
//  //		if (n->GetNumEdges() == 2)
//  //		{
//  //			neighbor_iterator nbi = n->getNeighborIter();
//  //			int n1 = n->nodeNeighborNext(nbi);
//  //			int n2 = n->nodeNeighborNext(nbi);
//  //			if ((g->GetNode(n1)->GetNumEdges() == 2) || (g->GetNode(n2)->GetNumEdges() == 2))
//  //			{
//  //				int nnum = aGraph->AddNode(newNode = new node("tunnel"));
//  //				n->setLabel(kParent, nnum);
//  //				
//  //				newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//  //				newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//  //				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//  //				newNode->setLabel(kNodeBlocked, 0);
//  //				newNode->setLabel(kXCoordinate, kUnknownPosition);
//  //				newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
//  //				
//  //				addTunnel(n, g, newNode);
//  //			}
//  //		}
//  //	}
//  ni = g->getNodeIter();
//  for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni)) {
//    if (n->GetLabelL(kParent) != -1) continue;
//    int numEdges = n->GetNumEdges();//getNumOutgoingEdges() + n->getNumIncomingEdges();
//			
//			// why abstract solo nodes; they just get confusing later?
//			//		if (numEdges == 0)
//			//		{
//			//			n->setLabel(kParent, aGraph->AddNode(newNode = new node("1c")));
//			//			newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//			//			newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//			//			newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//			//			newNode->setLabel(kNodeBlocked, 0);
//			//			newNode->setLabel(kXCoordinate, kUnknownPosition);
//			//			newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
//			//			continue;
//			//		}
//			if (numEdges == 1) {
//				// add to stack; we will merge these after we've finished everything else
//				remainingNodes.push_back(n);
//				continue;
//			}
//			
//			int neighbor[numEdges];
//			edge *e;
//			edge_iterator ei = n->getEdgeIter();
//			for (int x = 0; x < numEdges; x++) {
//				e = n->edgeIterNext(ei);
//				if (e == 0) {
//					cout << "That's impossible; we were told we had "
//					<< numEdges << ":(" << n->getNumOutgoingEdges()
//					<< "+" << n->getNumIncomingEdges()
//					<< ") edges, we're on #" << x << " and we got nil!"
//					<< endl;
//					numEdges = x;
//					continue;
//				}
//				if (e->getFrom() == n->GetNum()) neighbor[x] = e->getTo();
//				else                             neighbor[x] = e->getFrom();
//			}
//			
//			// check for all cliques involving 4 nodes
//			for (int x = 0; x < numEdges-2; x++) {
//				if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1) continue;
//				for (int y = x+1; y < numEdges-1; y++) {
//					if (g->GetNode(neighbor[y])->GetLabelL(kParent) != -1) continue;
//					for (int z = y+1; z < numEdges; z++) {
//						if (g->GetNode(neighbor[z])->GetLabelL(kParent) != -1) continue;
//						// each node has edge to us; now we also must have
//						// x<->y, y<->z, x<->z
//						//						if ((g->FindEdge(neighbor[x], neighbor[y]) || g->FindEdge(neighbor[y], neighbor[x])) &&
//						//								(g->FindEdge(neighbor[y], neighbor[z]) || g->FindEdge(neighbor[z], neighbor[y])) &&
//						//								(g->FindEdge(neighbor[z], neighbor[x]) || g->FindEdge(neighbor[x], neighbor[z])))
//						if (g->FindEdge(neighbor[x], neighbor[y]) &&
//								g->FindEdge(neighbor[y], neighbor[z]) &&
//								g->FindEdge(neighbor[z], neighbor[x])) {
//							// we have a 4-clique!
//							int nnum = aGraph->AddNode(newNode = new node("4c"));
//							n->SetLabelL(kParent, nnum);
//							g->GetNode(neighbor[x])->SetLabelL(kParent, nnum);
//							g->GetNode(neighbor[y])->SetLabelL(kParent, nnum);
//							g->GetNode(neighbor[z])->SetLabelL(kParent, nnum);
//							
//							newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//							newNode->SetLabelL(kNumAbstractedNodes, 4); // number of abstracted nodes
//							newNode->SetLabelL(kNodeBlocked, 0);
//							newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//							newNode->SetLabelF(kXCoordinate, kUnknownPosition);
//							newNode->SetLabelL(kFirstData, n->GetNum()); // nodes stored here
//							newNode->SetLabelL(kFirstData+1, neighbor[x]); // nodes stored here
//							newNode->SetLabelL(kFirstData+2, neighbor[y]); // nodes stored here
//							newNode->SetLabelL(kFirstData+3, neighbor[z]); // nodes stored here
//							
//							x = y = numEdges;
//							break;
//						}
//					}
//				}
//			}
//			
//			// check for all cliques involving 3 nodes
//			if (n->GetLabelL(kParent) == -1) {
//				for (int x = 0; x < numEdges-1; x++) {
//					if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1) continue;
//					for (int y = x+1; y < numEdges; y++) {
//						if (g->GetNode(neighbor[y])->GetLabelL(kParent) != -1) continue;
//						// each node has edge to us; now we also must have
//						// x<->y
//						//						if (g->FindEdge(neighbor[x], neighbor[y]) || g->FindEdge(neighbor[y], neighbor[x]))
//						if (g->FindEdge(neighbor[x], neighbor[y])) {
//							// we have a 3-clique!
//							int nnum = aGraph->AddNode(newNode = new node("3c"));
//							n->SetLabelL(kParent, nnum);
//							g->GetNode(neighbor[x])->SetLabelL(kParent, nnum);
//							g->GetNode(neighbor[y])->SetLabelL(kParent, nnum);
//							
//							newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//							newNode->SetLabelL(kNumAbstractedNodes, 3); // number of abstracted nodes
//							newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//							newNode->SetLabelL(kNodeBlocked, 0);
//							newNode->SetLabelF(kXCoordinate, kUnknownPosition);
//							newNode->SetLabelL(kFirstData, n->GetNum()); // nodes stored here
//							newNode->SetLabelL(kFirstData+1, neighbor[x]); // nodes stored here
//							newNode->SetLabelL(kFirstData+2, neighbor[y]); // nodes stored here
//							
//							x = numEdges;
//							break;
//						}
//					}
//				}
//			}
//			
//			//		if (n->GetLabelL(kParent) == -1)
//			//		{
//			//			// check for all cliques involving 2 nodes
//			//			for (int x = 0; x < numEdges; x++)
//			//			{
//			//				if (g->GetNode(neighbor[x])->GetLabelL(kParent) != -1)
//			//					continue;
//			//				// we have a 2-clique!
//			//				int nnum = aGraph->AddNode(newNode = new node("2c"));
//			//				n->setLabel(kParent, nnum);
//			//				g->GetNode(neighbor[x])->setLabel(kParent, nnum);
//			//
//			//				newNode->setLabel(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//			//				newNode->setLabel(kNumAbstractedNodes, 2); // number of abstracted nodes
//			//				newNode->setLabel(kNodeBlocked, 0);
//			//				newNode->setLabel(kXCoordinate, kUnknownPosition);
//			//				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//			//				newNode->setLabel(kFirstData, n->GetNum()); // nodes stored here
//			//				newNode->setLabel(kFirstData+1, neighbor[x]); // nodes stored here
//			//				break;
//			//			}
//			//		}
//			
//			// we didn't find a 3 or 4 clique
//			if (n->GetLabelL(kParent) == -1) remainingNodes.push_back(n);
//  }
//	
//  while (remainingNodes.size() > 0) {
//    node *orphan = (node*)remainingNodes.back();
//    if (!orphan) break;
//    remainingNodes.pop_back();
//    if (orphan->GetLabelL(kParent) != -1) continue;
//    int numEdges = orphan->getNumOutgoingEdges() + orphan->getNumIncomingEdges();
//    if (numEdges == 0) continue;
//		
//    edge_iterator ei = orphan->getEdgeIter();
//    for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei)) {
//      int neighbor = (e->getFrom() == orphan->GetNum())?e->getTo():e->getFrom();
//      if (g->GetNode(neighbor)->GetLabelL(kParent) == -1) {
//				unsigned int pNum;
//				pNum = aGraph->AddNode(newNode = new node("2c"));
//				orphan->SetLabelL(kParent, pNum);
//				g->GetNode(neighbor)->SetLabelL(kParent, pNum);
//				
//				newNode->SetLabelL(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//				newNode->SetLabelL(kNumAbstractedNodes, 2); // number of abstracted nodes
//				newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//				newNode->SetLabelL(kNodeBlocked, 0);
//				newNode->SetLabelF(kXCoordinate, kUnknownPosition);
//				newNode->SetLabelL(kFirstData, orphan->GetNum()); // nodes stored here
//				newNode->SetLabelL(kFirstData+1, neighbor); // nodes stored here
//				break;
//      }
//    }
//    if ((orphan->GetLabelL(kParent) == -1) && (orphan->GetNumEdges() == 1)) {
//      ei = orphan->getEdgeIter();
//      for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei)) {
//				int neighbor = (e->getFrom() == orphan->GetNum())?e->getTo():e->getFrom();
//				//printf("merging %d into %d (%d)\n", orphan->GetNum(), neighbor, g->GetNode(neighbor)->GetLabelL(kParent));
//				node *adoptee = g->GetNode(neighbor);
//				orphan->SetLabelL(kParent, adoptee->GetLabelL(kParent));
//				
//				node *adopteeParent = aGraph->GetNode(adoptee->GetLabelL(kParent));
//				adopteeParent->SetLabelL(kFirstData+adopteeParent->GetLabelL(kNumAbstractedNodes), orphan->GetNum());
//				adopteeParent->SetLabelL(kNumAbstractedNodes, adopteeParent->GetLabelL(kNumAbstractedNodes)+1);
//				break;
//      }
//    }
//    if (orphan->GetLabelL(kParent) == -1) {
//      orphan->SetLabelL(kParent, aGraph->AddNode(newNode = new node("stranded*")));
//      newNode->SetLabelL(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//      newNode->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
//      newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//      newNode->SetLabelL(kNodeBlocked, 0);
//      newNode->SetLabelF(kXCoordinate, kUnknownPosition);
//      newNode->SetLabelL(kFirstData, orphan->GetNum()); // nodes stored here
//    }
//		
//		
//    //				// we aren't going to push nodes into their neighbors for the moment, because it ruins the
//    //				// clique property of nodes.
//    ////      else {
//    ////				//printf("merging %d into %d (%d)\n", orphan->GetNum(), neighbor, g->GetNode(neighbor)->GetLabelL(kParent));
//    ////				node *adoptee = g->GetNode(neighbor);
//    ////				orphan->setLabel(kParent, adoptee->GetLabelL(kParent));
//    ////				
//    ////				node *adopteeParent = aGraph->GetNode((int)adoptee->GetLabelL(kParent));
//    ////				adopteeParent->setLabel(kFirstData+(int)adopteeParent->GetLabelL(kNumAbstractedNodes), orphan->GetNum());
//    ////				adopteeParent->setLabel(kNumAbstractedNodes, adopteeParent->GetLabelL(kNumAbstractedNodes)+1);
//    //// 			}
//    //		}
//    // else
//    //		if (orphan->GetLabelL(kParent) == -1)
//    //		{
//    //		orphan->setLabel(kParent, aGraph->AddNode(newNode = new node("stranded*")));
//    //		newNode->setLabel(kAbstractionLevel, orphan->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
//    //		newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//    //		newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//    //		newNode->setLabel(kNodeBlocked, 0);
//    //		newNode->setLabel(kXCoordinate, kUnknownPosition);
//    //		newNode->setLabel(kFirstData, orphan->GetNum()); // nodes stored here
//    //		}
//  }
//	
//  // now add all the edges
//  edge_iterator ei = g->getEdgeIter();
//  for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei)) {
//    int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
//    int to = g->GetNode(e->getTo())->GetLabelL(kParent);
//    edge *f=0;//, *g=0;
//							//if ((from != to) && (!(f = aGraph->FindEdge(to, from))) && (!(g = aGraph->FindEdge(from, to))))
//			if ((from != to) && (!(f = aGraph->FindEdge(to, from)))) {
//				//			double weight = (aGraph->GetNode(from)->GetLabelL(kNumAbstractedNodes))+
//				//															 (aGraph->GetNode(to)->GetLabelL(kNumAbstractedNodes));
//				//			weight /= 2;
//				//			weight += e->GetWeight();
//				double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
//				f = new edge(from, to, weight);
//				f->SetLabelL(kEdgeCapacity, 1);
//				aGraph->AddEdge(f);
//				if (verbose&kBuildGraph)
//					printf("Adding edge from %d (%d) to %d (%d) weight %1.2f\n", from, e->getFrom(), to, e->getTo(), weight);
//			}
//			else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
//			//		else if (g)
//			//			g->setLabel(kEdgeCapacity, g->GetLabelL(kEdgeCapacity)+1);
//  }
//	
//  return aGraph;
//}
	

void MapCliqueAbstraction::addTunnel(node *n, Graph *g, node *newNode)
{
	if (verbose&kBuildGraph) printf("Adding node %d to tunnel\n", n->GetNum());
	// check to see if we have neighbors with bf 2 which we can merge with
	neighbor_iterator nbi = n->getNeighborIter();
	int n1 = n->nodeNeighborNext(nbi);
	int n2 = n->nodeNeighborNext(nbi);
	if ((g->GetNode(n1)->GetLabelL(kParent) == -1) && (g->GetNode(n1)->GetNumEdges() == 2))
	{
		newNode->SetLabelL(kFirstData+newNode->GetLabelL(kNumAbstractedNodes), n1); // nodes stored here
		newNode->SetLabelL(kNumAbstractedNodes, newNode->GetLabelL(kNumAbstractedNodes)+1);
		g->GetNode(n1)->SetLabelL(kParent, newNode->GetNum());
		addTunnel(g->GetNode(n1), g, newNode);
	}
	if ((g->GetNode(n2)->GetLabelL(kParent) == -1) && (g->GetNode(n2)->GetNumEdges() == 2))
	{
		newNode->SetLabelL(kFirstData+newNode->GetLabelL(kNumAbstractedNodes), n2); // nodes stored here
		newNode->SetLabelL(kNumAbstractedNodes, newNode->GetLabelL(kNumAbstractedNodes)+1);
		g->GetNode(n2)->SetLabelL(kParent, newNode->GetNum());
		addTunnel(g->GetNode(n2), g, newNode);
	}
}

bool MapCliqueAbstraction::Pathable(unsigned int from, unsigned int to)
{
	return Pathable(abstractions[0]->GetNode(from), abstractions[0]->GetNode(to));
}

bool MapCliqueAbstraction::Pathable(node *from, node *to)
{
	//printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
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

void MapCliqueAbstraction::AddNode(node *)
{
}

void MapCliqueAbstraction::AddEdge(edge *, unsigned int)
{
}

// for now we'll immediately handle splits, but in the future we should queue up splits
// and process them in batch form(?)
void MapCliqueAbstraction::RemoveEdge(edge *e, unsigned int absLevel)
{
	if (e == 0)
		return;
	edge *ep = findEdgeParent(e, absLevel);
	if (ep)
	{
		ep->SetLabelL(kEdgeCapacity, ep->GetLabelL(kEdgeCapacity)-1);
		if (ep->GetLabelL(kEdgeCapacity) == 0) RemoveEdge(ep, absLevel+1);
		//printf("Removing edge %p in abstract Graph %d\n", e, absLevel);
		abstractions[absLevel]->RemoveEdge(e);
		delete e;
		return;
	}
	
	// it's an internal edge, so it might break a link in a parent abstract node
	if (absLevel+1 < abstractions.size())
	{
		node *pNode = abstractions[absLevel+1]->GetNode(abstractions[absLevel]->GetNode(e->getFrom())->GetLabelL(kParent));
		addNodeToRepairQ(pNode);
	}
	
	//printf("Removing edge %p in abstract Graph %d\n", e, absLevel);
	abstractions[absLevel]->RemoveEdge(e);
	delete e;
	return;
}

void MapCliqueAbstraction::addNodeToRepairQ(node *n)
{
	// key is unsigned, so it has to be >= 0
	if (n)
	{
		if ((n->key >= modifiedNodeQ.size()) ||
				((n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] != n)))
			//		if ((n->key < 0) || (n->key >= modifiedNodeQ.size()) ||
			//				((n->key >= 0) && (n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] != n)))
		{
			n->key = modifiedNodeQ.size();
			if (verbose&kRepairGraph)
				cout << "REM: Adding " << *n << " to modified queue" << endl;
			modifiedNodeQ.push_back(n);
		}
	}
}

void MapCliqueAbstraction::removeNodeFromRepairQ(node *n)
{
	// key is unsigned, so it has to be >= 0
	//	if ((n->key >= 0) && (n->key < modifiedNodeQ.size()) &&
	//			(modifiedNodeQ[n->key] == n))
	if ((n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] == n))
	{
		modifiedNodeQ[n->key] = modifiedNodeQ.back();
		modifiedNodeQ[n->key]->key = n->key;
		modifiedNodeQ.pop_back();
	}
}

void MapCliqueAbstraction::RemoveNode(node *n)
{
	if (n == 0) return;
	if (verbose&kRepairGraph)
		cout << "REM: Removing " << *n << endl;
	removeNodeFromRepairQ(n);
	edge_iterator ei;
	ei = n->getEdgeIter();
	unsigned int absLevel = n->GetLabelL(kAbstractionLevel);
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		edge *ep = findEdgeParent(e, absLevel);
		if (!ep)// edge is internal to a node
		{
			node *pNode = abstractions[absLevel+1]->GetNode(n->GetLabelL(kParent));
#ifdef INSTRUMENT_REPAIR
			hit[absLevel+1] += 1; // hits
#endif
			if (!pNode) continue;
			addNodeToRepairQ(pNode);
			continue;
		}
		ep->SetLabelL(kEdgeCapacity, ep->GetLabelL(kEdgeCapacity)-1);
		if (ep->GetLabelL(kEdgeCapacity) == 0)
		{
			RemoveEdge(ep, n->GetLabelL(kAbstractionLevel)+1);
		}
	}
	
	node *np = findNodeParent(n);
	if (np)
	{
		resetLocationCache(np);
		//np->setLabel(kXCoordinate, kUnknownPosition);
		if (np->GetLabelL(kNumAbstractedNodes) == 1)
		{
			// our parent might be in the modified node Q. If it is,
			// we can just take it out
			removeNodeFromRepairQ(np);
			
			if (verbose&kRepairGraph)
				printf("Removing parent!\n");
			RemoveNode(np);
			n->SetLabelL(kParent, -1);
		} else {
			// find this node (n) and removed it from the list
			for (int x = 0; x < np->GetLabelL(kNumAbstractedNodes); x++)
			{
				if (np->GetLabelL(kFirstData+x) == (long)n->GetNum())
				{
					np->SetLabelL(kFirstData+x,
												np->GetLabelL((kFirstData+np->GetLabelL(kNumAbstractedNodes)-1)));
					break;
				}
			}
			np->SetLabelL(kNumAbstractedNodes, np->GetLabelL(kNumAbstractedNodes)-1);
			resetLocationCache(np);
		}
	}
	unsigned int oldID;
	// now we can safely remove this node from the Graph
	node *changed = abstractions[absLevel]->RemoveNode(n, oldID);
#ifdef INSTRUMENT_REPAIR
	hit[absLevel] += 1; // hits
#endif
	// have to handle changing node here...rename it in the parent & children if necessary
	if (changed)
	{
		renameNodeInAbstraction(changed, oldID);
	}
}

void MapCliqueAbstraction::RepairAbstraction()
{
	// actually want to sort items...based on abstraction level, doing
	// lowest abstraction level first
	while (modifiedNodeQ.size() > 0)
	{
		node *temp, *changed = modifiedNodeQ.back();
		modifiedNodeQ.pop_back();
		// selection sort...assuming that modifiedNodeQ is small for now
		for (unsigned int x = 0; x < modifiedNodeQ.size(); x++)
		{
			if (modifiedNodeQ[x]->GetLabelL(kAbstractionLevel) <
					changed->GetLabelL(kAbstractionLevel))
			{
				temp = modifiedNodeQ[x];
				modifiedNodeQ[x] = changed;
				changed->key = x;
				changed = temp;
			}
		}
		if (verbose&kRepairGraph)
			cout << "REM: Choosing to repair: " << *changed << endl;
		int count = getChildGroups(changed);
		if (count != 1)
		{
			if (verbose&kRepairGraph)
				cout << "REM: We got " << count << " groups out of " << *changed << endl;
			splitNode(changed, count);
		}
	}
}

/*
 * Takes a node and does a simple bfs on its children to find connected
 * components. Marks the group of each child with kTemporary label &
 * returns the number of groups found.
 */
int MapCliqueAbstraction::getChildGroups(node *which)
{
	std::vector<node *> seenStack;
	seenStack.reserve(which->GetLabelL(kNumAbstractedNodes));
	unsigned int absLevel = which->GetLabelL(kAbstractionLevel);
	if (absLevel == 0)
		return 0;
	Graph *g = abstractions[absLevel-1];
#ifdef INSTRUMENT_REPAIR
	hit[absLevel-1] += 1; // hits
#endif

	for (int x = 0; x < which->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(which->GetLabelL(kFirstData+x));
		nextChild->SetLabelL(kTemporaryLabel, -1);
	}
	int currGroup = -1;
	for (int x = 0; x < which->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(which->GetLabelL(kFirstData+x));
		
		if (nextChild->GetLabelL(kTemporaryLabel) != -1)
			continue;
		
		currGroup++;
		nextChild->SetLabelL(kTemporaryLabel, currGroup);
		if (verbose&kRepairGraph)
			cout << *nextChild << " now assigned to group " << currGroup << endl;
		do {
			if (seenStack.size() > 0)
			{
				nextChild = seenStack.back();
				seenStack.pop_back();
			}
			edge_iterator ei = nextChild->getEdgeIter();
			for (edge *e = nextChild->edgeIterNext(ei); e; e = nextChild->edgeIterNext(ei))
			{
				unsigned int neighbor = (e->getFrom() == nextChild->GetNum()) ?
				(e->getTo()):(e->getFrom());
				if ((g->GetNode(neighbor)->GetLabelL(kParent) == (long)which->GetNum()) &&
						(g->GetNode(neighbor)->GetLabelL(kTemporaryLabel) == -1))
				{
					g->GetNode(neighbor)->SetLabelL(kTemporaryLabel, currGroup);
					if (verbose&kRepairGraph)
						cout << *g->GetNode(neighbor) << " now added to group " << currGroup << endl;
					seenStack.push_back(g->GetNode(neighbor));
				}
			}
		} while (seenStack.size() > 0);
	}
	return currGroup+1;
}


/*
 * Takes a node & splits it so that it is connected.
 */
void MapCliqueAbstraction::splitNode(node *parent, int numGroups)
{
	// get all children nodes that we are re-assigning
	std::vector<node *> children;
	children.reserve(parent->GetLabelL(kNumAbstractedNodes));
	for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
	{
		children[x] = abstractions[parent->GetLabelL(kAbstractionLevel)-1]->
		GetNode(parent->GetLabelL(kFirstData+x));
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
	}
	
	// 1. if parent has no neighbors, we can just split the parent into separate pieces
	//    since it has already been split. Then we're done.
	if (parent->GetNumEdges() == 0)
	{
		if (verbose&kRepairGraph)
			cout << "REM: parent has no neighbors, just splitting and ending" << endl;
		for (int x = 1; x < numGroups; x++)
			extractGroupIntoNewNode(parent, x);
		return;
	}
	
	std::vector<int> groupSize(numGroups);
	for (int x = 0; x < numGroups; x++) groupSize[x] = getGroupSize(parent, x);
	// now, progress through the following steps until we are done
	// 2. all single nodes with bf 1 should merge into neighbors
	// 3. other single nodes should try to join nodes if they can form a clique
	// 4. remaining single nodes abstract by themselves
	int reassigned = 0;
	for (int x = 0; x < numGroups; x++)
	{
		if (reassigned == numGroups-1) // done when we assign all but 1
			break;
		if (groupSize[x] == 1)
		{
			node *n = getNodeInGroup(parent, x);
			if (n->GetNumEdges() == 0)
			{
				if (verbose&kRepairGraph)
					cout << "REM: Reassigning group " << reassigned << " by splitting off group " << x << endl;
				extractGroupIntoNewNode(parent, x);
				reassigned++;
			}
			else if (n->GetNumEdges() == 1)
			{
				if (verbose&kRepairGraph)
					cout << "REM: Reassigning group " << reassigned << " by (neighbor) merging group " << x << endl;
				mergeGroupIntoNeighbor(parent, x);
				reassigned++;
			}
			else { // (n->GetNumEdges() > 1) 	   
				node *neighbor;
				if ((neighbor = findNeighborCliques(parent, x)))
				{
					if (verbose&kRepairGraph)
						cout << "REM: Reassigning group " << reassigned << " by (neighbor) merging " << x << endl;
					mergeGroupIntoNeighbor(parent, x, neighbor);
				}
				else {
					if (verbose&kRepairGraph)
						cout << "REM: Reassigning group " << reassigned << " by extraction " << x << endl;
					extractGroupIntoNewNode(parent, x);
				}
				reassigned++;
			}
		}
	}
	// 5. all groups >= 2 nodes should break off by themselves
	// we do this last so that we make sure to abstract off any single nodes properly
	// no use leaving them sitting here alone if they could have gone off with another node,
	// and we sent a 2+ group off instead
	for (int x = 0; x < numGroups; x++)
	{
		if (reassigned == numGroups-1) // done when we assign all but 1
			break;
		if (getGroupSize(parent, x) > 0)
		{
			if (verbose&kRepairGraph)
				cout << "REM: Reassigning (big) group " << reassigned << " by extracting " << x << endl;
			extractGroupIntoNewNode(parent, x);
			reassigned++;
		}
	}
}

/*
 * Find any node in parent that is a member of "group"
 */
node *MapCliqueAbstraction::getNodeInGroup(node *parent, int group)
{
	Graph *g = abstractions[parent->GetLabelL(kAbstractionLevel)-1];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
		for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(parent->GetLabelL(kFirstData+x));
		
		if (nextChild->GetLabelL(kTemporaryLabel) == group)
			return nextChild;
	}
	return 0;
}

/*
 * Count how many nodes are a member of "group"
 */
int MapCliqueAbstraction::getGroupSize(node *parent, int group)
{
	Graph *g = abstractions[parent->GetLabelL(kAbstractionLevel)-1];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
		int answer = 0;
	for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(parent->GetLabelL(kFirstData+x));
		if (nextChild->GetLabelL(kTemporaryLabel) == group)
			answer++;
	}
	return answer;
}

/*
 * check to see if the node in the group can be merged into another
 * node as part of a clique. Returns the neighbor node at the child level
 */
// PRECOND: group only has 1 node
node *MapCliqueAbstraction::findNeighborCliques(node *parent, int group)
{
	node *child = 0;
	Graph *g = abstractions[parent->GetLabelL(kAbstractionLevel)-1];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
		for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(parent->GetLabelL(kFirstData+x));
		if (nextChild->GetLabelL(kTemporaryLabel) == group)
		{
			child = nextChild;
			break;
		}
	}
	if (child)
		return findNeighborCliques(child);
	return 0;
}

/*
 * check to see if the node can be merged into another
 * node as part of a clique. Returns the neighbor node.
 */
node *MapCliqueAbstraction::findNeighborCliques(node *child)
{
	Graph *g = abstractions[child->GetLabelL(kAbstractionLevel)];
#ifdef INSTRUMENT_REPAIR
		hit[child->GetLabelL(kAbstractionLevel)] += 1; // hits
#endif
		edge_iterator ei = child->getEdgeIter();
	for (edge *e = child->edgeIterNext(ei); e; e = child->edgeIterNext(ei))
	{
		node *nextNode = g->GetNode((e->getFrom() == child->GetNum())?(e->getTo()):(e->getFrom()));
		if (checkNeighborClique(child, nextNode))
			return nextNode;
	}
	return 0;
}

/*
 * check to see if the node is connected to every other node in the group of
 * the neighbor. If a node has only 1 neighbor it doesn't have to be connected to
 * it, but it must be connected to at least 1 node and every node of degree 2 or greater
 */
bool MapCliqueAbstraction::checkNeighborClique(node *child, node *neighbor)
{
	node *neighborParent = abstractions[neighbor->GetLabelL(kAbstractionLevel)+1]->GetNode(neighbor->GetLabelL(kParent));
#ifdef INSTRUMENT_REPAIR
		hit[neighbor->GetLabelL(kAbstractionLevel)+1] += 1; // hits
#endif
		if (neighborParent == 0)
		return false;
	Graph *g = abstractions[child->GetLabelL(kAbstractionLevel)];
#ifdef INSTRUMENT_REPAIR
		hit[child->GetLabelL(kAbstractionLevel)] += 1; // hits
#endif

	int matches = 0;
	for (int x = 0; x < neighborParent->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->GetNode(neighborParent->GetLabelL(kFirstData+x));
		// we only require that we connect to every node in the abstraction
		// that has more than 1 neighbor
		if (g->FindEdge(nextChild->GetNum(), child->GetNum()))
			matches++;
		else if (nextChild->GetNumEdges() > 1)
			return false;
	}
	
	if (matches) return true;
	
	return false;
}

/*
 * Take all nodes in group of parent and move them into neighbor's group
 * neighbor is one level lower than the parent
 */
void MapCliqueAbstraction::mergeGroupIntoNeighbor(node *parent, int group, node *neighbor)
{
	Graph *g;
	if (neighbor == 0)
	{ // only 1 possible neighbor, so find it
		g = abstractions[parent->GetLabelL(kAbstractionLevel)-1];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
		for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
		{
			node *nextChild = g->GetNode(parent->GetLabelL(kFirstData+x));
			if (nextChild->GetLabelL(kTemporaryLabel) == group)
			{
				edge_iterator ei = nextChild->getEdgeIter();
				edge *e = nextChild->edgeIterNext(ei);
				if (e->getFrom() == nextChild->GetNum())
					neighbor = g->GetNode(e->getTo());
				else
					neighbor = g->GetNode(e->getFrom());
				break;
			}
		}
	}
	assert(neighbor->GetLabelL(kAbstractionLevel)+1 == parent->GetLabelL(kAbstractionLevel));
	if (verbose&kRepairGraph)
		cout << "Merging group " << group << " into " << *neighbor << endl;
	g = abstractions[parent->GetLabelL(kAbstractionLevel)];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)] += 1; // hits
#endif
		node *newParent = g->GetNode(neighbor->GetLabelL(kParent));
	if (verbose&kRepairGraph)
		cout << "(Child of " << *newParent << ")" << endl;
	// what happens if that node has no parent???
	if (newParent == 0)
	{
		printf("GOTTA HANDLE THE NO PARENT CASE IN mergeGroupIntoNeighbor --- but logic says we shouldn't hit this unless we add edges\n");
		exit(0);
	}
	if (newParent == parent)
	{
		printf("HOW DID THAT HAPPEN? oldParent = newParent\n");
		exit(0);
	}
	else {
		assert(parent->GetLabelL(kAbstractionLevel) == newParent->GetLabelL(kAbstractionLevel));
		transferGroup(group, parent, newParent);
	}
}

/*
 * take all member of group in parent and make them their own node
 */
void MapCliqueAbstraction::extractGroupIntoNewNode(node *parent, int group)
{
	// make new node...
	node *newNode;
	Graph *g = abstractions[parent->GetLabelL(kAbstractionLevel)];
#ifdef INSTRUMENT_REPAIR
		hit[parent->GetLabelL(kAbstractionLevel)] += 1; // hits
#endif
		
  /*unsigned int gpNum = */g->AddNode(newNode = new node("split node"));

	newNode->SetLabelL(kAbstractionLevel, parent->GetLabelL(kAbstractionLevel));
	newNode->SetLabelL(kNumAbstractedNodes, 0);
	newNode->SetLabelL(kParent, -1);
	newNode->SetLabelF(kXCoordinate, kUnknownPosition);
	newNode->SetLabelL(kNodeBlocked, 0);
	
	transferGroup(group, parent, newNode);
	
	// now, what do I do with newNode? It needs to see if it can be combine with it's neighbors, etc...
	insertNodeIntoHierarchy(newNode);
}

/*
 * Given a new node that is connected, but doesn't have a parent or have its edges
 * abstracted, and insert it into Graph.
 */
void MapCliqueAbstraction::insertNodeIntoHierarchy(node *newNode)
{
	node *neighbor;
	if (newNode->GetNumEdges() == 0)
		return;
	if (newNode->GetNumEdges() == 1)
	{
		Graph *g = abstractions[newNode->GetLabelL(kAbstractionLevel)];
#ifdef INSTRUMENT_REPAIR
		hit[newNode->GetLabelL(kAbstractionLevel)] += 1; // hits
#endif
		edge *e = newNode->getEdge(0);
		node *newParent = g->GetNode((e->getFrom() == newNode->GetNum())?(e->getTo()):(e->getFrom()));
		checkAndCreateParent(newParent);
		g = abstractions[newNode->GetLabelL(kAbstractionLevel)+1];
#ifdef INSTRUMENT_REPAIR
		hit[newNode->GetLabelL(kAbstractionLevel)+1] += 1; // hits
#endif
		newParent = g->GetNode(newParent->GetLabelL(kParent));
		
		newParent->SetLabelL(kFirstData+newParent->GetLabelL(kNumAbstractedNodes), newNode->GetNum());
		newParent->SetLabelL(kNumAbstractedNodes, newParent->GetLabelL(kNumAbstractedNodes)+1);
		//newParent->setLabel(kXCoordinate, kUnknownPosition);
		resetLocationCache(newParent);
		if (verbose&kRepairGraph)
		{
			printf("Collapsing node into neighbor: ");
			printf("New parent (%d) now has %ld abstracted nodes\n", newParent->GetNum(),
						 newParent->GetLabelL(kNumAbstractedNodes));
		}
		newNode->SetLabelL(kParent, newParent->GetNum());		
	}
	else if ((neighbor = findNeighborCliques(newNode)))
	{ // add newnode to neighbor's parent
		checkAndCreateParent(neighbor);
		node *parent = abstractions[neighbor->GetLabelL(kAbstractionLevel)+1]->GetNode(neighbor->GetLabelL(kParent));
#ifdef INSTRUMENT_REPAIR
		hit[neighbor->GetLabelL(kAbstractionLevel)+1] += 1; // hits
#endif
		newNode->SetLabelL(kParent, parent->GetNum());
		parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), newNode->GetNum());
		parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
		//parent->setLabel(kXCoordinate, kUnknownPosition);
		resetLocationCache(parent);
		if (verbose&kRepairGraph)
		{
			printf("Cliquing node into neighbor: ");
			printf("New parent (%d) now has %ld abstracted nodes\n", parent->GetNum(),
						 parent->GetLabelL(kNumAbstractedNodes));
		}
		
		unsigned int absLevel = newNode->GetLabelL(kAbstractionLevel);
		edge_iterator ei = newNode->getEdgeIter();
		for (edge *e = newNode->edgeIterNext(ei); e; e = newNode->edgeIterNext(ei))
		{
			abstractUpEdge(absLevel, e);
		}
	}
	else {
		if (verbose&kRepairGraph)
			cout << "We stay lonely: " << *newNode << endl;
		// add direct parent
		checkAndCreateParent(newNode);
		// add edges...
		unsigned int absLevel = newNode->GetLabelL(kAbstractionLevel);
		edge_iterator ei = newNode->getEdgeIter();
		for (edge *e = newNode->edgeIterNext(ei); e; e = newNode->edgeIterNext(ei))
		{
			abstractUpEdge(absLevel, e);
		}
		Graph *g = abstractions[absLevel+1];
#ifdef INSTRUMENT_REPAIR
		hit[absLevel+1] += 1; // hits
#endif
		insertNodeIntoHierarchy(g->GetNode(newNode->GetLabelL(kParent)));
	}
}

/*
 * Check to see if a node has a parent. If it doesn't, create one, and optionally
 * extend the abstraction level of the Graph as well.
 */
void MapCliqueAbstraction::checkAndCreateParent(node *which)
{
	if (which->GetLabelL(kParent) != -1)
		return;
	unsigned int absLevel = which->GetLabelL(kAbstractionLevel);
	if (absLevel+1 == abstractions.size())
	{
		abstractions.push_back(new Graph());
#ifdef INSTRUMENT_REPAIR
		hit.push_back(0);
#endif
	}
	node *parent;
	unsigned int id = abstractions[absLevel+1]->AddNode(parent = new node("created"));
#ifdef INSTRUMENT_REPAIR
		hit[absLevel+1] += 1; // hits
#endif
		which->SetLabelL(kParent, id);
	parent->SetLabelL(kAbstractionLevel, absLevel+1);
	parent->SetLabelL(kNumAbstractedNodes, 1);
	parent->SetLabelL(kParent, -1);
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
	parent->SetLabelL(kNodeBlocked, 0);
	parent->SetLabelL(kFirstData, which->GetNum());
}

/*
 * take all nodes in a group of old parent and move them into newParent
 */
void MapCliqueAbstraction::transferGroup(int group, node *oldParent, node *newParent)
{
	std::vector<node *> groupies;
	
	assert(oldParent->GetLabelL(kAbstractionLevel) == newParent->GetLabelL(kAbstractionLevel));
	
	groupies.reserve(oldParent->GetLabelL(kNumAbstractedNodes));
	Graph *g = abstractions[oldParent->GetLabelL(kAbstractionLevel)-1];
#ifdef INSTRUMENT_REPAIR
		hit[oldParent->GetLabelL(kAbstractionLevel)-1] += 1; // hits
#endif
	for (int x = 0; x < oldParent->GetLabelL(kNumAbstractedNodes); x++)
	{
		node *nextNode = g->GetNode(oldParent->GetLabelL(kFirstData+x));
		if (nextNode->GetLabelL(kTemporaryLabel) == group)
		{
			groupies.push_back(nextNode);
			
			// before I move this node over, I have to change its old edges...FIRST
			edge_iterator ei = nextNode->getEdgeIter();
			for (edge *e = nextNode->edgeIterNext(ei); e; e = nextNode->edgeIterNext(ei))
			{
				edge *ep = findEdgeParent(e, nextNode->GetLabelL(kAbstractionLevel));
				if (ep)
				{
					ep->SetLabelL(kEdgeCapacity, ep->GetLabelL(kEdgeCapacity)-1);
					if (ep->GetLabelL(kEdgeCapacity) == 0)
					{
						RemoveEdge(ep, nextNode->GetLabelL(kAbstractionLevel)+1);
					}
				}
			}				
			
			oldParent->SetLabelL(kFirstData+x, oldParent->GetLabelL(kFirstData+oldParent->GetLabelL(kNumAbstractedNodes)-1));
			oldParent->SetLabelL(kNumAbstractedNodes, oldParent->GetLabelL(kNumAbstractedNodes)-1);
			if (verbose&kRepairGraph)
				printf("Old parent (%d) now has %ld abstracted nodes\n", oldParent->GetNum(), oldParent->GetLabelL(kNumAbstractedNodes));
			newParent->SetLabelL(kFirstData+newParent->GetLabelL(kNumAbstractedNodes), nextNode->GetNum());
			newParent->SetLabelL(kNumAbstractedNodes, newParent->GetLabelL(kNumAbstractedNodes)+1);
			if (verbose&kRepairGraph)
				printf("New parent (%d) now has %ld abstracted nodes\n", newParent->GetNum(), newParent->GetLabelL(kNumAbstractedNodes));
			nextNode->SetLabelL(kParent, newParent->GetNum());
			resetLocationCache(oldParent);
			resetLocationCache(newParent);
			//oldParent->setLabel(kXCoordinate, kUnknownPosition);
			//newParent->setLabel(kXCoordinate, kUnknownPosition);
			x--;
		}
	}
	
	//	g = abstractions[oldParent->GetLabelL(kAbstractionLevel)-1];
	for (unsigned int x = 0; x < groupies.size(); x++)
	{
		node *which = groupies[x];
		// now, re-add parent edges
		edge_iterator ei = which->getEdgeIter();
		for (edge *e = which->edgeIterNext(ei); e; e = which->edgeIterNext(ei))
		{
			if (verbose&kRepairGraph)
				cout << "Group member: " << *which << " has edge " << *e << " which we want to abstract up" << endl;
			abstractUpEdge(which->GetLabelL(kAbstractionLevel), e);
		}
	}
	
}

/*
 * given an edge e at a particular abstraction level, keep abstracting the edge
 * until it disappears...
 *
 * note that if we connect an orphan node (ie one with degree 1) we might
 * break the assumptions of the abstraction & then want to break a node out
 *
 * same thing with connecting a node with degree 0 to a node with degree 1
 */
void MapCliqueAbstraction::abstractUpEdge(unsigned int absLevel, edge *e)
{
	// 1 find edge in parent
	Graph *g = abstractions[absLevel];
	Graph *pg = abstractions[absLevel+1];
#ifdef INSTRUMENT_REPAIR
		hit[absLevel] += 1; // hits
		hit[absLevel+1] += 1; // hits
#endif
		int par1, par2;
	par1 = g->GetNode(e->getFrom())->GetLabelL(kParent);
	par2 = g->GetNode(e->getTo())->GetLabelL(kParent);
	if (par1 == par2)
		return;
	if (par1 == -1)
	{ //addNodeToRepairQ(g->GetNode(e->getFrom()));
		if (verbose&kRepairGraph)
			cout << "This node has " << g->GetNode(e->getFrom())->GetNumEdges() <<
				" edge(s), but no parent: " << endl << *g->GetNode(e->getFrom()) << endl;
		return;
	}
	if (par2 == -1)
	{ //addNodeToRepairQ(g->GetNode(e->getFrom()));
		if (verbose&kRepairGraph)
			cout << "This node has " << g->GetNode(e->getTo())->GetNumEdges() <<
				" edge(s), but no parent: " << endl << *g->GetNode(e->getTo()) << endl;
		return;
	}
	edge *pe = pg->FindEdge(par1, par2);
	
	// 2 if it exists, just add to the count
	if (pe)
		pe->SetLabelL(kEdgeCapacity, pe->GetLabelL(kEdgeCapacity)+1);
	// 3 if it doesn't exist, add it, and recurse
	else {
		double edgeWeight = h(pg->GetNode(par1), pg->GetNode(par2));
		pg->AddEdge(pe = new edge(par1, par2, edgeWeight));
		pe->SetLabelL(kEdgeCapacity, 1);
		if (verbose&kRepairGraph)
			cout << "*** Abstracting " << *e << " with edge " << *pe << endl;
		abstractUpEdge(absLevel+1, pe);
	}
}

void MapCliqueAbstraction::resetLocationCache(node *n)
{
	unsigned int absLevel = n->GetLabelL(kAbstractionLevel);
	while (true)
	{
		n->SetLabelF(kXCoordinate, kUnknownPosition);
		GetNodeLoc(n);
		
		Graph *g = abstractions[n->GetLabelL(kAbstractionLevel)];
//#ifdef INSTRUMENT_REPAIR
//		hit[n->GetLabelL(kAbstractionLevel)] += 1; // hits
//#endif
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			node *p1, *p2;
			p1 = g->GetNode(e->getFrom());
			p2 = g->GetNode(e->getTo());
			double edgeWeight = h(p1, p2);
			e->setWeight(edgeWeight);
		}
		
		int parent = n->GetLabelL(kParent);
		if (parent == -1)
			break;
		absLevel++;
		n = abstractions[absLevel]->GetNode(parent);
//#ifdef INSTRUMENT_REPAIR
//		hit[absLevel] += 1; // hits
//#endif
	}
}

node *MapCliqueAbstraction::findNodeParent(node *n)
{
  unsigned int absLevel = n->GetLabelL(kAbstractionLevel);
  if (absLevel < abstractions.size()-1)
	{
#ifdef INSTRUMENT_REPAIR
		hit[absLevel+1] += 1; // hits
#endif
    return abstractions[absLevel+1]->GetNode(n->GetLabelL(kParent));
	}
  return 0;
}

/*
 * Given an edge at a level of abstraction, find the edge that
 * this edge abstracts into.
 */
edge *MapCliqueAbstraction::findEdgeParent(edge *e, unsigned int absLevel)
{
	if (absLevel >= abstractions.size()-1) return 0;
	
  Graph *g = abstractions[absLevel];
#ifdef INSTRUMENT_REPAIR
		hit[absLevel] += 1; // hits
		hit[absLevel+1] += 1; // hits
#endif
		
  node *from = g->GetNode(e->getFrom());
  node *to = g->GetNode(e->getTo());
	
	g = abstractions[absLevel+1];
	from = g->GetNode(from->GetLabelL(kParent));
	to = g->GetNode(to->GetLabelL(kParent));
	
	if (from == to) return 0;
	return g->FindEdge(from->GetNum(), to->GetNum());
	//	edge *ee = g->FindEdge(from->GetNum(), to->GetNum());
	//	if (ee)
	//		return ee;
	//	return g->FindEdge(to->GetNum(), from->GetNum());
}

void MapCliqueAbstraction::renameNodeInAbstraction(node *which, unsigned int oldID)
{
	unsigned int absLevel = which->GetLabelL(kAbstractionLevel);
	
  // adjust children
  if (absLevel > 0)
	{
    for (int x = 0; x < which->GetLabelL(kNumAbstractedNodes); x++)
		{
      abstractions[absLevel-1]->GetNode(which->GetLabelL(kFirstData+x))->SetLabelL(kParent, which->GetNum());
#ifdef INSTRUMENT_REPAIR
			hit[absLevel-1] += 1; // hits
#endif
    }
  }
	
  // adjust parent
  if (absLevel < abstractions.size()-1)
	{
    node *parent = abstractions[absLevel+1]->GetNode(which->GetLabelL(kParent));
#ifdef INSTRUMENT_REPAIR
		hit[absLevel+1] += 1; // hits
#endif
    if (parent)
		{
      for (int x = 0; x < parent->GetLabelL(kNumAbstractedNodes); x++)
			{
				if (parent->GetLabelL(kFirstData+x) == (long)oldID)
				{
					parent->SetLabelL(kFirstData+x, which->GetNum());
					break;
				}
			}
		}
	}
}

/** MeasureRepairHits
 * Measure the number of levels hit during repair. Always returns 0 unless compiled
 * with -D INSTRUMENT_REPAIR
 */
int MapCliqueAbstraction::MeasureRepairHits()
{
	int numHits = 0;
#ifdef INSTRUMENT_REPAIR
	for (unsigned int x = 0; x < hit.size(); x++)
	{
		if (hit[x] != 0)
			numHits++;
		printf("Level %d hit %d times\n", x, hit[x]);
	}
	hit.clear();
	hit.resize(abstractions.size());
#endif
	return numHits;
}
