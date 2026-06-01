/*
 *  $Id: praStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "FPUtil.h"
#include "PRAStar.h"

using namespace GraphAbstractionConstants;
const static bool verbose = false;

praStar::praStar()
:SearchAlgorithm()
{
	partialLimit = -1;
	fixedPlanLevel = -1;
	sprintf(algName,"PRA*(%d)", partialLimit);
	expandSearchRadius = true; planFromMiddle = true;
	cache = 0;
}

void praStar::setCache(path **p)
{
	cache = p;
}

path *praStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *_rp)
{
	lengths.resize(0);
	rp = _rp;
	// Reset the number of states expanded
	nodesExpanded = 0;
	nodesTouched = 0;
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
	{
		if (verbose)
		{
			if (!from)
				printf("praStar: from == 0\n");
			if (!to)
				printf("praStar: to == 0\n");
			if (from == to)
				printf("praStar: from == to\n");
			if (from && to && (!aMap->Pathable(from, to)))
				printf("praStar: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
		}
		return 0;
	}
	
	std::vector<node *> fromChain;
	std::vector<node *> toChain;
	
	map = aMap;
	
	if (verbose)
		printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
	if (aMap->GetAbstractGraph(0)->FindEdge(from->GetNum(), to->GetNum())) { // we are 1 step away
		if ((cache) && (*cache))
		{
			delete *cache;
			*cache = 0;
		}
		//printf("We're just 1 step away! (or is it 2?)\n");
		return new path(from, new path(to));
	}
	
	aMap->GetNumAbstractGraphs(from, to, fromChain, toChain);
	//	assert(aMap->GetAbstractGraph(fromChain.back()->GetLabelL(kAbstractionLevel))->
	//					FindEdge(fromChain.back()->GetNum(), toChain.back()->GetNum()));
	
	path *lastPath = 0;
	if (cache && ((*cache) != 0))
	{
		if (verbose) printf("Checking cache\n");
		// check to see if cache is consistent with the planning we're going
		// to do by finding the first node in the abstraction hierarchy
		if (fromChain.back()->GetLabelL(kAbstractionLevel) < (*cache)->n->GetLabelL(kAbstractionLevel))
		{
			delete (*cache);
			*cache = 0;
		}
		else {
			while ((fromChain.back() != (*cache)->n) && (fromChain.size() > 0))
			{
				toChain.pop_back();
				fromChain.pop_back();
			}
			if (fromChain.size() == 0)
			{ printf("Cache invalid; not in first node.\n"); std::cout << *(*cache)->n; exit(10); }
			// if we find it, we set it to be our lastPath
			if (verbose) printf("Setting last path to cache\n");
			lastPath = *cache;
			toChain.pop_back();
			fromChain.pop_back();
		}
		
	}
	if (fixedPlanLevel != -1)
	{
		//while (((int)fromChain.size() > 2) && ((int)toChain.size() > fixedPlanLevel + 1))
		while (((int)fromChain.size() > 1) && ((int)toChain.size() > fixedPlanLevel + 1))
		{
			toChain.pop_back();
			fromChain.pop_back();
		}
		
		//std::cout << "top praStar level: " << (toChain.size() - 1) << std::endl;
	}
	else if ((planFromMiddle) && (lastPath == 0))
	{
		unsigned int previousSize = fromChain.size();
		int minNode = (int)(2*sqrt(aMap->GetAbstractGraph(aMap->GetAbstractionLevel(fromChain[0]))->GetNumNodes()));
		while ((fromChain.size() > 2) && ((fromChain.size() > (previousSize)/2) ||
																			(aMap->GetAbstractGraph(fromChain.size())->GetNumNodes() < minNode)))
		{
//			printf("At size %d, %d nodes\n", fromChain.size(), aMap->GetAbstractGraph(fromChain.size())->GetNumNodes());
			toChain.pop_back();
			fromChain.pop_back();
		}
//		printf("Previous size: %d, nodes: %d, limit: %d now: %d\n", previousSize,
//					 (aMap->GetAbstractGraph(aMap->GetAbstractionLevel(fromChain[0]))->GetNumNodes()),
//					 minNode, toChain.size());
	}
	else if (lastPath == 0)
	{
		// there should be an edge directly from the last nodes here....
		//		assert(aMap->GetAbstractGraph(fromChain.back()->GetLabelL(kAbstractionLevel))->
		//					 FindEdge(fromChain.back()->GetNum(), toChain.back()->GetNum()));
		// it's possible this will be a path from the node back to itself
		// but if that's the case, we'll still be ok.
		lastPath = new path(fromChain.back(), new path(toChain.back()));
		//printf("%d <- %d (start)\n", toChain.back()->GetNum(), fromChain.back()->GetNum());
		toChain.pop_back();
		fromChain.pop_back();
	}

	lengths.resize(fromChain.size());
	do {
		unsigned int destParent = 0xFFFFFFFF;
		unsigned int dest;
		
		to = toChain.back();
		from = fromChain.back();
		dest = to->GetNum();
		if (verbose)
			printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->GetLabelL(kAbstractionLevel));		
		toChain.pop_back();
		fromChain.pop_back();
		
		if (verbose)
			printf("Building path from %d to %d (%ld/%ld)\n",
						 from->GetNum(), to->GetNum(), from->GetLabelL(kParent), to->GetLabelL(kParent));
		
		std::vector<unsigned int> eligibleNodeParents;
		
		if (lastPath)
		{
			// cut path down to size of partial path limit
			if (partialLimit > 0)
			{
				path *trav = lastPath;
				//for (int x = 0; x < partialLimit*(from->GetLabelL(kAbstractionLevel)+1); x++)
				for (int x = 0; x < partialLimit; x++)
					if (trav->next) 
						trav = trav->next;
				if ((trav->next != 0) || (trav->n->GetNum() != (unsigned)to->GetLabelL(kParent)))
				{
					destParent = trav->n->GetNum();
					if (trav->next)
						dest = trav->next->n->GetLabelL(kFirstData);
					else
						dest = trav->n->GetLabelL(kFirstData);
					delete trav->next;
					trav->next = 0;
					if (verbose) printf("Setting target parent to %d\n", destParent);
				}
				// set actual target to any node in destParent
				//				if (trav->next)
				//					to = aMap->GetAbstractGraph(trav->next->n->GetLabelL(kAbstractionLevel)
				//																			-1)->GetNode(trav->n->GetLabelL(kFirstData));
				// later try for best h value to actual target
				//for (unsigned int x = 1; x < trav->n->GetLabelL(kNumAbstractedNodes); x++)
				
			}
			
			// find eligible nodes for lower level expansions
			for (path *trav = lastPath; trav; trav = trav->next)
			{
				if (expandSearchRadius)
				{
					edge_iterator ei = trav->n->getEdgeIter();
					for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei))
					{
						if (e->getFrom() == trav->n->GetNum())
							eligibleNodeParents.push_back(e->getTo());
						else
							eligibleNodeParents.push_back(e->getFrom());
					}
				}
				eligibleNodeParents.push_back(trav->n->GetNum());
			}
		}
		if ((lastPath == 0) && (cache))
		{
			lastPath = getAbstractPath(map->GetAbstractGraph((unsigned int)from->
																											 GetLabelL(kAbstractionLevel)),
																 from->GetNum(), destParent, eligibleNodeParents,
																 kTemporaryLabel, dest);
			*cache = lastPath;
			lengths[fromChain.size()] = lastPath->length();
		}
		else {
			if ((cache && ((*cache) != lastPath)) || (!cache))
				delete lastPath;
			lastPath = getAbstractPath(map->GetAbstractGraph((unsigned int)from->
																											 GetLabelL(kAbstractionLevel)),
																 from->GetNum(), destParent, eligibleNodeParents,
																 kTemporaryLabel, dest);
			lengths[fromChain.size()] = lastPath->length();
		}
	} while (fromChain.size() > 0);
	cache = 0;
	return lastPath;
}

path *praStar::getAbstractPath(Graph *g, unsigned int source, unsigned int destParent,
															 std::vector<unsigned int> &eligibleNodeParents, int LABEL,
															 unsigned int dest)
{
	path *p = 0;
	edge *e;
	// extract actual path out
	unsigned int current = astar(g, source, destParent, eligibleNodeParents, LABEL, dest);
	if (current == source) return 0;
	
	while ((e = g->GetNode(current)->getMarkedEdge()))
	{
		if (verbose) printf("%d <- ", current);
		p = new path(g->GetNode(current), p);
		
		e->setMarked(true);
		//dest = current;
		
		if (e->getFrom() == current)
			current = e->getTo();
		else
			current = e->getFrom();
	}
	p = new path(g->GetNode(current), p);
	if (verbose) printf("%d\n", current);
	return p;
}

// for each primitive action, apply it as long as we can, checking to see if
// we ever hit our path again. If so, we can use the repeated primitive operator
// as a replacement for our path

// this should move into algorithm.cpp
path *praStar::smoothPath(path *p)
{
	// 1. add path to vector so we can detect it
	// 2. go each direction and see if we hit the path
	// 3. replace segment of previous path
	return p;
}

unsigned int praStar::astar(Graph *g, unsigned int source, unsigned int destParent,
														std::vector<unsigned int> &eligibleNodeParents, int LABEL,
														unsigned int dest)
{
	node *n=0;
	Heap *nodeHeap = new Heap(eligibleNodeParents.size());
	std::vector<node *> expandedNodes(100);
	edge_iterator ei;
	node *currBest = 0;
	bool expandedAnything = false;
	unsigned int openNode = source;
	
	int absLayer = g->GetNode(source)->GetLabelL(kAbstractionLevel);
	
	// mark location of eligible nodes
	for (unsigned int x = 0; x < eligibleNodeParents.size(); x++)
		map->GetAbstractGraph(absLayer+1)->GetNode(eligibleNodeParents[x])->key = x;
	
	// label start node cost 0
	n = g->GetNode(source);
	n->SetLabelF(LABEL, map->h(n, g->GetNode(dest)));
	n->markEdge(0);
	if (verbose) printf("Starting on %d with cost %1.4f\n", source, n->GetLabelF(LABEL));
	while (1)
	{
		nodesExpanded++;
		expandedNodes.push_back(n);
		n->key = expandedNodes.size()-1;
		
		if (verbose)
			printf("really working on %d with cost %1.4f\n", n->GetNum(), n->GetLabelF(LABEL));
		
		ei = n->getEdgeIter();
		
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == openNode) which = e->getTo();
			if (verbose) printf("considering neighbor %d\n", which);
			node *currNode = g->GetNode(which);
			
			if (nodeHeap->IsIn(currNode))
			{
				//nodesExpanded++;
				relaxEdge(nodeHeap, g, e, openNode, which, dest, LABEL);
			}
#ifdef LOCAL_PATH
			else if (rp && (absLayer==0) && (currNode->GetNum() != dest) &&
							 rp->nodeOccupied(currNode))
			{
				//printf("Can't path to %d, %d\n", (unsigned int)nextChild->GetLabelL(kFirstData), (unsigned int)nextChild->GetLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
			}
#else
			//else if (currNode->GetLabelL(kNodeBlocked) > 0)
			else if (((n->GetNum() != source) && (e->GetLabelL(kEdgeCapacity) <= 0)) ||
							 ((n->GetNum() == source) && (e->GetLabelL(kEdgeCapacity) < 0)))
			{
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
			}
#endif
			// this node is unexpanded if (1) it isn't on the expanded list
			else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				
				unsigned int whichParent = (unsigned int)currNode->GetLabelL(kParent);
				unsigned int parentKey = map->GetAbstractGraph(absLayer+1)->GetNode(whichParent)->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((eligibleNodeParents.size() == 0) ||
						((parentKey < eligibleNodeParents.size()) &&
						 (eligibleNodeParents[parentKey] == whichParent)))
				{
					currNode->SetLabelF(LABEL, MAXINT);
					currNode->SetKeyLabel(LABEL);
					currNode->markEdge(0);
					nodeHeap->Add(currNode);
					if (verbose)
						printf("Adding neighbor %d\n", which);
					//nodesExpanded++;
					relaxEdge(nodeHeap, g, e, openNode, which, dest, LABEL);
				}
				else { if (verbose) printf("%d not eligible\n", currNode->GetNum()); }
			}
			else { if (verbose) printf("%d already expanded\n", currNode->GetNum()); }
		}
		
		n = (node*)nodeHeap->Remove();
		if (n == 0)
		{
			if (verbose) printf("Error: We expanded every possible node!\n");
			break;
		}
		expandedAnything = true;
		
		openNode = n->GetNum();
		if (openNode == dest) { /*printf("Found goal %d\n", dest);*/ break; }
		
		if (verbose) printf("working on %d with cost %1.4f\n", openNode, n->GetLabelF(LABEL));
		
		if (currBest)
		{
			if (currBest->GetLabelL(kParent) == n->GetLabelL(kParent))
			{
				// these lines cause us to take the node with the best h() value
				// instead of the first explored node by A* in the abstraction
				if (currBest->GetLabelF(LABEL) >	n->GetLabelF(LABEL))
					currBest = n;
			}
			else if (n->GetLabelL(kParent) == (long)destParent)
			{
				currBest = n;
				//printf("Exited 'cause we're in our parent %ld/%d (2)\n", n->GetLabelL(kParent), destParent);
				break;
			}
		} else {
			currBest = n;
			if (n->GetLabelL(kParent) == (long)destParent)
			{
				//printf("Exited 'cause we're in our parent %ld/%d (1)\n", n->GetLabelL(kParent), destParent);
				break;
			}
		}
	}
	
	delete nodeHeap;
	
	if (!expandedAnything) return source;
	
	if ((currBest) && (openNode != dest))
	{
		dest = currBest->GetNum();
		if ((partialLimit > 0) && (currBest->GetLabelL(kParent) != (long)destParent))
		{
			if (verbose) printf("Error: We somehow didn't end up in our parent...\n");
		}
	}
	return dest;
}

void praStar::relaxEdge(Heap *nodeHeap, Graph *g, edge *e, int source, int nextNode, int dest,
												int LABEL)
{
	double weight;
	node *from = g->GetNode(source);
	node *to = g->GetNode(nextNode);
	node *d = g->GetNode(dest);
	weight = from->GetLabelF(LABEL)-map->h(from, d)+map->h(to, d)+e->GetWeight();
	if (fless(weight, to->GetLabelF(LABEL)))
	{
		if (verbose)
			printf("Updating %d to %1.4f from %1.4f\n", nextNode, weight, to->GetLabelF(LABEL));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
		to->SetLabelF(LABEL, weight);
		nodeHeap->DecreaseKey(to);
		// this is the edge used to get to this node in the min. path tree
		to->markEdge(e);
	}
}


//void praStar::addAbstractedNodesToHeap(Heap *nodeHeap, Graph *g, node *p, unsigned int source,
//																			 int LABEL)
//{
//  for (int cnt = 0; cnt < p->GetLabelL(kNumAbstractedNodes); cnt++) {
//    node *childNode = g->GetNode((unsigned int)p->GetLabelL(kFirstData+cnt));
//    if (nodeHeap->IsIn(childNode)) continue;
//    if (verbose) printf("%d ", childNode->GetNum());
//    childNode->setKeyLabel(LABEL);
//    childNode->SetLabelF(LABEL, MAXINT);
//    childNode->markEdge(0);
//    if (childNode->GetNum() != source) nodeHeap->Add(childNode);
//  }
//  if (verbose) printf("\n");
//}
//
//void praStar::addAbstractedNodesAndNeighborsToHeap(Heap *nodeHeap, Graph *g, node *p,
//																									 unsigned int source, int LABEL)
//{
//  for (int cnt = 0; cnt < p->GetLabelL(kNumAbstractedNodes); cnt++) {
//    unsigned int childNum;
//    node *childNode = g->GetNode(childNum = (unsigned int)p->GetLabelL(kFirstData+cnt));
//    childNode->setKeyLabel(LABEL);
//    childNode->SetLabelF(LABEL, MAXINT);
//    childNode->markEdge(0);
//    if ((childNode->GetNum() != source) && (!nodeHeap->IsIn(childNode))) {
//      if (verbose) printf("%d ", childNode->GetNum());
//      nodeHeap->Add(childNode);
//    }
//		
//    edge_iterator ei = childNode->getEdgeIter(); 
//    for (edge *e = childNode->edgeIterNext(ei); e; e = childNode->edgeIterNext(ei)) {
//      node *neighbor;
//      unsigned int which;
//      if ((which = e->getFrom()) == childNum) which = e->getTo();
//      neighbor = g->GetNode(which);
//      if (!nodeHeap->IsIn(neighbor)) {
//				neighbor->setKeyLabel(LABEL);
//				neighbor->SetLabelF(LABEL, MAXINT);
//				neighbor->markEdge(0);
//				if (neighbor->GetNum() != source) {
//					if (verbose) printf("%d ", neighbor->GetNum());
//					nodeHeap->Add(neighbor);
//				}
//      }
//    }
//  }
//  if (verbose) printf("\n");
//}
	


//void praStar::doLibraryRandomPath(bool repeat, bool optimal)
//{
//	static double lastLength, lastTime;
//	static node *r1, *r2;
//	if (verbose)
//		cout << "Clearing marked nodes" << endl;
//	ClearMarkedNodes();
//	
//	if (!repeat)
//	{
//		do {
//			do {
//				r1 = abstractions[0]->GetRandomNode();
//			} while (m->GetTerrainType((long)r1->GetLabelL(kFirstData), (long)r1->GetLabelL(kFirstData+1)) == kOutOfBounds);
//			do {
//				r2 = abstractions[0]->GetRandomNode();
//			} while (m->GetTerrainType((long)r2->GetLabelL(kFirstData), (long)r2->GetLabelL(kFirstData+1)) == kOutOfBounds);
//		} while (!Pathable(r1, r2));
//	}
//		
//	if (verbose)
//	{
//		cout << "Attempting path between nodes:" << endl;
//		cout << (*r1) << endl << (*r2) << endl;
//	}
//	
//	//	while (r1 != r2)
//	//	{
//	//		r1 = getPathStep(r1, r2);
//	//		if (verbose)
//	//			cout << "Stepping to " << (*r1) << endl;
//	//	}
//	
//	// ignoring return value! Leaking memory!
//	
//#ifdef OS_MAC
//	AbsoluteTime startTime = UpTime();
//#else
//	clock_t startTime, endTime;
//	long double duration;
//	startTime = clock();
//	
//#endif
//	
//	
//	path *p;
//	
//	if (optimal)
//		p = getLibraryPath(r1, r2);
//	else
//		p = getApproximatePath(r1, r2);
//	//		getPathStep(r1, r2);
//	
//	
//#ifdef OS_MAC
//	
//	AbsoluteTime stopTime = UpTime();
//	Nanoseconds diff = AbsoluteDeltaToNanoseconds(stopTime, startTime);
//	uint64_t nanosecs = UnsignedWideToUInt64(diff);
//	cout << nanosecs << " ns elapsed (" << (double)nanosecs/1000000.0 << " ms)" << endl;
//#else
//	endTime = clock();
//	duration=(long double)(endTime-startTime)/CLOCKS_PER_SEC;
//	cout << duration << " seconds elapsed" << endl;
//	
//#endif
//	
//	int cnt = 0;
//	double length = 0;
//	for (path *q = p; q; q = q->next)
//	{
//		if (q && q->next)
//		{
//			double t1, t2;
//			t1 = q->n->GetLabelL(kFirstData)-q->next->n->GetLabelL(kFirstData);
//			t2 = q->n->GetLabelL(kFirstData+1)-q->next->n->GetLabelL(kFirstData+1);
//			length += sqrt(t1*t1+t2*t2);
//		}
//		cnt++;
//	}
//	
//#ifdef OS_MAC
//	cout << "Path length " << cnt << " steps, " << length << ", " << (double)nanosecs/(1000*cnt) << " µs per step, ";
//	cout << (double)nanosecs/(1000*length) << " µs per distance" << endl;
//	
//	
//	if (!repeat)
//	{
//		lastLength = length;
//		lastTime = (double)nanosecs/1000000.0;
//	}
//	else {
//		cout << "Comparison: " << lastLength/length << "x longer; but " << ((double)nanosecs/1000000.0)/lastTime << "x faster." << endl;
//	}
//#else
//	cout << "Path length " << cnt << " steps, " << length << endl;
//	
//	if (!repeat)
//	{
//		lastLength = length;
//	}
//	else {
//		cout << "Comparison: " << lastLength/length << "x longer" << endl;
//	}
//#endif	
//	
//	clearDisplayLists();
//}
//
//
//using namespace PathFind; 	
//
//typedef AStar<MarkerFastClear, AStarOpenBuckets<MarkerFastClear, AStarCompareF>, AStarClosedLookup<MarkerFastClear> > ASTAR;
//
//path* praStar::getLibraryPath(node* from, node* to)
//{
//	printf("Getting the library path\n");	
//	static ASTAR *search = new ASTAR();
//	
//	//auto_ptr<Search> search(theAStar);
//	//search.reset(theAStar);
//	
//	search->setNodesLimit(100000000L);
//	
//	int start, target;
//	//path *p = 0;
//	MapEnv me(this, m, abstractions[0]);
//	//	MapEnv me(m);
//	
//	//me.getStartTarget(&start, &target);
//	start = from->GetNum();
//	target = to->GetNum();
//	
//	// 	SearchUtils searchUtils;
//	// 	
//	// 	if (searchUtils.checkPathExists(me, start, target))
//	// 		printf("Search Utils -> Path exists\n");
//	// 	else
//	// 		printf("Search Utils -> Path does not exist\n");
//		
//	search->findPath(me, start, target);
//		
//	const vector<int>& resultPath = search->GetPath();
//	
//	while(resultPath.size() < 1) {
//		printf("Bad library path!\n");
//		//return;
//		me.getStartTarget(&start, &target);
//		
//		search->findPath(me, start, target);
//		
//		resultPath = search->GetPath();
//	}
//	
//	//Display the path
//	path *p = 0;
//	edge *e;
//	
//	for (vector<int>::const_iterator i = resultPath.begin(); i != resultPath.end()-1; ++i) {
//		e = abstractions[0]->FindEdge(*(i+1), *i);
//		
//		if (!e) {
//			e = abstractions[0]->FindEdge(*i, *(i+1));
//			
//			if (!e) 
//				printf("Error, invalid edge: From (*i) = %d, To: (*(i+1)) = %d\n", *i, *(i+1));
//			else {
//				e->setMarked(true);
//			}
//		}
//		else {
//			e->setMarked(true);
//		}
//		p = new path(abstractions[0]->GetNode(*i), p);
//	}
//	return p;
//}
//
