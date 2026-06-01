/*
 *  $Id: craStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/23/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "CRAStar.h"
#include "FPUtil.h"
#include "AStar3.h"
#include <cfloat>
#include <limits.h>

using namespace GraphAbstractionConstants;
const int MAX_INT = 2147483647;
static const int verbose = 0;

craStar::craStar()
:SearchAlgorithm()
{
	partialLimit = -1;
	expandSearchRadius = true;
	sprintf(algName,"CRA*(%d)", partialLimit);
	
	absLevel = -1;
	smoothing = true;
	smType = BEGIN;
}

path *craStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	//	std::cout<<"find path from "<<*from<<"\nto "<<*to<<std::endl;
	
	std::vector<node *> fromChain;
	std::vector<node *> toChain;
	path *lastPath = 0;
	
	
	if (aMap->GetAbstractGraph(from->GetLabelL(kAbstractionLevel))->FindEdge(from->GetNum(), to->GetNum()))
		return new path(from, new path(to));
	
	setupSearch(aMap, fromChain, from, toChain, to);
	if (fromChain.size() == 0)
		return 0;
	
	
	
	//   do {
	// 		//	lastPath = buildNextAbstractPath(aMap, lastPath, fromChain, toChain, rp);
	// 		//	lastPath = buildNextAbstractPathQuick(aMap,lastPath,fromChain,toChain,rp);
	//   } while (lastPath->n->GetLabelL(kAbstractionLevel) > 0);
	
	path* abs = buildAbstractPath(aMap, fromChain, toChain, rp);
	
	if (partialLimit > 0)
	{
		
		path *trav = abs;
		path *thisPart = new path(trav->n);
		for (int x = 0; x < partialLimit-1; x++)
		{
			
			if (trav->next) {
				trav = trav->next;
				thisPart->tail()->next = new path(trav->n);
			}
			else
				break;
		}
		
		toChain.clear();
		findGoalNode(aMap,trav->n,toChain);
		
		lastPath = doRefinement(aMap, thisPart, fromChain,toChain);
		//delete thisPart;
	}
	else
		lastPath = doRefinement(aMap, abs, fromChain, toChain);
	
	//	if (verbose){
	// 	std::cout<<"before smoothing :";
	// 	lastPath->Print();
	// 	std::cout<<std::endl;
	//}
	

if (smoothing)
{
	
	path* p = lastPath;
	lastPath = smoothPath(aMap,lastPath);
	delete p;
}
return lastPath;
//return trimPath(lastPath, to);
}

/** 
* GIven an abstract level parent node n, find a new goal that is a 0-level child of n
* as well as the parent chain linking them. The parent chain will be stores in toChain.
*/
void craStar::findGoalNode(GraphAbstraction* aMap, node* n, std::vector<node *> &toChain)
{
	int currLevel = n->GetLabelL(kAbstractionLevel);
	if (currLevel==0)
	{
		
		toChain.push_back(n);
	}
	else{
		findGoalNode(aMap,aMap->GetAbstractGraph(currLevel-1)->GetNode(n->GetLabelL(kFirstData)), toChain);
		toChain.push_back(n);
	}
}

void craStar::setupSearch(GraphAbstraction *aMap,
													std::vector<node *> &fromChain, node *from,
													std::vector<node *> &toChain, node *to)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
	{
		if (verbose) {
			if (!from)
				printf("craStar: from == 0\n");
			if (!to)
				printf("craStar: to == 0\n");
			if (from == to)
				printf("craStar: from == to\n");
			if (from && to && (!aMap->Pathable(from, to)))
				printf("craStar: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
		}
		return;
	}
	
	if (verbose)
		printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
	
	aMap->GetNumAbstractGraphs(from, to, fromChain, toChain);
	
	unsigned int previousSize = fromChain.size();
	int minNode = (int)(2*sqrt(aMap->GetAbstractGraph(0)->GetNumNodes()));
	
	if (absLevel > 0)
	{
		
		while((int)fromChain.size()-1 > absLevel)
		{
			
			toChain.pop_back();
			fromChain.pop_back();
		}
	}
	else{ // dynamic level selection
		while ((fromChain.size() > 2) && ((fromChain.size() > (previousSize)/2) ||
																			(aMap->GetAbstractGraph(fromChain.size())->GetNumNodes() < minNode)))
		{
			toChain.pop_back();
			fromChain.pop_back();
		}
	}
}

/**
* Build an abstract path for quick refinement
 */
path* craStar::buildAbstractPath(GraphAbstraction *aMap, 
																 std::vector<node *> &fromChain,
																 std::vector<node *> &toChain,
																 reservationProvider *rp)
{
	path *result;
	aStarOld* astar = new aStarOld();
	
	node *to = 0;
	node *from = 0; 
	
	to = toChain.back();
	from = fromChain.back();
	
	result = astar->GetPath(aMap, from, to, rp);
	
	nodesExpanded += astar->GetNodesExpanded();
	nodesTouched += astar->GetNodesTouched();
	
	delete astar;
	
	return result;
}



/**
* Do a quick refinement of an abstract path
 */

path *craStar::doRefinement(GraphAbstraction *aMap, path* absPath, std::vector<node*> &fromChain, std::vector<node*> &toChain)
{
	assert((fromChain.size() == toChain.size()) && (toChain.back()->GetLabelL(kAbstractionLevel) == fromChain.back()->GetLabelL(kAbstractionLevel))
				 && (fromChain.back() == absPath->n));
	
	path* lastPath = absPath;
	
	while(fromChain.size() > 1)
	{
		
		// Reset from & to
		fromChain.pop_back();
		node* from = fromChain.back();
		
		if (verbose) std::cout<<"Finding path at level "<<from->GetLabelL(kAbstractionLevel)<<std::endl;
		
		toChain.pop_back();
		node* to = toChain.back();
		
		if (verbose) std::cout<<"From: "<<*from<<"\nto: "<<*to<<std::endl;
		
		path* apath = lastPath;
		
		int abstractLevel = apath->n->GetLabelL(kAbstractionLevel);
		
		if (verbose)
		{
			
			std::cout<<"abstract path\n";
			while(apath->next)
			{
				
				std::cout<<*(apath->n)<<std::endl;
				apath = apath->next;
			}
			std::cout<<*(apath->n)<<std::endl;
			apath = lastPath;
		}
		node* currentLow = from;
		
		assert(aMap->GetNthParent(currentLow,abstractLevel) == apath->n);
		
		apath = apath->next;
		path* returnPath = new path(currentLow);
		
		Graph* g = aMap->GetAbstractGraph(currentLow->GetLabelL(kAbstractionLevel));
		
		// check if the first node is an orphan
		if (currentLow->GetNumEdges()==1)
		{
			
			neighbor_iterator niter = currentLow->getNeighborIter();
			int neighborNode = currentLow->nodeNeighborNext(niter);
			nodesTouched++;
			node* neigh = g->GetNode(neighborNode);
			returnPath->tail()->next = new path(neigh);
			
			currentLow = neigh;
		}
		
		while(apath->next)
		{
			
			currentLow = getNextNode(aMap, currentLow, returnPath, apath, g, abstractLevel);
			apath = apath->next;
		}
		
		// do the last one
		currentLow = getNextNode(aMap, currentLow, returnPath, apath, g, abstractLevel);
		
		// Fix up the end; make sure we get to "to" 
		
		node* lastnode = currentLow;
		
		if (returnPath->tail()->n != to)
		{
			
			if (!g->FindEdge(currentLow->GetNum(),to->GetNum()))
			{
				
				// check if goal is an orphan
				if (to->GetNumEdges() == 1)
				{
					
					neighbor_iterator niter = to->getNeighborIter();
					int neighborNode = to->nodeNeighborNext(niter);
					nodesTouched++;
					node* neigh = g->GetNode(neighborNode);
					returnPath->tail()->next = new path(neigh);
					g->FindEdge(currentLow->GetNum(), neigh->GetNum())->setMarked(true);
					lastnode = neigh;
				}
			}
			
			g->FindEdge(lastnode->GetNum(), to->GetNum())->setMarked(true);
			returnPath->tail()->next = new path(to);
		}
		delete lastPath;
		lastPath = returnPath;
		
	}
	return lastPath;
}

/**
* Find the next node of the refined path. 
 */
node* craStar::getNextNode(GraphAbstraction *aMap, node* currentLow, path* returnPath, path* apath, Graph* g, int abstractLevel)
{		
	nodesExpanded++;
	
	std::vector<node*> neighbors(currentLow->GetNumEdges());
	int numInArray = 0; 
	double minHeur = DBL_MAX;
	int minIndex = -1;
	
	// Try to find a neighbor which has the next node in the abstract path as a parent
	neighbor_iterator niter = currentLow->getNeighborIter();
	int neighborNode = currentLow->nodeNeighborNext(niter);
	
	while(neighborNode != -1)
	{
		nodesTouched++;
		node* neigh = g->GetNode(neighborNode);
		if (aMap->GetNthParent(neigh,abstractLevel) == apath->n)
		{
			
			returnPath->tail()->next = new path(neigh);
			currentLow = neigh;
			break;
		}
		else if (aMap->GetNthParent(neigh,abstractLevel) == aMap->GetNthParent(currentLow,abstractLevel))
		{
			
			// same parent as current node --> add to array
			neighbors[numInArray] = neigh;
			double hdist = aMap->h(neigh,apath->n);
			if (minHeur > hdist)
			{
				
				minHeur = hdist;
				minIndex = numInArray;
			}
			
			numInArray++;
			
		}
		neighborNode = currentLow->nodeNeighborNext(niter);
	}
	
	if (neighborNode == -1)
	{
		// no direct neighbor with next abs node as parent
		// try neighbors of the neighbor node with min heuristic distance to next abstract node
		node* neigh = neighbors[minIndex];
		neighbor_iterator niter2 = neigh->getNeighborIter();
		int n2num = neigh->nodeNeighborNext(niter2);
		nodesExpanded++;
		while(n2num != -1)
		{
			
			nodesTouched++;
			node* n2 = g->GetNode(n2num);
			
			if (aMap->GetNthParent(n2,abstractLevel) == apath->n)
			{
				
				g->FindEdge(currentLow->GetNum(),neigh->GetNum())->setMarked(true);
				g->FindEdge(neigh->GetNum(), n2->GetNum())->setMarked(true);
				
				returnPath->tail()->next = new path(neigh, new path(n2));
				currentLow = n2;
				
				break;
			}
			n2num = neigh->nodeNeighborNext(niter2);
		}
		
		if (n2num == -1)
		{
			
			// not min heuristic node - try the rest of the list
			neighbors[minIndex] = neighbors[numInArray-1];
			
			for (int i=0; i<numInArray-1; i++)
			{
				
				nodesExpanded++;
				/*node*/ neigh = neighbors[i];
				neighbor_iterator niter3 = neigh->getNeighborIter();
				int nnum = neigh->nodeNeighborNext(niter3);
				
				// double check this code
				while(nnum != -1)
				{
					
					nodesTouched++;
					node* n2 = g->GetNode(nnum);
					if (aMap->GetNthParent(n2,abstractLevel) == apath->n)
					{
						
						g->FindEdge(currentLow->GetNum(),neigh->GetNum())->setMarked(true);
						g->FindEdge(neigh->GetNum(), n2->GetNum())->setMarked(true);						
						
						returnPath->tail()->next = new path(neigh, new path(n2));
						currentLow = n2;
						break;
					}
					nnum = neigh->nodeNeighborNext(niter3);
				}
			}			
		}
	}
	return currentLow;
}


path *craStar::buildNextAbstractPath(GraphAbstraction *aMap, path *lastPath,
																		 std::vector<node *> &fromChain,
																		 std::vector<node *> &toChain,
																		 reservationProvider *rp)
{
	node *to, *from, *hTarget = 0;
	to = toChain.back();
	toChain.pop_back();
	from = fromChain.back();
	fromChain.pop_back();
	
	if (verbose)
		printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->GetLabelL(kAbstractionLevel));		
	
	if (verbose)
		printf("Building path from %d to %d (%ld/%ld)\n",
					 from->GetNum(), to->GetNum(), from->GetLabelL(kParent), to->GetLabelL(kParent));
	
	std::vector<node *> eligibleNodeParents;
	
	if (lastPath)
	{
		// cut path down to size of partial path limit
		if (partialLimit > 0)
		{
			path *trav = lastPath;
			
			for (int x = 0; x < partialLimit; x++)
				if (trav->next) 
					trav = trav->next;
			// we don't need to reset the target if we have a complete path
			// but we do if our complete path doesn't end in our target node
			if ((trav->next != 0) || ((trav->next == 0) && ((int)trav->n->GetNum() != to->GetLabelL(kParent))))
			{
				to = trav->n;
				if (trav->next)
					hTarget = trav->next->n;
				else
					hTarget = to;
				delete trav->next;
				trav->next = 0;
				if (verbose) printf("Setting target parent to %d\n", to->GetNum());
			}
		}
		
		Graph *g = aMap->GetAbstractGraph(lastPath->n->GetLabelL(kAbstractionLevel));
		// find eligible nodes for lower level expansions
		for (path *trav = lastPath; trav; trav = trav->next)
		{
			if (expandSearchRadius)
			{
				edge_iterator ei = trav->n->getEdgeIter();
				for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei)) {
					if (e->getFrom() == trav->n->GetNum())
						eligibleNodeParents.push_back(g->GetNode(e->getTo()));
					else
						eligibleNodeParents.push_back(g->GetNode(e->getFrom()));
				}
			}
			eligibleNodeParents.push_back(trav->n);
		}
	}
	
	cAStar.setCorridor(&eligibleNodeParents);
	delete lastPath;
	path *result;
	if (hTarget != 0)
	{
		result = cAStar.getBestPath(aMap, from, to, hTarget, rp);
	}
	else {
		result = cAStar.GetPath(aMap, from, to, rp);
	}
	nodesExpanded += cAStar.GetNodesExpanded();
	nodesTouched += cAStar.GetNodesTouched();
	return result;
}

path *craStar::trimPath(path *lastPath, node *origDest)
{
	if (partialLimit != -1)
	{
		int parent = -1;
		path *change = 0, *last = 0;
		for (path *trav = lastPath; trav; trav = trav->next)
		{
			if (trav->n == origDest)
				return lastPath;
			if (trav->n->GetLabelL(kParent) != parent)
			{
				parent = trav->n->GetLabelL(kParent);
				change = last;
			}
			last = trav;
		}
		if (change)
		{
			delete change->next;
			change->next = 0;
		}
	}
	return lastPath;
}


/**
* copied from hpaStar.cpp
 *
 * smoothen the path by replacing parts of the path by 
 * straight lines.
 */
path* craStar::smoothPath(GraphAbstraction *m,path* p)
{
	findMinMax(p);
	
	if (verbose) std::cout<<"Smoothing the path\n";
	
	// put the path nodes in a vector
	lookup.clear();
//	path* pcopy = p->Clone();
//	path* ptr = pcopy;
	int tempLabel=0; 
	
	for (path *tmp = p; tmp != 0; tmp = tmp->next)
	{
		lookup.push_back(tmp->n);
		// set key = index in lookup
		tmp->n->SetLabelL(kTemporaryLabel,tempLabel);
		tempLabel++;
	}
	unsigned int n = 0; 
	path* smooth = 0;
	
	while(true)
	{
		
		//Skip blanks
		while(lookup[n]==0 && n<lookup.size()-1)
			n++;
		
		if (n>=lookup.size()-1)
		{
			
			break;
		}
		
		unsigned int last = lookup[n]->GetLabelL(kTemporaryLabel);
		
		if (last!=n)
		{
			
			for (unsigned int i=n; i<last && i<lookup.size(); i++)
			{
				
				lookup[i]=0;
			}
			n = last;
			continue;
		}
		
		int dir;
		for (dir = NORTH; dir <= NW; dir++)
		{
			
			// get a shortcut if it exists
			path* pathToNode = nextPathNode(m,lookup[n],dir);
			
			// paste the shortcut into our current path
			if (pathToNode)
			{
				int lastNode = pathToNode->tail()->n->GetLabelL(kTemporaryLabel);
				int curr = pathToNode->n->GetLabelL(kTemporaryLabel);
				
				if (lastNode > curr && !nextInLookup(lastNode, curr,lookup))
				{
					// make sure it's not the next one 
					
					unsigned int index = n;
					path* pathCopy = pathToNode;
						//path* backup = pathCopy;
					unsigned int end = pathToNode->tail()->n->GetLabelL(kTemporaryLabel);
						
					while(pathCopy->next)
					{
						
						//make sure we're not overwriting anything
						assert(index <= end);
						
						lookup[index]=pathCopy->n;
						pathCopy->n->SetLabelL(kTemporaryLabel,index);
						pathCopy = pathCopy->next;
						index++;
					}
					assert(index <= end);
					
					lookup[index]=pathCopy->n;
					pathCopy->n->SetLabelL(kTemporaryLabel,index);			
					
					index++;	
					
					while(index<=end)
					{
						
						lookup[index]= 0;
						index++;
					}
					
					if (smType==END)
					{
						
						n = end;
					}
					else if (smType==TWO_BACK)
						n = backTwoNodes(end,lookup);
					else		
						n++; 
					
					delete pathToNode; pathToNode = 0;
					//delete backup;
					break;
					
				}
				else if (dir==NW)
				{
					
					n++;
				}
				
				delete pathToNode;
			} 
			else if (dir==NW)
			{
				
				n++;
			}
		} //end for every direction
		
		
	}
	
	//Create smoothed path from lookup table
	for (unsigned int i=0; i<lookup.size(); i++)
	{
		
		if (lookup[i]!=0)
		{
			
			if (!smooth)
				smooth = new path(lookup[i],0);
			else
				smooth->tail()->next = new path(lookup[i],0);
		}
	}
	return smooth;
}

/**
* Find the index of the node two nodes back in the path
 */
int craStar::backTwoNodes(int i, std::vector<node*> lookupVal)
{
	i--;
	while(lookupVal[i]==NULL)
	{
		
		i--;
	}
	i--;
	while(lookupVal[i]==NULL)
	{
		
		i--;
	}
	return i;
}


/**
* find out whether last is the next 'real' index in the lookup table after
 * curr. This is to make sure we don't keep replacing little paths due to null's in
 * the lookup table. 
 */
bool craStar::nextInLookup(int last, int curr, std::vector<node*> lookupVal)
{
	if (last<curr) 
		return false;
	
	for (int i=curr+1; i<=last; i++)	//
	{
		if (i==last)
			return true;
		if (lookupVal[i]!=NULL)
			return false;
	}
	return true;	
}

/**
* shoot a ray in direction dir and see if you hit the path
 * Return the better path if you find it; 0 if you hit a  
 * wall or obstacle (ie. if you won't find the path) 
 */ 
path* craStar::nextPathNode(GraphAbstraction *m,node* n, int dir)
{
	Graph *g = m->GetAbstractGraph(0);
	
	int px = n->GetLabelL(kFirstData);
	int py = n->GetLabelL(kFirstData+1);
	
	node* next = getNextNode(static_cast<MapAbstraction*>(m),px,py,dir);
	
	if (next)
	{
		
		nodesTouched++;
		int nextKey = next->GetLabelL(kTemporaryLabel);
		edge* e = g->FindEdge(n->GetNum(), next->GetNum());
		
		if (e && (nextKey >= 0) && (nextKey < static_cast<int>(lookup.size())) && (lookup[nextKey]==next))
		{
			
			//we're done - we found the path
			return new path(n,new path(next));
		}
		else{	// look further for a path node		
			if (e)
			{
				
				path * p = nextPathNode(m,next,dir);				 
				if (p)
				{
					
					return new path(n,p);
				}
				else // haven't found a path node
					return 0;
			}
			else // no edge here
				return 0; 			
		}
	}
	else{ // we won't hit the path 
		return 0;
	}
}

/**
* get the next node from map coordinates (x,y) in direction
 * dir. Will return 0 if there's no node here. 
 */
node* craStar::getNextNode(MapAbstraction *m,int x, int y, int dir)
{
	if ((x < minx) || (x > maxx) || (y < miny) || (y>maxy))
		return 0;
	
	switch(dir)
	{
		case NORTH:
			return m->GetNodeFromMap(x,y+1);
			break;
		case EAST:
			return m->GetNodeFromMap(x+1,y);
			break;
		case SOUTH:
			return m->GetNodeFromMap(x, y-1);
			break;
		case WEST:
			return m->GetNodeFromMap(x-1, y);
			break;
		case NE:
			return m->GetNodeFromMap(x+1, y+1);
			break;
		case SE:
			return m->GetNodeFromMap(x+1, y-1);
			break;
		case SW:
			return m->GetNodeFromMap(x-1, y-1);
			break;
		case NW:
			return m->GetNodeFromMap(x-1, y+1);
			break;
		default:
			return 0;
	}
}

/**
* Find the box that bounds the path for more efficient path smoothing. 
 */ 
void craStar::findMinMax(path* p)
{
	minx = MAX_INT;
	miny = MAX_INT;
	maxx = -1;
	maxy = -1;
	
	path* pcopy = p;
	
	while(pcopy->next)
	{
		int x = pcopy->n->GetLabelL(kFirstData);
		int y = pcopy->n->GetLabelL(kFirstData+1);
		
		if (x < minx) minx = x;
		if (x > maxx) maxx = x;
		
		if (y < miny) miny = y;
		if (y > maxy) maxy = y;
		
		pcopy = pcopy->next;
	}
	
	int x = pcopy->n->GetLabelL(kFirstData);
	int y = pcopy->n->GetLabelL(kFirstData+1);
	
	if (x < minx) minx = x;
	if (x > maxx) maxx = x;
	
	if (y < miny) miny = y;
	if (y > maxy) maxy = y;
	
	assert((minx != MAX_INT)&&(miny != MAX_INT)&&(maxx != -1)&&(maxy != -1));
	
}
