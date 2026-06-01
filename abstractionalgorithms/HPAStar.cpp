/*
 * hpaStar.cpp
 * 
 * Created by Renee Jansen on 06/16/06
 *
 */

#include "HPAStar.h"
#include "AStar3.h"

#include "AStar.h"
#include "Timer.h"

using namespace GraphAbstractionConstants;

const static int verbose = 0;
const int MAX_INT = 2147483647;

hpaStar::hpaStar()
{
	partialLimit = -1;
	fromnum = 0; 
	tonum = 0;
	smoothing = true;
	smType = END;

	sprintf(algName,"HPA*(%d)",partialLimit);
}

/**
 * Set whether we want to do path smoothing. Default is true. 
 */
void hpaStar::setSmoothing(bool smooth){
	smoothing = smooth;
}

/**
 * Returns the HPA* path between from and to. 
 * 
 * Before calling this function, set the abstraction with hpaStar::setAbstraction. The abstraction
 * must be of type ClusterAbstraction.
 * 
 * Reservation provider isn't used 
 */
path* hpaStar::GetPath(GraphAbstraction *aMap, node* from, node* to, reservationProvider *)
{
	// Make sure a cluster abstraction has been set & it's the same as the abstraction being passed
	// in to this function
	assert(m==aMap);

	if ((from==to) || (from ==0) || (to==0)){
		if (verbose) std::cout<<"Returning an empty path\n";
		return 0;
	}

	fromnum = 0; 
	tonum = 0;

	Timer t;
	//if the nodes are in the same cluster, just do A*
	//this takes care of the case where both nodes are in some enclosed
	//area inside a cluster

	if (m->getClusterIdFromNode(from)==m->getClusterIdFromNode(to))
	{
		aStarOld astar;
		path* p = astar.GetPath(m,from,to);
		nodesExpanded = astar.GetNodesExpanded();
		nodesTouched = astar.GetNodesTouched();
		if (verbose)std::cout<<"Nodes are inside the same cluster\n";
		return p;
	}

	setUpSearch(from, to);

	// Check if there's a path at all
	if (!m->Pathable(from,to)){
		if (verbose) std::cout<<"Not Pathable\n";
		cleanUpSearch();
		return 0;
 	}

 	else
		if (verbose) std::cout<<"Pathable\n";

	//	t.StartTimer();
	path* abPath = findAbstractPath(fromnum,tonum);
	//std::cout<<"abstract path "<<t.EndTimer()<<std::endl;

	if (!abPath){
		cleanUpSearch();
		return 0;
	}
	path* mapPath = 0; 
	if (partialLimit > 0){
		path *trav = abPath;
		path *thisPart = new path(trav->n);
		for (int x = 0; x < partialLimit-1; x++){
			if (trav->next) {
				trav = trav->next;
				thisPart->tail()->next = new path(trav->n);
			}
		}
		

		node* upper = trav->tail()->n;
		point3d s(upper->GetLabelF(kXCoordinate),upper->GetLabelF(kYCoordinate),-1);
		int px;
		int py;
		m->GetMap()->GetPointFromCoordinate(s,px,py);

		node* newTo = m->GetNodeFromMap(px,py);
		mapPath = findMapPath(thisPart, from, newTo);
		delete thisPart;
	}
	else
	{
		//	t.StartTimer();
		mapPath = findMapPath(abPath,from,to);
		//std::cout<<"Found map path "<<t.EndTimer()<<std::endl;
	}
	
	path* finalPath = 0; 

	if (smoothing)
	{
		//t.StartTimer();
		finalPath = smoothPath(mapPath);
		//std::cout<<"Smoothing: "<<t.EndTimer()<<std::endl<<std::endl;
		delete mapPath;
	}
	else {
		finalPath = mapPath;
		mapPath = 0;
	}

	cleanUpSearch();
	
	delete abPath;

	return finalPath; 
}

/*
 * setUpSearch sets the number of nodes expanded and touched to 0
 * and inserts the start and goal nodes into the abstract Graph
 */ 
void hpaStar::setUpSearch(node* from, node* to)
{
	if (verbose) std::cout<<"HPA*: setting up search\n";
	// set nodes expanded & nodes touched to 0
	nodesExpanded = 0; 
	nodesTouched = 0;

	int touched =0;
	int expanded = 0; 
	// insert the nodes 
	//	Timer t; 
	//	t.StartTimer();
	fromnum = m->insertNode(from,expanded,touched);
	nodesExpanded += expanded;
	nodesTouched += touched;
	tonum = m->insertNode(to,expanded,touched);
	nodesExpanded += expanded;
	nodesTouched += touched;
	//	std::cout<<"Time taken to insert: "<<t.EndTimer();
}

/*
 * finds a path on the abstract level of a map
 */
path* hpaStar::findAbstractPath(node* from, node* to)
{
	if (verbose)	std::cout<<"HPA*: finding the abstract path\n";
	aStarOld astar;
	assert(from == fromnum);
	assert(to == tonum);
	path* p = astar.GetPath(m, fromnum, tonum); 
	nodesExpanded = astar.GetNodesExpanded();
	nodesTouched = astar.GetNodesTouched();

	if (verbose){
		std::cout<<"Abstract path: ";
		if (p)
			p->Print();
		else
			std::cout<<"Null";
		std::cout<<std::endl;
	}

	return (p);
}

/* 
 * finds the map-level path corresponding to the abstract path
 * It uses cached paths for each edge in the abstract path. 
 */ 
path* hpaStar::findMapPath(path* abPath, node* /*from*/,node* /*to*/)
{
	Graph *g = m->GetAbstractGraph(1);
	path* curr = abPath;

	path* returnme = 0;

	node* n = curr->n;
	curr = curr->next;
	node* n2 = curr->n;

	edge* e = g->FindEdge(n->GetNum(), n2->GetNum());

	path* lowlevel = m->getCachedPath(e);

	if (lowlevel)
	{ // edge was cached
		if ((lowlevel->n == m->getLowLevelNode(n2)) &&
			 (lowlevel->tail()->n == m->getLowLevelNode(n)))
		{
			// The cached path is backwards - have to reverse it
			lowlevel = lowlevel->reverse();
		}

		returnme = lowlevel;
	}
	else {
		returnme = new path(m->getLowLevelNode(n), new path(m->getLowLevelNode(n2)));
	}

	while(curr->next){
		n = n2;
		curr = curr->next;
		n2 = curr->n;

		/*		
		point3d s(n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),-1);
		int px;
		int py;
		m->GetMap()->GetPointFromCoordinate(s,px,py);

		point3d t(n2->GetLabelF(kXCoordinate),n2->GetLabelF(kYCoordinate),-1);
		
		m->GetMap()->GetPointFromCoordinate(t,px,py);
		*/

		e = g->FindEdge(n->GetNum(), n2->GetNum());

		lowlevel = m->getCachedPath(e);
		
		
		if (lowlevel)
		{

			if ((lowlevel->n == m->getLowLevelNode(n2)) &&
					(lowlevel->tail()->n == m->getLowLevelNode(n)))
			{
				lowlevel = lowlevel->reverse();
			}
			
			path *tailend = returnme->tail();
			if (tailend->n == lowlevel->n)
			{
				tailend->next = lowlevel->next;
				lowlevel->next = 0;
				delete lowlevel;
			}
			else{
				returnme->tail()->next = lowlevel;
			}			
		}
		else
		{
			returnme->tail()->next = new path(m->getLowLevelNode(n2));
		}
	}
	return returnme;
}

/**
 * Remove the start & goal nodes from the Graph
 */
void hpaStar::cleanUpSearch()
{
	// remove nodes from map
	if ((fromnum!=0) && (tonum != 0)){
		//		Timer t;
		//t.StartTimer();
		m->removeNodes(fromnum,tonum);
		//std::cout<<"It took "<<t.EndTimer()<<" to remove nodes\n";
	}
	fromnum = 0;
	tonum = 0;
}

/**
 * from HPA* smoothwizard.cpp
 *
 * smoothen the path by replacing parts of the path by 
 * straight lines.
 */
path* hpaStar::smoothPath(path* p)
{
	if (verbose) std::cout<<"Smoothing the path\n";

	//	findMinMax(p);
	int clusterSize = m->getClusterSize();

	// put the path nodes in a vector
	lookup.clear();
	int i=0; 
	//assert(pcopy);
	for (path *tmp = p; tmp != 0; tmp = tmp->next)
	{
		lookup.push_back(tmp->n);		
		// set key=index in lookup
		tmp->n->SetLabelL(kTemporaryLabel,i);
		i++;
	}

	unsigned int n = 0; 
	path* smooth = 0;

	while(true){
		//Skip blanks
		while(lookup[n]==0 && n<lookup.size()-1)
			n++;

		if (n>=lookup.size()-1){ 
			break;
		}

		unsigned int last = lookup[n]->GetLabelL(kTemporaryLabel);

		// Node's temporary label != last --> this node occurs more than
		// once. Cut out the in between path & go back to beginning of while loop
		if (last!=n)
		{
			std::cout<<"in here\n";
 			for (unsigned int j=n; j<last && j<lookup.size(); j++)
			{
 				lookup[j]=0;
			}
			n = last;
			continue;
		}

		int currX = lookup[n]->GetLabelL(kFirstData);
		int currY = lookup[n]->GetLabelL(kFirstData+1); 
		minx = currX - clusterSize;
		maxx = currX + clusterSize;																		 
		miny = currY - clusterSize;
		maxy = currY + clusterSize;

		int dir;
		for (dir = NORTH; dir <= NW; dir++)
		{
			// get a shortcut if it exists
			path* pathToNode = nextPathNode(lookup[n],dir);

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

					while(pathCopy->next){
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

					while(index<=end){
						lookup[index]= 0;
						index++;
					}

					if (smType==END){
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
				else if (dir==NW){
					n++;
				}
				delete pathToNode;
			} 
			else if (dir==NW){
				n++;
			}
		} //end for every direction


	}

	//Create smoothed path from lookup table
	for (unsigned int j=0; j<lookup.size(); j++){
		if (lookup[j]!=0){
			if (!smooth)
				smooth = new path(lookup[j],0);
			else
				smooth->tail()->next = new path(lookup[j],0);
		}
	}

	return smooth;
}

/**
 * Find the index of the node two nodes back in the path
 */
int hpaStar::backTwoNodes(int i, std::vector<node*> lookupVec)
{
	i--;
	while(lookupVec[i]==NULL){
		i--;
	}
	i--;
	while(lookupVec[i]==NULL){
		i--;
	}
	return i;
}


/**
 * find out whether last is the next 'real' index in the lookup table after
 * curr. This is to make sure we don't keep replacing little paths due to null's in
 * the lookup table. 
 */
bool hpaStar::nextInLookup(int last, int curr, std::vector<node*> lookupVec)
{
	if (last<curr) 
		return false;
	
	for (int i=curr+1; i<=last; i++)	//
	{
		if (i==last)
			return true;
		if (lookupVec[i]!=NULL)
			return false;
	}
	return true;	
}

/**
 * shoot a ray in direction dir and see if you hit the path
 * Return the better path if you find it; 0 if you hit a  
 * wall or obstacle (ie. if you won't find the path) 
 */ 
path* hpaStar::nextPathNode(node* n, int dir)
{
	Graph *g = m->GetAbstractGraph(0);

	int px = n->GetLabelL(kFirstData);
	int py = n->GetLabelL(kFirstData+1);
	
	node* next = getNextNode(px,py,dir);

	if (next){
		nodesTouched++;

		int nextKey = next->GetLabelL(kTemporaryLabel);
		edge* e = g->FindEdge(n->GetNum(), next->GetNum());
		
		if (e && (nextKey >= 0) && (nextKey < static_cast<int>(lookup.size())) && (lookup[nextKey]==next)){
			//we're done - we found the path
			return new path(n,new path(next));
		}
		else{	// look further for a path node		
			if (e){
				path * p = nextPathNode(next,dir);				 
				if (p){
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
node* hpaStar::getNextNode(int x, int y, int dir)
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

void hpaStar::findMinMax(path* p)
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
