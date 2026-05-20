/*
 *  Propagation.cpp
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code emphasizes on correctness.
 */

#include <sys/time.h>
#include <math.h>
#include <deque>
#include "Propagation.h"
#include <cstring>

using namespace PropUtil;

//#define MAXINT 2147483648

#define CLOSEDMODE 0
#define OPENMODE   1
#define NEWMODE    2
//#define WAITMODE   3
#define DELAYMODE  3

const static bool verbose = false;
const static bool drawtext = false;

//static unsigned long tickStart;
//
//static unsigned long tickGen;

void Prop::GetPath(GraphEnvironment *_env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath) {
	if (!InitializeSearch(_env,_g,from,to,thePath))
		return;

	struct timeval t0,t1;

    gettimeofday(&t0,0);
	while(!DoSingleSearchStep(thePath)) 
		{}
	gettimeofday(&t1,0);

//	double usedtime = t1.tv_sec-t0.tv_sec + (t1.tv_usec-t0.tv_usec)/1000000.0;

	//if (thePath.size() > 0)
	//	printf("\nNodes expanded=%ld, Nodes touched=%ld, Reopenings=%ld.\n",GetNodesExpanded(),GetNodesTouched(),NodesReopened);

	//char algname[20];
	

	//printf("Algorithm %s, time used=%lf sec, N/sec=%lf, solution cost=%lf, solution edges=%d.\n", algname,usedtime,GetNodesExpanded()/usedtime,solutionCost,(int)thePath.size());
}

bool Prop::InitializeSearch(GraphEnvironment *_env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath) {
	env = _env;
	grp = _g;
	nodesExpanded = 0;
	nodesTouched =  0;
	NodesReopened = 0;
	metaexpanded = 0;
	closedSize = 0;
	reopenings = 0;
	start = from;
	goal = to;
	justExpanded = from;

	closedList.clear();
	openQueue.reset();
	FCache.reset();
	delayCache.reset();

	thePath.clear();

	if (verID==PROP_A)
		strcpy(algname,"A*");
	else if (verID==PROP_B)
		strcpy(algname,"B");
	else if (verID==PROP_BP)
		strcpy(algname,"B'");
	else if (verID==PROP_APPROX)
		strcpy(algname,"Approx");
	else if (verID==PROP_BFS)
		strcpy(algname,"BFSRepair");
	else if (verID==PROP_DELAY)
		strcpy(algname,"Delay");
	else if (verID==PROP_DP)
		strcpy(algname,"Dual Prop");
	else if (verID==PROP_BPMX)
		strcpy(algname,"BPMX");
	else if (verID==PROP_DPMX)
		strcpy(algname,"DPMX");
	else if (verID==PROP_BPMXE)
		strcpy(algname,"BPMXE");
	else if (verID==PROP_DPDLMX)
		strcpy(algname,"DPDLMX");
	else {
		printf("I don't know what to do.\n");
		exit(-1);
	}

	//tickNewExp = tickReExp = tickBPMX = 0;
	//nNewExp = nReExp = nBPMX = 0;

	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}

	// step (1)
	SearchNode first(env->HCost(start, goal), 0, start, start);
	openQueue.Add(first);

	F = 0;

	return true;
}

bool Prop::DoSingleSearchStep(std::vector<graphState> &thePath) {
	switch (verID) 
	{
	case PROP_A:
		return DoSingleStepA(thePath);
	case PROP_B:
		return DoSingleStepB(thePath);
	case PROP_BP:
		return DoSingleStepBP(thePath);
	case PROP_APPROX:
		return DoSingleStepApprox(thePath);
	case PROP_BFS:
		return DoSingleStepBFS(thePath);
	case PROP_DELAY:
		return DoSingleStepDelay(thePath);
	case PROP_DP:
		return DoSingleStepDP(thePath);
	case PROP_BPMX:
		return DoSingleStepBPMX(thePath);
	case PROP_DPMX:
		return DoSingleStepDPMX(thePath);
	case PROP_BPMXE:
		return DoSingleStepBPMXE(thePath);
	case PROP_DPDLMX:
		return DoSingleStepDPDLMX(thePath);
	default:
		return true;
	}
	//if (verID == PROP_A)
	//	return DoSingleStepA(thePath);
	//if (verID == PROP_B)
	//	return DoSingleStepB(thePath);
	//else if (verID == PROP_BP) 
	//	return DoSingleStepBP(thePath);
	//else if (verID == PROP_APPROX)
	//	return DoSingleStepApprox(thePath);
	//else if (verID == PROP_BFS)
	//	return DoSingleStepBFS(thePath);
	//else  if (verID == PROP_DELAY)
	//	return DoSingleStepDelay(thePath);
	//else // PROP_DP
	//	return DoSingleStepDP(thePath);
}

void Prop::ComputeNewHMero3a(double &h, double &h_tmp, graphState neighbor, SearchNode& nb, double altH, int mode)
{	// this is necessary because the modified h is stored in the node, not the environment
	if (mode == OPENMODE || mode == CLOSEDMODE)//openQueue.IsIn(SearchNode(neighbor))) 
	{ 
		//SearchNode nb = openQueue.find(SearchNode(neighbor));
		h = max( nb.fCost - nb.gCost, altH);

		h_tmp = nb.fCost - nb.gCost;
	}
	//else if (mode == CLOSEDMODE)//closedList.find(neighbor) != closedList.end()) 
	//{
	//	//SearchNode nb = closedList.find(neighbor)->second;
	//	h = max( nb.fCost - nb.gCost, altH);

	//	h_tmp = nb.fCost - nb.gCost;
	//}
	else 
	{
		double envH = env->HCost(neighbor,goal);
		h = max(envH , altH);

		h_tmp = envH;
	}
}

void Prop::RelaxOpenNode(double f, double g, graphState neighbor, SearchNode &neighborNode, graphState topNodeID)
{

	if (f < neighborNode.fCost) // trick: if f equal, g is always less, causes key increased
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
	}
	else //if (f > neighborNode.fCost)
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		openQueue.IncreaseKey(neighborNode);  // adjust its position in OPEN
	}

}

void Prop::RelaxDelayNode(double f, double g, graphState neighbor, SearchNode &neighborNode, graphState topNodeID)
{

	if (g < neighborNode.gCost) // trick: if g equal, then f must be greater, which causes key increased
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		delayCache.DecreaseKey(neighborNode);  // adjust its position in OPEN
	}
	else //if (f > neighborNode.fCost)
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		delayCache.IncreaseKey(neighborNode);  // adjust its position in OPEN
	}

}

bool Prop::DoSingleStepA(std::vector<graphState> &thePath) {
// return false means the search is not finished, true otherwise

	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;
	SearchNode topNode = openQueue.Remove();
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	//if (topNode.rxp)
	//	nReExp++;
	//else
	//	nNewExp++;

	//tickStart = clock();

	if (verbose) {
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	//tickStart = clock();

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	//Categorize(neighbors);


	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;
		double h = env->HCost(neighbor,goal);
		double f = g + h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE)//!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end()) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) {
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE)//openQueue.IsIn(SearchNode(neighbor))) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose) {
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;

				if (verbose) {
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED

				//neighborNode.rxp = true;

				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	neighbors.clear();

	//if (topNode.rxp)
	//	tickReExp += clock() - tickStart;
	//else
	//	tickNewExp += clock() - tickStart;

	return false;
}

bool Prop::DoSingleStepB(std::vector<graphState> &thePath) {
// return false means the search is not finished, true otherwise

	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;


	SearchNode topNode;
	if (fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if (verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	//if (topNode.rxp)
	//	nReExp++;
	//else
	//	nNewExp++;
	//tickStart = clock();

	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) {
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	//Categorize(neighbors);


	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;
		double h = env->HCost(neighbor,goal);
		double f = g + h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE)//!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end()) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) {
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE)//openQueue.IsIn(SearchNode(neighbor))) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose) {
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;

				if (verbose) {
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED

				NodesReopened++;

				//neighborNode.rxp = true;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	neighbors.clear();

	//if (topNode.rxp)
	//	tickReExp += clock() - tickStart;
	//else
	//	tickNewExp += clock() - tickStart;

	return false;
}

/* Algorithm B' . step (3a) (3b) are inserted into algorithm B. step (7) is also changed, since we need to store the updated h */
bool Prop::DoSingleStepBP(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;

	// select the node to expand
	SearchNode topNode;
	if (fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if (verbose)
			printf("Expanding a node below F.\n");
	}
	//else if (fequal(openQueue.top().fCost, F))
	//{
	//	GetLowestGF(topNode);
	//}
	else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	//if (topNode.rxp)
	//	nReExp++;
	//else
	//	nNewExp++;
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	//tickStart = clock();

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	//Categorize(neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )


	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}

		ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight, mode);

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) //!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) //openQueue.IsIn(SearchNode(neighbor))) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					/* proved to be impossible */
					if (UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if (UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				//neighborNode.rxp = true;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if (fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if (verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	//if (topNode.rxp)
	//	tickReExp += clock() - tickStart;
	//else
	//	tickNewExp += clock() - tickStart;

	return false;
}


bool Prop::DoSingleStepApprox(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;


	// select the node to expand
	SearchNode topNode;
	//if (fless(openQueue.top().fCost , F)) 
	//{
	//	GetLowestG(topNode);
	//	if (verbose)
	//		printf("Expanding a node below F.\n");
	//}
	//else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	//Categorize(neighbors);

//	double hTop = topNode.fCost - topNode.gCost;
	//double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )


	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h = env->HCost(neighbor,goal);

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}
		
		//ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight,mode);

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		//minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) //!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) //openQueue.IsIn(SearchNode(neighbor))) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					/* proved to be impossible */
					//if (UpdateHOnly(neighborNode, h))
					//	openQueue.IncreaseKey(neighborNode);
					
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					//if (UpdateHOnly(neighborNode, h))
					//	closedList[neighbor] = neighborNode;

					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				// careful! gCost will be altered in next operation!
				double oldG = neighborNode.gCost;

					// this operation is optional, as appears in the variant version of the alg
					neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
					closedList[neighbor] = neighborNode; // this line was MISSING !!!

				if (fgreater(oldG - g , delta)) // if we can reduce g, and not by a small amount, then don't ignore
				{
					closedList.erase(neighbor);  // delete from CLOSED
					NodesReopened++;

					openQueue.Add(neighborNode); // add to OPEN
				}

			}

		}
	}

	/* step Mero (3b), update h of parent */
	//if (fgreater(minH2 , hTop)) 
	//{
	//	topNode.fCost = minH2 + topNode.gCost;  // f = h + g
	//	closedList[topNodeID] = topNode;

	//	if (verbose) 
	//	{
	//		printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
	//	}
	//}

	neighbors.clear();

	return false;
}

bool Prop::UpdateHOnly(SearchNode &neighborNode, double h)
{
	if (fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
	{
		double f = h + neighborNode.gCost;
		neighborNode.fCost = f;
		return true;
	}
	return false;
}

/* after using Categorize(), this algorithm is not quite right. to be fixed later ... */
bool Prop::DoSingleStepBFS(std::vector<graphState> &/*thePath*/) {
	return true;
}
// get lowest g node from OPEN, with f < F
void Prop::GetLowestG(SearchNode &gNode)
{	
	gNode = openQueue.FindSpecialMin(F);

	double fCost = gNode.fCost;
	gNode.fCost = -1; // set it to min then extract from top

	openQueue.DecreaseKey(gNode);
	openQueue.Remove();

	gNode.fCost = fCost;
}

// get lowest g node from OPEN, with f == F
void Prop::GetLowestGF(SearchNode &gNode)
{	
	gNode = openQueue.FindTieFMin(F);

	double fCost = gNode.fCost;
	gNode.fCost = -1; // set it to min then extract from top

	openQueue.DecreaseKey(gNode);
	openQueue.Remove();

	gNode.fCost = fCost;
}


// Nathan's alg
bool Prop::DoSingleStepDelay(std::vector<graphState> &/*thePath*/)
{
	return true;
}

// for DP
void Prop::ReverseProp(SearchNode& topNode) {
	//tickStart = clock();

////////////
	graphState topNodeID = topNode.currNode;
	double currentG = topNode.gCost;
	for (unsigned int x=0;x<neighbors.size();x++) 
	{
		graphState neighbor = neighbors[x];
		SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			double newG = env->GCost(topNodeID,neighbor) + neighborNode.gCost;
			if (currentG > newG) {
				currentG = newG;
				topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
			}
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				double newG = env->GCost(topNodeID,neighbor) + neighborNode.gCost;
				if (currentG > newG) {
					currentG = newG;
					topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
				}
			}
			else {
			}
		}
	}

	//nBPMX++;
	//tickBPMX += clock() - tickStart;

	metaexpanded += 1; //
	nodesExpanded += 1;
	nodesTouched += neighbors.size();
}

// for BPMX
void Prop::ReversePropX1(SearchNode& topNode) 
{
	//tickStart = clock();
	
	double maxh = topNode.fCost - topNode.gCost;
	for (unsigned int x=0;x<neighbors.size();x++) 
	{
		graphState neighbor = neighbors[x];
		SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - env->GCost(topNode.currNode,neighbor));
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				double oh = maxh;
				maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - env->GCost(topNode.currNode,neighbor));

				if (bpmxLevel > 1 && oh < maxh) {
					fifo.push_back(neighbor);
				}
			}
			else {
				maxh = max(maxh, env->HCost(neighbor,goal) - env->GCost(topNode.currNode,neighbor));
			}
		}
	}

	//nBPMX++;
	//tickBPMX += clock() - tickStart;

	metaexpanded += 1; //
	nodesExpanded += 1;
	nodesTouched += neighbors.size();

	topNode.fCost = maxh + topNode.gCost;
}


// for DPMX
// and DPDLMX
void Prop::ReversePropX2(SearchNode& topNode) {

	//tickStart = clock();
////////////
	graphState topNodeID = topNode.currNode;
	double currentG = topNode.gCost;
	double maxh = topNode.fCost - topNode.gCost;
	for (unsigned int x=0;x<neighbors.size();x++) 
	{
		graphState neighbor = neighbors[x];
		SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			double edge = env->GCost(topNodeID,neighbor);
			double newG = edge + neighborNode.gCost;
			if (fgreater(currentG , newG)) {
				currentG = newG;
				topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
			}
			maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - edge);
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				double edge = env->GCost(topNodeID,neighbor);
				double newG = edge + neighborNode.gCost;
				if (fgreater(currentG , newG)) {
					currentG = newG;
					topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
				}
				maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - edge);
			}
			else { // new node
				//neighborNode = delayCache.find(SearchNode(neighbor));
				//if (neighborNode.currNode == neighbor) {
				//	double edge = env->GCost(topNodeID, neighbor);
				//	double newG = edge + neighborNode.gCost;
				//	if (fgreater(currentG, newG)) {
				//		currentG = newG;
				//		topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
				//	}
				//	maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - edge);
				//}
				//else
					maxh = max(maxh, env->HCost(neighbor,goal) - env->GCost(topNodeID,neighbor));
			}
		}
	}

	topNode.fCost = maxh + topNode.gCost;

	//nBPMX++;
	//tickBPMX += clock() - tickStart;

	metaexpanded += 1; //
	nodesExpanded += 1;
	nodesTouched += neighbors.size();
}

void Prop::Broadcast(int level, int levelcount)
{ // we will only enque the newly updated (neighbor) nodes; or neighbor nodes being able to update others
	NodeLookupTable::iterator iter;

	//tickStart = clock();

	for (int i=0;i<levelcount;i++) {
		graphState front = fifo.front();
		fifo.pop_front();

		iter = closedList.find(front);
		if (iter == closedList.end())
			continue;

		SearchNode frontNode = iter->second;
		double frontH = frontNode.fCost - frontNode.gCost;

		myneighbors.clear();
		env->GetSuccessors(front, myneighbors);

		// backward pass
		for (unsigned int x = 0; x < myneighbors.size(); x++) 
		{
			graphState neighbor = myneighbors[x];
			 iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				double edgeWeight = env->GCost(front,neighbor);
				SearchNode neighborNode = iter->second;

				double neighborH = neighborNode.fCost - neighborNode.gCost;
				
				if (fgreater(neighborH - edgeWeight, frontH)) {
					frontH = neighborH - edgeWeight;
					frontNode.fCost = frontNode.gCost + frontH;
					fifo.push_back(neighbor);
				}
			}
			else {
				SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
				if (neighborNode.currNode == neighbor) {
					double edgeWeight = env->GCost(front,neighbor);

					double neighborH = neighborNode.fCost - neighborNode.gCost;
					
					if (fgreater(neighborH - edgeWeight, frontH)) {
						frontH = neighborH - edgeWeight;
						frontNode.fCost = frontNode.gCost + frontH;
					}
				}
				else {
					//neighborNode = delayCache.find(SearchNode(neighbor));
					//if (neighborNode.currNode == neighbor) {
					//	double edgeWeight = env->GCost(front,neighbor);
					//	double neighborH = neighborNode.fCost - neighborNode.gCost;

					//	if (fgreater(neighborH - edgeWeight, frontH)) {
					//		frontH = neighborH - edgeWeight;
					//		frontNode.fCost = frontNode.gCost + frontH;
					//		fifo.push_back(neighbor); // trick: its neighbors guaranteed been generated
					//	}
					//}
				}
			}
		}
		// store frontNode
		closedList[front] = frontNode;

		// forward pass
		for (unsigned int x = 0; x < myneighbors.size(); x++) 
		{
			graphState neighbor = myneighbors[x];
			NodeLookupTable::iterator theIter = closedList.find(neighbor);
			if (theIter != closedList.end()) {
				double edgeWeight = env->GCost(front,neighbor);
				SearchNode neighborNode = theIter->second;

				double neighborH = neighborNode.fCost - neighborNode.gCost;
				
				if (fgreater(frontH - edgeWeight, neighborH)) {
					neighborNode.fCost = neighborNode.gCost + frontH - edgeWeight;  // store neighborNode
					closedList[neighbor] = neighborNode;
					fifo.push_back(neighbor);
				}
			}
			else {
				SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
				if (neighborNode.currNode == neighbor) {
					double edgeWeight = env->GCost(front,neighbor);

					double neighborH = neighborNode.fCost - neighborNode.gCost;
					
					if (fgreater(frontH - edgeWeight, neighborH)) {
						neighborNode.fCost = neighborNode.gCost + frontH - edgeWeight;
						openQueue.IncreaseKey(neighborNode);
					}
				}
				else {
					neighborNode = delayCache.find(SearchNode(neighbor));
					double edgeWeight = env->GCost(front,neighbor);
					double neighborH = neighborNode.fCost - neighborNode.gCost;

					if (fgreater(frontH - edgeWeight, neighborH)) {
						neighborNode.fCost = neighborNode.gCost + frontH - edgeWeight;
						delayCache.IncreaseKey(neighborNode);
						fifo.push_back(neighbor); // trick: its neighbors guaranteed been generated
					}
				}
			}
		}
	}

	//nBPMX += 2;
	//tickBPMX += clock() - tickStart;

	metaexpanded += 2; //
	nodesExpanded += 2;
	nodesTouched += 2*levelcount;

	level++;
	if (level < bpmxLevel && fifo.size() > 0)
		Broadcast(level, fifo.size());
}


/* Algorithm Dual Propagation. Only applicable to undirected graphs! */
bool Prop::DoSingleStepDP(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;

	// select the node to expand
	SearchNode topNode;
	//if (fless(openQueue.top().fCost , F)) 
	//{
	//	GetLowestG(topNode);
	//	if (verbose)
	//		printf("Expanding a node below F.\n");
	//}
	//else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	//if (topNode.rxp)
	//	nReExp++;
	//else
	//	nNewExp++;

//	unsigned long tickTmp ;//= clock();

	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	//tickGen = clock() - tickTmp;

	//Categorize(neighbors);

	bool doReverse = false;

	int ichild = 0;

_REVERSE:
	// reverseProp() here, top node will be updated inside, so put topnode into closed afterwards
	if (doReverse) {
		if (bpmxLevel >= 1)
			ReversePropX2(topNode); 
		else
			ReverseProp(topNode);

		// counting supplement
		//if (nodesExpanded%2) {
		//	nodesExpanded += 1;

		//	if (topNode.rxp)
		//		nReExp++;
		//	else
		//		nNewExp++;
		//}

		//nodesExpanded += 1; //ichild / (double)neighbors.size();
	}

	//tickStart = clock();
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		//CleanUpOpen(topNode.gCost); // put nodes in open with f==F but g<g(goal) into closed, since they are likely part of the solution

		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	//neighbors.resize(0);
	//env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	//}
	unsigned int x;
	for ( x = 0,ichild=1; x<neighbors.size(); x++,ichild++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}
		
		if (bpmxLevel >= 1)
			ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight, mode);
		else {
			if (mode == NEWMODE)
				h = env->HCost(neighbor,goal);
			else
				h = neighborNode.fCost - neighborNode.gCost;
		}

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}

		if (bpmxLevel >= 1)
			if (fgreater(h - edgeWeight , hTop)) {
				//if (topNode.rxp) {
				//	tickReExp += clock() - tickStart + tickGen;
				//	tickGen = 0;
				//}
				//else {
				//	tickNewExp += clock() - tickStart + tickGen;
				//	tickGen = 0;
				//}

				topNode.fCost = topNode.gCost + h - edgeWeight; // hack: fix for when heuristic morphing
				doReverse = true;
				goto _REVERSE;
			}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					/* covered by backward H propagation check already */
					if (fless(neighborNode.gCost + edgeWeight , topNode.gCost)) {
						//if (topNode.rxp) {
						//	tickReExp += clock() - tickStart + tickGen;
						//	tickGen = 0;
						//}
						//else {
						//	tickNewExp += clock() - tickStart + tickGen;
						//	tickGen = 0;
						//}

						doReverse = true;
						goto _REVERSE;
					}
					// we may fail to update g, but still update h
					/* proved to be impossible */
					//if (UpdateHOnly(neighborNode, h))
					//	openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					if (fless(neighborNode.gCost + edgeWeight , topNode.gCost)) {

						//if (topNode.rxp) {
						//	tickReExp += clock() - tickStart + tickGen;
						//	tickGen = 0;
						//}
						//else {
						//	tickNewExp += clock() - tickStart + tickGen;
						//	tickGen = 0;
						//}

						doReverse = true;
						goto _REVERSE;
					}
					// we may fail to update g, but still update h
					if (bpmxLevel >= 1)
						if (UpdateHOnly(neighborNode, h))
							closedList[neighbor] = neighborNode;
					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				//neighborNode.rxp = true;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if (bpmxLevel >= 1)
		if (fgreater(minH2 , hTop)) 
		{
			topNode.fCost = minH2 + topNode.gCost;  // f = h + g
			closedList[topNodeID] = topNode;

			if (verbose) 
			{
				printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
			}
		}

	neighbors.clear();

	//if (topNode.rxp) {
	//	tickReExp += clock() - tickStart + tickGen;
	//}
	//else {
	//	tickNewExp += clock() - tickStart + tickGen;
	//}

	return false;
}

bool Prop::DoSingleStepBPMX(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;

	// select the node to expand
	SearchNode topNode;
	if (fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if (verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	//if (topNode.rxp)
	//	nReExp++;
	//else
	//	nNewExp++;

//	unsigned long tickTmp;// = clock();

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	//tickGen = clock() - tickTmp;

	bool doBPMX = false;
	//Categorize(neighbors);
	int ichild = 0;

_BPMX:

	if (doBPMX) {
		ReversePropX1(topNode);

		// counting supplement
		//if (nodesExpanded%2) {
		//	nodesExpanded += 1;

		//	if (topNode.rxp)
		//		nReExp++;
		//	else
		//		nNewExp++;
		//}

		//nodesExpanded += 1; //ichild / (double)neighbors.size();
	}

	//tickStart = clock();

	// topnode may be updated in ReverseProp, so put to closed afterwards
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )


	//while(true) 
	//{
		//SearchNode neighborNode;
		//graphState neighbor;
		//int mode;
		////double edgeWeight;

		//if (!NextNeighbor(neighborNode, neighbor, mode))
		//	break;
	unsigned int x;
	for (x = 0,ichild=1; x<neighbors.size(); x++,ichild++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}
		
		ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight, mode);

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}

		if (fgreater(h - edgeWeight , hTop)) {
			//if (topNode.rxp) {
			//	tickReExp += clock() - tickStart + tickGen;
			//	tickGen = 0;
			//}
			//else {
			//	tickNewExp += clock() - tickStart + tickGen;
			//	tickGen = 0;
			//}

			topNode.fCost = topNode.gCost + h - edgeWeight; // hack: fix for when heuristic morphing
			doBPMX = true;
			goto _BPMX;
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) //!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) //openQueue.IsIn(SearchNode(neighbor))) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					/* proved to be impossible */
					if (UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if (UpdateHOnly(neighborNode, h)) {
						closedList[neighbor] = neighborNode;

						if (bpmxLevel > 1)
							fifo.push_back(neighbor);
					}
					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				//neighborNode.rxp = true;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if (fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if (verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	if (fifo.size() > 0) {
		Broadcast(1,fifo.size()); // (level, levelcount)
	}

	//if (topNode.rxp)
	//	tickReExp += clock() - tickStart + tickGen;
	//else
	//	tickNewExp += clock() - tickStart + tickGen;

	return false;
}


bool Prop::DoSingleStepBPMXE(std::vector<graphState> &) 
{
	return true;
}

/* DP + Delay + BPMX */
bool Prop::DoSingleStepDPDLMX(std::vector<graphState> &thePath) 
{
	/* step (2) */

	//if (openQueue.size() == 0)
	//{
	//	thePath.resize(0); // no path found!
	//	closedList.clear();
	//	openQueue.reset();
	//	FCache.reset();
	//	env = 0;
	//	return true;
	//}
	

	/* step (3) */
	nodesExpanded += 1;

	// select the node to expand
	// this is where delay comes into play
	SearchNode topNode;

	if (goal == openQueue.top().currNode) 
	{
		bool found = false;
		while(delayCache.size() > 0 && delayCache.top().gCost < openQueue.top().fCost) {
			topNode = delayCache.Remove();
			if (topNode.fCost < openQueue.top().fCost) {
				found = true;
				break;
			}
		}
		if (!found) {
			topNode = openQueue.Remove();
		}
	}
	else if (delayCache.size() > 0) 
	{
		if (openQueue.size() ==0 || (reopenings < fDelay(nodesExpanded-NodesReopened) && delayCache.top().gCost < openQueue.top().fCost)) {
			topNode = delayCache.Remove();
			reopenings++;
		}
		else {
			topNode = openQueue.Remove();
			reopenings = 0;
		}
	}
	else if (openQueue.size() > 0) 
	{
		topNode = openQueue.Remove();
		reopenings = 0;
	}
	else 
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		delayCache.reset();
		env = 0;
		return true;
	}
	//if (fless(openQueue.top().fCost , F)) 
	//{
	//	GetLowestG(topNode);
	//	if (verbose)
	//		printf("Expanding a node below F.\n");
	//}
	//else 
	//{
	//	topNode = openQueue.Remove();
		
	//	if (fgreater(topNode.fCost,F)) 
	//	{
	//		F = topNode.fCost; // update F
	//		if (verbose) 
	//		{
	//			printf("F updated to %lf.\n",F);
	//		}
	//	}
	//}

	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	//Categorize(neighbors);

	bool doReverse = false;

	int ichild = 0;

_REVERSE:
	// reverseProp() here, top node will be updated inside, so put topnode into closed afterwards
	if (doReverse) {
		if (bpmxLevel >= 1) 
			ReversePropX2(topNode); 
		else
			ReverseProp(topNode);

		// counting supplement
		//if (nodesExpanded%2)
		//	nodesExpanded += 1;
		
		//nodesExpanded += 1; //ichild /(double)neighbors.size();
	}
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		//CleanUpOpen(topNode.gCost); // put nodes in open with f==F but g<g(goal) into closed, since they are likely part of the solution

		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		delayCache.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	//neighbors.resize(0);
	//env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	//}
	unsigned int x ;
	for ( x = 0,ichild=1; x<neighbors.size(); x++,ichild++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				neighborNode = delayCache.find(SearchNode(neighbor));
				if (neighborNode.currNode == neighbor) 
					mode = DELAYMODE;
				else
					mode = NEWMODE;
			}
		}
		
		if (bpmxLevel >= 1)
			ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight, mode);
		else {
			if (mode == NEWMODE)
				h = env->HCost(neighbor,goal);
			else
				h = neighborNode.fCost - neighborNode.gCost;
		}

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}

		if (bpmxLevel >= 1)
			if (fgreater(h - edgeWeight , hTop)) {
				topNode.fCost = topNode.gCost + h - edgeWeight; // hack: fix for when heuristic morphing
				doReverse = true;
				goto _REVERSE;
			}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					/* covered by backward H propagation check already */
					if (fless(neighborNode.gCost + edgeWeight , topNode.gCost)) {
						doReverse = true;
						goto _REVERSE;
					}
					// we may fail to update g, but still update h
					/* proved to be impossible */
					//if (UpdateHOnly(neighborNode, h))
					//	openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else if (mode == DELAYMODE) 
			{
				if (!fgreater(neighborNode.gCost,g))
				{ 
					if (fless(neighborNode.gCost + edgeWeight, topNode.gCost)) {
						doReverse = true;
						goto _REVERSE;
					}
					// fail to update g, but may still update h
					if (UpdateHOnly(neighborNode, h))
						delayCache.IncreaseKey(neighborNode);
					continue;
				}

				RelaxDelayNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					if (fless(neighborNode.gCost + edgeWeight , topNode.gCost)) {
						doReverse = true;
						goto _REVERSE;
					}
					// we may fail to update g, but still update h
					if (bpmxLevel >= 1)
						if (UpdateHOnly(neighborNode, h))
							closedList[neighbor] = neighborNode;
					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if (bpmxLevel >= 1)
		if (fgreater(minH2 , hTop)) 
		{
			topNode.fCost = minH2 + topNode.gCost;  // f = h + g
			closedList[topNodeID] = topNode;

			if (verbose) 
			{
				printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
			}
		}

	neighbors.clear();

	return false;
}

/*  not used */
bool Prop::DoSingleStepDPMX(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded += 1;

	// select the node to expand
	SearchNode topNode;
	if (fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if (verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if (fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	//Categorize(neighbors);

	// reverseProp() here, top node will be updated inside, so put topnode into closed afterwards
	ReversePropX2(topNode);

	//nodesExpanded += 1;
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if (verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		//CleanUpOpen(topNode.gCost); // put nodes in open with f==F but g<g(goal) into closed, since they are likely part of the solution

		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	//neighbors.resize(0);
	//env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	//while(true) 
	//{
	//	SearchNode neighborNode;
	//	graphState neighbor;
	//	int mode;
	//	//double edgeWeight;

	//	if (!NextNeighbor(neighborNode, neighbor, mode))
	//		break;
	//}
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;

		// determine neighbor type
		SearchNode neighborNode;
		int mode;
		neighborNode = openQueue.find(SearchNode(neighbor));
		if (neighborNode.currNode == neighbor) {
			mode = OPENMODE;
		}
		else {
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if (iter != closedList.end()) {
				neighborNode = iter->second;
				mode = CLOSEDMODE;
			}
			else {
				mode = NEWMODE;
			}
		}
		
		ComputeNewHMero3a(h, h_tmp, neighbor, neighborNode, hTop - edgeWeight, mode);

		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if (mode == NEWMODE) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if (verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if (mode == OPENMODE) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if (UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if (closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if (neighborNode.gCost <= g) {
				if (!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if (UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					continue;
				}

				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if (fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if (verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}

void Prop::ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath)
{
	SearchNode n;
	NodeLookupTable::iterator iter;

	closedSize = closedList.size();

	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else n = openQueue.find(SearchNode(goalNode));

	solutionCost = n.gCost;
	do {
		//solutionCost += env->GCost(n.prevNode,n.currNode);

		if (verbose)
			printf("%ld<-%ld,",n.currNode,n.prevNode);

		thePath.push_back(n.currNode);
		//n = closedList[n.prevNode];
		iter = closedList.find(n.prevNode);
		if (iter != closedList.end())
			n = iter->second;
		else
			n = openQueue.find(SearchNode(n.prevNode));

	} while (n.currNode != n.prevNode);
	//thePath.push_back(n.currNode);
	pathSize = thePath.size();
}

//void Prop::OpenGLDraw() const
//{
//	// node to expand: blue
//	// in open: green
//	// in closed: red
//	// in waitlist: yellow
//
//	//float r,gcost,b;
//	double x,y,z;
//	SearchNode sn;
//	graphState nodeID;
//	SearchNode topn;
//	char buf[100];
//
//	// draw nodes
//	node_iterator ni = grp->getNodeIter();
//	for (node* n = grp->nodeIterNext(ni); n; n = grp->nodeIterNext(ni))
//	{
//		graphGenerator::GetLoc(n,x,y,z);
//
//		nodeID = (graphState) n->GetNum();
//		// draw sphere first
//
//		// if it's just expanded
//		NodeLookupTable::iterator hiter;
//		if (nodeID == goal) 
//		{
//			glColor3f(1.0, 0.0, 1.0); // Magenta
//			DrawSphere(x,y,z,0.025);
//		}
//		else if (nodeID == justExpanded)
//		{
//			sn = closedList.find(nodeID)->second;
//			glColor3f(0,0,1);  // blue
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%d,%d,%d]",n->GetNum(), (int)sn.gCost, (int)(sn.fCost - sn.gCost), (int)sn.fCost);
//		}
//		// if in closed
//		else if ((hiter = closedList.find(nodeID)) != closedList.end())
//		{
//			sn = hiter->second;
//			glColor3f(1,0,0);  // red
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%d,%d,%d]",n->GetNum(), (int)sn.gCost, (int)(sn.fCost - sn.gCost), (int)sn.fCost);
//		}
//		// if in open
//		else if (openQueue.IsIn(SearchNode(nodeID)))
//		{
//			sn = openQueue.find(SearchNode(nodeID));
//
//
//			
//				glColor3f(0,1,0);  // green
//				DrawSphere(x,y,z,0.025);
//			
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%ld,%ld,%ld]",n->GetNum(), (long)sn.gCost, (long)(sn.fCost - sn.gCost), (long)sn.fCost);
//		}
//		else if (WaitList.IsIn(SearchNode(nodeID)))
//		{
//			sn = WaitList.find(SearchNode(nodeID));
//
//			glColor3f(1.0, 1.0, 0.0);  // yellow
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%ld,%ld,%ld]",n->GetNum(), (long)sn.gCost, (long)(sn.fCost - sn.gCost), (long)sn.fCost);
//		}
//		// neither, ignore
//		else 
//		{
//			continue;
//			
//			glColor3f(1,1,1); // white
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [?,%ld,?]",n->GetNum(), (long)env->HCost(nodeID,goal));
//		}
//
//		// draw the text info, in black
//		if (drawtext)
//			DrawText(x,y,z-0.15,0,0,0,buf);
//	}
//
//	// draw edges
//	edge_iterator ei = grp->getEdgeIter();
//	for (edge* e = grp->edgeIterNext(ei); e; e = grp->edgeIterNext(ei))
//	{
//		DrawEdge(e->getFrom(), e->getTo(), e->GetWeight());
//	}
//}

void Prop::DrawText(double x, double y, double z, float r, float gg, float b, char* str)
{
	//glPushMatrix();
	// rotate ?
  /*
	glPushMatrix();
	glColor3f(r,gg,b);
	glTranslatef(x,y,z);
	glScalef(1.0/(20*120.0), 1.0/(20*120.0), 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	
	int i=0;
	while(str[i]) 
	{
// 		glutStrokeCharacter(GLUT_STROKE_ROMAN,str[i]);
		i++;
	}
	glPopMatrix();
  */
}

void Prop::DrawEdge(unsigned int from, unsigned int to, double weight)
{
	double x1,y1,z1;
	double x2,y2,z2;
	char buf[100] = {0};

	node* nfrom = grp->GetNode(from);
	node* nto = grp->GetNode(to);

	graphGenerator::GetLoc(nfrom,x1,y1,z1);
	graphGenerator::GetLoc(nto,x2,y2,z2);
	/*
	// draw line segment
	glBegin(GL_LINES);
	glColor3f(1,0,0); // red
	glVertex3f(x1,y1,z1);
	glVertex3f(x2,y2,z2);
	glEnd();

	// draw weight info
	if (drawtext) {
		sprintf(buf,"%ld",(long)weight);
		DrawText((x1+x2)/2, (y1+y2)/2, (z1+z2)/2 - 0.15, 1, 0, 0, buf); // in red
	}
	*/
}

