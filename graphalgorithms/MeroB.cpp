/*
 *  MeroB.cpp
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code is not optimized for speed, but rather emphasizing on correctness.
 */

#include "MeroB.h"
#include <cstring>

const static bool verbose = false;

void MeroB::GetPath(GraphEnvironment *_env, Graph* _g, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	if (!InitializeSearch(_env,_g,from,to,thePath))
		return;
	
	while(!DoSingleSearchStep(thePath)) 
		{}
	
	if (thePath.size() > 0 && verbose)
		printf("\nNodes expanded=%lld, Nodes touched=%lld.\n",GetNodesExpanded(),GetNodesTouched());
}

bool MeroB::InitializeSearch(GraphEnvironment *_env, Graph* _g, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	env = _env;
	if (heuristic == 0)
		heuristic = env;
	g = _g;
	nodesTouched = nodesExpanded = 0;
	start = from;
	goal = to;
	
	closedList.clear();
	openQueue.reset();
	FCache.reset();
	
	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}
	
	// step (1)
	MeroBUtil::SearchNode first(heuristic->HCost(start, goal), 0, start, start);
	openQueue.Add(first);
	
	//if (verID == MB_B || verID == MB_BP)
	F = 0;
		
	return true;
}

bool MeroB::DoSingleSearchStep(std::vector<graphState> &thePath)

{
	if (verID == MB_BP) 
		return DoSingleStepBP(thePath);
	else if (verID == MB_B)
		return DoSingleStepB(thePath);
	else
		return DoSingleStepA(thePath);
}

/* The steps refer to the pseudo codes of the corresponding algorithms */

bool MeroB::DoSingleStepA(std::vector<graphState> &thePath)
{
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
	nodesExpanded++;
	MeroBUtil::SearchNode topNode = openQueue.Remove();
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose)
	{
		printf("Expanding node %ld , gcost=%lf, h=%lf, f=%lf.\n",
					 topNodeID, topNode.gCost, topNode.fCost-topNode.gCost, topNode.fCost);
	}
	
	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		//closedList.clear();
		//openQueue.reset();
		//env = 0;

		// don't reset everything yet, since they are required by the rendering funciton
		return true;
	}
	
	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);
	
	for (unsigned int x = 0; x<neighbors.size(); x++)
	{
		nodesTouched++;
		
		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double gcost = topNode.gCost + edgeWeight;
		double h = heuristic->HCost(neighbor,goal);
		double f = gcost + h;
		
		/* step (6), neither in OPEN nor CLOSED */
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end())
		{
			MeroBUtil::SearchNode n(f, gcost, neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose)
			{
				printf("Adding node %ld to OPEN, gcost=%lf, h=%lf, f=%lf.\n",neighbor,gcost,f-gcost,f);
			}
		}
		
		/* step (7) */
		else 
		{
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor)))
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost))
					continue;
				
				if (verbose)
				{
					printf("Adjusting node %ld in OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,gcost,neighbor,topNodeID); // parent is changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else if (closedList.find(neighbor) != closedList.end())
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost))
					continue;
				
				if (verbose)
				{
					printf("Moving node %ld from CLOSED to OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,gcost,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	
	return false;
}

/* Algorithm B. step (3) is modified from algorithm A.*/
bool MeroB::DoSingleStepB(std::vector<graphState> &thePath)
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
	nodesExpanded++;
	
	// put those with f < F into cache
	FCache.reset();
	while(openQueue.size() > 0)
		
	{
		MeroBUtil::SearchNode tmpNode = openQueue.top();
		if (fless(tmpNode.fCost, F))
		{
			FCache.Add(openQueue.Remove());
		}
		else
			break;
	}
	
	// select the node to expand
	MeroBUtil::SearchNode topNode;
	if (FCache.size() > 0) 
	{
		topNode = FCache.Remove();
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
	
	// move remaining nodes from FCache back to openQueue
	while (FCache.size() > 0)
	{
		openQueue.Add(FCache.Remove());
	}
	
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose)
	{
		printf("Expanding node %ld , gcost=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}
	
	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		//closedList.clear();
		//openQueue.reset();
		FCache.reset();
		//env = 0;

		// don't reset everything yet, since they are required by the rendering funciton
		return true;
	}
	
	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);
	
	for (unsigned int x = 0; x<neighbors.size(); x++)
	{
		nodesTouched++;
		
		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double gcost = topNode.gCost + edgeWeight;
		double h = heuristic->HCost(neighbor,goal);
		double f = gcost + h;
		
		/* step (6), neither in OPEN nor CLOSED */
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() )
			
		{
			MeroBUtil::SearchNode n(f,gcost,neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose)
			{
				printf("Adding node %ld to OPEN, gcost=%lf, h=%lf, f=%lf.\n",neighbor,gcost,f-gcost,f);
			}
		}
		/* step (7) */
		else {
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor)))
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost))
					continue;
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,gcost,neighbor,topNodeID); // parent is changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else if (closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost))
					continue;
				
				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",
								 neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,gcost,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	return false;
}
	
/* Algorithm B' . step (3a) (3b) are inserted into algorithm B. step (7) is also changed, since we need to store the updated h */
bool MeroB::DoSingleStepBP(std::vector<graphState> &thePath) 
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
	nodesExpanded++;
	
	// put those with f < F into cache
	FCache.reset();
	while(openQueue.size() > 0) 
	{
		MeroBUtil::SearchNode tmpNode = openQueue.top();
		if (fless(tmpNode.fCost , F)) 
		{
			FCache.Add(openQueue.Remove());
		}
		else
			break;
	}
	
	// select the node to expand
	MeroBUtil::SearchNode topNode;
	if (FCache.size() > 0) 
	{
		topNode = FCache.Remove();
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
	
	// move remaining nodes from FCache back to openQueue
	while(FCache.size() > 0) 
	{
		openQueue.Add(FCache.Remove());
	}
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose) 
	{
		printf("Expanding node %ld , gcost=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}
	
	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		//closedList.clear();
		//openQueue.reset();
		FCache.reset();
		//env = 0;

		// don't reset everything yet, since they are required by the rendering funciton
		return true;
	}
	
	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);
	
	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )
	
	for (unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;
		
		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double gcost = topNode.gCost + edgeWeight;
		
		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		// this is necessary because the modified h is stored in the node, not the environment
		if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor))) { 
			MeroBUtil::SearchNode nb = openQueue.find(MeroBUtil::SearchNode(neighbor));
			h = max( nb.fCost - nb.gCost, hTop - edgeWeight);
			
			h_tmp = nb.fCost - nb.gCost;
		}
		else if (closedList.find(neighbor) != closedList.end()) 
		{
			MeroBUtil::SearchNode nb = closedList.find(neighbor)->second;
			h = max( nb.fCost - nb.gCost, hTop - edgeWeight);
			
			h_tmp = nb.fCost - nb.gCost;
		}
		else 
		{
			h = max( heuristic->HCost(neighbor,goal), hTop - edgeWeight);
			
			h_tmp = heuristic->HCost(neighbor,goal);
		}
		
		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = gcost + h;
		
		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);
		
		/* step (6), neither in OPEN nor CLOSED */
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			MeroBUtil::SearchNode n(f,gcost,neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose) 
			{
				printf("Adding node %ld to OPEN, gcost=%lf, h=%lf, f=%lf.\n",neighbor,gcost,f-gcost,f);
			}
		}
		/* step (7) */
		else 
		{
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost)) 
				{
					if (fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
					{
						// we may fail to update gcost, but still update h
						f = h + neighborNode.gCost;
						neighborNode.fCost = f;
						openQueue.IncreaseKey(neighborNode);
					}
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %ld in OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				if (fless(f , neighborNode.fCost)) 
				{
					neighborNode.copy(f,gcost,neighbor,topNodeID); // parent is changed
					openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
				}
				else 
				{
					neighborNode.copy(f,gcost,neighbor,topNodeID); // parent is changed
					openQueue.IncreaseKey(neighborNode);  // adjust its position in OPEN
				}
			}
			else if (closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= gcost)
				if (!fgreater(neighborNode.gCost,gcost)) 
				{
					if (fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
					{
						// we may fail to update gcost, but still update h
						f = h + neighborNode.gCost;
						neighborNode.fCost = f;
						closedList[neighbor] = neighborNode;
					}
					continue;
				}
				
				if (verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, gcost=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gcost,f-gcost,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,gcost,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	/* step Mero (3b), update h of parent */
	if (fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + gcost
		closedList[topNodeID] = topNode;
		
		if (verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}
	
	return false;
}
		
void MeroB::ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath)
{
	MeroBUtil::SearchNode n;
	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else {
		n = openQueue.find(MeroBUtil::SearchNode(goalNode));
	}
	
	do {
		thePath.push_back(n.currNode);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	//thePath.push_back(n.currNode);
}


//void MeroB::OpenGLDraw(int) const
//{
//	OpenGLDraw();
//}

//void MeroB::OpenGLDraw() const
//{
//	//float r,gcost,b;
//	double x,y,z;
//	const MeroBUtil::SearchNode sn;
//	graphState nodeID;
//	const MeroBUtil::SearchNode topn;
//	char buf[100];
//
//	// draw nodes
//	node_iterator ni = g->getNodeIter();
//	for (node* n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
//	{
//		MeroBUtil::graphGenerator::GetLoc(n,x,y,z);
//
//		nodeID = (graphState) n->GetNum();
//		// draw sphere first
//
//		// if in closed
//		MeroBUtil::NodeLookupTable::const_iterator hiter;
//		if ((hiter = closedList.find(nodeID)) != closedList.end())
//		{
//			sn = hiter->second;
//			glColor3f(1,0,0);  // red
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%d,%d,%d]",n->GetNum(), (int)sn.gCost, (int)(sn.fCost - sn.gCost), (int)sn.fCost);
//		}
//		// if in open
//		else if (openQueue.IsIn(MeroBUtil::SearchNode(nodeID)))
//		{
//			sn = openQueue.find(MeroBUtil::SearchNode(nodeID));
//
//			FCache.reset();
//			while(openQueue.size() > 0) 
//			{
//				MeroBUtil::SearchNode tmpNode = openQueue.top();
//				if (fless(tmpNode.fCost , F)) 
//				{
//					FCache.Add(openQueue.Remove());
//				}
//				else
//					break;
//			}
//
//			if (FCache.size() > 0)
//			{
//				topn = FCache.top();
//			}
//			else 
//			{
//				topn = openQueue.top();
//			}
//
//			while(FCache.size() > 0) 
//			{
//				openQueue.Add(FCache.Remove());
//			}
//
//			// if on top, blue
//			if (topn.currNode == sn.currNode)
//			{
//				glColor3f(0,0,1);
//				DrawSphere(x,y,z,0.025);
//			}
//			else // green
//			{
//				glColor3f(0,1,0);
//				DrawSphere(x,y,z,0.025);
//			}
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [%ld,%ld,%ld]",n->GetNum(), (long)sn.gCost, (long)(sn.fCost - sn.gCost), (long)sn.fCost);
//		}
//		// neither in open nor closed, white
//		else 
//		{
//			continue;
//			
//			glColor3f(1,1,1); // white
//			DrawSphere(x,y,z,0.025);
//
//			memset(buf,0,100);
//			sprintf(buf,"%d [?,%ld,?]",n->GetNum(), (long)h->HCost(nodeID,goal));
//		}
//
//		// draw the text info, in black
//		DrawText(x,y,z+0.05,0,0,0,buf);
//	}
//
//	// draw edges
//	edge_iterator ei = g->getEdgeIter();
//	for (edge* e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//	{
//		DrawEdge(e->getFrom(), e->getTo(), e->GetWeight());
//	}
//}
//
//void MeroB::DrawText(double x, double y, double z, float r, float gg, float b, char* str)
//{
//	//glPushMatrix();
//	// rotate ?
//
//	glPushMatrix();
//	glColor3f(r,gg,b);
//	glTranslatef(x,y,z);
//	glScalef(1.0/(20*120.0), 1.0/(20*120.0), 1);
//	glRotatef(180, 0.0, 0.0, 1.0);
//	glRotatef(180, 0.0, 1.0, 0.0);
//	
//	int i=0;
//	while(str[i]) 
//	{
//		//glutStrokeCharacter(GLUT_STROKE_ROMAN,str[i]);
//		i++;
//	}
//	glPopMatrix();
//}
//
//void MeroB::DrawEdge(unsigned int from, unsigned int to, double weight)
//{
//	double x1,y1,z1;
//	double x2,y2,z2;
//	char buf[100] = {0};
//
//	node* nfrom = g->GetNode(from);
//	node* nto = g->GetNode(to);
//
//	MeroBUtil::graphGenerator::GetLoc(nfrom,x1,y1,z1);
//	MeroBUtil::graphGenerator::GetLoc(nto,x2,y2,z2);
//
//	// draw line segment
//	glBegin(GL_LINES);
//	glColor3f(1,1,0); // yellow
//	glVertex3f(x1,y1,z1);
//	glVertex3f(x2,y2,z2);
//	glEnd();
//
//	// draw weight info
//	sprintf(buf,"%ld",(long)weight);
//	DrawText((x1+x2)/2, (y1+y2)/2, (z1+z2)/2 + 0.05, 1, 0, 0, buf); // in red
//}
//
