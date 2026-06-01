/*
 *  $Id: ClusterAbstraction.cpp
 *  hog2
 *
 *  Created by Renee Jansen on 06/06/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "ClusterAbstraction.h"
#include "GenericAStar.h"
#include <cfloat>
#include <cmath>
#include <limits>

using namespace GraphAbstractionConstants;

class ClusterSearchEnvironment : public OldSearchCode::SearchEnvironment
{
public:
	ClusterSearchEnvironment(GraphAbstraction *_aMap, int _level)
	:aMap(_aMap), level(_level) {}
	
	void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
	{
		node *n = aMap->GetAbstractGraph(level)->GetNode(nodeID);
		neighbor_iterator ni = n->getNeighborIter();
		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
		{
			if (inCorridor(tmp))
				neighbors.push_back(tmp);
		}
	}
	
	double heuristic(uint32_t node1, uint32_t node2)
	{
		return aMap->h(aMap->GetAbstractGraph(level)->GetNode(node1),
									 aMap->GetAbstractGraph(level)->GetNode(node2));
	}

	double gcost(uint32_t node1, uint32_t node2)
  {
		return heuristic(node1, node2);
	}

	void setCorridor(std::vector<node *> &corr)
	{
		corridor = corr;
		if (corr.size() > 0)
		{
			corridorLevel = aMap->GetAbstractionLevel(corr[0]);
			for (unsigned int x = 0; x < corridor.size(); x++)
				corridor[x]->SetLabelL(kTemporaryLabel, x);
		}
	}
	
	bool inCorridor(uint32_t nodeID)
	{
		if (corridor.size() == 0)
			return true;
		node *n = aMap->GetAbstractGraph(level)->GetNode(nodeID);
		node *parent = aMap->GetNthParent(n, corridorLevel);
		if (parent)
		{
			unsigned int loc = parent->GetLabelL(kTemporaryLabel);
			if ((loc < corridor.size()) && (corridor[loc] == parent))
				return true;
		}
		return false;
	}
private:
	GraphAbstraction *aMap;
	int level;
	int corridorLevel;
	std::vector<node *> corridor;
};


const static int verbose = 0;

/**
* Add an abstract node corresponding to an entrance to the cluster
 */
void Cluster::AddNode(int n)
{
	nodes.push_back(n);
}

/**
* create a cluster abstraction for the given map. Clusters are square, 
 * with height = width = clustersize. 
 */ 
ClusterAbstraction::ClusterAbstraction(Map *map, int _clusterSize)
:MapAbstraction(map),clusterSize(_clusterSize)
{
	abstractions.push_back(GetMapGraph(map));
	createClustersAndEntrances();
	linkEntrancesAndClusters();
	createAbstractGraph();
}

ClusterAbstraction::~ClusterAbstraction()
{
	Graph* g = abstractions[1];
	edge_iterator ei = g->getEdgeIter();
	edge* e = g->edgeIterNext(ei);
	while(e)
	{
		path* p = temp[e];
		temp.erase(e);
		delete p;
		
		//	ei = g->getEdgeIter();
		e = g->edgeIterNext(ei);
	}
	paths.clear();
}

/**
* create clusters on the abstract level and create corresponding 
 * entrances. 
 */ 
void ClusterAbstraction::createClustersAndEntrances()
{
	Map* map = MapAbstraction::GetMap();
	int row=0, col=0, clusterId=0;
	int horizSize,vertSize;
	
	if (verbose) std::cout<<"Creating clusters and entrances\n";
	
	for (int j = 0; j < map->GetMapHeight(); j+=clusterSize)
	{
		col=0; 
		for (int i = 0; i< map->GetMapWidth(); i+=clusterSize)
		{
			horizSize = min(clusterSize, map->GetMapWidth()-i);
			vertSize = min(clusterSize, map->GetMapHeight()-j);
			Cluster cluster(clusterId++,row,col,i,j,horizSize,vertSize);
			
			addCluster(cluster);
			if (j > 0 && j < map->GetMapHeight())
			{
				
				createHorizEntrances(i,i+horizSize-1,j-1,row-1,col);
			}
			if (i > 0 && i < map->GetMapWidth())
			{
				
				createVertEntrances(j,j+vertSize-1,i-1,row,col-1);
			}
			col++;
		}
		row++;
	}  
	
	rows = row;
	columns = col;
	
	if (verbose) std::cout<<"map is "<<map->GetMapHeight()<<" x "<<map->GetMapWidth()<<std::endl
		<<"rows of clusters: "<<rows<<"\ncolumns of clusters: "<<columns<<std::endl;
}

/*
 * return the minimum of two integers (get from some library?)
 */
int ClusterAbstraction::min(int i, int j)
{
	
	if (i<j) return i;
	else return j;
}

/*
 * create the abstract Graph: create a node for each entrance and connect entrances with their 
 * "counterparts" in adjacent clusters, then set up the parent/child relationship between these
 * abstract nodes in the map Graph, then create intra edges. 
 */
void ClusterAbstraction::createAbstractGraph()
{
	abstractions.push_back(new Graph());
	Graph *g = abstractions[1];
	
	addAbsNodes(g);
	setUpParents(g);
	computeClusterPaths(g);
	
	// 	std::cout<<"1st level of abstraction\n";
	// 	g->Print(std::cout);
	// 	std::cout<<"end of Graph\n\n";
	createConnectivityGraph();
	//		abstractions[2]->Print(std::cout);
	
	if (verbose) std::cout<<"abstract Graph has "<<abstractions[1]->GetNumNodes()<<" nodes\n";
	if (verbose) std::cout<<"abstract graphs has "<<abstractions[1]->GetNumEdges()<<" edges\n";
}

/* 
* add a cluster to the abstraction
 */
void ClusterAbstraction::addCluster(Cluster c)
{
	clusters.push_back(c);
}

/* 
* add an entrance to the abstraction
 */
void ClusterAbstraction::addEntrance(Entrance e)
{
	entrances.push_back(e);
}

/**
* based on the function from HPA* abswizard.cpp with the same name
 *
 * Create horizontal entrances. Between 2 obstacles, there is either one or two entrances,
 * depending on how many tiles there are between these obstacles. 
 */ 
void ClusterAbstraction::createHorizEntrances(int start, int end, int latitude, int row, int col)
{
	Map* map = MapAbstraction::GetMap();
	Graph* g = abstractions[0];
	
	for (int i=start; i<=end; i++)
	{
		
		
		while((i<=end) && (!GetNodeFromMap(i,latitude) || !GetNodeFromMap(i,latitude+1) 
											 ||!(g->FindEdge(GetNodeFromMap(i,latitude)->GetNum(), GetNodeFromMap(i,latitude+1)->GetNum()))))
		{
			i++;
		}
		
		if (i>end)
			return;
		
		int begin = i;
		i++;
		//int begin = i;
		//		std::cout<<GetNodeFromMap(i,latitude)<<" "<<GetNodeFromMap(i,latitude+1)<<std::endl;
		while((i<=end)&&(GetNodeFromMap(i,latitude)) && (GetNodeFromMap(i,latitude+1))
					&& (g->FindEdge(GetNodeFromMap(i,latitude)->GetNum(), GetNodeFromMap(i,latitude+1)->GetNum()))
					&& ((i==start) || ((g->FindEdge(GetNodeFromMap(i-1,latitude)->GetNum(), GetNodeFromMap(i,latitude)->GetNum()))
														 && (g->FindEdge(GetNodeFromMap(i-1,latitude+1)->GetNum(), GetNodeFromMap(i,latitude+1)->GetNum())))))
		{
			i++;
		}
		
		//add entrance(s)
		if ((i - begin) >= MAX_ENTRANCE_WIDTH)
		{
			// create two new entrances, one for each end
			Entrance entrance1(latitude, begin,-1,-1,
												 map->GetNodeNum(begin,latitude),
												 map->GetNodeNum(begin,latitude+1),
												 row, col, 1, HORIZONTAL);
			addEntrance(entrance1);
			Entrance entrance2(latitude, (i - 1),-1,-1,
												 map->GetNodeNum(i-1,latitude),
												 map->GetNodeNum(i-1,latitude+1),
												 row, col, 1, HORIZONTAL);
			addEntrance(entrance2);
		}
		else 
		{
			// create one entrance in the middle 
			Entrance entrance(latitude, ((i - 1) + begin)/2,-1,-1,
												map->GetNodeNum(((i - 1) + begin)/2,latitude),
												map->GetNodeNum(((i - 1) + begin)/2,latitude+1),
												row, col, (i - begin), HORIZONTAL);
			addEntrance(entrance);
		}
		i--;
	}
}

/**
* based on the function from HPA* abstiling.cpp with the same name
 *
 * create vertical entrances 
 */
void ClusterAbstraction::createVertEntrances(int start, int end, int meridian, int row, int col)
{
	Map* map = MapAbstraction::GetMap();
	
	Graph* g = abstractions[0];
	for (int i=start; i<=end; i++)
	{
		
		
		while((i<=end)&& (!GetNodeFromMap(meridian,i) || !GetNodeFromMap(meridian+1,i)
											|| (!(g->FindEdge(GetNodeFromMap(meridian,i)->GetNum(), GetNodeFromMap(meridian+1,i)->GetNum())))))
		{
			i++;
		}
		
		
		
		if (i>end)
			return;
		
		int begin = i;
		
		i++;
		
		while((i<=end) && (GetNodeFromMap(meridian,i)) && (GetNodeFromMap(meridian+1,i))
					&& (g->FindEdge(GetNodeFromMap(meridian,i)->GetNum(), GetNodeFromMap(meridian+1,i)->GetNum()))
					&& ((i==start) || ((g->FindEdge(GetNodeFromMap(meridian,i-1)->GetNum(), GetNodeFromMap(meridian,i)->GetNum()))
														 && (g->FindEdge(GetNodeFromMap(meridian+1,i-1)->GetNum(), GetNodeFromMap(meridian+1,i)->GetNum())))))
		{
			i++;
		}
		
		//add entrance(s)
		if ((i - begin) >= MAX_ENTRANCE_WIDTH)
		{
			// create two entrances, one for each end
			Entrance entrance1(begin, meridian,-1,-1,
												 map->GetNodeNum(meridian,begin),
												 map->GetNodeNum(meridian+1,begin),
												 row, col, 1, VERTICAL);
			addEntrance(entrance1);
			Entrance entrance2((i - 1), meridian,-1,-1,
												 map->GetNodeNum(meridian,i-1),
												 map->GetNodeNum(meridian+1,i-1),
												 row, col, 1, VERTICAL);
			addEntrance(entrance2);
		}
		else
		{
			// create one entrance
			Entrance entrance(((i - 1) + begin)/2, meridian,-1,-1,
												map->GetNodeNum(meridian,((i - 1) + begin)/2),
												map->GetNodeNum(meridian+1,((i - 1) + begin)/2),
												row, col, (i - begin), VERTICAL);
			addEntrance(entrance);
		}
		
		i--;
		
	}
	
	
}

/** 
* from original HPA* abstiling.cpp
*/ 
void ClusterAbstraction::linkEntrancesAndClusters()
{
	if (verbose) std::cout<<"linking entrances and clusters\n";
	
	int cluster1Id;
	int cluster2Id;
	for (unsigned int i = 0; i < entrances.size(); i++)
	{
		
		Entrance &entrance = entrances[i];
		//   std::cout<<"entrance getrow: "<<entrance.getRow()<<" entrance.getcol: "<<entrance.getCol()<<std::endl;
		switch (entrance.getOrientation())
		{
			case HORIZONTAL:
			{
				cluster1Id = getClusterId(entrance.getRow(), entrance.getCol());
				cluster2Id = getClusterId(entrance.getRow() + 1, entrance.getCol());
				
				// update entrance
				entrance.setCluster1Id(cluster1Id);
				entrance.setCluster2Id(cluster2Id);
			}
				break;
			case VERTICAL:
			{
				cluster1Id = getClusterId(entrance.getRow(), entrance.getCol());
				cluster2Id = getClusterId(entrance.getRow(), entrance.getCol() + 1);
				entrance.setCluster1Id(cluster1Id);
				entrance.setCluster2Id(cluster2Id);
			}
				break;
			default:
				assert(false);
				break;
		}
	}
} 

/**
* Add a node for cluster for each entrance
 */ 
void ClusterAbstraction::addAbsNodes(Graph* g)
{
	Map* map = MapAbstraction::GetMap();
	if (verbose) std::cout<<"adding abstract nodes\n";
	int newnode1=-1;
	int newnode2=-1;
	
	recVec ans;
	double r;
	int num=-1;
	
	for (unsigned int i=0; i<entrances.size(); i++)
	{
		
		Entrance &entrance = entrances[i];
		int cluster1Id = entrance.getCluster1Id();
		int cluster2Id = entrance.getCluster2Id();
		switch (entrance.getOrientation())
		{
			case HORIZONTAL:
			{
				// create node for 1st cluster
				map->GetOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row(), ans.x,ans.y,ans.z,r);
				
				Cluster& c1 = getCluster(entrance.getCluster1Id());
				num = nodeExists(c1,ans.x,ans.y,g);
				
				if (num==-1)
				{
					
					node* n1 = new node("");
					newnode1 = g->AddNode(n1);
					n1->SetLabelL(kParent,-1);
					n1->SetLabelL(kAbstractionLevel,1);
					n1->SetLabelF(kXCoordinate,ans.x);
					n1->SetLabelF(kYCoordinate,ans.y);
					n1->SetLabelF(kZCoordinate,-10*r);
					n1->SetLabelL(kNumAbstractedNodes,0);
					c1.AddNode(newnode1);
				}
				else{
					newnode1 = num;
				}
				
				map->GetOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row()+1, ans.x,ans.y,ans.z,r);
				Cluster& c2 = getCluster(cluster2Id);	  
				
				num = nodeExists(c2,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n2= new node("");
					newnode2 = g->AddNode(n2);
					n2->SetLabelL(kParent,-1);
					n2->SetLabelL(kAbstractionLevel,1);
					n2->SetLabelF(kXCoordinate,ans.x);
					n2->SetLabelF(kYCoordinate,ans.y);
					n2->SetLabelF(kZCoordinate,-10*r);
					n2->SetLabelL(kNumAbstractedNodes,0);
					
					c2.AddNode(newnode2);
				}
				else{
					newnode2=num;
				}
				//add inter-edge
				g->AddEdge(new edge(newnode1,newnode2,1));
				
			}
				break;
			case VERTICAL:
			{
				map->GetOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row(), ans.x,ans.y,ans.z,r);
				
				Cluster& c1 = getCluster(cluster1Id);
				num = nodeExists(c1,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n1 = new node("");
					newnode1 = g->AddNode(n1);
					n1->SetLabelL(kParent,-1);
					n1->SetLabelL(kAbstractionLevel,1);
					n1->SetLabelF(kXCoordinate,ans.x);
					n1->SetLabelF(kYCoordinate,ans.y);
					n1->SetLabelF(kZCoordinate,-10*r);
					n1->SetLabelL(kNumAbstractedNodes,0);
					
					c1.AddNode(newnode1);
				}
				else{
					newnode1 = num;
				}
				
				
				map->GetOpenGLCoord(entrance.getCenter1Col()+1, entrance.getCenter1Row(),ans.x,ans.y,ans.z,r);
				
				Cluster& c2 = getCluster(cluster2Id);
				num = nodeExists(c2,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n2 = new node("");
					
					newnode2 = g->AddNode(n2);
					n2->SetLabelL(kParent,-1);
					n2->SetLabelL(kAbstractionLevel,1);
					n2->SetLabelF(kXCoordinate,ans.x);
					n2->SetLabelF(kYCoordinate,ans.y);
					n2->SetLabelF(kZCoordinate,-10*r);
					n2->SetLabelL(kNumAbstractedNodes,0);
					
					c2.AddNode(newnode2);
				}
				else{
					newnode2=num;
				}
				//add inter-edge
				g->AddEdge(new edge(newnode1, newnode2, 1));
				
			}	     
				break;
			default:
				assert(false);
				break;
		}
	}
}

/*
 * Compute the paths inside each cluster. For each pair of entrances inside a cluster, find out if 
 * there is a path that only uses nodes inside the cluster. If there is, add an edge to the abstract
 * Graph with the path distance as its weight and cache the path in the hash map.  
 * 
 * can make this more efficient?
 */
void ClusterAbstraction::computeClusterPaths(Graph* g)
{//int total=0;
	Map* map = MapAbstraction::GetMap();
	if (verbose) std::cout<<"computing cluster paths\n";
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		Cluster& c = clusters[i];
		
		std::vector<node*> corridor; 
		for (int l=0; l<c.GetNumNodes(); l++)
		{
			corridor.push_back(g->GetNode(c.getIthNodeNum(l)));
		}
		for (unsigned int j=0; j < c.parents.size(); j++)
			corridor.push_back(c.parents[j]);
		
		//int num = 0;
		for (int j=0; j<c.GetNumNodes(); j++)
		{
			for (int k=j+1; k<c.GetNumNodes();k++)
			{
				
				// find bottom level nodes
				node* absStart = g->GetNode(c.getIthNodeNum(j));
				node* absGoal = g->GetNode(c.getIthNodeNum(k));
				
				// find start/end coordinates (same in abstract and bottom level)
				double startx = absStart->GetLabelF(kXCoordinate);	
				double starty = absStart->GetLabelF(kYCoordinate);
				double startz = absStart->GetLabelF(kZCoordinate);
				
				double goalx = absGoal->GetLabelF(kXCoordinate);
				double goaly = absGoal->GetLabelF(kYCoordinate);
				double goalz = absGoal->GetLabelF(kZCoordinate);
				
				point3d s(startx,starty,startz);
				point3d gl(goalx,goaly,goalz);
				
				int px;
				int py;
				
				map->GetPointFromCoordinate(s,px,py);
				
				node* start = GetNodeFromMap(px,py);
				
				map->GetPointFromCoordinate(gl,px,py);
				
				node* goal = GetNodeFromMap(px,py);
				
				int startnum = c.getIthNodeNum(j);
				int goalnum = c.getIthNodeNum(k);
				
				//find path
//				corridorAStar astar;
//				astar.setCorridor(&corridor);
//				path* p = astar.GetPath(static_cast<MapAbstraction*>(this), start, goal);

				GenericAStar astar;
				ClusterSearchEnvironment cse(this, GetAbstractionLevel(start));
				cse.setCorridor(corridor);
				std::vector<uint32_t> resultPath;
				astar.GetPath(&cse, start->GetNum(), goal->GetNum(),
											resultPath);
				path *p = 0;
				for (unsigned int x = 0; x < resultPath.size(); x++)
					p = new path(GetAbstractGraph(start)->GetNode(resultPath[x]), p);

				if (p!=0)
				{
					//get its length
					double dist = distance(p); 
					
					//create edge
					edge* newedge = new edge(startnum, goalnum, dist);
					g->AddEdge(newedge);
					
					//store path
					paths[newedge] = p;
				}
			}
		}
	}
}

/**
* given a cluster row and column (NOT map row/column), return the cluster's ID.
 */
int ClusterAbstraction::getClusterId(int row, int col) const
{
	// assert? 
	return row * columns + col;
}

/**
* given a node's coordinates, find the cluster it's in
 */ 
int ClusterAbstraction::getClusterIdFromCoord(int row, int col) const
{
	// put in assert?
	int crow = row / clusterSize;
	int ccol = col / clusterSize; 
	return getClusterId(crow, ccol);
}

/**
* get the cluster from it's ID
 */
Cluster& ClusterAbstraction::getCluster(int id)
{
	assert (0 <= id && id < (int)clusters.size());
	return clusters[id];
}

/**
* check if a node already exists in a certain cluster with a particular set of 
 * coordinates
 */

//this could probably be done more efficiently. Reference to the node in the cluster? 
//(instead of node number) What about storing more efficiently ? hash map inside 
//createabsnodes? 
int ClusterAbstraction::nodeExists(const Cluster& c,double x,double y, Graph* g)
{
	int n=-1;
	for (int i=0; i< c.GetNumNodes(); i++)
	{
		
		n = c.getIthNodeNum(i);
		node* no = g->GetNode(n);
		if ((no->GetLabelF(kXCoordinate)==x) && (no->GetLabelF(kYCoordinate)==y))
		{
			
			return n;
		}
	}
	return -1;
}

/**
* Given a map node, return the ID of the cluster it's in. 
 *
 * Only works for map 
 */
int ClusterAbstraction::getClusterIdFromNode(node* n)
{
	
	Map* map = MapAbstraction::GetMap();
	if (n->GetLabelL(kAbstractionLevel) == 0)
		return getClusterIdFromCoord(n->GetLabelL(kFirstData+1),n->GetLabelL(kFirstData));	
	else{
		point3d s(n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),-1);
		int px;
		int py;
		
		map->GetPointFromCoordinate(s,px,py);
		return getClusterIdFromCoord(py,px);
	}
}


/**
* Nodes are assigned the closest entrance node in the abstract Graph as their parent.
 * Connected components that cannot reach any entrance nodes will be assigned their own
 * parent node. 
 *
 * Connected component code borrowed from MapSectorAbstraction.cpp
 */
void ClusterAbstraction::setUpParents(Graph* g)
{
	
	
	if (verbose)	std::cout<<"Setting up parents\n";
	Map* map = MapAbstraction::GetMap();
	
	std::vector<node*> dummies; 
	int numOrigNodes = g->GetNumNodes();
	
	// create ALL DUMMY PARENTS first
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		
		Cluster& c = clusters[i];
		
		node* dummyParent = new node("");
		dummyParent->SetLabelL(kAbstractionLevel, 1);
		dummyParent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
		dummyParent->SetLabelL(kParent, -1); 
		dummyParent->SetLabelF(kXCoordinate, kUnknownPosition);
		dummyParent->SetLabelL(kNodeBlocked, 0);
		
		
		g->AddNode(dummyParent);
		dummies.push_back(dummyParent);
		
		for (int x=c.getHOrig(); x<c.getHOrig()+c.getWidth(); x++)
		{
			
			for (int y=c.getVOrig(); y<c.getVOrig()+c.GetHeight(); y++)
			{
				
				if (map->GetNodeNum(x,y) >= 0)
				{
					node* n = GetNodeFromMap(x,y);
					buildNodeIntoParent(n, dummyParent);
				}
			}
		}
	}
	
	int numNodesAfter = g->GetNumNodes();
	
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		
		Cluster& c = clusters[i];
		//Create the corridor
		std::vector<node*> corridor; 
		corridor.push_back(dummies[i]);
		
		for (int l=0; l<c.GetNumNodes(); l++)
		{
			corridor.push_back(g->GetNode(c.getIthNodeNum(l)));
		}
		
//		corridorAStar astar;
//		astar.setCorridor(&corridor);	
		
		// 		std::cout<<"corridor has: "<<std::endl;
		// 		for (unsigned int bla = 0; bla<corridor.size(); bla++)
		// 			std::cout<<corridor[bla]<<" ("<<corridor[bla]->GetLabelL(kNumAbstractedNodes)<<") ";
		// 		std::cout<<std::endl;
		
		//Find parent for each node 
		for (int x=c.getHOrig(); x<c.getHOrig()+c.getWidth(); x++)
		{
			for (int y=c.getVOrig(); y<c.getVOrig()+c.GetHeight(); y++)
			{
				if (map->GetNodeNum(x,y) >= 0)
				{
					node* mnode = GetNodeFromMap(x,y);
					
					// reset minimum 
					double minDist = DBL_MAX;
					node* entrance = 0;
					
					//for every abstract (entrance node) in this cluster
					for (int k=0; k<c.GetNumNodes(); k++)
					{
						//get the entrance
						int nodenum = c.getIthNodeNum(k);
						
						node* n = g->GetNode(nodenum);						
						node* low = getLowLevelNode(n);	
						
						if (low==mnode)
						{
							entrance = n;
							break;
						}
						
						//See if there's a path within this cluster
//						astar.setCorridor(&corridor);
//						path* p = astar.GetPath(static_cast<MapAbstraction*>(this),low,mnode);
						GenericAStar astar;
						ClusterSearchEnvironment cse(this, GetAbstractionLevel(low));
						cse.setCorridor(corridor);
						std::vector<uint32_t> resultPath;
						astar.GetPath(&cse, low->GetNum(), mnode->GetNum(),
													resultPath);
						path *p = 0;
						for (unsigned int t = 0; t < resultPath.size(); t++)
							p = new path(GetAbstractGraph(low)->GetNode(resultPath[t]), p);
						
						if (p!=0)
						{
							// 								std::cout<<"corridor has: "<<std::endl;
							// 								for (unsigned int bla = 0; bla<corridor.size(); bla++)
							// 									std::cout<<corridor[bla]<<" ("<<corridor[bla]->GetLabelL(kNumAbstractedNodes)<<") ";
							// 								std::cout<<std::endl;
							// 							}
							
							// find distance to this point 
							point3d s(n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),-1);
							int px;
							int py;
							map->GetPointFromCoordinate(s,px,py);
							
							// calculate the distance to this entrance
							
							double dist = distance(p);
							
							if (dist<minDist)
							{
								
								minDist=dist;  
								entrance=n;
								
							}
							delete p;
					}
				}
					
					//std::cout<<entrance<<std::endl;
					if (entrance)
					{
						
						point3d s(entrance->GetLabelF(kXCoordinate),entrance->GetLabelF(kYCoordinate),-1);
						int px;
						int py;
						map->GetPointFromCoordinate(s,px,py);
						buildNodeIntoParent(mnode, entrance);
					}
					//					else
					//std::cout<<" no parent\n";
			}//end if (not out of bounds)
		}    
	}
}
// 	for (unsigned int i=0;i<dummies.size(); i++){
// 		// make sure no node has a dummy for a parent
// 		for (int j=0; j<dummies[i]->GetLabelL(kNumAbstractedNodes); j++){
// 			node* child = abstractions[0]->GetNode(dummies[i]->GetLabelL(kFirstData+j));
// 			if (dummies[i]->GetNum()==5289)
// 				std::cout<<"going to check\n";
// 			if (child->GetLabelL(kParent) == dummies[i]->GetNum()){
// 				if (dummies[i]->GetNum()==5289)
// 					std::cout<<"removing "<<child->GetNum()<<std::endl;
// 				child->SetLabelL(kParent,-1);
// 			}
// 		}

// 		g->RemoveNode(dummies[i]);
// 		delete dummies[i];
// 	}

//finish by giving not-yet-abstracted nodes a parent

node_iterator ni = abstractions[0]->getNodeIter();
for (node *next = abstractions[0]->nodeIterNext(ni); next;
		 next = abstractions[0]->nodeIterNext(ni))
{
	// if it isn't abstracted, do a bfs according to the cluster and abstract these nodes together
	if ((next->GetLabelL(kParent) == -1) || ((next->GetLabelL(kParent) >= numOrigNodes) && next->GetLabelL(kParent) < numNodesAfter))
	{
		node *parent;
		g->AddNode(parent = new node("??"));
		parent->SetLabelL(kAbstractionLevel, next->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
		parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
		parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
		parent->SetLabelF(kXCoordinate, kUnknownPosition);
		parent->SetLabelL(kNodeBlocked, 0);
		abstractionBFS(next, parent, getClusterIdFromNode(next),numOrigNodes,numNodesAfter);
	}
}
for (unsigned int i=dummies.size()-1;i>0; --i)
{
	
	
	int num = dummies[i]->GetNum();
	
	
	g->RemoveNode(dummies[i]);
	node* n = g->GetNode(num); 
	
	if (n)
	{
		
		//update children
		int numnodes = n->GetLabelL(kNumAbstractedNodes);
		for (int j=0; j<numnodes; j++)
		{
			
			(abstractions[0]->GetNode(n->GetLabelL(kFirstData+j)))->SetLabelL(kParent, num);
		}
	}
	
	delete dummies[i];
}
}

/**
* 'borrowed' from MapSectorAbstraction.cpp
 */
void ClusterAbstraction::abstractionBFS(node *which, node *parent, int cluster, int numOrigNodes, int numNodesAfter)
{
	if ((which == 0) || (getClusterIdFromNode(which) != cluster) ||
			((which->GetLabelL(kParent) < numOrigNodes) && (which->GetLabelL(kParent>=0))) ||
			((which->GetLabelL(kParent) >= numNodesAfter) && (which->GetLabelL(kParent)>=0)))
		return;
	
	buildNodeIntoParent(which, parent);
	
	getCluster(cluster).addParent(parent);
	
	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		abstractionBFS(abstractions[0]->GetNode(tmp), parent, cluster,numOrigNodes,numNodesAfter);
	}
}

/**
* Add a 2nd level to the Graph. If two nodes in the abstract (level 1) Graph have the 
 * same parent, there is a path between them. 
 */
void ClusterAbstraction::createConnectivityGraph()
{
	// 	std::cout<<" begin\n";
	// 	abstractions[1]->Print(std::cout);
	// 	std::cout<<"end of this\n";
	Graph* g = new Graph; 
	abstractions.push_back(g); 
	
	node_iterator ni = abstractions[1]->getNodeIter();
	for (node *next = abstractions[1]->nodeIterNext(ni); next;
			 next = abstractions[1]->nodeIterNext(ni))
	{
		//		std::cout<<"looking at "<<next->GetNum()<<std::endl;
		//std::cout<<"parent: "<<next->GetLabelL(kParent)<<std::endl;
		
		// if it isn't abstracted, do a bfs according to the cluster and abstract these nodes together
		if (next->GetLabelL(kParent) == -1)
		{
			//		std::cout<<"here inside if\n";
			node *parent;
			g->AddNode(parent = new node("??"));
			parent->SetLabelL(kAbstractionLevel, next->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->SetLabelF(kXCoordinate, kUnknownPosition);
			parent->SetLabelL(kNodeBlocked, 0);
			connectedBFS(next, parent);
		}
	}
	//	g->Print(std::cout);
}

void ClusterAbstraction::connectedBFS(node *which, node *parent)
{
	if ((which == 0) || (which->GetLabelL(kParent) != -1))
	{
		
		return;
	}
	buildNodeIntoParent(which, parent);
	
	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		connectedBFS(abstractions[1]->GetNode(tmp), parent);
	}
}

/**
* Check if there is a path between two nodes
 */
bool ClusterAbstraction::Pathable(node* from, node* to)
{
	node* fParent = abstractions[1]->GetNode(from->GetLabelL(kParent));
	node* tParent = abstractions[1]->GetNode(to->GetLabelL(kParent));
	
	if (fParent->GetLabelL(kParent) == tParent->GetLabelL(kParent))
		return true;
	else
		return false;
}


/*
 * 'borrowed' from SectorAbstraction.cpp 
 */ 
void ClusterAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert((n->GetLabelL(kAbstractionLevel)+1 == parent->GetLabelL(kAbstractionLevel))&& parent);
	
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
}

/**
* insert a start or goal node in to the abstract Graph (for searching purposes). 
 * The node is inserted in the same location as it is in on the map-Graph. Edges are
 * added between this node and all entrance nodes in its cluster when there is a path 
 * between the nodes. The weight of the edge is set to the distance of the map-level
 * path 
 */
node* ClusterAbstraction::insertNode(node* n, int& expanded, int& touched)
{
	
	expanded = 0;
	touched = 0;
	
	if (verbose)std::cout<<"in insert node\n";
	Map* map = MapAbstraction::GetMap();
	Graph* g = abstractions[1];
	
	point3d s(n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),-1);
	int px;
	int py;
	
	map->GetPointFromCoordinate(s,px,py);
	
	// 	Cluster& c = getCluster(getClusterIdFromCoord(py,px));
	Cluster& c = getCluster(getClusterIdFromCoord(n->GetLabelL(kFirstData+1), n->GetLabelL(kFirstData)));
	//	std::cout<<"cluster: "<<&c<<std::endl;
	//	std::cout<<"id: "<<getClusterIdFromCoord(py,px)<<std::endl;
	// 		for (int l=0; l<c.GetNumNodes(); l++){
	// 			std::cout<<g->GetNode(c.getIthNodeNum(l))<<std::endl;
	// 			//	corridor.push_back(g->GetNode(c.getIthNodeNum(l)));
	// 		}
	recVec ans;
	double r;
	map->GetOpenGLCoord((int)n->GetLabelL(kFirstData),(int)n->GetLabelL(kFirstData+1) , ans.x,ans.y,ans.z,r);
	
	//	std::cout<<"cluster num: "<<getClusterIdFromCoord(py,px)<<std::endl;
	//	std::cout<<"level "<<n->GetLabelL(kAbstractionLevel)<<std::endl;
	//	int nnum = nodeExists(c,n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),g);
	
	int nnum = nodeExists(c,ans.x,ans.y,g);
	
	//		std::cout<<"nnum is "<<nnum<<std::endl;
	if (nnum==-1)
	{
		node* newnode = new node("");
		int nodenum = g->AddNode(newnode);
		newnode->SetLabelF(kXCoordinate, n->GetLabelF(kXCoordinate));
		newnode->SetLabelF(kYCoordinate, n->GetLabelF(kYCoordinate));
		newnode->SetLabelF(kZCoordinate, n->GetLabelF(kZCoordinate)*2); //should never have to draw this so it doesn't matter
		newnode->SetLabelL(kParent,-1); 
		newnode->SetLabelL(kNumAbstractedNodes,0);
		newnode->SetLabelL(kAbstractionLevel,1);

		//  		std::vector<node*> corridor; 
		//  		for (int l=0; l<c.GetNumNodes(); l++){
		//  			std::cout<<g->GetNode(c.getIthNodeNum(l))<<std::endl;
		//  			corridor.push_back(g->GetNode(c.getIthNodeNum(l)));
		//  		}
		
		//		std::cout<<"nodes: "<<c.GetNumNodes()<<std::endl;
		for (int k=0; k<c.GetNumNodes(); k++)
		{
			//get the entrance
			int num = c.getIthNodeNum(k);
			
			node* tempgoal = g->GetNode(num);
			
			double goalx = tempgoal->GetLabelF(kXCoordinate);
			double goaly = tempgoal->GetLabelF(kYCoordinate);
			double goalz = tempgoal->GetLabelF(kZCoordinate);
			
			point3d gl(goalx,goaly,goalz);
			
			map->GetPointFromCoordinate(gl,px,py);
			node* goal = GetNodeFromMap(px,py);
			
			//  		std::cout<<"corridor has: "<<std::endl;
			//  		for (unsigned int bla = 0; bla<corridor.size(); bla++){
			// 			printMapCoord(corridor[bla]);
			// 		std::cout<<" "<<corridor[bla]<<" ("<<corridor[bla]->GetLabelL(kNumAbstractedNodes)<<") ";
			// 		}
			//  		std::cout<<std::endl;
			
			std::vector<node*> corridor;
			for (unsigned int i=0; i<c.parents.size(); i++)
				corridor.push_back(c.parents[i]);
			for (int j=0; j<c.GetNumNodes(); j++)
				corridor.push_back(g->GetNode(c.getIthNodeNum(j)));
			
			//find path
//			corridorAStar astar;
//			astar.setCorridor(&corridor);
//			path* p = astar.GetPath(static_cast<MapAbstraction*>(this),n , goal);
			GenericAStar astar;
			ClusterSearchEnvironment cse(this, GetAbstractionLevel(n));
			cse.setCorridor(corridor);
			std::vector<uint32_t> resultPath;
			astar.GetPath(&cse, n->GetNum(), goal->GetNum(),
										resultPath);
			path *p = 0;
			for (unsigned int x = 0; x < resultPath.size(); x++)
				p = new path(GetAbstractGraph(n)->GetNode(resultPath[x]), p);
			
			expanded += astar.GetNodesExpanded();
			touched += astar.GetNodesTouched();
			
			if (p!=0)
			{
				
				//get its length
				double dist = distance(p);
				//create edge
				edge* newedge = new edge(nodenum,num,dist);
				if (newnode->GetLabelL(kParent)==-1)
				{
					
					newnode->SetLabelL(kParent,(g->GetNode(num))->GetLabelL(kParent));
				}
				
				g->AddEdge(newedge);
				
				//store path
				path* p2 = p;
				temp[newedge] = p;
				
				newPaths.push_back(p2);
			}
		}
		
		if (newnode->GetLabelL(kParent)==-1)
		{
			
			// This node has its own parent in the connectivity Graph
			node* par = new node("");
			abstractions[2]->AddNode(par);
			par->SetLabelF(kXCoordinate, kUnknownPosition);
			par->SetLabelL(kParent,-1); 
			par->SetLabelL(kNumAbstractedNodes,0);
			par->SetLabelL(kAbstractionLevel,2);		
			
			buildNodeIntoParent(newnode,par);
		}
		
		
		//		std::cout<<"inserted\n";
		return newnode;
	}
	else{
		//		std::cout<<"no need to insert\n";
		return g->GetNode(nnum);
	}
	
}



/**
* remove a start or goal node after the search has been completed. The node is
 * removed, as well as all the edges that were added to connect it to the cluster. 
 * The edges are also removed from the cache.
 */ 
void ClusterAbstraction::removeNodes(node* start, node* goal)
{
	if (verbose) std::cout<<"in remove node\n";
	
	Map* map = MapAbstraction::GetMap();
	
	// Check cluster if it's one of the original nodes
	// Only delete if the node is not part of the original abstraction
	point3d s(start->GetLabelF(kXCoordinate),start->GetLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->GetPointFromCoordinate(s,px,py);
	
	Cluster& c = getCluster(getClusterIdFromCoord(py,px));
	Graph* g = abstractions[1];
	int num = nodeExists(c,start->GetLabelF(kXCoordinate),start->GetLabelF(kYCoordinate),g);
	if (num==-1)
	{
		
		if (start->GetNumEdges() == 0)
		{
			
			abstractions[2]->RemoveNode(start->GetLabelL(kParent));
		}
		edge_iterator ei = start->getEdgeIter();
		edge* e =start->edgeIterNext(ei); 
		
		while(e)
		{
			
			temp.erase(e);
			g->RemoveEdge(e);
			delete e;
			ei = start->getEdgeIter();
			e = start->edgeIterNext(ei);
		}
		int snum = start->GetNum();
		assert((snum == g->GetNumNodes()-1) || (snum == g->GetNumNodes()-2));
		g->RemoveNode(start);
		delete start;
		//	std::cout<<"removing node\n";
	}
	//	else
	//	std::cout<<"don't need to remove this\n";
	
	point3d gl(goal->GetLabelF(kXCoordinate),goal->GetLabelF(kYCoordinate),-1);
	int px2;
	int py2;
	map->GetPointFromCoordinate(gl,px2,py2);
	
	Cluster& c2 = getCluster(getClusterIdFromCoord(py2,px2));
	num = nodeExists(c2,goal->GetLabelF(kXCoordinate),goal->GetLabelF(kYCoordinate),g);
	if (num==-1)
	{
		
		if (goal->GetNumEdges() == 0)
		{
			
			abstractions[2]->RemoveNode(goal->GetLabelL(kParent));
			
		}
		edge_iterator ei = goal->getEdgeIter();
		//				std::cout<<"goal->GetNumEdges "<<goal->GetNumEdges()<<std::endl;
		edge* e = goal->edgeIterNext(ei); 
		while(e)
		{
			
			temp.erase(e);
			int gnum = goal->GetNum();
			assert((gnum == g->GetNumNodes()-1) || (gnum == g->GetNumNodes()-2));
			g->RemoveEdge(e);
			delete e;
			ei = goal->getEdgeIter();
			e = goal->edgeIterNext(ei);
		}
		
		g->RemoveNode(goal);
		delete goal;
		//		std::cout<<"removing goal node\n";
	}
	//	else std::cout<<"don't need to remove goal\n";
	
	temp.clear();
	for (unsigned int i=0; i<newPaths.size(); i++)
	{
		
		delete newPaths[i];
	}
	newPaths.clear();
}

/**
* get the map-level path that corresponds to abstract edge e
 */ 
path* ClusterAbstraction::getCachedPath(edge* e) 
{
	if (temp[e]) return temp[e]->Clone();
	else if (paths[e]) return paths[e]->Clone();
	else return 0;
}

/**
* get the map-level node that is at the (x,y) coordinates of the abstract
 * node
 */
node* ClusterAbstraction::getLowLevelNode(node* abstract) 
{
	Map* map = MapAbstraction::GetMap();
	
	point3d s(abstract->GetLabelF(kXCoordinate),abstract->GetLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->GetPointFromCoordinate(s,px,py);
	
	return MapAbstraction::GetNodeFromMap(px,py);
}

void ClusterAbstraction::printMapCoord(node* n)
{
	
	Map* map = MapAbstraction::GetMap();
	Graph* g = abstractions[1];
	point3d s(n->GetLabelF(kXCoordinate),n->GetLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->GetPointFromCoordinate(s,px,py);
	
	node* parent = g->GetNode(n->GetLabelL(kParent));
	
	point3d s2(parent->GetLabelF(kXCoordinate),parent->GetLabelF(kYCoordinate),-1);
	int px2;
	int py2;
	map->GetPointFromCoordinate(s2,px2,py2);
	
	std::cout<<"("<<px<<","<<py<<")"; // parent "<<parent<<"\n";
																		//("<<px2<<","<<py2<<")\n";
}

void ClusterAbstraction::printPathAsCoord(path* p)
{
	if (p->n)
	{
		
		path* curr = p;
		printMapCoord(curr->n);
		while(curr->next)
		{
			
			curr = curr->next;
			printMapCoord(curr->n);
		}
	}
}

void ClusterAbstraction::OpenGLDraw() const
{
	MapAbstraction::OpenGLDraw();
	GLdouble xx, yy, zz, rr;
	glColor3f(0.25, 0.0, 0.75);
	Map *map = GetMap();
	map->GetOpenGLCoord(0, 0, xx, yy, zz, rr);
	glBegin(GL_LINES);
	int numXSectors = (map->GetMapWidth()+clusterSize-1)/clusterSize;
	int numYSectors = (map->GetMapHeight()+clusterSize-1)/clusterSize;
	for (int y = 0; y <= numYSectors; y++)
	{
		glVertex3f(xx-rr, yy-rr+2*y*rr*clusterSize, zz-5*rr);
		glVertex3f(xx+2*numXSectors*rr*clusterSize, yy-rr+2*y*clusterSize*rr, zz-5*rr);
	}
	for (int x = 0; x <= numXSectors; x++)
	{
		glVertex3f(xx-rr+2*x*rr*clusterSize, yy-rr, zz-5*rr);
		glVertex3f(xx-rr+2*x*rr*clusterSize, yy-rr+2*numYSectors*rr*clusterSize, zz-5*rr);
	}
	glEnd();		
}
