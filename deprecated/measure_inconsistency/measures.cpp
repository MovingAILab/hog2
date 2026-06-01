#include <cstdio>
#include <cstring>
#include <vector>
#include <time.h>
#include <unordered_map>
#include <math.h>
#include "FPUtil.h"
#include "UnitSimulation.h"
#include "GraphEnvironment.h"
#include "MapAbstraction.h"
#include "Propagation.h"

using namespace std;
using namespace __gnu_cxx;
using namespace PropUtil;

class SimpleNode {
public:
	SimpleNode() 
	{
		depth = 0;
		me = 0;
		parent = 0; 
	}
	SimpleNode(graphState m, graphState p, int d) 
	{
		depth = d;
		me = m;
		parent = p;
	}

	graphState parent;
	graphState me;
	int depth;
};


double IRE(Graph* g, GraphEnvironment* env, graphState goal, vector<double>& ires);
double IRN(Graph* g, vector<double>& ires, vector<double>& irns);
void processArgs(int argc, char* argv[]);
void DFSVisit(GraphEnvironment* env, vector<SimpleNode> &thePath, int radius, hash_map<int,int>& pathcounts, hash_map<int,vector<double> > &pathcosts, double gval);
void DIJKStat(Graph* g, GraphEnvironment* env, graphState to, vector<double>& dijkedges, hash_map<int,int>& edgedistrib, int& total, int& negative);
void Dijkstra(GraphEnvironment* env, graphState from, graphState to, hash_map<int,double>& distance);
void LocalAStar(GraphEnvironment* env, graphState from, graphState to, int& totalExpansion, int& reopening);


#define IREN 0
#define TRAN 1
#define DIJK 2
#define CORRE 3
#define LOCAL 4
#define ALL  5


int MEASURE = IREN;
char mfname[50] = {0};
int fig = 0;
int N = 20;
int D = 5;
int samples = 50;
int EX = 200;

/*
Measures:
1. IRE/IRN  (G fixed)
2. transpositions  (none fixed)
3. distribution of edge vals in Dijkstra  (G fixed)
4. correlation of h & dist  (G fixed)
5. run A* locally, count reopenings  (G fixed)
*/

int main(int argc, char* argv[])
{
	processArgs(argc, argv);

	Graph* g = 0;
	Map* mp = 0;
	GraphEnvironment* env = 0;

	mp = new Map(mfname);
	g = GraphSearchConstants::GetGraph(mp);
	env = new GraphEnvironment(g, new GraphMapInconsistentHeuristic(mp, g));

	printf("Total Number of Nodes=%d\n",g->GetNumNodes());

	if(MEASURE == IREN || MEASURE == ALL)
	{
		printf("\nMeasure IRE/IRN\n");

		graphState from = g->GetRandomNode()->GetNum();
		graphState to = g->GetRandomNode()->GetNum();

		vector<double> ires,irns;
		double ire = IRE(g,env,to,ires);
		double irn = IRN(g,ires,irns);

		printf("Goal=%ld\n",to);
		printf("\nire=%lf, irn=%lf\n",ire,irn);
	}
	if(MEASURE == TRAN || MEASURE == ALL)
	{ // using recursive version of DFS
		printf("\nMeasure TRAN\n");

		srand(time(0));

		vector<SimpleNode> thePath;
		hash_map<int,int> pathcounts;
		hash_map<int,vector<double> > pathcosts;
		hash_map<int,int> pathstats;  // for histogram: distribution of nodes with different numbers of paths
		hash_map<int,int> pathcoststats;  // for histogram: distribution of nodes with different numbers of distinct g-costs
		double pathssum = 0;
		double pathcostsum = 0;
		int nodes = 0;

		for(unsigned int s=0; s<samples; s++)
		{
			thePath.clear();
			pathcounts.clear();
			pathcosts.clear();

			graphState from = g->GetRandomNode()->GetNum();
			SimpleNode n0(from,from,0);
			thePath.push_back(n0);

			DFSVisit(env, thePath, D, pathcounts, pathcosts, 0);

			for(hash_map<int,int>::iterator iter=pathcounts.begin(); iter!=pathcounts.end(); iter++)
			{
				pathssum += iter->second;
				nodes++;
				pathstats[iter->second]++;
			}

			for(hash_map<int,vector<double> >::iterator iter=pathcosts.begin(); iter!=pathcosts.end(); iter++)
			{
				// use insertion sort to sort it
				for(unsigned int is=1;is<(iter->second).size();is++)
				{
					double tmp = (iter->second)[is];
					int js;
					for(js=is-1;js>=0;js--)
					{
						if(fgreater((iter->second)[js],tmp)) {
							(iter->second)[js+1] = (iter->second)[js];
						}
						else {
							break;
						}
					}
					(iter->second)[js+1] = tmp;
				}

				// remove duplicates
				for(vector<double>::iterator it=iter->second.begin();;)
				{
					if(it+1 != iter->second.end()) {
						if(fequal(*it,*(it+1))) {
							it = iter->second.erase(it);
						}
						else
							it++;
					}
					else
						break;
				}

				pathcostsum += iter->second.size();
				pathcoststats[iter->second.size()]++;
			}
		}

		printf("\nAverage distinct paths=%lf, average num of distinct g-costs=%lf, D=%d, samples=%d\n",pathssum/nodes,pathcostsum/nodes,D,samples);

		printf("Distinct paths:\n");
		for(hash_map<int,int>::iterator iter=pathstats.begin();iter!=pathstats.end();iter++)
		{
			printf("[%d,%d];",iter->first,iter->second);
		}

		printf("\nDistinct g:\n");
		for(hash_map<int,int>::iterator iter=pathcoststats.begin();iter!=pathcoststats.end();iter++)
		{
			printf("[%d,%d];",iter->first,iter->second);
		}
		printf("\n");
	}

	if(MEASURE == DIJK || MEASURE == ALL)
	{
		printf("\nMeasure DIJK\n");

		graphState from = g->GetRandomNode()->GetNum();
		graphState to = g->GetRandomNode()->GetNum();

		vector<double> dijkedges;
		hash_map<int,int> edgedistrib;
		int total, negative;

		DIJKStat(g,env,to,dijkedges,edgedistrib,total,negative);

		printf("Goal=%ld\n",to);
		printf("Edge distribution:\n");
		for(hash_map<int,int>::iterator iter=edgedistrib.begin();iter!=edgedistrib.end();iter++)
		{
			printf("[%d,%d];",iter->first,iter->second);
		}
		printf("\nTotal edges=%d, negative edges=%d, ration=%lf\n",total,negative,(double)negative/total);
	}

	if(MEASURE == CORRE || MEASURE == ALL)
	{
		printf("\nMeasure CORRE\n");

		graphState from = g->GetRandomNode()->GetNum();
		graphState to = g->GetRandomNode()->GetNum();

		hash_map<int,double> H_to_dist;  // accumulated h for each (int) value of dist (index is dist)
		hash_map<int,int> num_to_dist;   // number of nodes for each value of dist  (index is dist)

		for(int i=0;i<samples;i++)
		{
			from = g->GetRandomNode()->GetNum();

			hash_map<int,double> distance;  // the index is nodeid
			Dijkstra(env,from,to,distance);

			// categorize the values according to dist
			for(hash_map<int,double>::iterator iter=distance.begin();iter!=distance.end();iter++)
			{
				graphState st = iter->first;
				H_to_dist[(int)iter->second] += env->HCost(st,to);
				num_to_dist[(int)iter->second]++;
			}
		}

		// print it out
		printf("Goal=%ld\n",to);
		printf("Dist->H distribution, EX=%d, samples=%d:\n",EX,samples);
		for(hash_map<int,double>::iterator iter=H_to_dist.begin();iter!=H_to_dist.end();iter++)
		{
			printf("[%d,%lf];",iter->first,iter->second/(num_to_dist[iter->first]));
		}
		printf("\n");
	}

	if(MEASURE == LOCAL || MEASURE == ALL)
	{
		printf("\nMeasure LOCAL A*\n");

		graphState from = g->GetRandomNode()->GetNum();
		graphState to = g->GetRandomNode()->GetNum();

		int totalExpansion = 0;
		int reopening = 0;

		for(int i=0;i<samples;i++)
		{
			from = g->GetRandomNode()->GetNum();

			LocalAStar(env,from,to,totalExpansion,reopening); // expansion & reopening will be accumulated here, no reset
		}

		printf("Goal=%ld\n",to);
		printf("Total expansions=%d, reopenings=%d, ration=%lf, EX=%d, samples=%d\n",totalExpansion,reopening,(double)reopening/totalExpansion,EX,samples);
	}

	return 0;
}

double IRE(Graph* g, GraphEnvironment* env, graphState goal, vector<double>& ires)
{
	edge_iterator eiter =  g->getEdgeIter();
	edge* e;
	double sum = 0;
	while(e = g->edgeIterNext(eiter))
	{
		graphState from = (graphState)e->getFrom();
		graphState to = (graphState)e->getTo();
		ires.push_back(abs(env->HCost(from, goal) - env->HCost(to, goal)));
		sum += ires.back();
	}

	return sum / ires.size();
}

/* defined on undirected graphs only */
double IRN(Graph* g, vector<double>& ires, vector<double>& irns)
{
	double sum = 0;
	node_iterator niter = g->getNodeIter();
	node* n;
	while( n = g->nodeIterNext(niter))
	{
		edge_iterator eiter = n->getEdgeIter();
		edge* e;
		double mx = 0;
		while(e = n->edgeIterNext(eiter))
		{
			mx = max(mx, ires[e->getEdgeNum()]);
		}
		irns.push_back(mx);
		sum += irns.back();
	}

	return sum / irns.size();
}

void processArgs(int argc, char* argv[])
{
	// -map filename -M measure -fig figure -N n -D depth -s samples 

	mfname[0] = 0;
	int i = 1;
	while(i<argc) 
	{
		if(strcmp(argv[i],"-map")==0) 
		{
			strncpy(mfname, argv[i+1], 50);
			i += 2;
		}
		else if(strcmp(argv[i],"-M")==0) 
		{
			if(strcmp(argv[i+1],"IREN")==0)
				MEASURE = IREN;
			else if(strcmp(argv[i+1],"TRAN") == 0)
				MEASURE = TRAN;
			else if(strcmp(argv[i+1],"DIJK") == 0)
				MEASURE = DIJK;
			else if(strcmp(argv[i+1],"CORRE") == 0)
				MEASURE = CORRE;
			else if(strcmp(argv[i+1],"LOCAL") == 0)
				MEASURE = LOCAL;
			else if(strcmp(argv[i+1],"ALL") == 0)
				MEASURE = ALL;
			i += 2;
		}
		else if(strcmp(argv[i],"-D")==0)
		{
			D = max(atoi(argv[i+1]),1);
			i += 2; 
		}
		else if(strcmp(argv[i],"-N")==0)
		{
			N = max(atoi(argv[i+1]),5);
			i += 2;
		}
		else if(strcmp(argv[i],"-fig")==0)
		{
			fig = max(atoi(argv[i+1]),0);
			i += 2;
		}
		else if(strcmp(argv[i],"-s")==0)
		{
			samples = max(atoi(argv[i+1]),1);
			i += 2;
		}
		else if(strcmp(argv[i],"-EX")==0)
		{
			EX = max(atoi(argv[i+1]),20);
			i += 2;
		}
		else 
		{
			i++;
		}
	}
	
	if(strlen(mfname)) 
	{
		FILE * mf = fopen(mfname,"r");
		if(!mf)
		{
			printf("Map file not exists.\n");
			exit(-1);
		}
		else
		{
			fclose(mf);
		}
	}
}

void DFSVisit(GraphEnvironment* env, vector<SimpleNode> &thePath, int radius, hash_map<int,int>& pathcounts, hash_map<int,vector<double> > &pathcosts, double gval)
{
	vector<graphState> neighbors;

	SimpleNode current = thePath.back();
	if(current.depth >= radius)
		return;

	env->GetSuccessors(current.me, neighbors);

	for(unsigned int x=0; x<neighbors.size(); x++)
	{
		graphState neighbor = neighbors[x];
		if(neighbor == current.parent)
			continue;

		bool loop = false;
		vector<SimpleNode>::iterator iter;
		for(iter=thePath.begin();iter!=thePath.end();iter++)
		{
			if(neighbor == iter->me) {
				loop = true;  // this neighbor is on the path
				break;
			}
		}

		if(loop)
			continue;

		pathcounts[neighbor]++;
		double newg = gval + env->GCost(current.me,neighbor);
		pathcosts[neighbor].push_back(newg);

		SimpleNode sn(neighbor, current.me, current.depth+1);
		thePath.push_back(sn);
		DFSVisit(env, thePath, radius, pathcounts, pathcosts, newg);  // recursion
		thePath.pop_back();
	}
}

void DIJKStat(Graph* g, GraphEnvironment* env, graphState goal, vector<double>& dijkedges, hash_map<int,int>& edgedistrib, int& total, int& negative)
{
	edge_iterator eiter =  g->getEdgeIter();
	edge* e;
	total = 0;
	negative = 0;
	while(e = g->edgeIterNext(eiter))
	{
		graphState from = (graphState)e->getFrom();
		graphState to = (graphState)e->getTo();
		double dijkEdge = env->GCost(from,to) - abs(env->HCost(from, goal) - env->HCost(to, goal));
		dijkedges.push_back(dijkEdge);
		edgedistrib[(int)dijkEdge]++;

		total++;
		if(fless(dijkEdge , 0))
			negative++;
	}
}

void Dijkstra(GraphEnvironment* env, graphState from, graphState to, hash_map<int,double>& distance)
{
	GQueue OpenList;
	NodeLookupTable ClosedList;
	int nodesExpanded = 0;

	SearchNode start(0,0,from,from);
	OpenList.Add(start);

	while(nodesExpanded < EX)
	{
		if(OpenList.size()==0)
			break;

		nodesExpanded++;
		
		SearchNode topNode = OpenList.Remove();
		graphState topNodeID = topNode.currNode;
		ClosedList[topNodeID] = topNode;

		distance[topNodeID] = topNode.gCost; // record what we need

		if(topNodeID == to)
		{
			break;
		}

		vector<graphState> neighbors;
		env->GetSuccessors(topNodeID, neighbors);

		for(unsigned int x=0;x<neighbors.size();x++)
		{
			graphState neighbor = neighbors[x];
			double edgeWeight = env->GCost(topNodeID,neighbor);
			double g = topNode.gCost + edgeWeight;
			double h = env->HCost(neighbor,to); 
			double f = g + h; // in principle this is not needed, but GGreater uses it for tie breaking

			SearchNode sn = OpenList.find(SearchNode(neighbor));
			if(sn.currNode == neighbor)  // in OPEN
			{
				if(fgreater(sn.gCost, g))
				{
					sn.copy(f,g,neighbor,topNodeID);
					OpenList.DecreaseKey(sn);
				}
			}
			else if(ClosedList.find(neighbor) != ClosedList.end()) {  // in CLOSED
			}
			else  // new node
			{
				sn.copy(f,g,neighbor,topNodeID);
				OpenList.Add(sn);
			}
		}
	}
}

void LocalAStar(GraphEnvironment* env, graphState from, graphState to, int& totalExpansion, int& reopening)
{
	PQueue OpenList;
	NodeLookupTable ClosedList;
	int nodesExpanded = 0;
	
	SearchNode start(0,0,from,from);
	OpenList.Add(start);

	while(nodesExpanded < EX)
	{
		if(OpenList.size()==0)
			break;

		nodesExpanded++;

		SearchNode topNode = OpenList.Remove();
		graphState topNodeID = topNode.currNode;
		ClosedList[topNodeID] = topNode;

		if(topNodeID == to)
			break;

		vector<graphState> neighbors;
		env->GetSuccessors(topNodeID, neighbors);

		for(unsigned int x=0;x<neighbors.size();x++)
		{
			graphState neighbor = neighbors[x];
			double edgeWeight = env->GCost(topNodeID,neighbor);
			double g = topNode.gCost + edgeWeight;
			double h = env->HCost(neighbor,to); 
			double f = g + h; // in principle this is not needed, but GGreater uses it for tie breaking

			SearchNode sn = OpenList.find(SearchNode(neighbor));
			NodeLookupTable::iterator iter;
			if(sn.currNode == neighbor)  // in OPEN
			{
				if(fgreater(sn.gCost, g))
				{
					sn.copy(f,g,neighbor,topNodeID);
					OpenList.DecreaseKey(sn);
				}
			}
			else if((iter = ClosedList.find(neighbor)) != ClosedList.end()) {  // in CLOSED
				sn = iter->second;
				if(fgreater(sn.gCost,g))
				{
					sn.copy(f,g,neighbor,topNodeID);
					ClosedList.erase(neighbor);

					reopening++;  // passed from the parameter
					OpenList.Add(sn);
				}
			}
			else  // new node
			{
				sn.copy(f,g,neighbor,topNodeID);
				OpenList.Add(sn);
			}
		}
	}

	totalExpansion += nodesExpanded;
}
