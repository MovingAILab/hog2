/*
 *  GraphCanonicalHeuristic.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GraphCanonicalHeuristic.h"
#include "TemplateAStar.h"
#include "NodeLimitAbstraction.h"
#include "MapLineAbstraction.h"

GraphCanonicalHeuristic::GraphCanonicalHeuristic(Map *m, int numClosest, int sizeFactor)
//GraphCanonicalHeuristic::GraphCanonicalHeuristic(Map *m, int DBSize)
:lengths(), centers(), centerDist(), map(m), g(0), mo(0)
{
//	if (DBSize == -1)
//	{
//		DBSize = (int)pow(m->GetMapWidth()*m->GetMapHeight(), 0.25)/2;
//		printf("Dynamically setting size to %d\n", DBSize);
//	}
	m_SizeFactor = sizeFactor;
	m_NumClosest = numClosest;
	g = GraphSearchConstants::GetGraph(map);
//	g = GraphSearchConstants::GetFourConnectedGraph(map);
	ga = new GraphEnvironment(m, g, new GraphMapHeuristic(m, g));
	//ma = new MapEnvironment(m->Clone());
	BuildPDB();
}

GraphCanonicalHeuristic::GraphCanonicalHeuristic(char *file)
:lengths(), centers(), centerDist(), map(0), g(0), mo(0)
{
	Load(file);
}

GraphCanonicalHeuristic::GraphCanonicalHeuristic(Graph *gr, GraphHeuristic *gh, int numClosest, int sizeFactor)
{
	m_SizeFactor = sizeFactor;
	m_NumClosest = numClosest;
	//this->g = g->cloneAll();
	this->g = gr;
	ga = new GraphEnvironment(gr, gh);
	map = 0;
	BuildPDB();
}

void GraphCanonicalHeuristic::Load(char *file)
{
	delete map;
	delete g;
	delete mo;

	FILE *f = fopen(file, "r");
	if (!f)
	{
		printf("Unable to open %s\n", file);
		return;
	}
	
	float ver;
	fscanf(f, "VERSION %f", &ver);
	if (ver != 1.0)
	{
		printf("Unknown version, not able to load data!\n");
		return;
	}
	
	fscanf(f, "%d %d %d", &m_SizeFactor, &m_NumClosest, &m_NumCanonicals);
	int numNodes;
	fscanf(f, "%d\n", &numNodes);

	map = new Map(f);
	g = GraphSearchConstants::GetGraph(map);
	ga = new GraphEnvironment(map, g, new GraphMapHeuristic(map, g));

//	ma = new MapEnvironment(map->Clone());
//	g = GraphSearchConstants::GetGraph(map);
	
	centers.resize(numNodes);
	for (unsigned int x = 0; x < centers.size(); x++)
		fscanf(f, "%d", &centers[x]);
	
	//std::vector<double> centerDist;
	centerDist.resize(m_NumClosest);
	for (unsigned int x = 0; x < centerDist.size(); x++)
	{
		centerDist[x].resize(numNodes);
		for (unsigned int y = 0; y < centerDist[x].size(); y++)
			fscanf(f, "%f ", &centerDist[x][y]);
	}
	fscanf(f, "\n");
	
	whichPDB.resize(m_NumClosest);
	for (unsigned int x = 0; x < whichPDB.size(); x++)
	{
		whichPDB[x].resize(numNodes);
		for (unsigned int y = 0; y < whichPDB[x].size(); y++)
			fscanf(f, "%d ", &whichPDB[x][y]);
	}
	fscanf(f, "\n");
	
	lengths.resize(m_NumCanonicals);
	//std::vector<std::vector<double> > lengths;
	for (unsigned int x = 0; x < lengths.size(); x++)
	{
		lengths[x].resize(m_NumCanonicals);
		for (unsigned int y = 0; y < lengths[x].size(); y++)
		{
			fscanf(f, "%f ", &lengths[x][y]);
		}
	}
	fclose(f);

	mo  = new MapOverlay(map);
	mo->SetTransparentValue(0);
	std::vector<int> randoms;
	//node *r = g->GetRandomNode();
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			node *n = g->GetNode(map->GetNodeNum(x, y));
			if (n)
			{
				graphState a;
				a = n->GetNum();
				//b = r->GetNum();
				if (centerDist[0][a] == 0)
				{
					mo->SetOverlayValue(x, y, 1.0);
				}
				else {
					while (whichPDB[0][a] >= (int)randoms.size())
					{
						randoms.push_back(random()%4096);
					}
					mo->SetOverlayValue(x, y, randoms[whichPDB[0][a]]);
				}
				//mo->SetOverlayValue(x, y, HCost(a, b));
				//				printf("(%d, %d)=%d center distances: %f\n", x, y, n->GetNum(), centerDist[0][n->GetNum()]);
			}
			else
				mo->SetOverlayValue(x, y, 0);
		}
	}
}


void GraphCanonicalHeuristic::Save(char *file)
{
	FILE *f = fopen(file, "w+");
	if (!f)
		return;
	assert(map);
	
	fprintf(f, "VERSION 1.0\n");
	fprintf(f, "%d %d %d\n", m_SizeFactor, m_NumClosest, m_NumCanonicals);
	fprintf(f, "%d\n", (int)whichPDB[0].size());

	map->Save(f);

	//std::vector<int> centers;
	for (unsigned int x = 0; x < centers.size(); x++)
		fprintf(f, "%d ", centers[x]);
	fprintf(f, "\n");

	//std::vector<double> centerDist;
	for (unsigned int x = 0; x < centerDist.size(); x++)
		for (unsigned int y = 0; y < centerDist[x].size(); y++)
			fprintf(f, "%f ", centerDist[x][y]);
	fprintf(f, "\n");

	for (unsigned int x = 0; x < whichPDB.size(); x++)
		for (unsigned int y = 0; y < whichPDB[x].size(); y++)
			fprintf(f, "%d ", whichPDB[x][y]);
	fprintf(f, "\n");
	
	//std::vector<std::vector<double> > lengths;
	for (unsigned int x = 0; x < lengths.size(); x++)
	{
		for (unsigned int y = 0; y < lengths[x].size(); y++)
		{
			fprintf(f, "%f ", lengths[x][y]);
		}
		fprintf(f, "\n");
	}
	fprintf(f, "\n");
	fclose(f);
}

GraphCanonicalHeuristic::~GraphCanonicalHeuristic()
{
	delete map;
	delete g;
	delete mo;
	//delete msa;
}

Graph *GraphCanonicalHeuristic::GetGraph()
{
	return g;
}

Map *GraphCanonicalHeuristic::GetMap()
{
	return map;
}

double GraphCanonicalHeuristic::GetDistToCanonicalState(graphState &which)
{
	double min = centerDist[0][which];
	for (unsigned int x = 1; x < centerDist.size(); x++)
		if (centerDist[x][which] < min)
			min = centerDist[x][which];
	return min;
}

graphState GraphCanonicalHeuristic::GetCanonicalStateID(graphState &which)
{
	return whichPDB[0][which];
}

void GraphCanonicalHeuristic::ChooseStartGoal(graphState &start, graphState &goal)
{
	graphState tmp;
	if (centerDist[0][goal] > centerDist[0][start])
	{
		tmp = start;
		start = goal;
		goal = tmp;
	}
}

double GraphCanonicalHeuristic::HCost(const graphState &state1, const graphState &state2) const
{
	node *n1 = g->GetNode(state1);
	node *n2 = g->GetNode(state2);
	if ((n1 == 0) || (n2 == 0))
		return 0;
	double val = 0;

	//printf("[");
	for (unsigned int x = 0; x < whichPDB.size(); x++)
	{
		for (unsigned int y = 0; y < whichPDB.size(); y++)
		{
			int p1 = whichPDB[x][n1->GetNum()];
			int p2 = whichPDB[y][n2->GetNum()];
			double newval;
			if (p1 != p2)
			{
				//printf("%f-%f-%f, ", lengths[p1][p2], centerDist[x][state1], centerDist[y][state2]);
				newval = lengths[p1][p2] - centerDist[x][state1] - centerDist[y][state2];
			}
			else
				newval = abs(centerDist[x][state1] - centerDist[y][state2]);
			val = max(val, newval);
		}
	}
	if (ga)
	{
		graphState g1 = n1->GetNum(), g2 = n2->GetNum();
		val = max(val, ga->HCost(g1, g2));
		//printf("= %f", val);
	}
	//printf("]\n");
	// compare with h-function...
	return val;
}


void GraphCanonicalHeuristic::BuildPDB()
{
	m_NumCanonicals = sqrt((m_SizeFactor-2*m_NumClosest)*g->GetNumNodes());
	if (m_NumCanonicals < m_NumClosest)
		m_NumCanonicals = m_NumClosest;
	
	printf("Getting centers (%d closest)\n", m_NumCanonicals);
	GetCenters();
	//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	
	// for each center, do a BFS and find the length to all other centers
	printf("Solving for values\n");
	GetPDBValues();
	
	ComputerCenterDistances();

	mo = 0;
	if (map)
	{
		mo  = new MapOverlay(map);
		mo->SetTransparentValue(0);
		node *r = g->GetRandomNode();
		for (int x = 0; x < map->GetMapWidth(); x++)
		{
			for (int y = 0; y < map->GetMapHeight(); y++)
			{
				node *n = g->GetNode(map->GetNodeNum(x, y));
				if (n)
				{
					graphState a, b;
					a = n->GetNum();
					b = r->GetNum();
	//				mo->SetOverlayValue(x, y, whichPDB[0][a]);
					mo->SetOverlayValue(x, y, HCost(a, b));
	//				printf("(%d, %d)=%d center distances: %f\n", x, y, n->GetNum(), centerDist[0][n->GetNum()]);
				}
				else
					mo->SetOverlayValue(x, y, 0);
			}
		}
	}
}

void GraphCanonicalHeuristic::ComputerCenterDistances()
{
//	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
//	//MapEnvironment ma(msa->GetMap()->Clone());
//	
//	//Graph *g = msa->GetAbstractGraph(0);
//	centerDist.resize(g->GetNumNodes());
//	for (int count = 0; count < g->GetNumNodes(); count++)
//	{
//		int locx, locy;
//		node *n = g->GetNode(count);
//		
//		msa->GetTileFromNode(msa->GetParent(n), locx, locy);
//		xyLoc loc1(locx, locy);
//		msa->GetTileFromNode(n, locx, locy);
//		xyLoc loc2(locx, locy);
//		
//		std::vector<xyLoc> path;
//		astar.GetPath(ma, loc1, loc2, path);
//		double len = 0.0;
//		for (unsigned int z = 1; z < path.size(); z++)
//			len += ma->GCost(path[z-1], path[z]);
//		//printf("Local Results = %f (length %d) [h=%f]\n", len, (int)path.size(), ma->HCost(loc1, loc2));
//		
//		centerDist[n->GetNum()] = len;
//	}
}

void GraphCanonicalHeuristic::GetCenters()
{
	int limit = m_NumCanonicals;

	if ((limit < 10) || (map == 0))
	{
		while ((int)centers.size() < limit)
		{
			int val = g->GetRandomNode()->GetNum();
 			while (g->GetNode(val)->GetLabelF(GraphSearchConstants::kZCoordinate) != 0)
			{
				printf("Reject!\n");
				val = g->GetRandomNode()->GetNum();
			}
			for (unsigned int x = 0; x < centers.size(); x++)
				if ((centers[x] == val) || (g->FindEdge(val, centers[x])))
					val = -1;
			if (val != -1)
				centers.push_back(val);
		}
		return;
	}

	if (map)
	{
		Graph *gr = 0;
		MapAbstraction *nla;
		bool found = false;
		nla = new MapLineAbstraction(map->Clone());
		for (unsigned int x = 1; x < nla->getNumAbstractGraphs(); x++)
		{
			gr = nla->GetAbstractGraph(x);
			if (gr == 0)
				break;
			printf("Line %d Wanted %d have %d\n", x, limit, gr->GetNumNodes());
			if (abs(gr->GetNumNodes() - limit) < 1+limit/10)
			{
				printf("Stopping");
				found = true;
				break;
			}
		}
		
		if (!found)
		{
			printf("Cleaning up failed line abstraction\n");
			delete nla;
			nla = 0;
		}
		while (!found)
		{
			static double factor = 25.0;
			double val = random()%75;
			val = val/factor+1;
			int count = (val*(double)g->GetNumNodes());
			count /= limit;
			printf("Trying NLA(%d)\n", count);
			nla = new NodeLimitAbstraction(map->Clone(), count);
			gr = nla->GetAbstractGraph(1);

			printf("Wanted %d have %d\n", limit, gr->GetNumNodes());
			if (abs(gr->GetNumNodes() - limit) < 1+limit/10)
			{
				printf("Stopping");
				break;
			}
//			if (gr->GetNumNodes() > limit)// too many nodes
//				factor++;
//			else {
//				factor--;
//			}

			delete nla;
			nla = 0;
		}
		m_NumCanonicals = gr->GetNumNodes();
	//	Graph *g = msa->GetAbstractGraph(1);
	//	// for every high-level node
		for (int x = 0; x < gr->GetNumNodes(); x++)
		{
			// find closest low-level node
			int locx, locy;
			nla->GetTileFromNode(gr->GetNode(x), locx, locy);

			// use the ids in the real map/graph to add centers
			centers.push_back(map->GetNodeNum(locx, locy));
			//printf("Center[%d] = (%d, %d)\n", (int)centers.size()-1, locx, locy);
		}
		delete nla;
	}
}

void GraphCanonicalHeuristic::GetPDBValues()
{
	//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	astar.SetStopAfterGoal(false);

	centerDist.resize(m_NumClosest);
	whichPDB.resize(m_NumClosest);
	for (int x = 0; x < m_NumClosest; x++)
	{
		centerDist[x].resize(g->GetNumNodes());
		whichPDB[x].resize(g->GetNumNodes());

		for (unsigned int y = 0; y < centerDist[x].size(); y++)
			centerDist[x][y] = 0x7FFFFFFF;
	}
	
	//Graph *g = msa->GetAbstractGraph(0);
	lengths.resize(centers.size());
	for (unsigned int x = 0; x < centers.size(); x++)
	{
		lengths[x].resize(centers.size());
		lengths[x][x] = 0;
		for (unsigned int y = x+1; y < centers.size(); y++)
			lengths[y].resize(centers.size());
	}

	for (unsigned int x = 0; x < centers.size(); x++)
	{
		printf("Solving from [%d of %d] (%d)\n", x, (int)centers.size(), centers[x]);
//		int locx, locy;
//		locx = g->GetNode(centers[x])->GetLabelL(GraphSearchConstants::kMapX);
//		locy = g->GetNode(centers[x])->GetLabelL(GraphSearchConstants::kMapY);
////		msa->GetTileFromNode(g->GetNode(centers[x]), locx, locy);
//		xyLoc loc1(locx, locy);
//		xyLoc locNone(0xFFFF, 0xFFFF);

		std::vector<graphState> path;
		graphState g1 = centers[x], g2 = centers[x];
		ga->SetDirected(true);
		astar.GetPath(ga, g1, g2, path);

		// mark closest points
		double len;
		for (int y = 0; y < g->GetNumNodes(); y++)
		{
//			locx = g->GetNode(y)->GetLabelL(GraphSearchConstants::kMapX);
//			locy = g->GetNode(y)->GetLabelL(GraphSearchConstants::kMapY);
//			xyLoc loc2(locx, locy);
			g1 = y;
			if (astar.GetClosedListGCost(g1, len))
			{
				for (unsigned int z = 0; z < centerDist.size(); z++)
				{
					if (len < centerDist[z][y])
					{
						for (unsigned int w = centerDist.size()-1; w > z; w--)
						{
							centerDist[w][y] = centerDist[w-1][y];
							//printf("centerDist[%d][%d] = %f\n", w, y, centerDist[w-1][y]);
							whichPDB[w][y] = whichPDB[w-1][y];
							//printf("whichPDB[%d][%d] = %f\n", w, y, whichPDB[w-1][y]);
						}
						//printf("centerDist[%d][%d] = %f\n", z, y, len);
						//printf("whichPDB[%d][%d] = %f\n", z, y, x);
						centerDist[z][y] = len;
						whichPDB[z][y] = x;
						break;
					}
				}
			}
		}
		for (unsigned int y = x+1; y < centers.size(); y++)
		{
//			locx = g->GetNode(centers[y])->GetLabelL(GraphSearchConstants::kMapX);
//			locy = g->GetNode(centers[y])->GetLabelL(GraphSearchConstants::kMapY);
//			//msa->GetTileFromNode(g->GetNode(centers[y]), locx, locy);
//			xyLoc loc2(locx, locy);

			len = -1;
			g1 = centers[y];
			if (astar.GetClosedListGCost(g1, len))
			{ }
			//printf("(%d, %d)<->(%d, %d) = %f\n", loc1.x, loc1.y, loc2.x, loc2.y, len);
			//printf("lengths[%d][%d] = %f\n", x, y, len);
			lengths[x][y] = len;
			lengths[y][x] = len;
		}
	}
	for (unsigned int w = 0; w < whichPDB.size(); w++)
		for (unsigned int x = 0; x < whichPDB[w].size(); x++)
			if (centerDist[w][x] == 0x7FFFFFFF)
				centerDist[w][x] = 0;

}


void GraphCanonicalHeuristic::OpenGLDraw() const
{
	if (map)
	{
		//map->SetTileSet(kFast);
		map->OpenGLDraw(kPolygons);
	}
//	if (ga)
//	{
//		ga->SetColor(1, 1, 1, 1);
//		for (unsigned int x = 0; x < centers.size(); x++)
//		{
//			graphState g1 = centers[x];
//			ga->OpenGLDraw(g1);
//		}
//	}
//	if (mo)
//		mo->OpenGLDraw();
}
