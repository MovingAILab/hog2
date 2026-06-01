/*
 *  $Id: MapAbstraction.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapAbstraction.h"

using namespace GraphAbstractionConstants;

MapAbstraction::~MapAbstraction()
{ 
	delete m;
}

void MapAbstraction::GetRandomTileFromNode(node *n, int &x, int &y)
{
	while (GetAbstractionLevel(n) != 0)
		n = GetNthChild(n, random()%GetNumChildren(n));
	x = n->GetLabelL(kFirstData);
	y = n->GetLabelL(kFirstData+1);
}

void MapAbstraction::GetTileFromNode(node *n, int &x, int &y)
{
	if (GetAbstractionLevel(n) != 0)
	{
		GetTileUnderLoc(x, y, GetNodeLoc(n));
		node *t = GetNodeFromMap(x, y);
		if (GetNthParent(t, GetAbstractionLevel(n)) == n)
		{
			//			printf("Nth parent is %d (%d), not %d (%d)\n",
			//						 t->GetNum(), GetAbstractionLevel(t),
			//						 n->GetNum(), GetAbstractionLevel(n));
			return;
		}
		while (GetAbstractionLevel(n) != 0)
			n = GetNthChild(n, 0);
	}
	x = n->GetLabelL(kFirstData);
	y = n->GetLabelL(kFirstData+1);
	//printf("(%d, %d) in x/y\n", x, y);
}

void MapAbstraction::ToggleDrawAbstraction(int which)
{
	bool drawThis = ((levelDraw>>which)&0x1);
	if (!drawThis)
		levelDraw |= (1<<which);
	else
		levelDraw = levelDraw&(~(1<<which));
}

void MapAbstraction::OpenGLDraw() const
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		if ((levelDraw >> x) & 1)
			DrawGraph(abstractions[x], (x>1)&&((levelDraw>>(x-1))&1));
		//glCallList(displayLists[x]);
	}
}


void MapAbstraction::DrawGraph(Graph *g, bool drawLevel) const
{
	if ((g == 0) || (g->GetNumNodes() == 0)) return;
	
	int abLevel = g->GetNode(0)->GetLabelL(kAbstractionLevel);	
	
	//  if (verbose&kBuildGraph) printf("Drawing Graph abstraction %d!\n", abLevel);
	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	
	//	glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), .25+((GLfloat)(abLevel%15-8)/30.0), 1);
	//	switch (g->GetNode(0)->GetLabelL(kAbstractionLevel)%3)
	//	{
	//		case 0: glColor4f(1, 0, 0, 1); break;
	//		case 1: glColor4f(0, 1, 0, 1); break;
	//		case 2: glColor4f(1, 1, 0, 1); break;
	//		case 3: glColor4f(0, 0, 1, .5); break;
	//		case 4: glColor4f(1, 0, 1, .5); break;
	//		case 5: glColor4f(1, 1, 0, .5); break;
	//		case 6: glColor4f(0, 1, 0, .5); break;
	//		case 7: glColor4f(0, 1, 0, .5); break;
	//		default:glColor4f(0, 0, 1, .5); break;
	//	}
	node_iterator ni;
	//	ni = g->getNodeIter();
	//	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	//	{
	//		if (n->GetLabelL(kNodeBlocked) > 0)
	//		{
	//			glColor3f(1, 1, 1);
	//			recVec rv = GetNodeLoc(n);
	//			glVertex3f(rv.x, 0, rv.z);
	//			glVertex3f(rv.x, rv.y, rv.z);
	//		}
	//	}
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		if (e->GetLabelL(kEdgeCapacity) == 0)      glColor4f(.5, .5, .5, 1);
		else if (e->GetLabelL(kEdgeCapacity) <= 0) glColor4f(.2, .2, .2, 1);
		else if (e->getMarked())                  glColor4f(1, 1, 1, 1);
		else if (abLevel%2)
			glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
		else glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
		//glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), .5+((GLfloat)(abLevel%15-8)/30.0), 1);
		
		// Color the edge widths
		// 		else if (e->getWidth() <= 0.0)
		// 			glColor4f(1, 0, 0, 1);
		// 		else if (e->getWidth() <= 0.8)
		// 			glColor4f(1, 0, 1, 1);
		// 		else if (e->getWidth() <= 1.1)
		// 			glColor4f(1, 1, 0, 1);
		// 		else if (e->getWidth() <= 1.5)
		// 			glColor4f(0, 1, 1, 1);
		// 		else if (e->getWidth() <= 2.1)
		// 			glColor4f(0, 1, 0, 1);
		//if (e->getMarked()) {	
		recVec rv = GetNodeLoc(n);
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->GetNode(e->getTo());
		rv = GetNodeLoc(n);
		
		glVertex3f(rv.x, rv.y, rv.z);
		//}
	}
	//node_iterator
	ni = g->getNodeIter();
	
	if (drawLevel)
	{
		for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
			DrawLevelConnections(n);
	}
	glEnd();
	//  if (verbose&kBuildGraph) printf("Done\n");
}

void MapAbstraction::DrawLevelConnections(node *n) const
{
	//	int x, y;
	//	double offsetx, offsety;
	//	recVec ans;
	//if (n->getNumOutgoingEdges()+n->getNumIncomingEdges() == 0) return;
	
	if (n->GetLabelL(kAbstractionLevel) == 0) return;
	else {
		glColor4f(.6, .6, .6, .6);
		recVec v = GetNodeLoc(n);
		for (int cnt = 0; cnt < n->GetLabelL(kNumAbstractedNodes); cnt++)
		{
			recVec v1 = GetNodeLoc(abstractions[n->GetLabelL(kAbstractionLevel)-1]->GetNode(n->GetLabelL(kFirstData+cnt)));
			glVertex3f(v.x, v.y, v.z);
			glVertex3f(v1.x, v1.y, v1.z);
		}
	}
	//return ans;
}

void MapAbstraction::GetTileUnderLoc(int &x, int &y, const recVec &v)
{
	double width = (GetMap()->GetMapWidth()+1)/2.0;
	double height = (GetMap()->GetMapHeight()+1)/2.0;
	//double offsetx, offsety;
	//offsetx = offsety = .5;
	x = (int)(width*(v.x+1.0));
	y = (int)(height*(v.y+1.0));
	//	printf("(%1.2f, %1.2f) in openGL converted to (%d, %d) in x/y\n",
	//				 v.x, v.y, x, y);
}

recVec MapAbstraction::GetNodeLoc(node *n) const
{
	int x, y;
	//  double offsetx, offsety;
	recVec ans;
	
	if (n->GetLabelF(kXCoordinate) != kUnknownPosition)
	{
		ans.x = n->GetLabelF(kXCoordinate);
		ans.y = n->GetLabelF(kYCoordinate);
		ans.z = n->GetLabelF(kZCoordinate);
		return ans;
	}
	
	//	double width = GetMap()->GetMapWidth();
	//	double height = GetMap()->GetMapHeight();
	
	if (n->GetLabelL(kAbstractionLevel) == 0)
	{
		x = n->GetLabelL(kFirstData);
		y = n->GetLabelL(kFirstData+1);
		
		Map *mp = GetMap();
		double r;
		mp->GetOpenGLCoord(x,y,ans.x,ans.y,ans.z,r);
		//    switch (n->GetLabelL(kFirstData+2)) {
		//			case kTopLeft: offsetx = .3; offsety = .3; break;
		//			case kTopRight: offsetx = .6; offsety = .3; break;
		//			case kBottomLeft: offsetx = .3; offsety = .6; break;
		//			case kBottomRight: offsetx = .6; offsety = .6; break;
		//			case kNone:
		//			default: offsetx = .5; offsety = .5; break;
		//    }
		//    ans.x = (double)x/width+offsetx/width-.5;
		//    ans.z = (double)y/height+offsety/width-.5;
	}
	else {
		int totNodes = 0;
		ans.x = ans.y = ans.z = 0;
		for (int cnt = 0; cnt < n->GetLabelL(kNumAbstractedNodes); cnt++)
		{
			int absLevel = n->GetLabelL(kAbstractionLevel)-1;
			node *nextChild = abstractions[absLevel]->GetNode(n->GetLabelL(kFirstData+cnt));
			recVec tmp = GetNodeLoc(nextChild);
			int weight = nextChild->GetLabelL(kNumAbstractedNodes);
			totNodes += weight;
			ans.x += weight*tmp.x;
			ans.y += weight*tmp.y;
			ans.z += weight*tmp.z;
		}
		ans.x /= totNodes;//n->GetLabelL(kNumAbstractedNodes); 
		ans.y /= totNodes;//n->GetLabelL(kNumAbstractedNodes); 
		ans.z /= totNodes;//n->GetLabelL(kNumAbstractedNodes); 
	}
	Map *mp = GetMap();
	double r, a, b, c;
	mp->GetOpenGLCoord(0,0,a,b,c,r);
	// height
	ans.z -= 5.0*r;
	//ans.z = -(double)5.0*r*(n->GetLabelL(kAbstractionLevel)+1.1);
	
	n->SetLabelF(kXCoordinate, ans.x);
	n->SetLabelF(kYCoordinate, ans.y);
	n->SetLabelF(kZCoordinate, ans.z);
	return ans;
}

void MapAbstraction::ClearMarkedNodes()
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		edge_iterator ei = abstractions[x]->getEdgeIter();
		for (edge *e = abstractions[x]->edgeIterNext(ei); e; e = abstractions[x]->edgeIterNext(ei))
			e->setMarked(false);
	}
}

// estimate the cost from a to b
double MapAbstraction::h(node *a, node *b)
{
	if ((a == 0) || (b == 0))
		return 999999999.99;
	
	recVec rv1 = GetNodeLoc(a);
	recVec rv2 = GetNodeLoc(b);
	double answer = OctileDistance((double)rv1.x,(double)rv1.y,(double)rv2.x,(double)rv2.y);
	//		if (fabs(rv1.x-rv2.x) < fabs(rv1.y-rv2.y)) {
	//			answer = root2m1*fabs(rv1.x-rv2.x)+fabs(rv1.y-rv2.y);
	//		} else {
	//			answer = root2m1*fabs(rv1.y-rv2.y)+fabs(rv1.x-rv2.x);
	//		}
	answer *= GetMap()->GetCoordinateScale();
//	if (a->GetNumEdges() < 4)
//		return 9999999;
//	if (b->GetNumEdges() < 4)
//		return 9999999;
	//answer+=(8-a->GetNumEdges()+8-b->GetNumEdges())*5;
	//if (answer > 1)
		return answer;
	//return 1;
}

/** Computes octile distance. No scaling */
double MapAbstraction::OctileDistance(double x1, double y1, double x2, double y2)
{
	double answer = 0.0;
	const double root2m1 = ROOT_TWO-1;//sqrt(2.0)-1;
		if (fabs(x1-x2) < fabs(y1-y2))
			answer = root2m1*fabs(x1-x2)+fabs(y1-y2);
	else
		answer = root2m1*fabs(y1-y2)+fabs(x1-x2);
	return answer;
}




/**
* GetMapGraph(map)
 *
 * Given a map, this function uses the external map interfaces to turn it
 * into a Graph, and sets the appropriate node numbers for that map. This
 * function should not be called multiple times on the same map, because
 * the original Graph map lose it's association with the map.
 */
Graph *GetMapGraph(Map *m)
{
//	return GraphSearchConstants::GetGraph(m);
	// printf("Getting Graph representation of world\n");
	char name[32];
	Graph *g = new Graph();
	node *n;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			Tile &currTile = m->GetTile(x, y);
			currTile.tile1.node = kNoGraphNode;
			currTile.tile2.node = kNoGraphNode;
			
			if (m->AdjacentEdges(x, y, kInternalEdge))
			{
				if (m->GetTerrainType(x, y) == kOutOfBounds)
					continue;
				sprintf(name, "(%d, %d)", x, y);
				currTile.tile1.node = g->AddNode(n = new node(name));
				n->SetLabelL(kAbstractionLevel, 0); // level in abstraction tree
				n->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
				n->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
				n->SetLabelF(kXCoordinate, kUnknownPosition);
				n->SetLabelL(kNodeBlocked, 0);
				n->SetLabelL(kFirstData, x);
				n->SetLabelL(kFirstData+1, y);
				n->SetLabelL(kFirstData+2, kNone);
			}
			else {
				if (m->GetTerrainType(x, y, kLeftEdge) != kOutOfBounds)
				{
					sprintf(name, "(%d/%d)", x, y);
					currTile.tile1.node = g->AddNode(n = new node(name));
					n->SetLabelL(kAbstractionLevel, 0); // level in abstraction tree
					n->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
					n->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
					n->SetLabelF(kXCoordinate, kUnknownPosition);
					n->SetLabelL(kNodeBlocked, 0);
					n->SetLabelL(kFirstData, x);
					n->SetLabelL(kFirstData+1, y);
					if (currTile.split == kForwardSplit)
						n->SetLabelL(kFirstData+2, kTopLeft);
					else
						n->SetLabelL(kFirstData+2, kBottomLeft);
				}
				
				if (m->GetTerrainType(x, y, kRightEdge) != kOutOfBounds)
				{
					sprintf(name, "(%d\\%d)", x, y);
					currTile.tile2.node = g->AddNode(n = new node(name));
					n->SetLabelL(kAbstractionLevel, 0); // level in abstraction tree
					n->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
					n->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
					n->SetLabelF(kXCoordinate, kUnknownPosition);
					n->SetLabelL(kNodeBlocked, 0);
					n->SetLabelL(kFirstData, x);
					n->SetLabelL(kFirstData+1, y);
					if (currTile.split == kForwardSplit)
						n->SetLabelL(kFirstData+2, kBottomRight);
					else
						n->SetLabelL(kFirstData+2, kTopRight);
				}
			}
		}
	}
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			//cout << "Trying (x, y) = (" << x << ", " << y << ")" << endl;
			AddMapEdges(m, g, x, y);
			//			if (!g->verifyGraph())
			//			{
			//				cerr << "Broken at (x, y) = (" << x << ", " << y << ")" << endl;
			//			}
		}
	}
	// printf("Done\n");
	
	// verify Graph is correct
#if 0
	{
		node_iterator ni = g->getNodeIter();
		for (n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
		{
			int numEdges = n->getNumOutgoingEdges() + n->getNumIncomingEdges();
			
			edge *ee;
			edge_iterator eie = n->getEdgeIter();
			for (int x = 0; x < numEdges; x++)
			{
				ee = n->edgeIterNext(eie);
				if (ee == 0)
				{ cout << "**That's impossible; we were told we had " << numEdges << ":(" << n->getNumOutgoingEdges() << "+" << n->getNumIncomingEdges() <<
					") edges, we're on #" << x << " and we got nil!";
					cout << "(node " << n->GetNum() << ")" << endl;
					break;
				}
			}			
		}
	}
#endif
	
	return g;
}

/**
* AddMapEdges(map, Graph, x, y)
 *
 * This is a helper function for GetMapGraph that does the work of adding
 * the Graph edges. Each edge is only added once, so while the Graph has
 * directional edges, we treat them as being bidirectional.
 */
static const int gEdgeProb = 100;
static const int gStraightEdgeProb = 100;

void AddMapEdges(Map *m, Graph *g, int x, int y)
{
	// check 4 surrounding edges
	// when we get two of them, we add the corresponding diagonal edge(?)...not yet
	// make sure the edge we add isn't there already!
	// make sure we get the right node # when we add the edge
	edge *e = 0;
	
	// left edge is always tile1, right edge is always tile 2
	if ((x >= 1) && (m->AdjacentEdges(x, y, kLeftEdge)) && (m->GetTile(x, y).tile1.node != kNoGraphNode))
	{
		if (m->AdjacentEdges(x-1, y, kInternalEdge) && (m->GetTile(x-1, y).tile1.node != kNoGraphNode))
		{
			if ((random()%100) < gStraightEdgeProb)
			{
				e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y).tile1.node, 1);
				g->AddEdge(e);
			}
		}
		else if (m->GetTile(x-1, y).tile2.node != kNoGraphNode)
		{
			if ((random()%100) < gStraightEdgeProb)
			{
				e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y).tile2.node, 1);
				g->AddEdge(e);
			}
		}
		if (e)
			e->SetLabelL(kEdgeCapacity, 1);
	}
	e = 0;
	// top edge (may be tile 1 or tile 2)
	if ((y >= 1) && (m->AdjacentEdges(x, y, kTopEdge)))
	{
		if ((m->AdjacentEdges(x, y, kInternalEdge)) || (m->GetSplit(x, y) == kForwardSplit))
		{
			if (m->GetTile(x, y).tile1.node != kNoGraphNode)
			{
				if (m->AdjacentEdges(x, y-1, kInternalEdge) || (m->GetSplit(x, y-1) == kBackwardSplit))
				{
					if (m->GetTile(x, y-1).tile1.node != kNoGraphNode)
					{
						if ((random()%100) < gStraightEdgeProb)
						{
							e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x, y-1).tile1.node, 1);
							g->AddEdge(e);
						}
					}
				}
				else if (m->GetTile(x, y-1).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < gStraightEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x, y-1).tile2.node, 1);
						g->AddEdge(e);
					}
				}
			}
		}
		else {
			if (m->AdjacentEdges(x, y-1, kInternalEdge) || (m->GetSplit(x, y-1) == kBackwardSplit))
			{
				if ((m->GetTile(x, y).tile2.node != kNoGraphNode) && (m->GetTile(x, y-1).tile1.node != kNoGraphNode))
				{
					if ((random()%100) < gStraightEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x, y-1).tile1.node, 1);
						g->AddEdge(e);
					}
				}
			}
			else if ((m->GetTile(x, y).tile2.node != kNoGraphNode) && (m->GetTile(x, y-1).tile2.node != kNoGraphNode))
			{
				if ((random()%100) < gStraightEdgeProb)
				{
					e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x, y-1).tile2.node, 1);
					g->AddEdge(e);
				}
			}
		}
		if (e)
			e->SetLabelL(kEdgeCapacity, 1);
	}
	e = 0;
	// diagonal UpperLeft edge, always node 1...
	// (1) we can cross each of the boundaries between tiles
	if ((x >= 1) && (y >= 1) && (m->AdjacentEdges(x, y, kLeftEdge)) && (m->AdjacentEdges(x, y, kTopEdge)) &&
			(m->AdjacentEdges(x, y-1, kLeftEdge)) && (m->AdjacentEdges(x-1, y, kTopEdge)) &&
			(m->GetTile(x, y).tile1.node != kNoGraphNode))
	{
		// (2) we can cross the inner tile boundaries
		if (((m->AdjacentEdges(x-1, y, kInternalEdge)) || (m->GetSplit(x-1, y) == kBackwardSplit)) &&
				((m->AdjacentEdges(x, y-1, kInternalEdge)) || (m->GetSplit(x, y-1) == kBackwardSplit)) &&
				((m->AdjacentEdges(x-1, y-1, kInternalEdge)) || (m->GetSplit(x-1, y-1) == kForwardSplit)) &&
				((m->AdjacentEdges(x, y, kInternalEdge)) || (m->GetSplit(x, y) == kForwardSplit)))
		{
			// (3) find what tiles to connect
			if (m->AdjacentEdges(x-1, y-1, kInternalEdge))
			{
				if (m->GetTile(x-1, y-1).tile1.node != kNoGraphNode)
				{
					if ((random()%100) < gEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y-1).tile1.node, ROOT_TWO);
						g->AddEdge(e);
					}
				}
			}
			else if (m->GetTile(x-1, y-1).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < gEdgeProb)
				{
					e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y-1).tile2.node, ROOT_TWO);
					g->AddEdge(e);
				}
			}
			if (e)
				e->SetLabelL(kEdgeCapacity, 1);
		}
	}
	e = 0;
	// diagonal UpperRight edge
	// (1) we can cross each of the boundaries between tiles
	if ((y >= 1) && (x < m->GetMapWidth()-1) && (m->AdjacentEdges(x, y, kRightEdge)) && (m->AdjacentEdges(x, y, kTopEdge)) &&
			(m->AdjacentEdges(x, y-1, kRightEdge)) && (m->AdjacentEdges(x+1, y, kTopEdge)) &&
			(m->GetTile(x+1, y-1).tile1.node != kNoGraphNode))
	{
		// (2) we can cross the inner tile boundaries
		if (((m->AdjacentEdges(x+1, y, kInternalEdge)) || (m->GetSplit(x+1, y) == kForwardSplit)) &&
				((m->AdjacentEdges(x, y-1, kInternalEdge)) || (m->GetSplit(x, y-1) == kForwardSplit)) &&
				((m->AdjacentEdges(x+1, y-1, kInternalEdge)) || (m->GetSplit(x+1, y-1) == kBackwardSplit)) &&
				((m->AdjacentEdges(x, y, kInternalEdge)) || (m->GetSplit(x, y) == kBackwardSplit)))
		{
			// (3) connect
			if (m->AdjacentEdges(x, y, kInternalEdge))
			{
				if (m->GetTile(x, y).tile1.node != kNoGraphNode)
				{
					if ((random()%100) < gEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x+1, y-1).tile1.node, ROOT_TWO);
						g->AddEdge(e);
					}
				}
			}
			else if (m->GetTile(x, y).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < gEdgeProb)
				{
					e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x+1, y-1).tile1.node, ROOT_TWO);
					g->AddEdge(e);
				}
			}
			if (e)
				e->SetLabelL(kEdgeCapacity, 1);
		}
	}	
}
