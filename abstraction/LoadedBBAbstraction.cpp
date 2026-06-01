/*
 *  LoadedBBAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/12/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include "LoadedBBAbstraction.h"
#include "FPUtil.h"
#include <math.h>
#include <string.h>

using namespace std;
using namespace GraphAbstractionConstants;

enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const static int verbose = kMiscMessages;//kMiscMessages;//kRepairGraph;

const double unknownPosition = -5.0;

bool LoadedBBAbstraction::BoundingBox::pointInBox(double x, double y, double z)
{
//	printf("checking (%lf, %lf, %lf) in (%lf, %lf, %lf)<=>(%lf, %lf, %lf)\n",
//				 x, y, z, x1, y1, z1, x2, y2, z2);
	return (((fless(x, x1) && (!fless(x, x2))) || (fless(x, x2) && (!fless(x, x1)))) &&
					((fless(y, y1) && (!fless(y, y2))) || (fless(y, y2) && (!fless(y, y1)))) &&
					((fless(z, z1) && (!fless(z, z2))) || (fless(z, z2) && (!fless(z, z1)))));
}

void LoadedBBAbstraction::BoundingBox::OpenGLDraw() const
{
	glBegin(GL_LINE_LOOP);
	glVertex3f(x1, y1, z1);
	glVertex3f(x2, y1, z1);
	glVertex3f(x2, y2, z1);
	glVertex3f(x1, y2, z1);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex3f(x2, y2, z2);
	glVertex3f(x1, y2, z2);
	glVertex3f(x1, y1, z2);
	glVertex3f(x2, y1, z2);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(x1, y1, z1);
	glVertex3f(x1, y1, z2);
	glVertex3f(x2, y1, z1);
	glVertex3f(x2, y1, z2);
	glVertex3f(x1, y2, z1);
	glVertex3f(x1, y2, z2);
	glVertex3f(x2, y2, z1);
	glVertex3f(x2, y2, z2);
	glEnd();
}
//double x1, x2, y1, y2, z1, z2;

/**
 * Construct a new Graph hierarchy.
 *
 * Constructions a new Graph abstraction hierarchy from the Graph using the
 * designated abstraction method.
 */
LoadedBBAbstraction::LoadedBBAbstraction(char *fname, char *boxFile)
:GraphAbstraction()
{
	levelDraw = 1;

	loadBoxes(boxFile);
	buildAbstractions(loadGraph(fname));
}

LoadedBBAbstraction::~LoadedBBAbstraction()
{
  cleanMemory();
}

void LoadedBBAbstraction::loadBoxes(char *boxFile)
{
	FILE *f = fopen(boxFile, "r");
	if (f)
	{
		while (!feof(f))
		{
			//printf("Loading bounding box %u\n", boxes.size());
			BoundingBox b;
			int num;
			fscanf(f, "%d %lf %lf %lf %lf %lf %lf\n",
						 &num, &b.x1, &b.y1, &b.z1, &b.x2, &b.y2, &b.z2);
			boxes.push_back(b);
		}
		fclose(f);
	}
	else {
		printf("Unable to open box file %s\n", boxFile);
		exit(2);
	}
}

double LoadedBBAbstraction::h(node *a, node *b)
{
	double d1 = a->GetLabelF(kXCoordinate)-b->GetLabelF(kXCoordinate);
	double d2 = a->GetLabelF(kYCoordinate)-b->GetLabelF(kYCoordinate);
	double d3 = a->GetLabelF(kZCoordinate)-b->GetLabelF(kZCoordinate);
	return sqrt(d1*d1+d2*d2+d3*d3);
}

void LoadedBBAbstraction::ToggleDrawAbstraction(int which)
{
  bool drawThis = ((levelDraw>>which)&0x1);
  if (!drawThis)
    levelDraw |= (1<<which);
  else
    levelDraw = levelDraw&(~(1<<which));
}

void LoadedBBAbstraction::OpenGLDraw() const
{
	glDisable(GL_LIGHTING);
  for (unsigned int x = 0; x < abstractions.size(); x++)
	{
    if ((levelDraw >> x) & 1) DrawGraph(abstractions[x]);
    //glCallList(displayLists[x]);
  }
	if (levelDraw&1)
	{
		for (unsigned int x = 0; x < boxes.size(); x++)
		{
			glColor3f(0, 0, 1);
			boxes[x].OpenGLDraw();
		}
	}
	glEnable(GL_LIGHTING);
}

void LoadedBBAbstraction::DrawGraph(Graph *g) const
{
		if ((g == 0) || (g->GetNumNodes() == 0)) return;
		
		int abLevel = g->GetNode(0)->GetLabelL(kAbstractionLevel);	
		
		glBegin(GL_LINES);
		glNormal3f(0, 1, 0);
		node_iterator ni;
		edge_iterator ei = g->getEdgeIter();
		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei)) {
			node *n;
			n = g->GetNode(e->getFrom());
			
			if (e->GetLabelL(kEdgeCapacity) == 0)      glColor4f(.5, .5, .5, 1);
			else if (e->GetLabelL(kEdgeCapacity) <= 0) glColor4f(.2, .2, .2, 1);
			else if (e->getMarked())                  glColor4f(1, 1, 1, 1);
			else if (abLevel%2)
				glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
			else glColor4f(((GLfloat)(abLevel%15)/15.0), 1-((GLfloat)(abLevel%15)/15.0), 0, 1);
			recVec rv = GetNodeLoc(n);
			glVertex3f(rv.x, rv.y, rv.z);
			
			n = g->GetNode(e->getTo());
			rv = GetNodeLoc(n);
			
			glVertex3f(rv.x, rv.y, rv.z);
		}
		ni = g->getNodeIter();
		for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
			DrawLevelConnections(n);
		glEnd();
		//  if (verbose&kBuildGraph) printf("Done\n");
}

void LoadedBBAbstraction::DrawLevelConnections(node *n) const
{
  //	int x, y;
  //	double offsetx, offsety;
  //	recVec ans;
  //if (n->getNumOutgoingEdges()+n->getNumIncomingEdges() == 0) return;
	
  if (n->GetLabelL(kAbstractionLevel) == 0) return;
  else {
    glColor4f(.6, .6, .6, .6);
    recVec v = GetNodeLoc(n);
    for (int cnt = 0; cnt < n->GetLabelL(kNumAbstractedNodes); cnt++) {
      recVec v1 = GetNodeLoc(abstractions[n->GetLabelL(kAbstractionLevel)-1]->GetNode(n->GetLabelL(kFirstData+cnt)));
      glVertex3f(v.x, v.y, v.z);
      glVertex3f(v1.x, v1.y, v1.z);
    }
  }
  //return ans;
}

recVec LoadedBBAbstraction::GetNodeLoc(node *n) const
{
	//  double offsetx, offsety;
  recVec ans;
	
  if (n->GetLabelF(kXCoordinate) != unknownPosition) {
    ans.x = n->GetLabelF(kXCoordinate);
    ans.y = n->GetLabelF(kYCoordinate);
    ans.z = n->GetLabelF(kZCoordinate);
    return ans;
  }
	
	int totNodes = 0;
	ans.x = ans.y = ans.z = 0;
	for (int cnt = 0; cnt < n->GetLabelL(kNumAbstractedNodes); cnt++) {
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

  n->SetLabelF(kXCoordinate, ans.x);
  n->SetLabelF(kYCoordinate, ans.y);
  n->SetLabelF(kZCoordinate, ans.z);
  return ans;
}


Graph *LoadedBBAbstraction::loadGraph(char *fname)
{
	FILE *f = fopen(fname, "r");
	if (!f)
	{
		printf("Error opening %s for loading\n", fname);
		exit(1);
	}
	char nextLine[255];
	std::vector<unsigned int> IDS;
	bool nodes = true;
	Graph *g = new Graph();
	fgets(nextLine, 255, f);
	const double VERSION = 1.0;
	double version;
	sscanf(nextLine, "VERSION %lf", &version);
	if (version != VERSION)
	{
		printf("Got %lf; code can only handle version 1.0\n", version);
		exit(1);
	}
	
	while ((!feof(f)) && fgets(nextLine, 255, f))
	{
		if (nextLine[0] == '#')
			continue;
		if (strncmp("edges", nextLine, 5) == 0)
		{
			nodes = false;
			continue;
		}
		if (nodes)
		{
			// ID x y z
			int ID;
			double x, y, z;
			sscanf(nextLine, "%d %lf %lf %lf", &ID, &x, &y, &z);
			//printf("Loaded node %d\n", ID);
			if (IDS.size() <= (unsigned int)ID)
				IDS.resize(ID+1);
			node *n;
			IDS[ID] = g->AddNode(n = new node("l1"));
			n->SetLabelL(kAbstractionLevel, 0); // level in abstraction tree
			n->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
			n->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			n->SetLabelF(kXCoordinate, x);
			n->SetLabelF(kYCoordinate, y);
			n->SetLabelF(kZCoordinate, z);
			n->SetLabelL(kNodeBlocked, 0);
		}
		else {
			int ID1, ID2;
			sscanf(nextLine, "%d %d", &ID1, &ID2);
			g->AddEdge(new edge(IDS[ID1], IDS[ID2], h(g->GetNode(ID1), g->GetNode(ID2))));
			//printf("Loaded edge between %d and %d\n", ID1, ID2);
		}
	}
	return g;
}

void LoadedBBAbstraction::VerifyHierarchy()
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
//				int x1, y1;
//				GetTileFromNode(n, x1, y1);
//				if (n != GetNodeFromMap(x1, y1))
//					cout << "VERIFY: node doesn't correspond to underlying map" << endl << *n << endl;
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
			// VERIFY edge weights
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

void LoadedBBAbstraction::cleanMemory()
{
  while (abstractions.size() > 0) {
    delete abstractions.back();
    abstractions.pop_back();
  }
//	clearDisplayLists();
//	while (displayLists.size() > 0)
//		displayLists.pop_back();
}

//void LoadedBBAbstraction::clearDisplayLists()
//{
//  for (unsigned int x = 0; x < displayLists.size(); x++) {
//    if (displayLists[x] != 0) glDeleteLists(displayLists[x], 1);
//    displayLists[x] = 0;
//  }
//}

void LoadedBBAbstraction::buildAbstractions(Graph *_g)
{
	int totalNodes = 0;
  cleanMemory();
	abstractions.push_back(_g);
  Graph *g = abstractions[0];
//	if (displayLists.size() != 1)
//		displayLists.push_back(0);
  //if (verbose)
      printf("Base Graph (0) has %d nodes\n", g->GetNumNodes());
			g->printStats();
	
  for (int x = 1; ; x++)
	{
    if (g->GetNumEdges() == 0) break;

    if (verbose&kMiscMessages) printf("Building abstraction #%2d\n", x);

    abstractions.push_back(abstractGraph(g));
		g = abstractions.back();

    if (verbose&kMiscMessages)
		{
      printf("Abstract Graph #%2d has %d nodes\n", x, g->GetNumNodes());
			g->printStats();
		}
		totalNodes += g->GetNumNodes();
//    displayLists.push_back(0);
  }
	// printf("%d nodes, excluding bottom level", totalNodes);
}

Graph *LoadedBBAbstraction::abstractGraph(Graph *g)
{
	// for each node:
	// 1. find bounding box
	// 2. add node and all neighbors which are in the bounding box
	node_iterator ni = g->getNodeIter();
	node *newNode;
	Graph *aGraph = new Graph();

	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		if (n->GetLabelL(kParent) != -1) continue;
		int which;
		if (n->GetLabelL(kAbstractionLevel) == 0)
			which = findBoundingBox(n);
		else
			which = -1;
		//printf("%u abstracted into box %d\n", n->GetNum(), which);
		newNode = createNewParent(aGraph, n);
		//printf("%u created as parent of %u\n", newNode->GetNum(), n->GetNum());
		if ((which == -1) && (n->GetLabelL(kAbstractionLevel) == 0))
			addNodeToParent(n, newNode);
		else
			addNeighborsInBox(g, n, which, newNode);
	}
	
	// now add all the edges
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
		int to = g->GetNode(e->getTo())->GetLabelL(kParent);
		edge *f=0;
		
		if ((from != to) && (!(f = aGraph->FindEdge(to, from))))
		{
			double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
			f = new edge(from, to, weight);
			f->SetLabelL(kEdgeCapacity, 1);
			aGraph->AddEdge(f);
			//printf("Adding edge to Graph!\n");
		}
		else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
		//		else if (g)
		//			g->setLabel(kEdgeCapacity, g->GetLabelL(kEdgeCapacity)+1);
	}
	 
	return aGraph;
}

int LoadedBBAbstraction::findBoundingBox(node *n)
{
	for (unsigned int x = 0; x < boxes.size(); x++)
	{
		if (boxes[x].pointInBox(n->GetLabelF(kXCoordinate),
																					 n->GetLabelF(kYCoordinate),
																					 n->GetLabelF(kZCoordinate)))
			return x;
//		if (boxes[boxes.size()-x-1].pointInBox(n->GetLabelF(kXCoordinate),
//																					 n->GetLabelF(kYCoordinate),
//																					 n->GetLabelF(kZCoordinate)))
//			return boxes.size()-x-1;
	}
	return -1;
}

void LoadedBBAbstraction::addNeighborsInBox(Graph *g, node *n, int which, node *parent)
{
	//printf("Thinking about adding %d to parent %d\n", n->GetNum(), parent->GetNum());
	if (n->GetLabelL(kParent) != -1)
		return;
	if ((which != -1) && (!boxes[which].pointInBox(n->GetLabelF(kXCoordinate),
																								 n->GetLabelF(kYCoordinate),
																								 n->GetLabelF(kZCoordinate))))
		return;

	addNodeToParent(n, parent);
	neighbor_iterator nbi = n->getNeighborIter();
	for (node *next = g->GetNode(n->nodeNeighborNext(nbi));
			 next; next = g->GetNode(n->nodeNeighborNext(nbi)))
		addNeighborsInBox(g, next, which, parent);
}

void LoadedBBAbstraction::addNodeToParent(node *n, node *parent)
{
	//printf("Adding %d to parent %d\n", n->GetNum(), parent->GetNum());
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1); // number of abstracted nodes
}

node *LoadedBBAbstraction::createNewParent(Graph *g, node *n)
{
	node *newNode = new node("l1");
	g->AddNode(newNode);
	newNode->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
	newNode->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
	newNode->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
	newNode->SetLabelF(kXCoordinate, unknownPosition);
	newNode->SetLabelL(kNodeBlocked, 0);

	return newNode;
}

bool LoadedBBAbstraction::Pathable(unsigned int from, unsigned int to)
{
	return Pathable(abstractions[0]->GetNode(from), abstractions[0]->GetNode(to));
}

bool LoadedBBAbstraction::Pathable(node *from, node *to)
{
  //printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
  while (from != to) {
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


void LoadedBBAbstraction::AddNode(node *)
{
}

void LoadedBBAbstraction::AddEdge(edge *, unsigned int)
{
}

//	// for now we'll immediately handle splits, but in the future we should queue up splits
//// and process them in batch form(?)
//void LoadedBBAbstraction::RemoveEdge(edge *e, unsigned int absLevel)
//{
//  return;
//}
//

node *LoadedBBAbstraction::findNodeParent(node *n)
{
  unsigned int absLevel = n->GetLabelL(kAbstractionLevel);
  if (absLevel < abstractions.size()-1)
    return abstractions[absLevel+1]->GetNode(n->GetLabelL(kParent));
  return 0;
}
