//
//  Prim.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/9/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include  <queue>
#include "Prim.h"

class Compare
{
public:
	bool operator() (edge *e1, edge *e2)
	{
		return e1->GetWeight() > e2->GetWeight();
	}
};

void Prim(Graph *g)
{
	std::priority_queue<edge *, std::vector<edge *>, Compare> q;

	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		e->setMarked(false);
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		n->SetKeyLabel(0);
	}
	node *n = g->GetNode(0);
	n->SetKeyLabel(1);
	for (int x = 0; x < n->GetNumEdges(); x++)
	{
		edge *e = n->getEdge(x);
		q.push(e);
	}
	while (q.size() > 0)
	{
		edge *e = q.top();
		q.pop();
		if (e->getMarked())
			continue;
		if (g->GetNode(e->getFrom())->GetKeyLabel() == 0 ||
			g->GetNode(e->getTo())->GetKeyLabel() == 0)
		{
//			printf("*Adding %d-%d cost %d to MST\n", e->getFrom(), e->getTo(), (int)e->GetWeight());
			e->setMarked(true);
		}
		
		node *n = g->GetNode(e->getFrom());
		if (n->GetKeyLabel() == 1)
		{
			n = g->GetNode(e->getTo());
			if (n->GetKeyLabel() == 1)
				continue;
		}
//		printf("*Processing %d\n", n->GetNum());
		n->SetKeyLabel(1);
		for (int x = 0; x < n->GetNumEdges(); x++)
		{
			edge *e = n->getEdge(x);
			if (e->getMarked() == true)
				continue;
			if (e->getFrom() == n->GetNum())
			{
				if (g->GetNode(e->getTo())->GetKeyLabel() == 0)
				{
					q.push(e);
//					printf("--Adding edge (%d %d)\n", e->getFrom(), e->getTo());
				}
			}
			else {
				if (g->GetNode(e->getFrom())->GetKeyLabel() == 0)
				{
					q.push(e);
//					printf("--Adding edge (%d %d)\n", e->getTo(), e->getFrom());
				}
			}
		}
	}
}
