/*
 *  GraphEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "GraphEnvironment.h"
#include "Constants.h"
#include "Heap.h"
#include "FloydWarshall.h"
#include "SVGUtil.h"

using namespace GraphSearchConstants;

//int GraphMapInconsistentHeuristic::hmode=2;
//int GraphMapInconsistentHeuristic::HN=10;
//double GraphMapPerfectHeuristic::prob=0.5;

GraphEnvironment::GraphEnvironment(Graph *_g, GraphHeuristic *gh)
:g(_g), h(gh)
{
	m = 0;
 	directed = false;
	drawEdgeCosts = false;
	integerEdgeCosts = false;
	drawNodeLabels = false;
	nodeScale = 1.0;
}

GraphEnvironment::GraphEnvironment(Map *_m, Graph *_g, GraphHeuristic *gh)
:g(_g), h(gh)
{
	m = _m;
 	directed = false;
	drawEdgeCosts = false;
	integerEdgeCosts = false;
	drawNodeLabels = false;
	nodeScale = 1.0;
}

//GraphEnvironment::GraphEnvironment(Map *m)
//{
//	h = 0;
//	directed = false;
//	g = GetMapGraph(m);
//	//BuildHeuristics(m, g);
//}

GraphEnvironment::~GraphEnvironment()
{
	// delete g; ??
	//	delete g;
//	delete h;
}

int GraphEnvironment::GetNumSuccessors(const graphState &stateID) const
{
	node *n = g->GetNode(stateID);
	
	if (n == 0)
	{
		return 0;
	}
	
	if (directed)
	{
		return n->getNumOutgoingEdges();
	}
	return n->GetNumEdges();
}

void GraphEnvironment::GetSuccessors(const graphState &stateID, std::vector<graphState> &neighbors) const
{
	neighbors.resize(0);
	node *n = g->GetNode(stateID);

	if (n == 0)
	{
		return;
	}

	if (directed)
	{
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			neighbors.push_back(e->getTo());
		}
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			if (stateID != e->getTo())
				neighbors.push_back(e->getTo());
			else
				neighbors.push_back(e->getFrom());
		}
	}
}

void GraphEnvironment::GetActions(const graphState &stateID, std::vector<graphMove> &actions) const
{
	actions.resize(0);
	node *n = g->GetNode(stateID);

	if (n == 0)
	{
		return;
	}

	if (directed)
	{
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			actions.push_back(graphMove(e->getFrom(),e->getTo()));
		}
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			if (stateID != e->getTo())
				actions.push_back(graphMove(e->getFrom(),e->getTo()));
			else
				actions.push_back(graphMove(e->getTo(),e->getFrom()));
		}
	}

}

graphMove GraphEnvironment::GetAction(const graphState &s1, const graphState &s2) const
{
	return graphMove(s1, s2);
}

void GraphEnvironment::ApplyAction(graphState &s, graphMove a) const
{
	assert(s == a.from);
	s = a.to;
}

bool GraphEnvironment::InvertAction(graphMove &a) const
{
	uint32_t tmp = a.from;
	a.from = a.to;
	a.to = tmp;
	if (g->findDirectedEdge(a.from, a.to))
		return true;
	return false;
}

double GraphEnvironment::HCost(const graphState &state1, const graphState &state2) const
{
	if (h)
		return h->HCost(state1, state2);
	if (state1 == state2)
		return 0;
	return 1;// this should be the min edge cost in the graph...
}

double GraphEnvironment::GCost(const graphState &, const graphMove &move) const
{
	edge *e = g->FindEdge(move.from, move.to);
	assert(e);
	return e->GetWeight();
}

double GraphEnvironment::GCost(const graphState &state1, const graphState &state2) const
{
	edge *e = g->FindEdge(state1, state2);
//	if (!e)
//		return -1000.0;
	assert(e);
	return e->GetWeight();
}

bool GraphEnvironment::GoalTest(const graphState &state, const graphState &goal) const
{
	return state == goal;
}

uint64_t GraphEnvironment::GetStateHash(const graphState &state) const
{
	return g->GetNode(state)->getUniqueID();
}

void GraphEnvironment::GetStateFromHash(uint64_t hash, graphState &s) const
{
	s = hash;
}


uint64_t GraphEnvironment::GetActionHash(graphMove act) const
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}


//
//void GraphEnvironment::OpenGLDraw() const
//{
//	if ((g == 0) || (g->GetNumNodes() == 0)) return;
//	
//	if (m)
//	{
//		m->OpenGLDraw();
//		return;
//	}
//	
//	// Hittable background behind graph
//	glBegin(GL_TRIANGLE_FAN);
//	glColor4f(0, 0, 0, 0.01);
//	glVertex3f(-1, -1, 0.01);
//	glVertex3f(1, -1, 0.01);
//	glVertex3f(1, 1, 0.01);
//	glVertex3f(-1, 1, 0.01);
//	glEnd();
//	glBegin(GL_LINES);
//	glNormal3f(0, 1, 0);
//
//	double off = 0;
//	edge_iterator ei = g->getEdgeIter();
//	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//	{
//		//int x, y;
//		//double offsetx, offsety;
//		node *n;
//		n = g->GetNode(e->getFrom());
//		assert(n!=0);
//
//		{
//			GLfloat r, g, b, t;
//			GetColor(r, g, b, t);
//			glColor4f(r, g, b, t);
//		}
//		if (e->getMarked())
//		{
//			glColor3f(1.0, 0.0, 0.0);
//			off = -0.01;
//		}
//		else {
//			off = 0;
//		}
//		
//		double x, y, z;
//		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//		glVertex3f(x, y, z+off);
//		
//		n = g->GetNode(e->getTo());
//		assert(n!=0);
//		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//		
//		glVertex3f(x, y, z+off);
//	}
//	glEnd();
//
//	char label[5];
//	if (drawEdgeCosts)
//	{
//		glLineWidth(3.0);
//		glColor4f(0.0, 0.0, 0.0, 1.0);
//		edge_iterator ei = g->getEdgeIter();
//		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//		{
//			if (integerEdgeCosts)
//				sprintf(label, "%d", int(e->GetWeight()));
//			else
//				sprintf(label, "%1.2f", e->GetWeight());
//			node *n;
//			n = g->GetNode(e->getFrom());
//			
//			double x, y, z;
//			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			
//			n = g->GetNode(e->getTo());
//			x += n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y += n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z += n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			
//			DrawText(x/2, y/2, z/2-0.003, 0.2, label);
//		}
//		glLineWidth(1.0);
//	}
//	if (drawNodeLabels)
//	{
//		glLineWidth(3.0);
//		glColor4f(0.0, 0.0, 0.0, 1.0);
//		for (int x = 0; x < g->GetNumNodes(); x++)
//		{
//			node *n = g->GetNode(x);
//			DrawTextCentered(n->GetLabelF(GraphSearchConstants::kXCoordinate),
//							 n->GetLabelF(GraphSearchConstants::kYCoordinate),
//							 n->GetLabelF(GraphSearchConstants::kZCoordinate)-0.003,
//							 0.2, n->GetName());
//		}
//		glLineWidth(1.0);
//	}
//	
//}
//
//void GraphEnvironment::OpenGLDraw(const graphState &s) const
//{
//	if (m)
//	{
//		double xx, yy, zz, rad;
//		int x1, y1;
//		node *n = g->GetNode(s);
//		x1 = n->GetLabelL(GraphSearchConstants::kMapX);
//		y1 = n->GetLabelL(GraphSearchConstants::kMapY);
//		m->GetCoord(x1, y1, xx, yy, zz, rad);
//		GLfloat red, gre, blue, t;
//		GetColor(red, gre, blue, t);
//		glColor4f(red, gre, blue, t);
//		//glColor3f(0.5, 0.5, 0.5);
//		//DrawSphere(xx, yy, zz, rad);
//		glBegin(GL_QUADS);
//		glVertex3d(xx+rad, yy+rad, zz-rad);
//		glVertex3d(xx-rad, yy+rad, zz-rad);
//		glVertex3d(xx-rad, yy-rad, zz-rad);
//		glVertex3d(xx+rad, yy-rad, zz-rad);
//		glEnd();
//	}
//	else {
//		
//		if (drawNodeLabels)// draw states as boxes
//		{
//
//			GLfloat r, gr, b, t;
//			GetColor(r, gr, b, t);
//			glColor4f(r, gr, b, t);
//			
//			node *n = g->GetNode(s);
//			double x, y, z, rad;
//			x = (double)n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = (double)n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = (double)n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			//rad = 20*(double)0.4/(g->GetNumNodes());
//			rad = nodeScale*(double)0.4/(g->GetNumNodes());
//			DrawSquare(x, y, z-0.002, rad);
//			glLineWidth(2.0);
////			if (r+gr+b < 1.5)
////				glColor4f(1, 1, 1, t);
////			else
//			glColor4f(0, 0, 0, t);
//			OutlineRect(x-rad, y-rad, x+rad, y+rad, z-0.0021);
//			glLineWidth(1.0);
//		}
//		//(double)2.0/(g->GetNumNodes()*g->GetNumNodes()));
//
//
//		if (1) // draw states by their edges
//		{
//			node *n = g->GetNode(s);
//			glLineWidth(2.0);
//			glBegin(GL_LINES);
//
//			GLfloat r, gr, b, t;
//			GetColor(r, gr, b, t);
//			glColor4f(r, gr, b, t);
//
//			edge_iterator ei = n->getEdgeIter();
//			for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
//			{
//				node *n;
//				n = g->GetNode(e->getFrom());
//				
//				double x, y, z;
//				x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//				y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//				z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//				glVertex3f(x, y, z);
//				
//				n = g->GetNode(e->getTo());
//				x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//				y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//				z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//				
//				glVertex3f(x, y, z);
//			}
//			glEnd();
//			glLineWidth(1.0);
//		}
//	}
//}
//
//void GraphEnvironment::GLLabelState(const graphState &s, const char *txt) const
//{
////	glLineWidth(3.0);
//	glColor4f(0.0, 0.0, 0.0, 1.0);
//	node *n = g->GetNode(s);
//	double rad = nodeScale*(double)0.4/(g->GetNumNodes());
////	glPushMatrix();
////	glTranslatef(rad, -rad, 0);
//	DrawText(n->GetLabelF(GraphSearchConstants::kXCoordinate)+rad,
//			 n->GetLabelF(GraphSearchConstants::kYCoordinate)-rad,
//			 n->GetLabelF(GraphSearchConstants::kZCoordinate)-0.003,
//			 0.2, txt);
////	glPopMatrix();
////	glLineWidth(1.0);
//}
//
//void GraphEnvironment::OpenGLDraw(const graphState &, const graphMove &) const
//{
//	// if we want to draw a set of moves we use this to do so
//}
//
//void GraphEnvironment::GLDrawLine(const graphState &from, const graphState &to) const
//{
//	{
//		GLfloat r, g, b, t;
//		GetColor(r, g, b, t);
//		glColor4f(r, g, b, t);
//	}
//
//	
//	glBegin(GL_LINES);
//	
//	double x, y, z;
//
//	node *n = g->GetNode(from);
//	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	glVertex3f(x, y, z);
//	
//	n = g->GetNode(to);
//	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	glVertex3f(x, y, z);
//
//	glEnd();
//}

void GraphEnvironment::DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, float mix) const
{
	DrawLERP(disp, a, b, mix,
[](float a, float b, float mix) { return (1-mix)*a+mix*b;},
[](float a, float b, float mix) { return (1-mix)*a+mix*b;});
}

void GraphEnvironment::DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, float mix,
			  std::function<float(float, float, float)> l1,
			  std::function<float(float, float, float)> l2) const
{
	if ((a == 0) || (a->GetNumNodes() == 0) || b == 0 || b->GetNumNodes() == 0 || b->GetNumNodes() != a->GetNumNodes())
		return;

	rgbColor mainColor = GetColor();
//	float mixX = (mix<0.6)?(5*mix/3):1;
//	float mixY = (mix<0.4)?0:5*(mix-0.4)/3;

	edge_iterator ei = a->getEdgeIter();
	for (edge *e = a->edgeIterNext(ei); e; e = a->edgeIterNext(ei))
	{
//		if (e->getFrom() > e->getTo())
//			continue;
		//int x, y;
		//double offsetx, offsety;
		node *n;
		node *n2;
		double x1, y1;
		double x2, y2;

		n = a->GetNode(e->getFrom());
		n2 = b->GetNode(e->getFrom());
		x1 = l1(n->GetLabelF(GraphSearchConstants::kXCoordinate), n2->GetLabelF(GraphSearchConstants::kXCoordinate), mix);
		y1 = l2(n->GetLabelF(GraphSearchConstants::kYCoordinate), n2->GetLabelF(GraphSearchConstants::kYCoordinate), mix);
//		x1 = (1-mixX)*+mixX*n2->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y1 = (1-mixY)*n->GetLabelF(GraphSearchConstants::kYCoordinate)+mixY*n2->GetLabelF(GraphSearchConstants::kYCoordinate);
		
		n = a->GetNode(e->getTo());
		n2 = b->GetNode(e->getTo());
		x2 = l1(n->GetLabelF(GraphSearchConstants::kXCoordinate), n2->GetLabelF(GraphSearchConstants::kXCoordinate), mix);
		y2 = l2(n->GetLabelF(GraphSearchConstants::kYCoordinate), n2->GetLabelF(GraphSearchConstants::kYCoordinate), mix);
//		x2 = (1-mixX)*n->GetLabelF(GraphSearchConstants::kXCoordinate)+mixX*n2->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y2 = (1-mixY)*n->GetLabelF(GraphSearchConstants::kYCoordinate)+mixY*n2->GetLabelF(GraphSearchConstants::kYCoordinate);
		
//		auto i = g->GetNumNodes();
//		auto rad = nodeScale*(double)0.4/(std::max(i, 8));
		disp.DrawLine(Graphics::point(x1, y1), Graphics::point(x2, y2), 0.005, mainColor);
	}

}

void GraphEnvironment::DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, graphState sa, graphState sb, float mix,
			  std::function<float(float, float, float)> l1,
			  std::function<float(float, float, float)> l2) const
{
	if ((a == 0) || (a->GetNumNodes() == 0) || b == 0 || b->GetNumNodes() == 0 || b->GetNumNodes() != a->GetNumNodes())
		return;

	rgbColor mainColor = GetColor();
//	float mixX = (mix<0.6)?(5*mix/3):1;
//	float mixY = (mix<0.4)?0:5*(mix-0.4)/3;

	edge_iterator ei = a->getEdgeIter();
	for (edge *e = a->edgeIterNext(ei); e; e = a->edgeIterNext(ei))
	{
//		if (e->getFrom() > e->getTo())
//			continue;
		//int x, y;
		//double offsetx, offsety;
		node *n;
		node *n2;
		double x1, y1;

		n = a->GetNode(sa);
		n2 = b->GetNode(sb);
		x1 = l1(n->GetLabelF(GraphSearchConstants::kXCoordinate), n2->GetLabelF(GraphSearchConstants::kXCoordinate), mix);
		y1 = l2(n->GetLabelF(GraphSearchConstants::kYCoordinate), n2->GetLabelF(GraphSearchConstants::kYCoordinate), mix);
		
		disp.FillCircle(Graphics::point(x1, y1), 0.005f*7, mainColor);
	}

}


void GraphEnvironment::Draw(Graphics::Display &disp) const
{
	if ((g == 0) || (g->GetNumNodes() == 0)) return;
	
	rgbColor mainColor = GetColor();
	
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		double x1, y1;
		double x2, y2;
		x1 = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y1 = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		
		n = g->GetNode(e->getTo());
		x2 = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y2 = n->GetLabelF(GraphSearchConstants::kYCoordinate);

		auto i = g->GetNumNodes();
		auto rad = nodeScale*(double)0.4/(std::max(i, 8));
//		disp.DrawLine(Graphics::point(x1, y1), Graphics::point(x2, y2), 1.0, mainColor);
//		DrawLine(disp, x1, y1, x2, y2, 1);
		if (directed) // arrow goes to node
		{
			double len = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
			double ratio = (rad+0.1*rad)/len;
			DrawLine(disp, x1, y1, (x2*(1-ratio)+ratio*x1), (y2*(1-ratio)+ratio*y1), 1);
		}
		else {
			DrawLine(disp, x1, y1, x2, y2, 1);
		}
//		disp.DrawLine(Graphics::point(x1, y1), Graphics::point(x2, y2), 0.2*rad, mainColor);
//		disp.DrawLine(Graphics::point(x1, y1), Graphics::point(x2, y2), 0.1*rad*width, SearchEnvironment::color);
	}
	
	char label[5];
	if (drawEdgeCosts)
	{
//		glLineWidth(3.0);
//		glColor4f(0.0, 0.0, 0.0, 1.0);
		edge_iterator ei = g->getEdgeIter();
		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
		{
			if (integerEdgeCosts)
				sprintf(label, "%d", int(e->GetWeight()));
			else
				sprintf(label, "%1.2f", e->GetWeight());
			node *n;
			n = g->GetNode(e->getFrom());
			assert(n!=0);

			double x, y, z;
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
			
			n = g->GetNode(e->getTo());
			assert(n!=0);
			x += n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y += n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z += n->GetLabelF(GraphSearchConstants::kZCoordinate);
			
			//DrawText(x/2, y/2, z/2-0.003, 0.2, label);
			auto i = g->GetNumNodes();
			double rad = nodeScale*(double)0.4/(std::max(i, 8));
			disp.DrawText(label, Graphics::point(x*0.5f, y*0.5f), Colors::black, rad, Graphics::textAlignCenter);
		}
//		glLineWidth(1.0);
	}
	if (drawNodeLabels)
	{
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			node *n = g->GetNode(x);
			auto i = g->GetNumNodes();
			double rad = nodeScale*(double)0.4/(std::max(i, 8));
			disp.DrawText(n->GetName(), Graphics::point(n->GetLabelF(GraphSearchConstants::kXCoordinate),
														n->GetLabelF(GraphSearchConstants::kYCoordinate)),
						  Colors::black, rad, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		}
		}
	
}

void GraphEnvironment::Draw(Graphics::Display &disp, const graphState &l) const
{
	rgbColor c  = GetColor();
	node *n = g->GetNode(l);
	float x, y, z, rad;
	x = (float)n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = (float)n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = (float)n->GetLabelF(GraphSearchConstants::kZCoordinate);
	auto i = g->GetNumNodes();
	rad = nodeScale*0.4f/(std::max(i, 8));
	Graphics::rect rec;
	rec.left= x-rad;
	rec.top = y-rad;
	rec.right = x+rad;
	rec.bottom = y+rad;
	disp.FillCircle({x, y}, rad, c);
	disp.FrameCircle({x, y}, rad, Colors::black, 0.1f*rad);
}

void GraphEnvironment::DrawStateLabel(Graphics::Display &disp, const graphState &l1, const char *txt) const
{
	node *n = g->GetNode(l1);
	double x, y, z, rad;
	x = (double)n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = (double)n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = (double)n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	rad = 1.5*nodeScale*(double)0.4/(g->GetNumNodes());
	auto i = g->GetNumNodes();
	rad = nodeScale*(double)0.4/(std::max(i, 8));

//	disp.DrawText(txt, Graphics::point(x+rad, y-rad), GetColor(), 0.05);
//	disp.DrawText(txt, Graphics::point(x+rad, y-rad), GetColor(), rad);
	disp.DrawText(txt, Graphics::point(x+rad*ONE_OVER_ROOT_TWO+0.15f*rad, y-rad*ONE_OVER_ROOT_TWO-0.15f*rad), GetColor(), rad,
				  Graphics::textAlignLeft, Graphics::textBaselineBottom);
}

void GraphEnvironment::DrawLine(Graphics::Display &disp, const graphState &from, const graphState &to, double width) const
{
	float x1, y1;
	float x2, y2;

	node *n = g->GetNode(from);
	x1 = (float)n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y1 = (float)n->GetLabelF(GraphSearchConstants::kYCoordinate);
	//z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	//glVertex3f(x, y, z);
	
	n = g->GetNode(to);
	x2 = (float)n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y2 = (float)n->GetLabelF(GraphSearchConstants::kYCoordinate);
	//z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	//glVertex3f(x, y, z);
	DrawLine(disp, x1, y1, x2, y2, width);
}

void GraphEnvironment::DrawLine(Graphics::Display &disp, float x1, float y1, float x2, float y2, double width) const
{
	auto i = g->GetNumNodes();
	auto rad = nodeScale*(double)0.4/(std::max(i, 8));
	if (directed)
		disp.DrawArrow(Graphics::point(x1, y1), Graphics::point(x2, y2), 0.1f*rad*width, SearchEnvironment::color);
	else
		disp.DrawLine(Graphics::point(x1, y1), Graphics::point(x2, y2), 0.1f*rad*width, SearchEnvironment::color);
}

Graphics::point GraphEnvironment::GetLocation(const graphState &s) const
{
	float x1, y1;

	node *n = g->GetNode(s);
	x1 = (float)n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y1 = (float)n->GetLabelF(GraphSearchConstants::kYCoordinate);
	Graphics::point p(x1, y1, 0);
	return p;
}


namespace GraphSearchConstants {

	/**
	* GetMapGraph(map)
	 *
	 * Given a map, this function uses the external map interfaces to turn it
	 * into a Graph, and sets the appropriate node numbers for that map. This
	 * function should not be called multiple times on the same map, because
	 * the original Graph map lose it's association with the map.
	 */
	Graph *GetUndirectedGraph(Map *m)
	{ return GetEightConnectedGraph(m, false); }

	Graph *GetGraph(Map *m)
	{ return GetEightConnectedGraph(m); }

	Graph *GetEightConnectedGraph(Map *m, bool directed)
	{
		Graph *g = new Graph();
		AddNodesToGraph(m, g);
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			for (int x = 0; x < m->GetMapWidth(); x++)
			{
				AddEdges(m, g, x, y, directed);//, 1.0, 1.5);
			}
		}
		// printf("Done\n");
		return g;
	}
	
	Graph *GetFourConnectedGraph(Map *m, bool directed)
	{
		Graph *g = new Graph();
		AddNodesToGraph(m, g);
		for (int y = 0; y < m->GetMapHeight(); y++)
		{
			for (int x = 0; x < m->GetMapWidth(); x++)
			{
				AddEdges(m, g, x, y, directed, 1.0, 0.0, 100, 0);//, 1.0, 1.5);
			}
		}
		// printf("Done\n");
		return g;
	}

	void AddNodesToGraph(Map *m, Graph *g)
	{
		char name[32];
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
					double xx, yy, zz, rr;
					m->GetCoord(x, y,xx, yy, zz, rr);
					if (2 != m->GetTerrainType(x, y)>>terrainBits)
						continue;
					sprintf(name, "(%d, %d)", x, y);
					currTile.tile1.node = g->AddNode(n = new node(name));
					n->SetLabelF(kXCoordinate, xx);
					n->SetLabelF(kYCoordinate, yy);
					n->SetLabelF(kZCoordinate, 00);
					n->SetLabelL(kMapX, x);
					n->SetLabelL(kMapY, y);
				}
				else {
					if (m->GetTerrainType(x, y, kLeftEdge) == kGround)
					{
						double xx, yy, zz, rr;
						m->GetCoord(x, y,xx, yy, zz, rr);
						if (2 != m->GetTerrainType(x, y)>>terrainBits)
							continue;
						sprintf(name, "(%d, %d)", x, y);
						currTile.tile1.node = g->AddNode(n = new node(name));
						n->SetLabelF(kXCoordinate, xx);
						n->SetLabelF(kYCoordinate, yy);
						n->SetLabelF(kZCoordinate, 00);
						n->SetLabelL(kMapX, x);
						n->SetLabelL(kMapY, y);
						//					if (currTile.split == kForwardSplit)
						//						n->SetLabelL(kFirstData+2, kTopLeft);
						//					else
						//						n->SetLabelL(kFirstData+2, kBottomLeft);
					}
					
					if (m->GetTerrainType(x, y, kRightEdge) == kGround)
					{
						double xx, yy, zz, rr;
						m->GetCoord(x, y,xx, yy, zz, rr);
						if (m->GetTerrainType(x, y) == kOutOfBounds)
							continue;
						sprintf(name, "(%d, %d)", x, y);
						currTile.tile1.node = g->AddNode(n = new node(name));
						n->SetLabelF(kXCoordinate, xx);
						n->SetLabelF(kYCoordinate, yy);
						n->SetLabelF(kZCoordinate, 00);
						n->SetLabelL(kMapX, x);
						n->SetLabelL(kMapY, y);
						//					if (currTile.split == kForwardSplit)
						//						n->SetLabelL(kFirstData+2, kBottomRight);
						//					else
						//						n->SetLabelL(kFirstData+2, kTopRight);
					}
				}
			}
		}
	}

	/**
	* AddEdges(map, Graph, x, y)
	 *
	 * This is a helper function for GetMapGraph that does the work of adding
	 * the Graph edges. Each edge is only added once, so while the Graph has
	 * directional edges, we treat them as being bidirectional.
	 */
	void AddEdges(Map *m, Graph *g, int x, int y,
				  bool directed,
				  double straigtEdgeCost,
				  double diagEdgeCost,
				  int straightEdgeProb,
				  int diagEdgeProb)
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
				if ((random()%100) < straightEdgeProb)
				{
					e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y).tile1.node, straigtEdgeCost);
					g->AddEdge(e);
					if (directed)
					{
						e = new edge(m->GetTile(x-1, y).tile1.node, m->GetTile(x, y).tile1.node, straigtEdgeCost);
						g->AddEdge(e);
					}
				}
			}
			else if (m->GetTile(x-1, y).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < straightEdgeProb)
				{
					e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y).tile2.node, straigtEdgeCost);
					g->AddEdge(e);
					if (directed)
					{
						e = new edge(m->GetTile(x-1, y).tile2.node, m->GetTile(x, y).tile1.node, straigtEdgeCost);
						g->AddEdge(e);
					}
				}
			}
		}

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
							if ((random()%100) < straightEdgeProb)
							{
								e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x, y-1).tile1.node, straigtEdgeCost);
								g->AddEdge(e);
								if (directed)
								{
									e = new edge(m->GetTile(x, y-1).tile1.node, m->GetTile(x, y).tile1.node, straigtEdgeCost);
									g->AddEdge(e);
								}
							}
						}
					}
					else if (m->GetTile(x, y-1).tile2.node != kNoGraphNode)
					{
						if ((random()%100) < straightEdgeProb)
						{
							e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x, y-1).tile2.node, straigtEdgeCost);
							g->AddEdge(e);
							if (directed)
							{
								e = new edge(m->GetTile(x, y-1).tile2.node, m->GetTile(x, y).tile1.node, straigtEdgeCost);
								g->AddEdge(e);
							}
						}
					}
				}
			}
			else {
				if (m->AdjacentEdges(x, y-1, kInternalEdge) || (m->GetSplit(x, y-1) == kBackwardSplit))
				{
					if ((m->GetTile(x, y).tile2.node != kNoGraphNode) && (m->GetTile(x, y-1).tile1.node != kNoGraphNode))
					{
						if ((random()%100) < straightEdgeProb)
						{
							e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x, y-1).tile1.node, straigtEdgeCost);
							g->AddEdge(e);
							if (directed)
							{
								e = new edge(m->GetTile(x, y-1).tile1.node, m->GetTile(x, y).tile2.node, straigtEdgeCost);
								g->AddEdge(e);
							}
						}
					}
				}
				else if ((m->GetTile(x, y).tile2.node != kNoGraphNode) && (m->GetTile(x, y-1).tile2.node != kNoGraphNode))
				{
					if ((random()%100) < straightEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x, y-1).tile2.node, straigtEdgeCost);
						g->AddEdge(e);
						if (directed)
						{
							e = new edge(m->GetTile(x, y-1).tile2.node, m->GetTile(x, y).tile2.node, straigtEdgeCost);
							g->AddEdge(e);
						}
					}
				}
			}
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
						if ((random()%100) < diagEdgeProb)
						{
							e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y-1).tile1.node, diagEdgeCost);
							g->AddEdge(e);
							if (directed)
							{
								e = new edge(m->GetTile(x-1, y-1).tile1.node, m->GetTile(x, y).tile1.node, diagEdgeCost);
								g->AddEdge(e);
							}
						}
					}
				}
				else if (m->GetTile(x-1, y-1).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < diagEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x-1, y-1).tile2.node, diagEdgeCost);
						g->AddEdge(e);
						if (directed)
						{
							e = new edge(m->GetTile(x-1, y-1).tile2.node, m->GetTile(x, y).tile1.node, diagEdgeCost);
							g->AddEdge(e);
						}
					}
				}
			}
		}

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
						if ((random()%100) < diagEdgeProb)
						{
							e = new edge(m->GetTile(x, y).tile1.node, m->GetTile(x+1, y-1).tile1.node, diagEdgeCost);
							g->AddEdge(e);
							if (directed)
							{
								e = new edge(m->GetTile(x+1, y-1).tile1.node, m->GetTile(x, y).tile1.node, diagEdgeCost);
								g->AddEdge(e);
							}
						}
					}
				}
				else if (m->GetTile(x, y).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < diagEdgeProb)
					{
						e = new edge(m->GetTile(x, y).tile2.node, m->GetTile(x+1, y-1).tile1.node, diagEdgeCost);
						g->AddEdge(e);
						if (directed)
						{
							e = new edge(m->GetTile(x+1, y-1).tile1.node, m->GetTile(x, y).tile2.node, diagEdgeCost);
							g->AddEdge(e);
						}
					}
				}
			}
		}	
	}
}

GraphMapInconsistentHeuristic::GraphMapInconsistentHeuristic(Map *map, Graph *graph)
:GraphDistanceHeuristic(graph), m(map)
{
	displayHeuristic = 0;
	compressed = false;
	SetMode(kMax);
	SetNumUsedHeuristics(1000);
//	std::vector<std::vector<double> > values;
//	FloydWarshall(graph, values);
//	std::vector<int> randomizer;
//	randomizer.resize(values.size());
//	for (int x = 0; x < values.size(); x++)
//		randomizer[x] = x;
//	for (int x = values.size(); x > 0; x--)
//	{
//		int tmp = randomizer[x-1];
//		int switcher = random()%x;
//		randomizer[x-1] = randomizer[switcher];
//		randomizer[switcher] = tmp;
//	}
//	for (int x = 0; x < values.size(); x++)
//		AddHeuristic(values[randomizer[x]], x);
//	for (int x = 0; x < HN /*10*/; x++)
//	{
//		node *n = g->GetRandomNode();
//		graphState loc = n->GetNum();
//		std::vector<double> values;
//		GetOptimalDistances(n, values);
//		AddHeuristic(values, loc);
//	}
} 

double GraphMapInconsistentHeuristic::HCost(const graphState &state1, const graphState &state2) const
{
	long x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
	long y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
	long x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
	long y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
	
	double a = ((x1>x2)?(x1-x2):(x2-x1));
	double b = ((y1>y2)?(y1-y2):(y2-y1));
	double val = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);

	if (hmode == kIgnore)
		return val;
//	if (0 != (x1+x2+y1+y2)%16) // TODO: Remove this line
//		return val;
	//for (unsigned int x = 0; x < heuristics.size(); x++)
	if (hmode == kRandom)
	{
		int x = (x1+x2+y1+y2)%heuristics.size();
		for (int y = 0; y < numHeuristics; y++)
		{
			int offset = heuristics.size()/numHeuristics;
			double hval = heuristics[(x+y*offset)%heuristics.size()][state1]-heuristics[(x+y*offset)%heuristics.size()][state2];
			if (hval < 0) hval = -hval;
			if (fgreater(hval, val))
				val = hval;
		}
	}
	else if (hmode == kMax) // hmode == 2, taking the max
	{
		for (unsigned int i = 0; i < heuristics.size() && i < (unsigned int)numHeuristics; i++)
		{
			double hval = heuristics[i][state1]-heuristics[i][state2];
			if (hval < 0)
				hval = -hval;
			if (fgreater(hval,val))
				val = hval;
		}
	}
	else if (hmode == kGridMax)  // hmode == 3, return max at grid points, otherwise 0
	{
		if ( (x1+x2) % 4 == 0 && (y1+y2) % 4 == 0)
		{
			for (unsigned int i=0;i<heuristics.size();i++)
			{
				double hval = heuristics[i][state1]-heuristics[i][state2];
				if (hval < 0)
					hval = -hval;
				if (fgreater(hval,val))
					val = hval;
			}
		}
		else
			val = 0;
	}
	else if (hmode == kCompressed)
	{
		// at each state we can only look up the heuristic graphstate%H where H is the # of heuristics
		static std::vector<double> vals;
		static std::vector<double> errors;
		static graphState lastGoal = -1;
		
		if (lastGoal != state2)
		{
			FillInCache(vals, errors, state2);
			lastGoal = state2;
		}
		
		if (compressed)
		{
			for (unsigned int x = 0; x < heuristics.size(); x++)
			{
				double hval = vals[x*numHeuristics+state1%numHeuristics]-heuristics[x][state1];
				if (hval < 0)
					hval = -hval;
				hval -= errors[x*numHeuristics+state1%numHeuristics];
				if (fgreater(hval,val))
					val = hval;
			}
		}
		else {
			for (unsigned int x = (state1%numHeuristics); x < heuristics.size(); x+=numHeuristics)
			{
				double hval = vals[x]-heuristics[x][state1];
				if (hval < 0)
					hval = -hval;
				hval -= errors[x];
				if (fgreater(hval,val))
					val = hval;
			}
		}
	}

	return val;
}

// this *should* be correct except for the division at the end
void GraphMapInconsistentHeuristic::Compress()
{
	hmode = kCompressed;
	compressed = true;

	for (unsigned int state1 = 0; state1 < heuristics[0].size(); state1++)
	{
		for (unsigned int x = (state1%numHeuristics), y = 0; x < heuristics.size(); x+=numHeuristics, y++)
		{
			heuristics[y][state1] = heuristics[x][state1];
		}
	}
	assert((heuristics.size()%numHeuristics) == 0);
	heuristics.resize(heuristics.size()/numHeuristics);
}

void GraphMapInconsistentHeuristic::FillInCache(std::vector<double> &vals,
												std::vector<double> &errors,
												graphState state2) const
{
	int unused;
	if (numHeuristics == 0)
	{
		assert(!"No heuristics being used");
		// Note: This would fix it, but breaks const correctness
		// This is old code, so need to re-analyze if we use again
		//numHeuristics = heuristics.size();
	}
	if (!compressed)
	{
		unused = heuristics.size(); // set these values to the uncompressed size
		vals.resize(heuristics.size());
		errors.resize(heuristics.size());
	}
	else {
		unused = numHeuristics*heuristics.size();
		vals.resize(unused);
		errors.resize(unused);
	}
	for (unsigned int x = 0; x < vals.size(); x++)
		vals[x] = errors[x] = -1;

	node *next = g->GetNode(state2);
	next->SetLabelF(kTemporaryLabel, 0.0);
	next->SetKeyLabel(kTemporaryLabel);
	Heap h;
	h.Add(next);

	if (!compressed)
	{
		for (unsigned int x = (state2%numHeuristics); x < heuristics.size(); x+=numHeuristics)
		{
			vals[x] = heuristics[x][state2];
			errors[x] = 0;
			unused--;
		}
	}
	else {
		for (unsigned int x = 0; x < heuristics.size(); x++)
		{
			vals[x*numHeuristics+state2%numHeuristics] = heuristics[x][state2];
			errors[x*numHeuristics+state2%numHeuristics] = 0;
			unused--;
		}
	}

	while ((unused > 0) && (!h.Empty()))
	{
		next = (node*)h.Remove();

		double cost = next->GetLabelF(kTemporaryLabel);
		neighbor_iterator ni = next->getNeighborIter();
		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
		{
			double edgeCost = g->FindEdge(next->GetNum(), tmp)->GetWeight();

			if (compressed)
			{
				for (unsigned int x = 0; x < heuristics.size(); x++)
				{
					if (vals[x*numHeuristics+tmp%numHeuristics] == -1)
					{
						unused--;
						vals[x*numHeuristics+tmp%numHeuristics] = heuristics[x][tmp];
						errors[x*numHeuristics+tmp%numHeuristics] = cost+edgeCost;
					}
				}
			}
			else {
				for (unsigned int x = (tmp%numHeuristics); x < heuristics.size(); x+=numHeuristics)
				{
					if (vals[x] == -1)
					{
						unused--;
						vals[x] = heuristics[x][tmp];
						errors[x] = cost+edgeCost;
					}
				}
			}
			
			node *nb = g->GetNode(tmp);
			if (h.IsIn(nb))
			{
				if (fgreater(nb->GetLabelF(kTemporaryLabel), cost+edgeCost))
				{
					nb->SetLabelF(kTemporaryLabel, cost+edgeCost);
					h.DecreaseKey(nb);
				}
			}
			else {
				nb->SetKeyLabel(kTemporaryLabel);
				nb->SetLabelF(kTemporaryLabel, cost+edgeCost);
				h.Add(nb);
			}
		}
	}	
}

//void GraphMapInconsistentHeuristic::OpenGLDraw() const
//{
//	//static int counter = 50;
//	//counter = (counter+1);
//	if (heuristics.size() == 0)
//	{
//		printf("No heuristics\n");
//		return;
//	}
//
////	long x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
////	long y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
////	long x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
////	long y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
//	
//	GraphEnvironment ge(m, g, 0);
//
//	double max = 0;
//	for (unsigned int a = 0; a < heuristics.back().size(); a++)
//	{
//		if (heuristics.back()[a] > max)
//			max = heuristics.back()[a];
//	}
//	
//	for (unsigned int a = 0; a < heuristics.back().size(); a++)
//	{
////		double x, y, z;
//		if ((hmode == kCompressed) &&
//			((a%heuristics.size() != displayHeuristic) || (heuristics.size() == displayHeuristic)))
//			continue;
//		node *n = g->GetNode(a);
//		
//		if (n)
//		{
//			if (heuristics.size() == displayHeuristic)
//			{
//				ge.SetColor(heuristics[a%heuristics.size()][a]/max, 0, 1-heuristics[a%heuristics.size()][a]/max, 1);
//				ge.OpenGLDraw(a);
//			}
//			else {
//				if (heuristics[displayHeuristic][a] != 0)
//				{
//					ge.SetColor(heuristics[displayHeuristic][a]/max, 0, 1-heuristics[displayHeuristic][a]/max, 1);
//					ge.OpenGLDraw(a);
//				}
//				else {
//					ge.SetColor(1, 1, 1, 1);
//					ge.OpenGLDraw(a);
//				}
//			}
////			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
////			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
////			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
////			
////			glColor3f(0.0, sizes[a]/maxSize, 0.0);
////			if (dist[a] == 0)
////				glColor3f(1, 1, 1);
////			DrawSphere(x, y, z, approxSize);
//		}
//	}
//}
//
//
//void GraphDistanceHeuristic::OpenGLDraw() const
//{
//	//static int counter = 50;
//	//counter = (counter+1);
//	if (heuristics.size() == 0)
//		return;
//
//	double approxSize = 2.0/sqrt(g->GetNumNodes());
//	for (unsigned int a = 0; a < locations.size(); a++)
//	{
//		double x, y, z;
//		node *n = g->GetNode(locations[a]);
//		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//		
//		rgbColor r(1.0, 1.0, 1.0);//GetColor((counter+locations[a])%100, 0, 100, 4);
//		
//		glColor3f(0.0, 0.0, 1.0);
//		DrawSphere(x, y, z, approxSize);
//	}
//	double maxWeight = 0;
//	for (unsigned int a = 0; a < weight.size(); a++)
//		if (weight[a] > maxWeight)
//			maxWeight = weight[a];
//	
//	double maxSize = 0;
//	for (unsigned int a = 0; a < sizes.size(); a++)
//		if (sizes[a] > maxSize)
//			maxSize = sizes[a];
//	
//	for (unsigned int a = 0; a < weight.size(); a++)
//	{
//		double x, y, z;
//		node *n = g->GetNode(a);
//		if (n)
//		{
//			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//					
//			glColor3f(0.0, sizes[a]/maxSize, 0.0);
//			if (dist[a] == 0)
//				glColor3f(1, 1, 1);
//			DrawSphere(x, y, z, approxSize);
//		}
//	}
//}

double GraphDistanceHeuristic::HCost(const graphState &state1, const graphState &state2) const
{
	double val = 0;
	for (unsigned int i = 0; i < heuristics.size(); i++)
	{
		double hval = heuristics[i][state1]-heuristics[i][state2];
		if (hval < 0)
			hval = -hval;
		if (fgreater(hval,val))
			val = hval;
	}
	return val;
}

void GraphDistanceHeuristic::ChooseStartGoal(graphState &start, graphState &goal)
{
	if (heuristics.size() == 0)
		return;
	double minStart=-1, minGoal=-1;

	minStart = heuristics[0][start];
	minGoal = heuristics[0][goal];
	for (unsigned int x = 1; x < heuristics.size(); x++)
	{
		if (heuristics[x][start] < minStart)
			minStart = heuristics[x][start];
		if (heuristics[x][goal] < minGoal)
			minGoal = heuristics[x][goal];
	}
	if (minStart < minGoal)
	{
		graphState tmp;
		tmp = start;
		start = goal;
		goal = tmp;
	}
}

void GraphDistanceHeuristic::AddHeuristic(node *n)
{
	if (placement == kFarPlacement)
		n = FindFarNode(n);
	else if (placement == kAvoidPlacement)
		n = FindAvoidNode(n);
	else if (n == 0)
		n = g->GetRandomNode();
	
	std::vector<double> values;
//	std::cout << "Adding differential heuristic based at " << n->GetLabelL(kMapX) << ", " << n->GetLabelL(kMapY) << std::endl;
	GetOptimalDistances(n, values);
	AddHeuristic(values, n->GetNum());
}

void GraphDistanceHeuristic::AddHeuristic(std::vector<double> &values, graphState location)
{
	heuristics.push_back(values);
	locations.push_back(location);
}



void GraphDistanceHeuristic::GetOptimalDistances(node *n, std::vector<double> &values)
{
	values.resize(g->GetNumNodes());
	for (unsigned int x = 0; x < values.size(); x++)
		values[x] = -1.0;
	n->SetLabelF(kTemporaryLabel, 0.0);
	n->SetKeyLabel(kTemporaryLabel);
	Heap h;
	h.Add(n);
	while (!h.Empty())
	{
		node *next = (node*)h.Remove();
//		printf("Heap size %d, working on node %d cost %f\n", h.size(), next->GetNum(),
//			   next->GetLabelF(kTemporaryLabel));
		double cost = next->GetLabelF(kTemporaryLabel);
		values[next->GetNum()] = next->GetLabelF(kTemporaryLabel);
		neighbor_iterator ni = next->getNeighborIter();
		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
		{
			if (values[tmp] == -1)
			{
				node *nb = g->GetNode(tmp);
				if (h.IsIn(nb))
				{
					if (fgreater(nb->GetLabelF(kTemporaryLabel),
								 cost + g->FindEdge(next->GetNum(), tmp)->GetWeight()))
					{
						nb->SetLabelF(kTemporaryLabel,
									  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
						h.DecreaseKey(nb);
					}
				}
				else {
					nb->SetKeyLabel(kTemporaryLabel);
					nb->SetLabelF(kTemporaryLabel,
								  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
					h.Add(nb);
				}
			}
		}
	}
}

node *GraphDistanceHeuristic::FindAvoidNode(node *n)
{
	if (locations.size() == 0)
		return FindFarNode(0);
	n = g->GetRandomNode();
	// 1. Select vertex r
	if (n == 0)
	{
		int bestSum = MAXINT;
		int bestId = 0;
		for (unsigned int x = 0; x < heuristics[0].size(); x+=1)
		//for (unsigned int x = 0; x < 5; x++)
		{
			if (heuristics[0][x] == -1)
				continue;
			int sum = 0;
			for (unsigned int y = 0; y < heuristics.size(); y++)
				sum += heuristics[y][x];
			int diff = 0;
			sum /= heuristics.size();
			for (unsigned int y = 0; y < heuristics.size(); y++)
				diff = max(diff, fabs(sum-heuristics[y][x]));
			if (diff < bestSum)
			{
				bestId = x;
				bestSum = diff;
			}
		}
		n = g->GetNode(bestId);
//		n = g->GetRandomNode(); // could be done better
//		while (heuristics[0][n->GetNum()] < 1)
//			n = g->GetRandomNode();
	}
//	std::vector<double> dist;
//	std::vector<double> weight;
//	std::vector<double> sizes;
	//printf("Starting from %d\n", n->GetNum());

	// 2. build shortest path tree
	GetOptimalDistances(n, dist);
	weight.resize(dist.size());
	sizes.resize(dist.size());
	// 3. calculate difference between heuristic and actual distance for each node
	for (unsigned int x = 0; x < dist.size(); x++)
	{
		sizes[x] = -1;
		if (dist[x] != -1)
		{
			weight[x] = fabs(dist[x]-HCost(n->GetNum(), x));
		}
		else {
			weight[x] = -1;
		}
	}
	// 4. compute the size of each node (the sum of weights (#3) in the subtree if no existing landmark)
	ComputeSizes(n, dist, weight, sizes);
	// 5. select node, w, with highest weight
	int best = 0;
	for (unsigned int x = 1; x < sizes.size(); x++)
		if (fless(sizes[best], sizes[x]))
			best = x;
	// 6. follow the children of w by largest weight until a leaf is reached & return
	return FindBestChild(best, dist, sizes);
}

void GraphDistanceHeuristic::ComputeSizes(node *n, std::vector<double> &dist,
										  std::vector<double> &weight, std::vector<double> &sizes)
{
	neighbor_iterator ni = n->getNeighborIter();
	int nodeSize = -1;
	for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
	{
		node *nb = g->GetNode(tmp);
		if (sizes[nb->GetNum()] == -1) // not yet computed
		{
			// on shortest path
			if (fequal(dist[nb->GetNum()] - dist[n->GetNum()], g->FindEdge(n->GetNum(), nb->GetNum())->GetWeight()))
			{
				//printf("%d has successor %d\n", n->GetNum(), nb->GetNum());
				ComputeSizes(nb, dist, weight, sizes);
				if (sizes[nb->GetNum()] == 0)
				{
					nodeSize = 0;
				}
				else if (nodeSize == -1)
				{
					nodeSize = sizes[nb->GetNum()];
				}
				else if (nodeSize != 0) {
					nodeSize += sizes[nb->GetNum()];
				}
			}
		}
	}

	if (nodeSize == -1)
	{
		sizes[n->GetNum()] = weight[n->GetNum()];
	}
	else if (nodeSize != 0) {
		sizes[n->GetNum()] = nodeSize+weight[n->GetNum()];
	}
	else {
		sizes[n->GetNum()] = 0;
	}
			
	for (unsigned int x = 0; x < locations.size(); x++)
		if (locations[x] == n->GetNum())
			sizes[n->GetNum()] = 0;
	
	//printf("size at %d is %1.1f\n", n->GetNum(), sizes[n->GetNum()]);
}

node *GraphDistanceHeuristic::FindBestChild(int best, std::vector<double> &dist,
										   std::vector<double> &sizes)
{
	int nextChild = best;
	int nextSize = 0;
	node *n;
	do {
//		printf("Child %d has size %1.1f\n", nextChild, nextSize);
		n = g->GetNode(nextChild);
		nextChild = -1;
		nextSize = 0;
		neighbor_iterator ni = n->getNeighborIter();
		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
		{
			node *nb = g->GetNode(tmp);
			// on shortest path
			if (fequal(dist[nb->GetNum()] - dist[n->GetNum()], g->FindEdge(n->GetNum(), nb->GetNum())->GetWeight()))
			{
				if (nextChild == -1)
				{
					//printf("Next Child %d has size %1.1f\n", nb->GetNum(), sizes[nb->GetNum()]);
					nextSize = sizes[nb->GetNum()];
					nextChild = nb->GetNum();
				}
				else if ((fless(nextSize, sizes[nb->GetNum()])))
				{
					//printf("Next Child %d has size %1.1f\n", nb->GetNum(), sizes[nb->GetNum()]);
					nextSize = sizes[nb->GetNum()];
					nextChild = nb->GetNum();
				}
			}
		}
	} while (nextChild != -1);
	return n;
}	
node *GraphDistanceHeuristic::FindFarNode(node *n)
{
	std::vector<double> values;
	values.resize(g->GetNumNodes());
	for (unsigned int x = 0; x < values.size(); x++)
		values[x] = -1;

	Heap h;
	for (unsigned int x = 0; x < locations.size(); x++)
	{
		n = g->GetNode(locations[x]);
		n->SetLabelF(kTemporaryLabel, 0.0);
		n->SetKeyLabel(kTemporaryLabel);
		h.Add(n);
	}
	if ((locations.size() == 0) && n)
	{
		n->SetLabelF(kTemporaryLabel, 0.0);
		n->SetKeyLabel(kTemporaryLabel);
		h.Add(n);
	}
	else if (n == 0) { // cheap way of finding largest region with high liklihood
		for (int x = 0; x < 10; x++)
		{
			n = g->GetNode(x*g->GetNumNodes()/10);
			n->SetLabelF(kTemporaryLabel, 0.0);
			n->SetKeyLabel(kTemporaryLabel);
			h.Add(n);
		}
	}

	while (!h.Empty())
	{
		node *next = (node*)h.Remove();
		//		printf("Heap size %d, working on node %d cost %f\n", h.size(), next->GetNum(),
		//			   next->GetLabelF(kTemporaryLabel));
		double cost = next->GetLabelF(kTemporaryLabel);
		values[next->GetNum()] = next->GetLabelF(kTemporaryLabel);
		neighbor_iterator ni = next->getNeighborIter();
		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
		{
			if (values[tmp] == -1)
			{
				node *nb = g->GetNode(tmp);
				if (h.IsIn(nb))
				{
					if (fgreater(nb->GetLabelF(kTemporaryLabel),
								 cost + g->FindEdge(next->GetNum(), tmp)->GetWeight()))
					{
						nb->SetLabelF(kTemporaryLabel,
									  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
						h.DecreaseKey(nb);
					}
				}
				else {
					nb->SetKeyLabel(kTemporaryLabel);
					nb->SetLabelF(kTemporaryLabel,
								  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
					h.Add(nb);
				}
			}
		}
		if (h.Empty())
		{
//			printf("Selecting node at (%ld, %ld)\n", next->GetLabelL(GraphSearchConstants::kMapX),
//				   next->GetLabelL(GraphSearchConstants::kMapY));
			return next;
		}
	}
	return 0;
}


//GraphMapInconsistentHeuristic::GraphMapInconsistentHeuristic(Map *map, Graph *graph)
//:m(map), g(graph)
//{
//	for (int x = 0; x < HN /*10*/; x++)
//	{
//		node *n = g->GetRandomNode();
//		graphState loc = n->GetNum();
//		std::vector<double> values;
//		GetOptimalDistances(n, values);
//		AddHeuristic(values, loc);
//	}
//} 
//
//double GraphMapInconsistentHeuristic::HCost(graphState &state1, graphState &state2)
//{
//	int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
//	int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
//	int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
//	int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
//	
//	double a = ((x1>x2)?(x1-x2):(x2-x1));
//	double b = ((y1>y2)?(y1-y2):(y2-y1));
//	double val = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
//
//	if (hmode == 0)
//		return val;
//
//	//for (unsigned int x = 0; x < heuristics.size(); x++)
//	if (hmode == 1) {
//		int x = (x1+x2+y1+y2)%heuristics.size();
//		{
//			double hval = heuristics[x][state1]-heuristics[x][state2];
//			if (hval < 0) hval = -hval;
//			if (fgreater(hval, val))
//				val = hval;
//		}
//	}
//	else if (hmode == 2) { // hmode == 2, taking the max
//		for (unsigned int i=0;i<heuristics.size();i++) {
//			double hval = heuristics[i][state1]-heuristics[i][state2];
//			if (hval < 0)
//				hval = -hval;
//			if (fgreater(hval,val))
//				val = hval;
//		}
//	}
//	else {  // hmode == 3, return max at grid points, otherwise 0
//		if ( (x1+x2) % 4 == 0 && (y1+y2) % 4 == 0) {
//			for (unsigned int i=0;i<heuristics.size();i++) {
//				double hval = heuristics[i][state1]-heuristics[i][state2];
//				if (hval < 0)
//					hval = -hval;
//				if (fgreater(hval,val))
//					val = hval;
//			}
//		}
//		else
//			val = 0;
//	}
//
//	return val;
//}
//
//void GraphMapInconsistentHeuristic::AddHeuristic(std::vector<double> &values,
//												 graphState location)
//{
//	heuristics.push_back(values);
//	locations.push_back(location);
//}
//
//
//void GraphMapInconsistentHeuristic::GetOptimalDistances(node *n, std::vector<double> &values)
//{
//	values.resize(g->GetNumNodes());
//	for (unsigned int x = 0; x < values.size(); x++)
//		values[x] = -1.0;
//	n->SetLabelF(kTemporaryLabel, 0.0);
//	n->SetKeyLabel(kTemporaryLabel);
//	Heap h;
//	h.Add(n);
//	while (!h.Empty())
//	{
//		node *next = (node*)h.Remove();
////		printf("Heap size %d, working on node %d cost %f\n", h.size(), next->GetNum(),
////			   next->GetLabelF(kTemporaryLabel));
//		double cost = next->GetLabelF(kTemporaryLabel);
//		values[next->GetNum()] = next->GetLabelF(kTemporaryLabel);
//		neighbor_iterator ni = next->getNeighborIter();
//		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
//		{
//			if (values[tmp] == -1)
//			{
//				node *nb = g->GetNode(tmp);
//				if (h.IsIn(nb))
//				{
//					if (fgreater(nb->GetLabelF(kTemporaryLabel),
//								 cost + g->FindEdge(next->GetNum(), tmp)->GetWeight()))
//					{
//						nb->SetLabelF(kTemporaryLabel,
//									  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
//						h.DecreaseKey(nb);
//					}
//				}
//				else {
//					nb->SetKeyLabel(kTemporaryLabel);
//					nb->SetLabelF(kTemporaryLabel,
//								  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
//					h.Add(nb);
//				}
//			}
//		}
//	}
//}


/*
AbstractionGraphEnvironment::AbstractionGraphEnvironment( GraphAbstraction *_gabs, unsigned int level, GraphHeuristic *gh ):
	GraphEnvironment( _gabs->GetAbstractGraph(level), gh ), gabs(_gabs)
{
	// compute graph scale
	node_iterator ni = g->getNodeIter();
	node *n;
	double min_x = DBL_MAX, max_x = DBL_MIN;
	double min_y = DBL_MAX, max_y = DBL_MIN;
	double min_z = DBL_MAX, max_z = DBL_MIN;
	n = g->nodeIterNext( ni );
	while( n != NULL ) {
		double x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		double y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		double z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		if ( x < min_x ) min_x = x;
		if ( x > max_x ) max_x = x;
		if ( y < min_y ) min_y = y;
		if ( y > max_y ) max_y = y;
		if ( z < min_z ) min_z = z;
		if ( z > max_z ) max_z = z;
		n = g->nodeIterNext( ni );
	}
	graphscale = 0.;
	int count = 0;
	if ( fgreater(max_x, min_x) ) { graphscale += max_x-min_x; count++; }
	if ( fgreater(max_y, min_y) ) { graphscale += max_y-min_y; count++; }
	if ( fgreater(max_z, min_z) ) { graphscale += max_z-min_z; count++; }
	// average over all the dimensions and divide by root(nodecount,dimensions)
	graphscale /= count * pow( (double)g->GetNumNodes(), 1./(double)count );
};

AbstractionGraphEnvironment::~AbstractionGraphEnvironment() {
	//~GraphEnvironment();
};

void AbstractionGraphEnvironment::OpenGLDraw() const {
	if ((g == 0) || (g->GetNumNodes() == 0)) return;

	glBegin(GL_LINES);

	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		glColor3f( 0,0,0 );
		//glColor3f(1, 0, 0);
		//if (e->getMarked())
		//	glColor3f(1, 1, 1);
		
 double x, y, z;
		x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		glVertex3f(x, y, z);
		
		n = g->GetNode(e->getTo());
		x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		
		glVertex3f(x, y, z);
	}
	glEnd();
};

*/
