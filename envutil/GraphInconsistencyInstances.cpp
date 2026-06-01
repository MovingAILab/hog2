//
//  GraphInconsistencyInstances.cpp
//  Inconsistency
//
//  Created by Nathan Sturtevant on 2/19/19.
//  Copyright © 2019 NS Software. All rights reserved.
//

#include "GraphInconsistencyInstances.h"
#include "GraphEnvironment.h"

namespace GraphInconsistencyExamples {
	
	// This graph is from Mero, 1984
	Graph *GetPolyGraph(int numExampleNodes)
	{
		Graph *g = new Graph();
		node *n;
		
		n = new node("S");
		n->SetLabelL(kHeuristic, 0);
		g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, 0);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, -0.7);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		n = new node("m");
		n->SetLabelL(kHeuristic, 0);
		g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, 0);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.33333);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		char nodeName[2] = {'a', 0};
		for (int x = 0; x < numExampleNodes; x++)
		{
			n = new node(nodeName);
			nodeName[0]++;
			n->SetLabelL(kHeuristic, numExampleNodes+x);
			
			n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6*(double(x)/(numExampleNodes-1.0)));
			n->SetLabelF(GraphSearchConstants::kYCoordinate, -0.18);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
			
			g->AddNode(n);
			g->AddEdge(new edge(0, n->GetNum(), 1));
			g->AddEdge(new edge(n->GetNum(), 1, numExampleNodes-x));
		}
		
		nodeName[0] = 'A';
		unsigned int last = 1;
		for (int x = 0; x < numExampleNodes; x++)
		{
			n = new node(nodeName);
			nodeName[0]++;
			n->SetLabelL(kHeuristic, 0);
			g->AddNode(n);
			if (x == numExampleNodes-1)
				g->AddEdge(new edge(last, n->GetNum(), numExampleNodes-1));
			else
				g->AddEdge(new edge(last, n->GetNum(), 1));
			last = n->GetNum();
			
			n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6*(double(x)/(numExampleNodes-1.0)));
			n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.7);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		}
		return g;
	}
	
	Graph *GetExpoGraph(int numExampleNodes, bool a)
	{
		Graph *g = new Graph();
		char nodeName[2] = {'a', 0};
		nodeName[0]+=numExampleNodes-1;
		for (int x = 0; x <= numExampleNodes; x++)
		{
			node *n;
			if (x == 0)
			{
				g->AddNode(n = new node("G"));
			}
			else {
				g->AddNode(n = new node(nodeName));
				nodeName[0]--;
			}
			if (x == 0 || x == 1)
			{
				n->SetLabelL(kHeuristic, 0);
			}
			else {
				n->SetLabelL(kHeuristic, (1<<(x-1)) + 2*x - 3);
			}
			
			double val = 1.5*PI*x/(numExampleNodes+1.0)-PI/4;
			n->SetLabelF(GraphSearchConstants::kXCoordinate, 0.8*sin(val));
			n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.8*cos(val));
			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		}
		
		g->AddEdge(new edge(1,0,(1<<(numExampleNodes-1)) + numExampleNodes - 2));
		for (int x = 1; x <= numExampleNodes; x++)
		{
			for (int y = x+1; y <= numExampleNodes; y++)
			{
				g->AddEdge(new edge(y, x, (1<<(y-2)) + y - (1<<(x-1)) - x));
			}
		}
		
		if (!a)
			g->GetNode(numExampleNodes)->SetLabelL(kHeuristic, 0);

		return g;
	}
	
	// This graph is from Martelli, 1979
	Graph *GetExpoGraphA(int N)
	{
		return GetExpoGraph(N, true);
	}
	
	// This graph is from Mero, 1984
	Graph *GetExpoGraphB(int N)
	{
		return GetExpoGraph(N, false);
	}

	
	Graph *GetWeightedInconsistency(float w, int numExampleNodes)
	{
		{
			Graph *g = new Graph();
			node *n;

			char nodeName[3] = {'t', '0', 0};
			for (int x = 0; x < numExampleNodes; x++)
			{
				n = new node(nodeName);
				nodeName[1]++;
				n->SetLabelF(kHeuristic, numExampleNodes-x-1);
				n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6*(double(x)/(numExampleNodes-1)));
				n->SetLabelF(GraphSearchConstants::kYCoordinate, -0.5);
				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				g->AddNode(n);
				if (x > 0)
					g->AddEdge(new edge(n->GetNum()-1, n->GetNum(), 1));
			}
			nodeName[0] = 'b';
			nodeName[1] = '0';
			for (int x = 0; x < numExampleNodes; x++)
			{
				n = new node(nodeName);
				nodeName[1]++;
				n->SetLabelF(kHeuristic, numExampleNodes-x-1);
				n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6*(double(x)/(numExampleNodes-1)));
				n->SetLabelF(GraphSearchConstants::kYCoordinate, 0);
				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				g->AddNode(n);
				g->AddEdge(new edge(n->GetNum()-numExampleNodes, n->GetNum(), (x+1.0)*(w-1)/w));///(std::max(2.0f, w/(w-1.0f)))));
				if (x > 0)
					g->AddEdge(new edge(n->GetNum()-1, n->GetNum(), 1));
			}
			n = new node("e");
			n->SetLabelF(kHeuristic, numExampleNodes);
			n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6*(double(0)/(numExampleNodes-1)));
			n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.5);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
			g->AddNode(n);
			g->AddEdge(new edge(n->GetNum()-numExampleNodes, n->GetNum(), 1));

			n = new node("g");
			n->SetLabelF(kHeuristic, 0);
			n->SetLabelF(GraphSearchConstants::kXCoordinate, -0.8+1.6);
			n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.5);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
			g->AddNode(n);
			g->AddEdge(new edge(n->GetNum()-1, n->GetNum(), numExampleNodes));

			return g;
		}
	}

}
