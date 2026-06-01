/*
 *  $Id: width.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "GraphAbstraction.h"
#include "Map.h"
#include "Width.h"

using namespace std;

///**
// * Finds the node with the given x and y coordinates
// *
// * This is quite inefficient and should be done in a better way.
// */
//node* findNodeAt(int x, int y, Graph* g) 
//{
//  node_iterator i = g->getNodeIter();
//	
//  for (node* n = g->nodeIterNext(i); n; n = g->nodeIterNext(i))
//	{
//        if (n->GetLabelL(kFirstData) == x && n->GetLabelL(kFirstData+1) == y)
//      return n;
//  }
//  return 0;
//}
//
///**
// * Finds the minimum edge width of all edges from the node
// */
//float findMin(node* n)
//{
//  float min;
//  edge_iterator i = n->getEdgeIter();
//  edge* e = n->edgeIterNext(i);
//	
//  if (!e) return 0.0f;
//	
//  min = e->getWidth();
//	
//  for (e = n->edgeIterNext(i); e; e = n->edgeIterNext(i))
//	{
//        if (e->getWidth() < min) min = e->getWidth();
//  }
//	
//  return min;
//}
//
//
///**
// * Finds the maximum edge width of all edges from the node
// */
//float findMax(node* n)
//{
//	if (!n)
//	{
//    		return 0.0f;
//	}
//
//	float max;
//	edge_iterator i = n->getEdgeIter();
//	edge* e = n->edgeIterNext(i);
//	
//	if (!e)
//	{
//    		return 0.0f;
//	}
//	
//	max = e->getWidth();
//	
//	for (e = n->edgeIterNext(i); e; e = n->edgeIterNext(i))
//	{
//    		if (e->getWidth() > max)
//	{
//    			max = e->getWidth();
//		}
//	}
//	
//	return max;
//}
//
//
//
///**
// * Finds the maximum edge width of all nodes abstracted by x.
// */
//float findMaxAbstracted(node* x, Graph* g)
//{
//	float max = 0.0f;
//	float tempMax;
//	int numNodes = x->GetLabelL(kNumAbstractedNodes);
//	node* n;
//	
//	for (int i = 0; i < numNodes; i++)
//	{
//    		n = g->GetNode(x->GetLabelL(kFirstData+i));
//		tempMax = findMax(n);
//		
//		if (tempMax > max)
//	{
//    			max = tempMax;
//		}
//	}
//		
//	return max;
//}
//
//
///**
// * Finds the minimum node width of all nodes abstracted by x
// */
//float findMinAbstractedNode(node* x, Graph* g)
//{
//  float min = 0.0f;
//  int numNodes = x->GetLabelL(kNumAbstractedNodes);
//  node* n;
//	
//  n = g->GetNode(x->GetLabelL(kFirstData));
//  min = n->getWidth();
//	
//  edge_iterator ei;
//	
//  for (int i = 0; i < numNodes; i++)
//	{
//        n = g->GetNode(x->GetLabelL(kFirstData+i));
//		
//    ei = n->getEdgeIter();
//    for (edge* e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
//	{
//          if (g->GetNode(e->getTo())->getWidth() < min)
//	min = g->GetNode(e->getTo())->getWidth();
//				
//      if (g->GetNode(e->getFrom())->getWidth() < min)
//	min = g->GetNode(e->getFrom())->getWidth();
//    }
//		
//    if (n->getWidth() < min)
//	{
//          min = n->getWidth();
//    }
//  }
//	
//  return min;
//}
//
//
///**
// * Finds the minimum edge width in a spanning tree of maximal weight constructed from
// * the edges between the nodes abstracted within n.
// *
// * The Graph g should be the abstracted Graph level one below that of which 'n' is in.
// */
//float minSpanningTree(node* n, Graph* g)
//{
//  int numNodes = n->GetLabelL(kNumAbstractedNodes);
//	
//  if (numNodes <= 0) return 0.0f;
//	
//  node* current;
//  edge_iterator ei;
//  vector<edge*> edges;
//	
//  // Put all the edges in a vector
//  for (int i = 0; i < numNodes; i++)
//	{
//        current = g->GetNode(n->GetLabelL(kFirstData+i));
//		
//    ei = current->getEdgeIter();
//    for (edge* e = current->edgeIterNext(ei); e; e = current->edgeIterNext(ei))
//	{
//          if (!edgeInVector(e, edges)) edges.push_back(e);
//    }
//  }
//	
//  // Sort the edges based on their width (from increasing to decreasing)
//  sortEdgeWidths(&edges);
//
//  // Calculate the spanning tree of maximal weight
//  vector<edge*> tree;
//  vector<int> nodes;
//	
//  for (unsigned int i = 0; i < edges.size(); i++)
//	{
//        if (!hasCycle(edges[i], nodes))
//	{
//          tree.push_back(edges[i]);
//      nodes.push_back(edges[i]->getTo());
//      nodes.push_back(edges[i]->getFrom());
//    }
//  }
//	
//  if (tree.empty())
//    return 0.0f;
//	
//  return tree.back()->getWidth();
//}
//
//
///**
// * Returns true if 'e' is in 'edges' (based on it's from and to values),
// * false otherwise.
// */
//bool edgeInVector(edge* e, vector<edge*> edges)
//{
//  for (unsigned int i = 0; i < edges.size(); i++)
//	{
//        if ((e->getFrom() == edges[i]->getFrom() && e->getTo() == edges[i]->getTo()) ||
//       (e->getFrom() == edges[i]->getTo() && e->getTo() == edges[i]->getFrom()))
//	{
//          return true;
//    }
//  }
//  return false;
//}
//
//
///**
// * Sort all the edges in descending order, based on their edge width.
// * Insertion sort (shouldn't be many values in the vector).
// */
//void sortEdgeWidths(vector<edge*>* edges)
//{
//  unsigned int i, j;
//  edge* v;
//
//  for (i=1; i < edges->size(); i++)
//	{
//        v = (*edges)[i];
//    j = i;
//		
//    while ( (*edges)[j-1]->getWidth() < v->getWidth() )
//	{
//          (*edges)[j] = (*edges)[j-1]; 
//      j = j-1;
//      if (j <= 0) break;
//    }
//    (*edges)[j] = v;
//  }
//}
//
//
///**
// * Returns true if adding e will cause a cycle, false otherwise
// */
//bool hasCycle(edge* e, vector<int> nodes)
//{
//  bool b_to = false;
//  bool b_from = false;
//  int to = e->getTo();
//  int from = e->getFrom();
//	
//  for (unsigned int i = 0; i < nodes.size(); i++)
//	{
//        if (to == nodes[i])
//      b_to = true;
//    else if (from == nodes[i])
//      b_from = true;
//  }
//	
//  return b_to && b_from;
//}
//
