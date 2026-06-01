/*
 *  $Id: width.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * Helper functions for determining edge widths and node widths.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef WIDTH_H
#define WIDTH_H

#include "Graph.h"

// THESE MACROS ARE EVIL (side-effects)
//#define MIN(A,B) (((A) < (B)) ? (A) : (B))
//#define MAX(A,B) (((A) > (B)) ? (A) : (B))

//node* findNodeAt(int x, int y, Graph* g);
//edge* findEdgeBetween(int from, int to, Graph* g);
//float findMin(node* n);
//float findMax(node* n);
//float findMaxAbstracted(node* x, Graph* g);
//float findMinAbstractedNode(node* x, Graph* g);
//float minSpanningTree(node* n, Graph* g);
//bool edgeInVector(edge* e, std::vector<edge*> edges);
//void sortEdgeWidths(std::vector<edge*>* edges);
//bool hasCycle(edge* e, std::vector<int> nodes);

#endif
