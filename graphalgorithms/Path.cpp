/*
 *  $Id: path.cpp
 *  hog2
 *
 *  Created by Vadim Bulitko on 11/16/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Path.h"

// Returns the length of the path -- number of steps
unsigned path::length()
{
	return (n == NULL) ? 0 : ((next == NULL) ? 1 : (1+next->length()));
}

// Return the cummulative distance along a path
//double path::distance(GraphAbstraction* aMap)
//{
//	// check of the path is empty or has only one node
//	if ((n == NULL) || (next == NULL))
//		return 0.0;
//
//	// Otherwise, iterate through the path
//	return aMap->h(n,next->n) + next->distance(aMap);
//}

// Return the neighbor complexity of a path
// That is the cummulative degree of all vertices except the last one
unsigned path::degree()
{
	// check of the path is empty or has only one node
	if ((n == NULL) || (next == NULL))
		return 0;
	
	// Otherwise, iterate through the rest of the path
	return n->GetNumEdges() + next->degree();
}

// Print the path
void path::Print(bool beginning)
{
	if (beginning)
		printf("[");
	
	if (n != NULL)
		printf("%d",n->GetNum());
	else
		printf("NULL");
	
	if (next != NULL) {
		printf(",");
		next->Print(false);
	}
	else
		printf("]");
}

path *path::reverse()
{
	if (next == 0)
		return this;
	path *tmp = next->reverse();
	next->next = this;
	next = 0;
	return tmp;
}
