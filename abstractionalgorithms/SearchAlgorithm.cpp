/*
 *  $Id: SearchAlgorithm.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

//#ifdef OS_MAC
//#include <CoreServices/CoreServices.h>
//#endif

#include "SearchAlgorithm.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <stdint.h>
#include "Timer.h"

using namespace GraphAbstractionConstants;
using namespace std;

static const int verbose = 0;

void DoRandomPath(GraphAbstraction *aMap, SearchAlgorithm *sa, bool repeat)
{
	static double lastLength, lastTime;
	static node *r1 = 0, *r2 = 0;
	Graph *g = aMap->GetAbstractGraph(0);
	//if (verbose) cout << "Clearing marked nodes" << endl;
	//aMap->ClearMarkedNodes();
	
	//	r1 = ((MapAbstraction*)aMap)->GetNodeFromMap(257, 449);
	//	r2 = ((MapAbstraction*)aMap)->GetNodeFromMap(319, 458);
	if ((!repeat) || (r1 == 0) || (r2 == 0))
	{
		lastLength = 0;
		lastTime = 1;
		do {
			//      do {
			r1 = g->GetRandomNode();
			//      } while (aMap->GetMap()->GetTerrainType((long)r1->GetLabelL(kFirstData), (long)r1->GetLabelL(kFirstData+1)) == kOutOfBounds);
			//      do {
			r2 = g->GetRandomNode();
			//      } while (aMap->GetMap()->GetTerrainType((long)r2->GetLabelL(kFirstData), (long)r2->GetLabelL(kFirstData+1)) == kOutOfBounds);
		} while (!aMap->Pathable(r1, r2));
	}
	
	if (verbose)
	{
		cout << "Attempting path between nodes:" << endl;
		cout << (*r1) << endl << (*r2) << endl;
	}
	
	//	while (r1 != r2)
	//	{
	//		r1 = getPathStep(r1, r2);
	//		if (verbose)
	//			cout << "Stepping to " << (*r1) << endl;
	//	}
	
	// ignoring return value! Leaking memory!
	
	Timer t;
	t.StartTimer();
	
	path *p;
	
	p = sa->GetPath(aMap, r1, r2);
	//	if (optimal)
	//		//p = getLibraryPath(r1, r2);
	//		p = GetPath(r1, r2);
	//	else
	//		//p = getLibraryPath(r1, r2);
	//		p = getApproximatePath(r1, r2);
	//	//		while (r1 != r2)
	//	//			r1 = getPathStep(r1, r2);
	
	
	t.EndTimer();
	
	int cnt = 0;
	double length = 0;
	for (path *q = p; q; q = q->next)
	{
		if (q && q->next)
		{
			double t1, t2;
			t1 = q->n->GetLabelL(kFirstData)-q->next->n->GetLabelL(kFirstData);
			t2 = q->n->GetLabelL(kFirstData+1)-q->next->n->GetLabelL(kFirstData+1);
			length += sqrt(t1*t1+t2*t2);
		}
		cnt++;
	}
	
	cout << "Steps: " << cnt << ", len: " << length << ", time: " << t.GetElapsedTime();
	//<< ",  time/step: " << (double)nanosecs/(1000*cnt) << ", time/unit: " << (double)nanosecs/(1000*length);
		cout << "ms, h() = " << aMap->h(r1, r2) << ", nodes: " << sa->GetNodesExpanded() << endl;
		
		//cout << "DATA\t" << nanosecs << "\t" << length << endl;
		if (!repeat)
		{
			lastLength = length;
			lastTime = t.GetElapsedTime();
		} else {
			cout << "Comparison: " << lastLength/length << "x longer; but " << t.GetElapsedTime()/lastTime << "x faster." << endl;
		}
#endif	
		
		//aMap->clearDisplayLists();
}
