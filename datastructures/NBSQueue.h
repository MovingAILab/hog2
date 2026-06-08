//
//  NBSQueue.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/10/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef NBSQueue_h
#define NBSQueue_h

#include "BDOpenClosed.h"
#include "BDIndexOpenClosed.h"

//low g -> low f
template <class state, class DS>
struct NBSCompareOpenReady {
	bool operator()(const DS &i1, const DS &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(i1.g, i2.g))
		{
			return (!fless(f1, f2));
		}
		return (fgreater(i1.g, i2.g)); // low g over high
	}
};

template <class state, class DS>
struct NBSCompareOpenWaiting {
	bool operator()(const DS &i1, const DS &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(f1, f2))
		{
			return (!fgreater(i1.g, i2.g));
		}
		return (fgreater(f1, f2)); // low f over high
	}
};



template <typename state, int epsilon = 0, bool moveLessEqToOpen = true, class pQueue = BDOpenClosed<state, NBSCompareOpenReady<state, BDOpenClosedData<state>>, NBSCompareOpenWaiting<state, BDOpenClosedData<state>>>>
class NBSQueue {
public:
	bool GetNextPair(uint64_t &nextForward, uint64_t &nextBackward)
	{
		// move items with f<CLowerBound to ready
		while (forwardQueue.OpenWaitingSize() != 0)
		{
			if (moveLessEqToOpen)
			{
				if (flesseq(forwardQueue.PeekAt(kOpenWaiting).g+forwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
					forwardQueue.PutToReady();
				else
					break;
			}
			else if (!moveLessEqToOpen)
			{
				if (fless(forwardQueue.PeekAt(kOpenWaiting).g+forwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
					forwardQueue.PutToReady();
				else
					break;
			}
		}
		while (backwardQueue.OpenWaitingSize() != 0)
		{
			if (moveLessEqToOpen)
			{
				if (flesseq(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
					backwardQueue.PutToReady();
				else
					break;
			}
			else if (!moveLessEqToOpen)
			{
				if (fless(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
					backwardQueue.PutToReady();
				else
					break;
			}
		}
//		while (backwardQueue.OpenWaitingSize() != 0)
//		{
//			if (moveLessEqToOpen &&
//				flesseq(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
//			{
//				backwardQueue.PutToReady();
//			}
//			else if (!moveLessEqToOpen &&
//					 fless(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
//			{
//				backwardQueue.PutToReady();
//			}
//		}

		while (true)
		{
			if (forwardQueue.OpenSize() == 0)
				return false;
			if (backwardQueue.OpenSize() == 0)
				return false;
			if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0) &&
				(!fgreater(forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound)))
			{
				nextForward = forwardQueue.Peek(kOpenReady);
				nextBackward = backwardQueue.Peek(kOpenReady);
				return true;
			}
			bool changed = false;

			if (backwardQueue.OpenWaitingSize() != 0)
			{
				const auto i4 = backwardQueue.PeekAt(kOpenWaiting);
				if (!fgreater(i4.g+i4.h, CLowerBound))
				{
					changed = true;
					backwardQueue.PutToReady();
				}
			}
			if (forwardQueue.OpenWaitingSize() != 0)
			{
				const auto i3 = forwardQueue.PeekAt(kOpenWaiting);
				if (!fgreater(i3.g+i3.h, CLowerBound))
				{
					changed = true;
					forwardQueue.PutToReady();
				}
			}
			if (!changed)
			{
				CLowerBound = DBL_MAX;
				if (forwardQueue.OpenWaitingSize() != 0)
				{
					const auto i5 = forwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i5.g+i5.h);
				}
				if (backwardQueue.OpenWaitingSize() != 0)
				{
					const auto i6 = backwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i6.g+i6.h);
				}
				if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0))
					CLowerBound = std::min(CLowerBound, forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon);
			}

		}
		return false;
	}
	void Reset(int maxHash)
	{
		CLowerBound = 0;
		forwardQueue.Reset(maxHash);
		backwardQueue.Reset(maxHash);
	}
	double GetLowerBound() { return CLowerBound; }
	bool TerminateOnG() {
		if (forwardQueue.OpenReadySize() > 0 && backwardQueue.OpenReadySize() > 0)
			return CLowerBound == forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon;
		return false;
	}
	pQueue forwardQueue;
	pQueue backwardQueue;
private:
	double CLowerBound;
};

#endif /* NBSQueue_h */
