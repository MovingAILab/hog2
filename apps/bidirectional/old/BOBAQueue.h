//
//  BOBAQueue.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/10/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef BOBAQueue_h
#define BOBAQueue_h

#include "BDOpenClosed.h"

//low g -> low f
template <class state>
struct BOBACompareOpenReady {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(i1.g, i2.g))
		{
			return (!fless(f1, f2)); //equal g, low f over high
		}
		return (fgreater(i1.g, i2.g)); // low g over high
	}
};

template <class state>
struct BOBACompareOpenWaiting {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(f1, f2))
		{
			return (!fless(i1.g, i2.g)); // high g-cost over low
		}
		return (fgreater(f1, f2)); // low f over high
	}
};

template <typename state, int epsilon = 1>
class BOBAQueue {
public:
	bool GetNextPair(uint64_t &nextForward, uint64_t &nextBackward)
	{
		// move items with f<CLowerBound to ready
		while (forwardQueue.OpenWaitingSize() != 0 && fless(forwardQueue.PeekAt(kOpenWaiting).g+forwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
		{
			forwardQueue.PutToReady();
		}
		while (backwardQueue.OpenWaitingSize() != 0 && fless(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
		{
			backwardQueue.PutToReady();
		}

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

//			if (forwardQueue.OpenWaitingSize() != 0 && backwardQueue.OpenWaitingSize() != 0)
//			{
//				const auto i3 = forwardQueue.PeekAt(kOpenWaiting);
//				const auto i4 = backwardQueue.PeekAt(kOpenWaiting);
//				if ((!fgreater(i3.g+i3.h, CLowerBound)) && (!fgreater(i4.g+i4.h, CLowerBound)))
//				{
//					changed = true;
//					if (fgreater(i3.g, i4.g))
//						forwardQueue.PutToReady();
//					else
//						backwardQueue.PutToReady();
//				}
//				else if (!fgreater(i3.g+i3.h, CLowerBound))
//				{
//					changed = true;
//					forwardQueue.PutToReady();
//				}
//				else if (!fgreater(i4.g+i4.h, CLowerBound))
//				{
//					changed = true;
//					backwardQueue.PutToReady();
//				}
//			}
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
	void Reset()
	{
		CLowerBound = 0;
		forwardQueue.Reset();
		backwardQueue.Reset();
	}
	double GetLowerBound() { return CLowerBound; }
	BDOpenClosed<state, BOBACompareOpenReady<state>, BOBACompareOpenWaiting<state>> forwardQueue;
	BDOpenClosed<state, BOBACompareOpenReady<state>, BOBACompareOpenWaiting<state>> backwardQueue;
private:
	double CLowerBound;
};

#endif /* BOBAQueue_h */
