//
//  BOBAQueueGF.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/11/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef BOBAQueueGF_h
#define BOBAQueueGF_h

#include "BDOpenClosed.h"

//low g -> low f
template <class state>
struct BOBAGLowHigh {
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
struct BOBAFLowHigh {
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
class BOBAQueueGF {
public:
	bool GetNextPair(uint64_t &nextForward, uint64_t &nextBackward)
	{
		// move items with f<CLowerBound to ready
		while (forwardQueue.OpenWaitingSize() != 0 && fless(forwardQueue.PeekAt(kOpenWaiting).g, GForwardMin))
		{
			forwardQueue.PutToReady();
		}
		while (backwardQueue.OpenWaitingSize() != 0 && fless(backwardQueue.PeekAt(kOpenWaiting).g, GBackwardMin))
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
				(flesseq(forwardQueue.PeekAt(kOpenReady).g + forwardQueue.PeekAt(kOpenReady).h, CLowerBound)) &&
				(flesseq(backwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).h, CLowerBound)))
			{
				nextForward = forwardQueue.Peek(kOpenReady);
				nextBackward = backwardQueue.Peek(kOpenReady);
				return true;
			}
			bool changed = false;
			
			if (backwardQueue.OpenWaitingSize() != 0)
			{
				const auto i4 = backwardQueue.PeekAt(kOpenWaiting);
				if (flesseq(i4.g, GBackwardMin))
				{
					changed = true;
					backwardQueue.PutToReady();
				}
			}
			if (forwardQueue.OpenWaitingSize() != 0)
			{
				const auto i3 = forwardQueue.PeekAt(kOpenWaiting);
				if (flesseq(i3.g, GForwardMin))
				{
					changed = true;
					forwardQueue.PutToReady();
				}
			}
			if (!changed) // f-costs both greater than CLowerBound
			{
				// Choose to increase either:
				// * next forward g (+backwardg+epsilon)
				// * next backward g (+forwardg+epsilon)
				// * forward f (CLowerBound)
				// * backward f
				// and increase the relevant bound
				// (choose min of all)
				double gBound = GForwardMin+GBackwardMin+epsilon;
				double newGForward = DBL_MAX;
				double newGBackward = DBL_MAX;
				double fFBound = DBL_MAX;
				if (forwardQueue.OpenReadySize() != 0)
				{
					const auto i5 = forwardQueue.PeekAt(kOpenReady);
					fFBound = i5.g+i5.h;
				}
				if (forwardQueue.OpenWaitingSize() != 0)
				{
					const auto i5 = forwardQueue.PeekAt(kOpenWaiting);
					newGForward = i5.g;
				}
				double fBBound = DBL_MAX;
				if (backwardQueue.OpenReadySize() != 0)
				{
					const auto i5 = backwardQueue.PeekAt(kOpenReady);
					fFBound = i5.g+i5.h;
				}
				if (backwardQueue.OpenWaitingSize() != 0)
				{
					const auto i5 = backwardQueue.PeekAt(kOpenWaiting);
					newGBackward = i5.g;
				}
				//assert(fequal(CLowerBound, std::min(gBound, std::min(fFBound, fBBound))));

				if (fgreatereq(gBound, std::max(fFBound, fBBound)))
				{
					//choose the side with lower g-cost
					if (fless(newGForward, newGBackward))
					{
						forwardQueue.PutToReady();
						CLowerBound = std::min(newGForward+GBackwardMin+epsilon, std::max(fFBound, fBBound));
						GForwardMin = newGForward;
					}
					else {
						backwardQueue.PutToReady();
						CLowerBound = std::min(GForwardMin+newGBackward+epsilon, std::max(fFBound, fBBound));
						GBackwardMin = newGBackward;
					}
					//PutToReady();
					continue;
				}

				// Suppose: g+g+e = 4+4+1 = 9
				// newGForward = 12
				// newGBackward = 12
				// Ff = 10
				// Fb = 11
				// Then, increase CLowerBound to 11

				// Suppose: g+g+e = 4+4+1 = 9
				// newGForward = 6 (6+4+1 = 11)
				// newGBackward = 5 (4 + 5 + 1 = 10)
				// Ff = 10
				// Fb = 5
				// Then, increase CLowerBound to 10

				// Suppose: g+g+e = 4+4+1 = 9
				// newGForward = 10
				// newGBackward = 11
				// Ff = 12
				// Fb = 12
				// Then, increase CLowerBound to 10 and GForwardMin to 10

				// new values increase by:
				// newGForward-GForwardMin (increase forward g)
				// newGBackward-GBackwardMin (increase backward g)
				// max(fFBound-CLowerBound,fBBound-CLowerBound) (increase CLowerBound)
//				if (fless(newGForward-GForwardMin, std::min(newGBackward-GBackwardMin,
//															std::max(fFBound-CLowerBound,fBBound-CLowerBound))))
//				{
//					GForwardMin = newGForward;
//				}
//				else if (fless(newGBackward-GBackwardMin,std::max(fFBound-CLowerBound,fBBound-CLowerBound)))
//				{
//					GForwardMin = newGBackward;
//				}
//				else {
//					CLowerBound = std::max(fFBound, fBBound);
//				}
			}
			
		}
		return false;
	}
	void Reset()
	{
		CLowerBound = 0;
		GForwardMin = 0;
		GBackwardMin = 0;
		forwardQueue.Reset();
		backwardQueue.Reset();
	}
	double GetLowerBound() { return CLowerBound; }
	BDOpenClosed<state, BOBAGLowHigh<state>, BOBAFLowHigh<state>> forwardQueue;
	BDOpenClosed<state, BOBAGLowHigh<state>, BOBAFLowHigh<state>> backwardQueue;
private:
	double CLowerBound;
	double GForwardMin;
	double GBackwardMin;
};

#endif /* BOBAQueueGF_h */
