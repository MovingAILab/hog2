//
//  TSInstances.h
//  hog2 glut
//

#ifndef TSInstances_h
#define TSInstances_h

#include "TopSpin.h"
#include <random>
#include <algorithm>

namespace TS {

	template<int N>
	void GetTSInstance(TopSpinState<N> &s, int which)
	{
		std::vector<int> elts;
		std::mt19937 mt(which);
		for (int x = 0; x < N; x++)
			elts.push_back(x);
		std::shuffle(elts.begin(), elts.end(), mt);
		for (int x = 0; x < N; x++)
			s.puzzle[x] = elts[x];
	}
}
#endif /* TSInstances_h */
