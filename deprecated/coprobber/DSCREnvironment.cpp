#include "DSCREnvironment.h"

template<>
double DSCREnvironment<xyLoc,tDirection>::HCost(const xyLoc &s1, const xyLoc &s2 ) {
	// we can just take the max since all edge costs are 1
	// (including the diagonals)
	return( ceil( max(abs(s1.x-s2.x),abs(s1.y-s2.y)) / (double)cop_speed ) );
};

template<>
double DSCREnvironment<graphState,graphMove>::HCost(const graphState &s1, const graphState &s2 ) {
	// calling the environments hcost enables us to use the maximum norm
	// for abstraction hierarchies as well.
	return( ceil( env->HCost( s1, s2 ) / (double)cop_speed ) );
};

