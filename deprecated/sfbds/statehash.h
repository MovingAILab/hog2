// definition of hash functions for a state, i.e. a tuple of vertices

#ifndef SFBDS_STATE_HASH
#define SFBDS_STATE_HASH

/*
	notes:
		- define SFBDS_LITTLE_ENDIAN if you have an akward machine
		- for state=graphState we support a maximum of 2^32 vertices
		- for state=xyLoc we support maximal dimensions of 2^16 x 2^16
		- for state=PancakePuzzleState up to a maximal size of 12 pancakes
*/


//#define SFBDS_LITTLE_ENDIAN

#include <stdint.h>

template<class state>
uint64_t sfbds_state_hash( state s1, state s2 );

template<class state>
uint64_t regular_state_hash( state s1 );

uint64_t factorial( int val );

#endif
