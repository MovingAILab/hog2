#include "statehash.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "PancakePuzzle.h"

/************************************************************
| graphState
************************************************************/
template<>
uint64_t sfbds_state_hash<graphState>( graphState s1, graphState s2 ) {

	// sort the two states
	if( s1 > s2 ) {
		graphState temp = s1;
		s1 = s2;
		s2 = temp;
	}

	// take the last 32 bits of state 1 and the last 32 bits of state 2
	#ifdef SFBDS_LITTLE_ENDIAN
		return ( (s1 >> 32) | ( (s2 >> 32) << 32 ) );
	#else
		return ( (s1 << 32) | ( (s2 << 32) >> 32 ) );
	#endif
};

template<>
uint64_t regular_state_hash<graphState>( graphState s ) {
	return s;
};


/*************************************************************
| xyLoc
*************************************************************/
template<>
uint64_t sfbds_state_hash<xyLoc>( xyLoc s1, xyLoc s2 ) {

	// sort the states in x and alternatively in y
	if( s1.x > s2.x || (s1.x == s2.x && s1.y > s2.y) ) {
		xyLoc temp = s1; s1 = s2; s2 = temp;
	}

	// take the last 16 bits of s1.x and s1.y
	// plus the last 16 bits of s2.x and s2.y
	#ifdef SFBDS_LITTLE_ENDIAN
		return(
			((uint64_t)s1.x)>>48 |
			((uint64_t)s1.y)>>32 |
			((uint64_t)s2.x)>>16 |
			((uint64_t)s2.y) );
	#else
		return(
			((uint64_t)s1.x)<<48 |
			((uint64_t)s1.y)<<32 |
			((uint64_t)s2.x)<<16 |
			((uint64_t)s2.y) );
	#endif
};

template<>
uint64_t regular_state_hash<xyLoc>( xyLoc s ) {
	return( ((uint64_t)s.x)<<32 | ((uint64_t)s.y) );
};

/*************************************************************
| PancakePuzzleState
*************************************************************/
uint64_t factorial( int val )
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
			6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
			6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
};

template<>
uint64_t sfbds_state_hash<PancakePuzzleState>( PancakePuzzleState s1, PancakePuzzleState s2 ) {
	uint64_t result1 = 0;
	uint64_t result2 = 0;
	unsigned int i, j;
	for( i = 0; i < s1.puzzle.size(); i++ ) {
		result1 += s1.puzzle[i] * factorial( s1.puzzle.size() - i - 1 );
		result2 += s2.puzzle[i] * factorial( s2.puzzle.size() - i - 1 );
		for( j = i; j < s1.puzzle.size(); j++ ) {// it should actually start at i+1
			if( s1.puzzle[j] > s1.puzzle[i] ) s1.puzzle[j]--;
			if( s2.puzzle[j] > s2.puzzle[i] ) s2.puzzle[j]--;
		}
	}

	return( (result1<<32) | ((result2>>32)<<32) );
};

template<>
uint64_t regular_state_hash<PancakePuzzleState>( PancakePuzzleState s ) {
	uint64_t result = 0;
	unsigned int i, j;
	for( i = 0; i < s.puzzle.size(); i++ ) {
		result += s.puzzle[i] * factorial( s.puzzle.size() - i - 1 );
		for( j = i; j < s.puzzle.size(); j++ ) {// it should actually start at i+1
			if( s.puzzle[j] > s.puzzle[i] ) s.puzzle[j]--;
		}
	}
	return result;
};

