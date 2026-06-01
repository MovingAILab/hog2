
#ifndef MYHASH_H
#define MYHASH_H

#include <vector>
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"

// These functions can be used when an unknown amount of "for"-loops
// has to be used (e.g. num_players many). Then you can encode one
// for loop and dehash the current value...
unsigned long hash_permutation( unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states );
void dehash_permutation( unsigned long hash, unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states );


// hash function definition
template<class state>
uint64_t CRHash( const std::vector<state> &pos );

template<class state>
uint64_t CRHash( const state &s1, const state &s2 );

/*
// \see MyHash.cpp
template<>
uint64_t CRHash<xyLoc>( const std::vector<xyLoc> &pos );
template<>
uint64_t CRHash<graphState>( const std::vector<graphState> &pos );
*/

template<class state>
uint64_t StateHash( const state &s );

/*
// \see MyHash.cpp
template<>
uint64_t StateHash<xyLoc>( const xyLoc &s );
template<>
uint64_t StateHash<graphState>( const graphState &s );
*/


inline unsigned int uintplus( unsigned int a, unsigned int b ) {
	if( UINT_MAX - a < b ) return UINT_MAX;
	return (a+b);
};

#endif
