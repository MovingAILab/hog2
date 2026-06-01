#include "MyHash.h"
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"

unsigned long hash_permutation( unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states ) {
	unsigned long offset = 1, hash = 0;
	for( unsigned int i = 0; i < num_players; i++ ) {
		hash += player_states[i] * offset;
		offset *= num_player_states[i];
	}
	return hash;
}

void dehash_permutation( unsigned long hash, unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states ) {
	unsigned long offset = 1;
	for( unsigned int i = 0; i < num_players; i++ ) {
		offset = num_player_states[i];
		player_states[i] = hash%offset;
		hash /= offset;
	}
}

// specification for state=xyLoc
template<>
uint64_t CRHash<xyLoc>( const std::vector<xyLoc> &pos ) {
	return( ((uint64_t)pos[0].x)<<48 | (((uint64_t)pos[0].y)<<48)>>16 |
		(((uint64_t)pos[1].x)<<48)>>32 | (((uint64_t)pos[1].y)<<48)>>48 );
}

// hash function copied from CopRobberEnvironment.h
// only that this has to be outside of classes
// \see CopRobberEnvironment.h
template<>
uint64_t CRHash<graphState>( const std::vector<graphState> &node) {
	uint64_t hash = 0, t;
	double psize = 64./(double)node.size();
	double count = 0.;
	unsigned int i;
	unsigned char j;

	for( i = 0; i < node.size(); i++ ) {
		t = node[i];
		for( j = 0; j < floor(count+psize)-floor(count); j++ ) {
			hash |= (t & (1<<j))<<(unsigned int)floor(count);
		}
		count += psize;
	}
	return hash;
}


// specification for state=xyLoc
template<>
uint64_t CRHash<xyLoc>( const xyLoc &s1, const xyLoc &s2 ) {
	return( ((uint64_t)s1.x)<<48 | (((uint64_t)s1.y)<<48)>>16 |
		(((uint64_t)s2.x)<<48)>>32 | (((uint64_t)s2.y)<<48)>>48 );
}
// same for state=graphState
template<>
uint64_t CRHash<graphState>( const graphState &s1, const graphState &s2 ) {
	return( ((uint64_t)s1)<<32 | (((uint64_t)s2)<<32)>>32 );
};


template<>
uint64_t StateHash<xyLoc>( const xyLoc &s ) {
	return( ((uint64_t)s.x)<<32 | (((uint64_t)s.y)<<32)>>32 );
}

template<>
uint64_t StateHash<graphState>( const graphState &s ) {
	return( (uint64_t) s );
}

