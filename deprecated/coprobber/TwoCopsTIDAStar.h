#include <vector>
#include <set>
#include <ext/hash_set>
#include <unordered_map>
#include "GraphEnvironment.h"

#ifndef TWOCOPSTIDASTAR_H
#define TWOCOPSTIDASTAR_H

/*!
	two players IDA* implementation for two cops

	note: this code has mainly been copied from TIDAStar_optimized
	      and adopted to two cops
	note: it only runs with graph now since the hashing only really works with graphs anyways
	note: agents can always pass their turn
	note: all edge costs are 1, terminal costs are 0
*/
class TwoCopsTIDAStar {

	public:

	// constructor
	TwoCopsTIDAStar( GraphEnvironment *env );
	//~TwoCopsTIDAStar();

	unsigned int tida( graphState &r, graphState &c1, graphState &c2, bool minFirst );

	void set_usePerfectDistanceHeuristic( bool usePerfectDistanceHeuristic );

	unsigned int nodesExpanded, nodesTouched;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched;

	protected:

	// for internal use within the hashing and move generation functions
	typedef uint32_t Position;
	typedef graphState CRState[3]; // internal only

	GraphEnvironment *env;
	unsigned int numnodes;
	bool usePerfectDistanceHeuristic;
	std::vector<std::vector<double> > distance_heuristic;

	// hashing functions to convert a Position into CRState and vice versa
	Position CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( Position &hash, const CRState &s );

	void GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors );


	// TIDA* subroutine
	unsigned int tida_update( Position &pos, unsigned int bound, bool minFirst );

	bool GoalTest(const  Position &pos );
	// lower estimate on survival time of robber
	unsigned int HCost( Position &pos, bool &minFirst );

	void clear_bounds_cache();

	typedef std::unordered_map<Position, unsigned int> BoundCache;
	// lower bound cache
	BoundCache min_lcache, max_lcache;
	// upper bound cache
	BoundCache min_ucache, max_ucache;
};

#endif
