#include <vector>
#include <map>
#include <unordered_map>
#include "MyHash.h" // for uintplus
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"


#ifndef DSIPFPN_H
#define DSIPFPN_H
/*
	Depth-First Proof Number Search
	with incorporated heuristic pruning

	note: different speed system included and we only operate on graphs
	note: both, cop and robber are allowed to pass their turn

	note: there is maximally UINT32_MAX (joint) states allowed
*/

#define MYINFTY UINT32_MAX

class DSIDFPN {

	public:

	// types
	typedef MultiAgentEnvironment<graphState,graphMove>::MAState CRState;

	// constructor & destructor
	DSIDFPN( GraphEnvironment *env, unsigned int cop_speed = 1 );
	~DSIDFPN();

	double dsidfpn( graphState &robber_pos, graphState &cop_pos, bool minFirst = true );

	unsigned int distinctNodesGenerated;
	unsigned int nodesExpanded;

	double temp_global_bound;

	protected:

	// the environment variable
	DSCREnvironment<graphState,graphMove> *dscrenv;
	GraphEnvironment *env;
	unsigned int numnodes;

	// hashing functions for the state
	uint32_t CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( uint32_t &hash, CRState &s );

	// data structures

	// the search nodes that are used on the path from the root to the terminals
	struct SearchNode { // nodes that are used during the search
		unsigned int pos;
		bool minFirst;
		double bound, value;
		unsigned phi, delta;
		unsigned int th_phi, th_delta;
	};

	// bound cache
	typedef std::unordered_map<unsigned int, double> BoundCache;
	// lower and upper bound cache
	BoundCache min_lcache, max_lcache, min_ucache, max_ucache;

	// Transposition Table (TT)
	struct TTEntry { // current value, bound on proof number and disproof number
		double value;
		unsigned int pn, dn;
	};
	typedef std::unordered_map<unsigned int, TTEntry> OneLevelTT; // position => (dis)proof number bounds
	typedef std::map<double, OneLevelTT > TT; // Transposition Table with caching w.r.t gcost
	// transposition tables
	TT min_ttable, max_ttable;
	
	// cache functions
	void clear_bounds_cache(); // clears upper and lower bound cache
	void clear_tt(); // clears TT information
	void TTLookup( double &depth, unsigned int &pos, bool &minFirst,
	               double &value, unsigned int &phi, unsigned int &delta );
	void TTStore( double &depth, unsigned int &pos, bool &minFirst,
	              double &value, unsigned int &phi, unsigned int &delta );
	void UpdateBoundsCache( SearchNode &n );

	bool dfpn_iteration( SearchNode &n );
	// runs an iteration of df-pn
	void mid( SearchNode &n );
	// returns the index of the next child that has to be explored
	// if there are no children that could be selected (because all are (dis)proved) -1 is returned
	int select_child( std::vector<SearchNode> &successors, unsigned int &phi_c, unsigned int &delta_c, unsigned int &delta_two );
	// computes \Delta Min( node n )
	unsigned int delta_min( std::vector<SearchNode> &successors );
	// computes \Phi Sum( node n )
	unsigned int phi_sum( std::vector<SearchNode> &successors );
	// update the parents value from the successor's values
	double value_minimax( SearchNode &n, std::vector<SearchNode> &successors );
	// tries to prune a node (terminal or bounds)
	bool try_pruning_node( SearchNode &n );
	// neighbor generation
	void generate_moves( SearchNode &n, std::vector<SearchNode> &successors );

};

#endif
