#include <iostream>
#include "DSIDFPN.h"

/*------------------------------------------------------------------------------
| Constructor & Destructor
------------------------------------------------------------------------------*/
DSIDFPN::DSIDFPN( GraphEnvironment *_env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<graphState,graphMove>( _env, true, cop_speed ) ),
	env(_env),
	numnodes(env->GetGraph()->GetNumNodes())
{
	if( numnodes*numnodes > MYINFTY ) {
		std::cerr << "ERROR: Mode nodes in the graph than can be encoded in a hash." << std::endl;
		exit( 1 );
	}
};

DSIDFPN::~DSIDFPN() {
	delete dscrenv;
	// the caches don't have to be deleted since they get destroyed automatically
};

/*------------------------------------------------------------------------------
| DSIDFPN
------------------------------------------------------------------------------*/
double DSIDFPN::dsidfpn( graphState &robber_pos, graphState &cop_pos, bool minFirst ) {

	// reset statistics
	distinctNodesGenerated = 0;
	nodesExpanded = 0;

	// we cannot start with a terminal node
	if( robber_pos == cop_pos ) return 0.;

	// change the given positions into one
	CRState pos; pos.push_back( robber_pos ); pos.push_back( cop_pos );

	SearchNode r; // root
	r.pos      = CRHash_MemOptim( pos );
	r.minFirst = minFirst;
	r.value    = dscrenv->AccumulatedHCost( pos, r.minFirst );

	do {
		r.bound = r.value;
		// verbose
		temp_global_bound = r.value;
		std::cout << "--------- new iteration with bound " << r.bound << " ------" << std::endl;
		bool status = dfpn_iteration( r );
		// sanity check: if iteration proved the root then the result should be greater than the bound
		assert( status == (r.value > r.bound) );
	} while( r.value > r.bound );

	// cleanup
	clear_bounds_cache();

	return r.value;
};


bool DSIDFPN::dfpn_iteration( SearchNode &n ) {
	n.th_phi   = MYINFTY;
	n.th_delta = MYINFTY;
	n.phi      = 1;
	n.delta    = 1;
	mid( n );
	// clear the TT's
	clear_tt();
	if( (n.phi==0) != (n.delta==MYINFTY) ) std::cout << "ERROR at root: n.phi = " << n.phi << " n.delta = " << n.delta << std::endl;
	if( (n.phi==MYINFTY) != (n.delta==0) ) std::cout << "ERROR at root: n.phi = " << n.phi << " n.delta = " << n.delta << std::endl;
	if( n.phi == MYINFTY )
		return( n.minFirst );
	else
		return( !n.minFirst );
};


void DSIDFPN::mid( SearchNode &n ) {
	// verbose
	/*
	if( temp_global_bound == 21. ) {
		CRState pos;
		MemOptim_Hash_To_CRState( n.pos, pos );
		std::cout << "entering mid with node: (pos,minFirst,bound,value,phi,delta,th_phi,th_delta) =" << std::endl;
		std::cout << "((" << pos[0] << "," << pos[1] << ")," << n.minFirst << "," << n.bound << "," << n.value << "," << n.phi << "," << n.delta << "," << n.th_phi << "," << n.th_delta << ")" << std::endl;
	}

	if(  (n.phi == MYINFTY) || (n.phi==0) || (n.delta==0) || (n.delta==MYINFTY) ) {
		std::cout << "expanded (dis)proven node" << std::endl;
		exit( 1 );
	}

	if( (n.phi > distinctNodesGenerated + 1) || (n.delta > distinctNodesGenerated + 1) ) {
		std::cout << "phi or delta greater distinctNodesExpanded" << std::endl;
		exit( 1 );
	}
	*/

	std::vector<SearchNode> successors;
	generate_moves( n, successors );

	n.phi   = delta_min( successors );
	n.delta = phi_sum( successors );
	while( n.th_phi > n.phi && n.th_delta > n.delta ) {
		// verbose
		//std::cout << "intermediate values at mid: " << "(" << pos[0] << "," << pos[1] << ") " << n.minFirst << " " << delta_min( successors ) << " " << phi_sum( successors ) << std::endl;
		unsigned int delta_two, phi_c, delta_c;
		int index = select_child( successors, phi_c, delta_c, delta_two );
		if( phi_c != successors[index].phi ) {
			std::cout << "error 1" << std::endl;
			exit( 1 );
		} else if( delta_c != successors[index].delta ) {
			std::cout << "error 2" << std::endl;
			exit( 1 );
		}
		if( index >= 0 ) {
			// sanity checks
			if( successors[index].phi == MYINFTY )
				std::cout << "ERROR: phi shouldn't be infinity" << std::endl;
			if( n.delta == MYINFTY )
				std::cout << "ERROR: n.delta should not be infinity" << std::endl;
			if( n.delta > n.th_delta )
				std::cout << "ERROR: negative substraction" << std::endl;
			successors[index].th_phi   = uintplus( successors[index].phi, n.th_delta - n.delta );
			successors[index].th_delta = (n.th_phi < uintplus(delta_two,1) )?n.th_phi:uintplus(delta_two,1);
			mid( successors[index] );
		} else {
			std::cout << "ERROR: index = " << index << std::endl;
		}
		n.phi   = delta_min( successors );
		n.delta = phi_sum( successors );
		if( ((n.phi==0)!=(n.delta==MYINFTY)) || ((n.phi==MYINFTY)!=(n.delta==0)) ) {
			std::cout << "Here is the problem at: distinct(" << distinctNodesGenerated << ") expanded(" << nodesExpanded << ")" << std::endl;
			for( std::vector<SearchNode>::iterator it = successors.begin(); it != successors.end(); it++ ) {
				std::cout << "   child with phi=" << it->phi << " delta=" << it->delta << std::endl;
			}
		}
	}

	n.value = value_minimax( n, successors );
	TTStore( n.bound, n.pos, n.minFirst, n.value, n.phi, n.delta );
	UpdateBoundsCache( n );

	// verbose
	/*
	if( temp_global_bound == 21. ) {
		CRState pos;
		MemOptim_Hash_To_CRState( n.pos, pos );
		std::cout << "leaving mid with node: (pos,minFirst,bound,value,phi,delta,th_phi,th_delta) =" << std::endl;
		std::cout << "((" << pos[0] << "," << pos[1] << ")," << n.minFirst << "," << n.bound << "," << n.value << "," << n.phi << "," << n.delta << "," << n.th_phi << "," << n.th_delta << ")" << std::endl;
	}
	*/
	return;
};


int DSIDFPN::select_child( std::vector<SearchNode> &successors, unsigned int &phi_c, unsigned int &delta_c, unsigned int &delta_two ) {
	int index = -1;
	phi_c     = MYINFTY;
	delta_c   = MYINFTY;
	delta_two = MYINFTY;
	for( unsigned int i = 0; i < successors.size(); i++ ) {
		if( successors[i].delta < delta_c ) {
			index = (int)i;
			delta_two = delta_c;
			phi_c   = successors[i].phi;
			delta_c = successors[i].delta;
		}
		else if( successors[i].delta < delta_two )
			delta_two = successors[i].delta;

		// if this node is disproved return since the node above is then proved
		/* this should never be called
		if( successors[i].phi == MYINFTY ) {
			std::cout << "called" << std::endl;
			return -1;
		}
		*/
	}
	return index;
};

unsigned int DSIDFPN::delta_min( std::vector<SearchNode> &successors ) {
	unsigned int min = MYINFTY;
	for( std::vector<SearchNode>::iterator it = successors.begin(); it != successors.end(); it++ ) {
		if( it->delta < min ) min = it->delta;
		// sanity check
		if( it->delta == MYINFTY && it->phi != 0 )
			std::cout << "Here is another problem: delta=" << it->delta << " phi=" << it->phi << std::endl;
	}
	return min;
};

unsigned int DSIDFPN::phi_sum( std::vector<SearchNode> &successors ) {
	unsigned int sum = 0;
	for( std::vector<SearchNode>::iterator it = successors.begin(); it != successors.end(); it++ ) {
		sum = uintplus( sum, it->phi );
		// sanity check
		if( sum == MYINFTY && it->delta != 0 )
			std::cout << "Here is another problem: sum=" << sum << " delta=" << it->delta << std::endl;
		if( sum == MYINFTY ) break;
	}
	return sum;
};

double DSIDFPN::value_minimax( SearchNode &n, std::vector<SearchNode> &successors ) {
	double value = n.minFirst?DBL_MAX:-DBL_MAX;
	for( std::vector<SearchNode>::iterator it = successors.begin(); it != successors.end(); it++ ) {
		if( it->phi == MYINFTY || it->delta == MYINFTY ) {
			if( n.minFirst )
				value = min( value, it->value + (n.bound - it->bound) );
			else
				value = max( value, it->value + (n.bound - it->bound) );
		}
	}
	return value;
};


bool DSIDFPN::try_pruning_node( SearchNode &n ) {

	// first test whether this is a terminal node
	CRState pos;
	MemOptim_Hash_To_CRState( n.pos, pos );
	if( dscrenv->GoalTest( pos ) ) {
		n.value = dscrenv->TerminalCost( pos );
		if( n.minFirst ) {
			n.phi   = (n.bound < n.value)?MYINFTY:0;
			n.delta = (n.bound < n.value)?0:MYINFTY;
		} else {
			n.phi   = (n.bound < n.value)?0:MYINFTY;
			n.delta = (n.bound < n.value)?MYINFTY:0;
		}
		return true;
	}


	// now try against the upper bound cache
	BoundCache* current_cache = n.minFirst?&min_ucache:&max_ucache;
	BoundCache::iterator bcit = current_cache->find( n.pos );
	// upper bound found?
	if( bcit != current_cache->end() ) {
		if( bcit->second <= n.bound ) {
			n.value = bcit->second;
			n.phi   = n.minFirst?0:MYINFTY;
			n.delta = n.minFirst?MYINFTY:0;
			// verbose
			/*
			CRState pos;
			MemOptim_Hash_To_CRState( n.pos, pos );
			std::cout << "(" << env->GetGraph()->GetNode( pos[0] )->GetLabelL( GraphSearchConstants::kMapX ) << ",";
			std::cout << env->GetGraph()->GetNode( pos[0] )->GetLabelL( GraphSearchConstants::kMapY ) << ") ";
			std::cout << "(" << env->GetGraph()->GetNode( pos[1] )->GetLabelL( GraphSearchConstants::kMapX ) << ",";
			std::cout << env->GetGraph()->GetNode( pos[1] )->GetLabelL( GraphSearchConstants::kMapY ) << ") ";
			std::cout << "problem_set1_map4.map" << std::endl;
			*/
			return true;
		}
	}
	// try against lower bounds
	current_cache = n.minFirst?&min_lcache:&max_lcache;
	bcit = current_cache->find( n.pos );
	// lower bound found in cache?
	if( bcit != current_cache->end() ) {
		if( bcit->second > n.bound ) {
			n.value = bcit->second;
			n.phi   = n.minFirst?MYINFTY:0;
			n.delta = n.minFirst?0:MYINFTY;
			// verbose
			/*
			CRState pos;
			MemOptim_Hash_To_CRState( n.pos, pos );
			std::cout << "(" << env->GetGraph()->GetNode( pos[0] )->GetLabelL( GraphSearchConstants::kMapX ) << ",";
			std::cout << env->GetGraph()->GetNode( pos[0] )->GetLabelL( GraphSearchConstants::kMapY ) << ") ";
			std::cout << "(" << env->GetGraph()->GetNode( pos[1] )->GetLabelL( GraphSearchConstants::kMapX ) << ",";
			std::cout << env->GetGraph()->GetNode( pos[1] )->GetLabelL( GraphSearchConstants::kMapY ) << ") ";
			std::cout << "problem_set1_map4.map" << std::endl;
			//std::cout << "upper bound pruned child with bound " << n.bound << " and set value to " << n.value << std::endl;
			*/
			return true;
		}
	} else {
		// lower bound in heuristic?
		double hcost = dscrenv->AccumulatedHCost( pos, n.minFirst );
		if( hcost > n.bound ) {
			n.value = hcost;
			n.phi   = n.minFirst?MYINFTY:0;
			n.delta = n.minFirst?0:MYINFTY;
			// verbose
			//distinctNodesGenerated++;
			return true;
		}
	}

	return false;
};



void DSIDFPN::generate_moves( SearchNode &n, std::vector<SearchNode> &successors ) {
	// clear the cache
	successors.clear();
	// change the position into readable format
	CRState pos;
	MemOptim_Hash_To_CRState( n.pos, pos );
	// expand node and find all successors
	std::vector<graphState> succs;
	nodesExpanded++;
	if( n.minFirst ) {
		dscrenv->GetCopSuccessors( pos, succs );
	} else {
		dscrenv->GetRobberSuccessors( pos, succs );
	}
	CRState newpos = pos;
	SearchNode newnode;

	// verbose
	//if( temp_global_bound == 21. )
	//	std::cout << "children are:" << std::endl;

	// now change them into our compressed format and look them up in the TT
	for( std::vector<graphState>::iterator it = succs.begin(); it != succs.end(); it++ ) {
		// verbose
		//if( temp_global_bound == 21. )
		//	std::cout << *it;
		newpos[n.minFirst?1:0] = *it;
		newnode.pos      = CRHash_MemOptim( newpos );
		newnode.minFirst = !n.minFirst;
		if( n.minFirst ) newnode.bound = n.bound - dscrenv->CopGCost( pos, newpos );
		else             newnode.bound = n.bound - dscrenv->RobberGCost( pos, newpos );
		if( !try_pruning_node( newnode ) )
			// if the node has not been seen yet (i.e. is not in TT), then TTLookup returns default values
			TTLookup( newnode.bound, newnode.pos, newnode.minFirst, newnode.value, newnode.phi, newnode.delta );
		successors.push_back( newnode );

		// verbose
		//if( temp_global_bound == 21. )
		//	std::cout << "(phi=" << newnode.phi << ",delta=" << newnode.delta << ",bound=" << newnode.bound << ",value=" << newnode.value << ")" << std::endl;

		// speedup
		// Carsten: reenable
		//if( newnode.phi == MYINFTY ) break;
	}
	//std::cout << std::endl;
	return;
};



/*------------------------------------------------------------------------------
| Hashing
------------------------------------------------------------------------------*/
uint32_t DSIDFPN::CRHash_MemOptim( CRState &s ) {
	return( s[0] * numnodes + s[1] );
};

void DSIDFPN::MemOptim_Hash_To_CRState( uint32_t &hash, CRState &s ) {
	s.push_back( (graphState) (hash/numnodes) );
	s.push_back( (graphState) (hash%numnodes) );
	return;
};

/*------------------------------------------------------------------------------
| Caching
------------------------------------------------------------------------------*/
void DSIDFPN::clear_bounds_cache() {
	min_lcache.clear();
	max_lcache.clear();
	min_ucache.clear();
	max_ucache.clear();
	return;
};

void DSIDFPN::clear_tt() {
	min_ttable.clear();
	max_ttable.clear();
	return;
};

void DSIDFPN::TTLookup( double &bound, unsigned int &pos, bool &minFirst,
                        double &value, unsigned int &phi, unsigned int &delta ) {
	bool found_entry = false;
	// determine which TT to take, for the minimizer or maximizer
	TT* current_table = minFirst?&min_ttable:&max_ttable;
	// find whether we have a TT at that depth
	TT::iterator ttit = current_table->find( bound );
	if( ttit != current_table->end() ) {
		// find whether we have an entry for the position
		OneLevelTT::iterator olttit = ttit->second.find( pos );
		if( olttit != ttit->second.end() ) {
			// assign value, phi and delta
			value = olttit->second.value;
			if( minFirst ) { // if minimizer (=AND)
				phi   = olttit->second.dn;
				delta = olttit->second.pn;
			} else { // if maximizer (=OR)
				phi   = olttit->second.pn;
				delta = olttit->second.dn;
			}
			found_entry = true;
		}
	}
	if( !found_entry ) {
		// return standart values for newly extracted node
		phi = 1;
		delta = 1;
		value = DBL_MAX; //bound;
		distinctNodesGenerated++;
	}
	return;
};

void DSIDFPN::TTStore( double &bound, unsigned int &pos, bool &minFirst,
                       double &value, unsigned int &phi, unsigned int &delta ) {

	TT* current_table = minFirst?&min_ttable:&max_ttable;

	TT::iterator ttit = current_table->find( bound );
	if( ttit == current_table->end() ) { // there is no OneLevelTT for this depth => create one
		std::pair<TT::iterator,bool> insert_pair = current_table->insert( TT::value_type( bound, OneLevelTT() ) );
		if( !insert_pair.second ) {
			std::cerr << "ERROR: Could not create transposition table for bound " << bound << std::endl;
			exit( 1 );
		}
		ttit = insert_pair.first;
	}

	// create a TT entry
	TTEntry e;
	// fill it
	e.value = value;
	if( minFirst ) {
		e.pn = delta;
		e.dn = phi;
	} else {
		e.pn = phi;
		e.dn = delta;
	}
	// store it
	(ttit->second)[pos] = e;

	return;
};

void DSIDFPN::UpdateBoundsCache( SearchNode &n ) {
	// sanity checks
	if( ( (n.phi==0) != (n.delta==MYINFTY) ) || (n.phi==MYINFTY) != (n.delta==0) ) {
		CRState pos;
		MemOptim_Hash_To_CRState( n.pos, pos );
		std::cout << "ERROR in bound update: n.pos=(" << pos[0] << "," << pos[1] << ") n.minFirst=" << n.minFirst << std::endl;
		std::cout << "    n.value=" << n.value << " n.phi=" << n.phi << " n.delta=" << n.delta << std::endl;;
		std::cout << "    n.th_phi=" << n.th_phi << " n.th_delta=" << n.th_delta << std::endl;
	}
	//assert( (n.phi==0) == (n.delta==MYINFTY) );
	//assert( (n.phi==MYINFTY) == (n.delta==0) );

	if( n.phi == MYINFTY || n.delta == MYINFTY ) {
		if( n.minFirst ) {
			if( n.phi == MYINFTY ) min_lcache[n.pos] = n.value;
			else                   min_ucache[n.pos] = n.value;
		} else {
			if( n.phi == MYINFTY ) max_ucache[n.pos] = n.value;
			else                   max_lcache[n.pos] = n.value;
		}
	}
	return;
};

