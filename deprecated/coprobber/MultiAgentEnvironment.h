#ifndef MULTIAGENTENVIRONMENT_H
#define MULTIAGENTENVIRONMENT_H

#include <stdint.h>
#include <vector>
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"

/*------------------------------------------------------------------------------
| Types
------------------------------------------------------------------------------*/
template<class action>
struct extended_action {
	action a;
	bool noaction;

	extended_action(): noaction(true) {};
	extended_action( action _a ): a(_a), noaction(false) {};
};

template<class action>
static bool operator==( const extended_action<action> &ea1, const extended_action<action> &ea2 ) {
	if( ea1.noaction ) return( ea2.noaction );
	if( ea2.noaction ) return( false );
	return( ea1.a == ea2.a );
};

/*------------------------------------------------------------------------------
| Environment
------------------------------------------------------------------------------*/
template<class state, class action>
class MultiAgentEnvironment: public SearchEnvironment<std::vector<state>, std::vector<extended_action<action> > > {
	public:

	typedef std::vector<state> MAState;
	typedef extended_action<action> SAAction; // single agent action
	typedef std::vector<SAAction> MAMove;

};

/*------------------------------------------------------------------------------
| Occupancy
------------------------------------------------------------------------------*/
template<class state, class action>
class MultiAgentOccupancy: public OccupancyInterface<std::vector<state>,std::vector<extended_action<action> > > {
	public:
	typedef typename MultiAgentEnvironment<state,action>::MAState MAState;
};

#endif
