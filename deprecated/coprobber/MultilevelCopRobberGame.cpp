#include "MultilevelCopRobberGame.h"
#include "MyHash.h"

MultilevelCopRobberGame::MultilevelCopRobberGame( GraphAbstraction *_gabstraction, unsigned int _num_cops, bool simultaneous, bool _playerscanpass ):
	MultilevelMarkovGame<graphState,graphMove>( simultaneous ),
	gabstraction(_gabstraction), num_cops(_num_cops),
	playerscanpass(_playerscanpass)
{
	// assign some storage for the games
	unsigned int i, levels = gabstraction->getNumAbstractGraphs();
	crgames           = new CopRobberGame*[levels];
	graphheuristics   = new MultilevelGraphHeuristic*[levels];
	graphenvironments = new GraphEnvironment*[levels];
	for( i = 0; i < levels; i++ ) {
		crgames[i]           = NULL;
		graphheuristics[i]   = NULL;
		graphenvironments[i] = NULL;
	}

}

MultilevelCopRobberGame::~MultilevelCopRobberGame() {
	unsigned int i, levels = NumLevels();
	for( i = 0; i < levels; i++ ) {
		delete crgames[i];
		delete graphenvironments[i];
		delete graphheuristics[i];
	}
	delete [] crgames;
	delete [] graphenvironments;
	delete [] graphheuristics;
	// delete gabstraction; ???
}

unsigned int MultilevelCopRobberGame::NumLevels() {
	return gabstraction->getNumAbstractGraphs();
}

MarkovGame<graphState,graphMove>* MultilevelCopRobberGame::GetMarkovGame( unsigned int level ) {
	if( crgames[level] == NULL ) {
		Graph *g = gabstraction->GetAbstractGraph( level );
		graphheuristics[level]   = new MultilevelGraphHeuristic( gabstraction, level );
		graphenvironments[level] = new GraphEnvironment( g, graphheuristics[level] );
		crgames[level]           = new CopRobberGame( graphenvironments[level], num_cops, simultaneous, playerscanpass );
	}
	return crgames[level];
}

CopRobberGame::CRState MultilevelCopRobberGame::GetParent( CRState s, unsigned int slevel ) {
	CRState parent;
	node *nparent;

	for( unsigned int i = 0; i < s.size(); i++ ) {
		nparent = gabstraction->GetParent( gabstraction->GetAbstractGraph(slevel)->GetNode(s[i]) );
		if( nparent )
			parent.push_back( nparent->GetNum() );
		else {
			parent.clear();
			return parent;
		}
	}
	return parent;
}

unsigned int MultilevelCopRobberGame::GetNumChildren( CRState s, unsigned int slevel ) {
	unsigned int result = 0;
	Graph *g = gabstraction->GetAbstractGraph( slevel );

	for( unsigned int i = 0; i < s.size(); i++ ) {
		if( result ) {
			result *= gabstraction->GetNumChildren( g->GetNode( s[i] ) );
		} else {
			result = gabstraction->GetNumChildren( g->GetNode( s[i] ) );
		}
	}
	return result;
}

CopRobberGame::CRState MultilevelCopRobberGame::GetNthChild( CRState s, unsigned int slevel, unsigned int n ) {
	unsigned int i, num_players = s.size();
	unsigned int nps[num_players], ps[num_players];
	Graph *g = gabstraction->GetAbstractGraph( slevel );
	CRState result;

	for( i = 0; i < num_players; i++ ) {
		nps[i] = gabstraction->GetNumChildren( g->GetNode( s[i] ) );
	}
	dehash_permutation( n, num_players, nps, ps );
	for( i = 0; i < num_players; i++ ) {
		result.push_back( gabstraction->GetNthChild( g->GetNode( s[i] ), ps[i] )->GetNum() );
	}
	return result;
}
