#include "GraphEnvironment.h"

// this is a heuristic that uses the differential heuristic
// with 100 canonical states and maximizes with octile distance
// heuristic.


class MyDiffHeuristic: public GraphHeuristic {

	public:

	// constructor
	MyDiffHeuristic( Map *_m, unsigned int number_canonical_states ): m(_m)
	{
		g   = GraphSearchConstants::GetGraph(m);
		gdh = new GraphDistanceHeuristic( g );
		gmh = new GraphMapHeuristic( m, g );
		gdh->SetPlacement(kFarPlacement);
		for( unsigned int i = 0; i < number_canonical_states; i++ )
			gdh->AddHeuristic();
	};

	~MyDiffHeuristic() {
		// cleanup
		delete gdh;
		delete gmh;
	};

	Graph *GetGraph() { return g; };
	double HCost( const graphState &s1, const graphState &s2 ) const {
		return std::max( gdh->HCost( s1, s2 ), gmh->HCost( s1, s2 ) );
	};


	private:
		Map *m;
		Graph *g;
		GraphDistanceHeuristic *gdh;
		GraphMapHeuristic *gmh;

};

