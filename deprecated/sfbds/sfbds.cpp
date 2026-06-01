#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

// HOG2 includes
#include "Common.h"
// HOG2 graph and map environments
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"
#include "GraphCanonicalHeuristic.h"
#include "TemplateAStar.h"
// HOG2 Pancake
#include "PancakePuzzle.h"

// project includes
#include "astar.h"
#include "MyDiffHeuristic.h"
#include "optimalIDAStar.h"
#include "idastar.h"


// config parameters
#define NUMBER_CANONICAL_STATES 10


/*------------------------------------------------------------------------------
| SFBDS computation
------------------------------------------------------------------------------*/
int compute_sfbds( int argc, char* argv[] ) {

	// Syntax
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " " << argv[1] << " -map <mapname> -s <x,y> -g <x,y> -h <heuristic> -j <jumping_policy> [-np]" << std::endl;
		std::cout << "where <jumping_policy> can have the values:" << std::endl;
		std::cout << "  0 - always expand the start state (=A*)" << std::endl;
		std::cout << "  1 - always expand the goal state (=A* from goal to start)"
			<< std::endl;
		std::cout << "  2 - expand the state with fewer successors "
			<< "(in case of tie priority is given to the start state)" << std::endl;
		std::cout << "  3 - expand randomly weighted with the number of successors"
			<< std::endl;
		std::cout << std::endl;
		std::cout << "<heuristic> can be:" << std::endl;
		std::cout << "  octile    - octile distance heuristic in maps" << std::endl;
		std::cout << "  canonical - canonical heuristic" << std::endl;
		std::cout << "-np is for no advanced pruning" << std::endl;
		return(1);
	}

	// default values
	int expandheuristic_param = 0;
	bool advanced_pruning = true;
	char heuristicname[128];
	Map *m = NULL;
	xyLoc startloc(0,0), goalloc(0,0);

	// Command parsing
	int i = argc-1;
	while( i > 1 ) {
		if( strcmp( argv[i-1], "-j" ) == 0 ) {
			expandheuristic_param = atoi( argv[i] );
			i -= 2;
		} else if( strcmp( argv[i-1], "-h" ) == 0 ) {
			strcpy( heuristicname, argv[i] );
			i -= 2;
		} else if( strcmp( argv[i], "-np" ) == 0 ) {
			advanced_pruning = false;
			i--;
		} else if( strcmp( argv[i-1], "-map" ) == 0 ) {
			m = new Map( argv[i] );
			i -= 2;
		} else if( strcmp( argv[i-1], "-s" ) == 0 ) {
			sscanf( argv[i], "%hu,%hu", &startloc.x, &startloc.y );
			i -= 2;
		} else if( strcmp( argv[i-1], "-g" ) == 0 ) {
			sscanf( argv[i], "%hu,%hu", &goalloc.x, &goalloc.y );
			i -= 2;
		} else {
			std::cerr << "ERROR: unkown parameter discovered (" << argv[i-1] << "). Exiting." << std::endl;
			exit(1);
		}
	}

	// error checks for input
	if( m == NULL ) {
		std::cerr << "ERROR: no map submitted." << std::endl;
		return(1);
	}

	if( startloc == goalloc ) {
		std::cerr << "ERROR: start " << startloc << " and goal "
			<< goalloc << " are the same." << std::endl;
		return(1);
	}

	Graph *g                     = NULL;
	GraphMapHeuristic *gh        = NULL;
	GraphCanonicalHeuristic *gch = NULL;
	MyDiffHeuristic *gdh         = NULL;
	GraphEnvironment *env        = NULL;

	// set the environment with respective heuristic
	if( strcmp( heuristicname, "octile" ) == 0 ) {
		g = GraphSearchConstants::GetGraph( m );
		gh = new GraphMapHeuristic( m, g );
		env = new GraphEnvironment( g, gh );
	}
	else if( strcmp( heuristicname, "canonical" ) == 0 ) {
		gch = new GraphCanonicalHeuristic( m, 1, 10 );
		g   = gch->GetGraph();
		env = new GraphEnvironment( g, gch );
	}
	else if( strcmp( heuristicname, "differential" ) == 0 ) {
		gdh = new MyDiffHeuristic( m, NUMBER_CANONICAL_STATES );
		g   = gdh->GetGraph();
		env = new GraphEnvironment( g, gdh );
	} else {
		std::cerr << "ERROR: unknown heuristic submitted. Exiting." << std::endl;
		exit( 1 );
	}
	env->SetDirected( true );

	graphState start = m->GetNodeNum( startloc.x, startloc.y );
	graphState goal  = m->GetNodeNum( goalloc.x, goalloc.y );
	std::cout << "start: " << start << std::endl;
	std::cout << "goal: " << goal << std::endl;

	std::vector<graphState> path;
	SFBDSAStar<graphState,graphMove> *a = new SFBDSAStar<graphState,graphMove>( env );
	a->use_pruning( advanced_pruning );
	double result = a->astar( start, goal, path, expandheuristic_param );
	std::cout << "result value: " << result << std::endl;
	//std::cout << "result path: " << std::endl;
	//for( std::vector<graphState>::iterator it = path.begin(); it != path.end(); it++ )
	//	std::cout << *it << std::endl;
	std::cout << "number of jumps during search: " << a->numberOfJumps << std::endl;
	std::cout << "number of jumps in the solution: " << a->numberOfJumpsInSolution << std::endl;
	std::cout << "nodes expanded: " << a->nodesExpanded << std::endl;
	std::cout << "reopened nodes: " << a->reopenedNodes << std::endl;
	std::cout << "BPMX updates: " << a->bpmxUpdates << std::endl;
	std::cout << "one sided reopenings: " << a->oneSidedReopenings << std::endl;
	std::cout << "successors touched: " << a->successorsTouched << std::endl;
	std::cout << "nodes popped from open queue: " << a->nodesPoppedFromOpenQueue << std::endl;
	//std::cout << "predecessor operator pruning: " << a->predecessorOperatorPruning << std::endl;
	std::cout << "distance prunes at expansion: " << a->distancePruning << std::endl;
	std::cout << "distance prunes at successors: " << a->distanceSuccessorPruning << std::endl;
	std::cout << "closed list prunes: " << a->closedListPrunes << std::endl;

	delete a;
	delete env;
	if( strcmp( heuristicname, "canonical" ) == 0 )
		delete gch;
	else if( strcmp( heuristicname, "octile" ) == 0 ) {
		delete gh;
		delete g;
		delete m;
	}
	else if( strcmp( heuristicname, "differential" ) == 0 ) {
		delete gdh;
		delete g;
		delete m;
	}



	/*
	// old code for MapEnvironment

	MapEnvironment *env = new MapEnvironment( m );

	std::vector<xyLoc> path;

	SFBDSAStar<xyLoc,tDirection> *a = new SFBDSAStar<xyLoc,tDirection>( env );
	double result = a->astar( startloc, goalloc, path, expandheuristic_param );

	std::cout << "result value: " << result << std::endl;
	std::cout << "result path:" << std::endl;
	for( std::vector<xyLoc>::iterator it = path.begin(); it != path.end(); it++ ) {
		std::cout << *it << std::endl;
	}
	*/

	return 0;
};


/*------------------------------------------------------------------------------
| Template A*
------------------------------------------------------------------------------*/
int compute_templatea( int argc, char* argv[] ) {

	// Syntax
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " " << argv[1] << " -map <mapname> -s <x,y> -g <x,y> -h <heuristic>" << std::endl;
		return(1);
	}

	// default values
	char heuristicname[128];
	Map *m = NULL;
	xyLoc startloc(0,0), goalloc(0,0);

	// Command parsing
	int i = argc-1;
	while( i > 1 ) {
		if( strcmp( argv[i-1], "-h" ) == 0 ) {
			strcpy( heuristicname, argv[i] );
			i -= 2;
		} else if( strcmp( argv[i-1], "-map" ) == 0 ) {
			m = new Map( argv[i] );
			i -= 2;
		} else if( strcmp( argv[i-1], "-s" ) == 0 ) {
			sscanf( argv[i], "%hu,%hu", &startloc.x, &startloc.y );
			i -= 2;
		} else if( strcmp( argv[i-1], "-g" ) == 0 ) {
			sscanf( argv[i], "%hu,%hu", &goalloc.x, &goalloc.y );
			i -= 2;
		} else {
			std::cerr << "ERROR: unkown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// error checks for input
	if( m == NULL ) {
		std::cerr << "ERROR: no map submitted." << std::endl;
		return(1);
	}

	if( startloc == goalloc ) {
		std::cerr << "ERROR: start " << startloc << " and goal "
			<< goalloc << " are the same." << std::endl;
		return(1);
	}

	Graph *g                     = NULL;
	GraphMapHeuristic *gh        = NULL;
	GraphCanonicalHeuristic *gch = NULL;
	MyDiffHeuristic *gdh         = NULL;
	GraphEnvironment *env        = NULL;

	// set the environment with respective heuristic
	if( strcmp( heuristicname, "octile" ) == 0 ) {
		g = GraphSearchConstants::GetGraph( m );
		gh = new GraphMapHeuristic( m, g );
		env = new GraphEnvironment( g, gh );
	}
	else if( strcmp( heuristicname, "canonical" ) == 0 ) {
		gch = new GraphCanonicalHeuristic( m, 1, 10 );
		g   = gch->GetGraph();
		env = new GraphEnvironment( g, gch );
	}
	else if( strcmp( heuristicname, "differential" ) == 0 ) {
		gdh = new MyDiffHeuristic( m, NUMBER_CANONICAL_STATES );
		g   = gdh->GetGraph();
		env = new GraphEnvironment( g, gdh );
	} else {
		std::cerr << "ERROR: unknown heuristic submitted. Exiting." << std::endl;
		exit( 1 );
	}
	env->SetDirected( true );

	graphState start = m->GetNodeNum( startloc.x, startloc.y );
	graphState goal  = m->GetNodeNum( goalloc.x, goalloc.y );
	std::cout << "start: " << start << std::endl;
	std::cout << "goal: " << goal << std::endl;

	std::vector<graphState> path;
	TemplateAStar<graphState,graphMove,GraphEnvironment> *astar = new TemplateAStar<graphState,graphMove,GraphEnvironment>();
	astar->InitializeSearch( env, start, goal, path );
	do {
		std::cout << "expanding " << astar->CheckNextNode() << std::endl;
	} while( !astar->DoSingleSearchStep( path ) );

	// compute the cost of the entire path
	double result = 0.;
	for( unsigned int i = 0; i < path.size() - 1; i++ ) {
		result += env->GCost( path[i], path[i+1] );
	}

	std::cout << "result value: " << result << std::endl;
	std::cout << "result path: " << std::endl;
	//for( std::vector<graphState>::iterator it = path.begin(); it != path.end(); it++ )
	//	std::cout << *it << std::endl;
	std::cout << "nodes expanded: " << astar->GetNodesExpanded() << std::endl;
	std::cout << "unique nodes  : " << astar->GetUniqueNodesExpanded() << std::endl;
	std::cout << "nodes touched : " << astar->GetNodesTouched() << std::endl;

	delete astar;
	delete env;
	if( strcmp( heuristicname, "canonical" ) == 0 )
		delete gch;
	else if( strcmp( heuristicname, "octile" ) == 0 ) {
		delete gh;
		delete g;
		delete m;
	}
	else if( strcmp( heuristicname, "differential" ) == 0 ) {
		delete gdh;
		delete g;
		delete m;
	}

	return 0;
};




/*------------------------------------------------------------------------------
| Experiments for SFBDS
------------------------------------------------------------------------------*/
int compute_experiment( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 8 ) {
		std::cout << "Syntax: " << argv[0] << " experiment -f <file> -o <file> -h <heuristic> -j <jumping_policy> [-np]" << std::endl;
		std::cout << "where <file> is the input (-f) or output (-o) file of the experiments and" << std::endl;
		std::cout << "<heuristic> and <jumping_policy> are the same as for method sfbds" << std::endl;
		return( 1 );
	};

	FILE *fin = NULL;
	std::ofstream fout;
	bool advanced_pruning = true;
	int expandheuristic_param = 0;
	char heuristicname[128];

	// Command parsing
	int i = argc-1;
	while( i > 1 ) {
		if( strcmp( argv[i-1], "-f" ) == 0 ) {
			fin = fopen( argv[i], "r" );
			i -= 2;
		}
		else if( strcmp( argv[i-1], "-o" ) == 0 ) {
			fout.open( argv[i], std::ios::out );
			i -= 2;
		}
		else if( strcmp( argv[i], "-np" ) == 0 ) {
			advanced_pruning = false;
			i--;
		}
		else if( strcmp( argv[i-1], "-h" ) == 0 ) {
			strcpy( heuristicname, argv[i] );
			i -= 2;
		}
		else if( strcmp( argv[i-1], "-j" ) == 0 ) {
			expandheuristic_param = atoi( argv[i] );
			i -= 2;
		} else {
			std::cerr << "ERROR: unkown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// error handling
	if( fin == NULL ) {
		std::cerr << "ERROR: could not open input file" << std::endl;
		return 1;
	}
	if( !fout ) {
		std::cerr << "ERROR: could not open output file" << std::endl;
		return 1;
	}

	char mapfile[256];
	Map *m = NULL;
	Graph *g;
	GraphMapHeuristic *gh = NULL;
	GraphCanonicalHeuristic *gch = NULL;
	MyDiffHeuristic *gdh = NULL;
	GraphEnvironment *env = NULL;
	
	// resetting the random counter to make experiments repeatable
	srandom( 1 );

	// for each input in the experiment input file
	while( !feof( fin ) ) {

		// read the map file
		fscanf( fin, "%s\n", mapfile );
		fout << mapfile << std::endl;
		m = NULL;
		m = new Map( mapfile );

		if( m == NULL ) {
			std::cerr << "ERROR: could not open map " << mapfile << "." << std::endl;
			return 1;
		}

		if( strcmp( heuristicname, "canonical" ) == 0 ) {
			gch = new GraphCanonicalHeuristic( m, 1, 10 );
			g   = gch->GetGraph();
			env = new GraphEnvironment( g, gch );
		}
		else if( strcmp( heuristicname, "octile" ) == 0 ) {
			g   = GraphSearchConstants::GetGraph( m );
			gh  = new GraphMapHeuristic( m, g );
			env = new GraphEnvironment( g, gh );
		}
		else if( strcmp( heuristicname, "differential" ) == 0 ) {
			gdh = new MyDiffHeuristic( m, NUMBER_CANONICAL_STATES );
			g   = gdh->GetGraph();
			env = new GraphEnvironment( g, gdh );
		} else {
			std::cerr << "ERROR: unknown heuristic submitted. Exiting." << std::endl;
			exit( 1 );
		}
		env->SetDirected( true );

		// read in 1000 test instances
		for( i = 0; i < 1000; i++ ) {
			SFBDSAStar<graphState,graphMove> *astar = new SFBDSAStar<graphState,graphMove>( env );
			astar->use_pruning( advanced_pruning );

			xyLoc startloc, goalloc;
			fscanf( fin, "(%hu,%hu) (%hu,%hu)\n", &startloc.x, &startloc.y, &goalloc.x, &goalloc.y );
			fout << "(" << startloc.x << "," << startloc.y << ") ("
				<< goalloc.x << "," << goalloc.y << ") ";
			// verbose output
			//std::cout << "computing for " << startloc << " " << goalloc << " ..." << std::flush;

			graphState start = m->GetNodeNum( startloc.x, startloc.y );
			graphState goal  = m->GetNodeNum( goalloc.x, goalloc.y );

			// run the computation for the instances
			if( start == goal ) {
				std::cerr << "Warning: start and goal are the same: " << start
					<< " (" << startloc << ")" << std::endl;
			}

			std::vector<graphState> solution_path;
			double solution_value =
				astar->astar( start, goal, solution_path, expandheuristic_param );

			// solution output
			fout << solution_value << " ";
			fout << astar->nodesExpanded << " ";
			fout << astar->nodesPoppedFromOpenQueue << " ";
			fout << astar->successorsTouched << " ";
			//fout << astar->predecessorOperatorPruning << " ";
			fout << astar->distancePruning << " ";
			fout << astar->distanceSuccessorPruning << " ";
			fout << astar->closedListPrunes << " ";
			fout << astar->numberOfJumps << " ";
			fout << astar->numberOfJumpsInSolution << " ";
			fout << astar->reopenedNodes << " ";
			fout << astar->bpmxUpdates << " ";
			fout << astar->oneSidedReopenings;
			fout << std::endl;

			delete astar;
			// verbose output
			//std::cout << " done" << std::endl;
		}

		// cleanup
		delete env;
		if( strcmp( heuristicname, "canonical" ) == 0 )
			delete gch;
		else if( strcmp( heuristicname, "octile" ) == 0 ) {
			delete gh;
			delete g;
			delete m;
		}
		else if( strcmp( heuristicname, "differential" ) == 0 ) {
			delete gdh;
			delete g;
			delete m;
		}

	}

	fclose( fin );
	fout.close();

	return 0;
};


/*------------------------------------------------------------------------------
| Experiments with Template A*
------------------------------------------------------------------------------*/
int compute_experimenttemplatea( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 8 ) {
		std::cout << "Syntax: " << argv[0] << " " << argv[1] << " -f <file> -o <file> -h <heuristic>" << std::endl;
		std::cout << "where <file> is the input (-f) or output (-o) file of the experiments and" << std::endl;
		std::cout << "<heuristic> are the same as for method sfbds" << std::endl;
		return( 1 );
	};

	FILE *fin = NULL;
	std::ofstream fout;
	char heuristicname[128];

	// Command parsing
	int i = argc-1;
	while( i > 1 ) {
		if( strcmp( argv[i-1], "-f" ) == 0 ) {
			fin = fopen( argv[i], "r" );
			i -= 2;
		}
		else if( strcmp( argv[i-1], "-o" ) == 0 ) {
			fout.open( argv[i], std::ios::out );
			i -= 2;
		}
		else if( strcmp( argv[i-1], "-h" ) == 0 ) {
			strcpy( heuristicname, argv[i] );
			i -= 2;
		} else {
			std::cerr << "ERROR: unkown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// error handling
	if( fin == NULL ) {
		std::cerr << "ERROR: could not open input file" << std::endl;
		return 1;
	}
	if( !fout ) {
		std::cerr << "ERROR: could not open output file" << std::endl;
		return 1;
	}

	char mapfile[256];
	Map *m = NULL;
	Graph *g;
	GraphMapHeuristic *gh = NULL;
	GraphCanonicalHeuristic *gch = NULL;
	MyDiffHeuristic *gdh = NULL;
	GraphEnvironment *env = NULL;
	
	// resetting the random counter to make experiments repeatable
	srandom( 1 );

	// for each input in the experiment input file
	while( !feof( fin ) ) {

		// read the map file
		fscanf( fin, "%s\n", mapfile );
		fout << mapfile << std::endl;
		m = NULL;
		m = new Map( mapfile );

		if( m == NULL ) {
			std::cerr << "ERROR: could not open map " << mapfile << "." << std::endl;
			return 1;
		}

		if( strcmp( heuristicname, "canonical" ) == 0 ) {
			gch = new GraphCanonicalHeuristic( m, 1, 10 );
			g   = gch->GetGraph();
			env = new GraphEnvironment( g, gch );
		}
		else if( strcmp( heuristicname, "octile" ) == 0 ) {
			g   = GraphSearchConstants::GetGraph( m );
			gh  = new GraphMapHeuristic( m, g );
			env = new GraphEnvironment( g, gh );
		}
		else if( strcmp( heuristicname, "differential" ) == 0 ) {
			gdh = new MyDiffHeuristic( m, NUMBER_CANONICAL_STATES );
			g   = gdh->GetGraph();
			env = new GraphEnvironment( g, gdh );
		} else {
			std::cerr << "ERROR: unknown heuristic submitted. Exiting." << std::endl;
			exit( 1 );
		}
		env->SetDirected( true );

		// read in 1000 test instances
		for( i = 0; i < 1000; i++ ) {
			TemplateAStar<graphState,graphMove,GraphEnvironment> *astar = new TemplateAStar<graphState,graphMove,GraphEnvironment>();
			astar->SetUseBPMX( 1 );

			xyLoc startloc, goalloc;
			fscanf( fin, "(%hu,%hu) (%hu,%hu)\n", &startloc.x, &startloc.y, &goalloc.x, &goalloc.y );
			fout << "(" << startloc.x << "," << startloc.y << ") ("
				<< goalloc.x << "," << goalloc.y << ") ";
			// verbose output
			//std::cout << "computing for " << startloc << " " << goalloc << " ..." << std::flush;

			graphState start = m->GetNodeNum( startloc.x, startloc.y );
			graphState goal  = m->GetNodeNum( goalloc.x, goalloc.y );

			// run the computation for the instances
			if( start == goal ) {
				std::cerr << "Warning: start and goal are the same: " << start
					<< " (" << startloc << ")" << std::endl;
			}

			std::vector<graphState> solution_path;
			astar->GetPath( env, start, goal, solution_path );
			double solution_value = 0.;
			if( solution_path.size() > 0 )
				for( unsigned int i = 0; i < solution_path.size() - 1; i++ )
					solution_value += env->GCost( solution_path[i], solution_path[i+1] );

			// solution output
			fout << solution_value << " ";
			fout << astar->GetNodesExpanded() << " ";
			fout << astar->GetNodesTouched() << " ";
			fout << astar->GetUniqueNodesExpanded();
			fout << std::endl;

			delete astar;
			// verbose output
			//std::cout << " done" << std::endl;
		}

		// cleanup
		delete env;
		if( strcmp( heuristicname, "canonical" ) == 0 )
			delete gch;
		else if( strcmp( heuristicname, "octile" ) == 0 ) {
			delete gh;
			delete g;
			delete m;
		}
		else if( strcmp( heuristicname, "differential" ) == 0 ) {
			delete gdh;
			delete g;
			delete m;
		}

	}

	fclose( fin );
	fout.close();

	return 0;
};



/*------------------------------------------------------------------------------
| Testpoint computation for the experiments
------------------------------------------------------------------------------*/
int compute_testpoints( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 6 ) {
		std::cout << "Syntax: " << argv[0] << " testpoints -o <outputfile> -maps <map1> <mapi>" << std::endl;
		std::cout << "Returns an experiment file with 1000 test cases for each map given by -maps." << std::endl;
		std::cout << "Note that -maps has to be the last argument submitted." << std::endl;
		return( 1 );
	};

	std::string outputfilename;
	std::vector<std::string> mapfiles;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-o" ) == 0 ) {
			outputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-maps" ) == 0 ) {
			// get all the maps
			for( carg++; carg < argc; carg++ )
				mapfiles.push_back( argv[carg] );
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
			
	}

	std::ofstream fout;
	fout.open( outputfilename.c_str(), std::ios::out );
	if( !fout ) {
		std::cerr << "ERROR: could not open output file." << std::endl;
		exit(1);
	}

	// initializing the random number generator with 1
	srandom( 1 );

	// verbose output
	std::cout << "Creating file " << outputfilename <<
		" with 1000 testpoints for each of the following maps:" << std::endl;
	for( std::vector<std::string>::iterator it = mapfiles.begin(); it != mapfiles.end(); it++ ) {
		std::cout << *it;
		// open the map file
		Map *m = new Map( (*it).c_str() );
		Graph *g = GraphSearchConstants::GetGraph( m );

		std::cout << " ... " << std::flush;
		fout << (*it).c_str() << std::endl;

		int i = 0;
		while( i < 1000 ) {
			unsigned num1 = (unsigned int) floor(
				(double)random()/(double)RAND_MAX * (double)g->GetNumNodes() );

			std::vector<node*>* reachable_nodes = g->getReachableNodes( g->GetNode( num1 ) );
			unsigned num2 = (unsigned int)floor(
				(double)random()/(double)RAND_MAX * (double)(reachable_nodes->size()-1) );

			// if both are the same
			if( (*reachable_nodes)[num2]->GetNum() == num1 )
				continue;

			fout << "(";
			fout << g->GetNode(num1)->GetLabelL(GraphSearchConstants::kMapX);
			fout << ",";
			fout << g->GetNode(num1)->GetLabelL(GraphSearchConstants::kMapY);
			fout << ") ";
			fout << "(";
			fout << (*reachable_nodes)[num2]->GetLabelL(GraphSearchConstants::kMapX);
			fout << ",";
			fout << (*reachable_nodes)[num2]->GetLabelL(GraphSearchConstants::kMapY);
			fout << ")" << std::endl;
			delete reachable_nodes;
			i++;
		}

		delete g;
		delete m;

		std::cout << "done" << std::endl;
	}

	return 0;
}


/*------------------------------------------------------------------------------
| Code for Pancake puzzle
------------------------------------------------------------------------------*/
// test state generation
int compute_pancakestates( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 8 ) {
		std::cout << "Syntax: " << argv[0] << " pancakestates -s <size> -n <number> -o <outputfile>" << std::endl;
		std::cout << "  writes <number> random pancakestates into the <outputfile>" << std::endl;
		return( 1 );
	};

	std::string outputfilename;
	unsigned int puzzle_size = 0;
	unsigned int number_of_puzzles = 0;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-o" ) == 0 ) {
			outputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-n" ) == 0 ) {
			number_of_puzzles = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
			
	}

	std::ofstream fout( outputfilename.c_str(), std::ios::out);
	if( fout.fail() ) {
		std::cerr << "ERROR: could not open output file (" << outputfilename << ")" << std::endl;
		exit(1);
	}

	// create a puzzle
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	std::vector<PancakePuzzleState> states;
	puzzle->Create_Random_Pancake_Puzzles( states, puzzle_size, number_of_puzzles);
	for( std::vector<PancakePuzzleState>::iterator it = states.begin(); it != states.end(); it++ ) {
		fout << *it << std::endl;
	}
	fout.close();

	return 0;
};


// build PDB for the pancake puzzle
int compute_pancakepdb( int argc, char* argv[1] ) {

	// syntax
	if( argc < 8 ) {
		std::cout << "Syntax: " << argv[0] << " pancakepdb -o <filename> -s <size> -t <tile1> <tile2>" << std::endl;
		return( 1 );
	}

	std::string pdbfilename;
	std::vector<int> tiles;
	unsigned int puzzle_size = 0;

	// command line parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-o" ) == 0 ) {
			pdbfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-t" ) == 0 ) {
			for( carg++; carg < argc; carg++ )
				tiles.push_back( atoi( argv[carg] ) );
		} else {	
			std::cerr << "ERROR: unknown argument. Exiting." << std::endl;
			return( 1 );
		}
	}

	// build the puzzle
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	PancakePuzzleState start(puzzle_size);
	std::cout << "building PDB ... " << std::flush;
	puzzle->Build_Regular_PDB( start, tiles, pdbfilename.c_str() );
	std::cout << "done" << std::endl << std::flush;

	// clean up
	delete puzzle;
	return 0;
};


// pancake computation
int compute_pancake( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " pancake -s <size> -pdb <file> -j <number> -a <0|1> -p <puzzle>" << std::endl;
		return( 1 );
	};

	std::string pdbfilename;
	PancakePuzzleState puzzlestate;
	unsigned int puzzle_size = 0;
	unsigned int expandheuristic_param = 0;
	bool use_both_heuristic_lookups = true;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-a" ) == 0 ) {
			use_both_heuristic_lookups = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-p" ) == 0 ) {
			for( carg++; carg < argc; carg++ )
				puzzlestate.puzzle.push_back( atoi(argv[carg]) );
		} else if( strcmp( argv[carg], "-pdb" ) == 0 ) {
			pdbfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-j" ) == 0 ) {
			expandheuristic_param = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// create environment
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	// create goal
	PancakePuzzleState goal(puzzle_size);
	std::cout << "start: " << puzzlestate << std::endl;
	std::cout << "goal : " << goal << std::endl;
	// read PDB
	std::cout << "reading PDB ... " << std::flush;
	puzzle->Load_Regular_PDB( pdbfilename.c_str(), goal, false );
	std::cout << "done" << std::endl << std::flush;

	// create the SFBDS-A* object
	SFBDSAStar<PancakePuzzleState,unsigned> *astar = new SFBDSAStar<PancakePuzzleState,unsigned>( puzzle );
	// enable maximization of heuristic lookups
	astar->use_asymmetric_heuristic( use_both_heuristic_lookups );

	if( puzzlestate == goal ) {
		std::cerr << "Warning: start and goal are the same." << std::endl;
	}

	std::vector<PancakePuzzleState> path;
	// find the solution
	double solution_value = astar->astar( puzzlestate, goal, path, expandheuristic_param );

	// verbose
	std::cout << "solution path:" << std::endl;
	for( unsigned int i = 0; i < path.size(); i++ )
		std::cout << path[i] << std::endl;

	// solution output
	std::cout << "solution value: " << solution_value << std::endl
		<< "nodes expanded: " << astar->nodesExpanded << std::endl
		<< "nodes popped from openqueue: " << astar->nodesPoppedFromOpenQueue << std::endl
		<< "successors touched: " << astar->successorsTouched << std::endl
		<< "distance prunes: " << astar->distancePruning << std::endl
		<< "distance successor prunes: " << astar->distanceSuccessorPruning << std::endl
		<< "closed list prunes: " << astar->closedListPrunes << std::endl
		<< "number of jumps: " << astar->numberOfJumps << std::endl
		<< "jumps in solution: " << astar->numberOfJumpsInSolution << std::endl
		<< "reopened nodes: " << astar->reopenedNodes << std::endl
		<< "bpmx updates: " << astar->bpmxUpdates << std::endl
		<< "one side reopenings: " << astar->oneSidedReopenings << std::endl;

	// cleanup
	delete astar;
	delete puzzle;

	return 0;
};



// pancake experiment
int compute_pancakeex( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " pancakeex -s <size> -f <file> -o <file> -pdb <file> -j <number> -a <0|1> -np" << std::endl;
		std::cout << "where" << std::endl;
		std::cout << "  -s <size> is the number of pancakes" << std::endl;
		std::cout << "  -a <0|1> determines whether both regular and dual lookups should be made" << std::endl;
		std::cout << "    (enabled by default)" << std::endl;
		return( 1 );
	};

	std::string inputfilename;
	std::string outputfilename;
	std::string pdbfilename;
	unsigned int puzzle_size = 0;
	unsigned int expandheuristic_param = 0;
	bool use_both_heuristic_lookups = true;
	bool use_advanced_pruning = true;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-f" ) == 0 ) {
			inputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-o" ) == 0 ) {
			outputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-a" ) == 0 ) {
			use_both_heuristic_lookups = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-np" ) == 0 ) {
			use_advanced_pruning= false;
			carg++;
		} else if( strcmp( argv[carg], "-pdb" ) == 0 ) {
			pdbfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-j" ) == 0 ) {
			expandheuristic_param = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// create environment
	std::vector<PancakePuzzleState> states;
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	// read puzzles
	puzzle->read_in_pancake_puzzles( inputfilename.c_str(), false, puzzle_size, 0, states );
	std::cout << "read " << states.size() << " puzzles." << std::endl;
	// create goal
	PancakePuzzleState goal(puzzle_size);
	// read PDB
	std::cout << "reading PDB ... " << std::flush;
	puzzle->Load_Regular_PDB( pdbfilename.c_str(), goal, false );
	std::cout << "done" << std::endl << std::flush;
	// open output file
	std::ofstream fout( outputfilename.c_str(), std::ios::out );
	if( fout.fail() ) {
		std::cerr << "ERROR: could not open output file. Exiting." << std::endl;
		exit( 1 );
	}

	// for each Puzzle state
	for( std::vector<PancakePuzzleState>::iterator it = states.begin(); it != states.end(); it++ ) {
		// create the SFBDS-A* object
		SFBDSAStar<PancakePuzzleState,unsigned> *astar = new SFBDSAStar<PancakePuzzleState,unsigned>( puzzle );
		// enable maximization of heuristic lookups
		astar->use_asymmetric_heuristic( use_both_heuristic_lookups );
		astar->use_pruning( use_advanced_pruning );

		if( *it == goal ) {
			std::cerr << "Warning: start and goal are the same @ " << goal << std::endl;
		}

		// verbose
		//std::cout << "--------- " << *it << " -----------------" << std::endl;

		std::vector<PancakePuzzleState> path;
		// find the solution
		double solution_value = astar->astar( *it, goal, path, expandheuristic_param );

		// verbose
		//std::cout << "-------- path --------------" << std::endl;
		//for( unsigned int i = 0; i < path.size(); i++ )
		//	std::cout << path[i] << std::endl;

		// solution output
		fout << *it << " => ";
		fout << solution_value << " ";
		fout << astar->nodesExpanded << " ";
		fout << astar->nodesPoppedFromOpenQueue << " ";
		fout << astar->successorsTouched << " ";
		//fout << astar->predecessorOperatorPruning << " ";
		fout << astar->distancePruning << " ";
		fout << astar->distanceSuccessorPruning << " ";
		fout << astar->closedListPrunes << " ";
		fout << astar->numberOfJumps << " ";
		fout << astar->numberOfJumpsInSolution << " ";
		fout << astar->reopenedNodes << " ";
		fout << astar->bpmxUpdates << " ";
		fout << astar->oneSidedReopenings;
		fout << std::endl;

		delete astar;
	}

	// cleanup
	delete puzzle;

	return 0;
};



/*------------------------------------------------------------------------------
| Code for IDA*
------------------------------------------------------------------------------*/
// pancake computation
int compute_pancakeida( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " pancake -s <size> -pdb <file> -a <0|1> -bpmx -p <puzzle>" << std::endl;
		return( 1 );
	};

	std::string pdbfilename;
	PancakePuzzleState puzzlestate;
	unsigned int puzzle_size = 0;
	bool use_both_heuristic_lookups = true;
	bool useBPMX = false;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-a" ) == 0 ) {
			use_both_heuristic_lookups = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-p" ) == 0 ) {
			for( carg++; carg < argc; carg++ )
				puzzlestate.puzzle.push_back( atoi(argv[carg]) );
		} else if( strcmp( argv[carg], "-pdb" ) == 0 ) {
			pdbfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-bpmx" ) == 0 ) {
			useBPMX = true;
			carg++;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// create environment
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	// create goal
	PancakePuzzleState goal(puzzle_size);
	std::cout << "start: " << puzzlestate << std::endl;
	std::cout << "goal : " << goal << std::endl;
	// read PDB
	std::cout << "reading PDB ... " << std::flush;
	puzzle->Load_Regular_PDB( pdbfilename.c_str(), goal, false );
	std::cout << "done" << std::endl << std::flush;

	// create the optimalIDA* object
	OptimalIDAStar<PancakePuzzleState,unsigned> *optidastar = new OptimalIDAStar<PancakePuzzleState,unsigned>( puzzle );
	optidastar->SetUseMaxHeuristic( use_both_heuristic_lookups );
	optidastar->SetUseBPMX( useBPMX );
	std::cout << "running IDA* with " << (use_both_heuristic_lookups?"both":"regular") << " lookup(s)" << std::endl;
	std::cout << "running " << (useBPMX?"with":"without") << " BPMX" << std::endl;

	if( puzzlestate == goal ) {
		std::cerr << "Warning: start and goal are the same." << std::endl;
	}

	std::cout << "----------------- optimal IDA* -----------------" << std::endl;
	double result = optidastar->IDAStar( puzzlestate, goal );
	//std::cout << "bound used: " << bound << std::endl;
	std::cout << "result: " << result << "     ";
	std::cout << "nodes: " << optidastar->necessaryNodesExpanded << "    ";
	std::cout << "last iter: " << optidastar->necessaryNodesExpandedInLastIteration << std::endl;

	// cleanup
	delete optidastar;

	// regular IDA*
	IDAStar<PancakePuzzleState,unsigned> *idastar = new IDAStar<PancakePuzzleState,unsigned>( puzzle );
	idastar->SetUseMaxHeuristic( use_both_heuristic_lookups );
	idastar->SetUseBPMX( useBPMX );

	std::cout << "----------------- regular IDA* -----------------" << std::endl;
	result = idastar->RunIDAStar( puzzlestate, goal );
	std::cout << "result: " << result << "     ";
	std::cout << "nodes: " << idastar->nodesExpanded << "    ";
	std::cout << "last iter: " << idastar->nodesExpandedInLastIteration << std::endl;

	delete idastar;
	delete puzzle;

	return 0;
};


// pancake experiment with IDA*'s
int compute_pancakeidaex( int argc, char* argv[] ) {

	// Syntax handling
	if( argc < 10 ) {
		std::cout << "Syntax: " << argv[0] << " pancakeidaex -s <size> -f <file> -o <file> -pdb <file> -j <number> -a <0|1> -bpmx" << std::endl;
		std::cout << "where" << std::endl;
		std::cout << "  -s <size> is the number of pancakes" << std::endl;
		std::cout << "  -a <0|1> determines whether both regular and dual lookups should be made" << std::endl;
		std::cout << "    (enabled by default)" << std::endl;
		return( 1 );
	};

	std::string inputfilename;
	std::string outputfilename;
	std::string pdbfilename;
	unsigned int puzzle_size = 0;
	std::vector<unsigned> policies;
	bool use_both_heuristic_lookups = true;
	bool use_bpmx = false;

	// argument parsing
	int carg = 2;
	while( carg < argc ) {
		if( strcmp( argv[carg], "-f" ) == 0 ) {
			inputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-o" ) == 0 ) {
			outputfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-a" ) == 0 ) {
			use_both_heuristic_lookups = atoi( argv[carg+1] );
			carg += 2;
		} else if( strcmp( argv[carg], "-bpmx" ) == 0 ) {
			use_bpmx = true;
			carg++;
		} else if( strcmp( argv[carg], "-pdb" ) == 0 ) {
			pdbfilename = argv[carg+1];
			carg += 2;
		} else if( strcmp( argv[carg], "-j" ) == 0 ) {
			policies.push_back( atoi( argv[carg+1] ) );
			carg += 2;
		} else if( strcmp( argv[carg], "-s" ) == 0 ) {
			puzzle_size = atoi( argv[carg+1] );
			carg += 2;
		} else {
			std::cerr << "ERROR: unknown parameter discovered. Exiting." << std::endl;
			exit(1);
		}
	}

	// create environment
	std::vector<PancakePuzzleState> states;
	PancakePuzzle *puzzle = new PancakePuzzle( puzzle_size );
	// read puzzles
	puzzle->read_in_pancake_puzzles( inputfilename.c_str(), false, puzzle_size, 0, states );
	std::cout << "read " << states.size() << " puzzles." << std::endl;
	// create goal
	PancakePuzzleState goal(puzzle_size);
	// read PDB
	std::cout << "reading PDB ... " << std::flush;
	puzzle->Load_Regular_PDB( pdbfilename.c_str(), goal, false );
	std::cout << "done" << std::endl << std::flush;
	// open output file
	std::ofstream fout( outputfilename.c_str(), std::ios::out );
	if( fout.fail() ) {
		std::cerr << "ERROR: could not open output file. Exiting." << std::endl;
		exit( 1 );
	}

	// for each Puzzle state
	for( std::vector<PancakePuzzleState>::iterator it = states.begin(); it != states.end(); it++ ) {

		// regular IDA*
		IDAStar<PancakePuzzleState,unsigned> *idastar = new IDAStar<PancakePuzzleState,unsigned>( puzzle );
		idastar->SetUseMaxHeuristic( use_both_heuristic_lookups );
		idastar->SetUseBPMX( use_bpmx );
		idastar->SetJumpingPolicy( 0 ); // regular A*
		double result = idastar->RunIDAStar( *it, goal );

		// output the result
		fout << result << " " << idastar->nodesExpanded << " "
			<< idastar->nodesExpandedInLastIteration << " " << std::flush;

		double r;
		// for each policy
		for( std::vector<unsigned>::iterator policy = policies.begin(); policy != policies.end(); policy++ ) {

			idastar->SetJumpingPolicy( *policy );
			r = idastar->RunIDAStar( *it, goal );

			// sanity check
			if( !fequal( r, result ) )
				std::cerr << "Warning: policy " << *policy << " did not find same solution!" << std::endl;

			fout << idastar->nodesExpanded << " " << idastar->nodesExpandedInLastIteration << " " << std::flush;
		}

		// optimal IDA*
		OptimalIDAStar<PancakePuzzleState,unsigned> *optidastar = new OptimalIDAStar<PancakePuzzleState,unsigned>( puzzle );
		optidastar->SetUseMaxHeuristic( use_both_heuristic_lookups );
		optidastar->SetUseBPMX( use_bpmx );
		unsigned nE;
		r = optidastar->IDAStarIteration( *it, goal, result, nE );

		// sanity check
		if( !fequal( r, result ) )
			std::cerr << "Warning: optimal IDA* did not find same solution!" << std::endl;

		// output the optimal number of nodes
		fout << nE << " => " << *it << std::endl;

		delete idastar;
		delete optidastar;
	}

	// cleanup
	delete puzzle;

	return 0;
};




/*------------------------------------------------------------------------------
| Code for visualization of SFBDS
------------------------------------------------------------------------------*/
bool _simulation_ended = false;
bool _simulation_paused = true;
Map *_simulation_map = NULL;
xyLoc _simulation_startloc(0,0), _simulation_goalloc(0,0);
unsigned int _simulation_expandheuristic_param = 0;
char _simulation_heuristicname[128];
bool _simulation_advanced_pruning = true;
GraphEnvironment* _simulation_environment = NULL;
SFBDSAStar<graphState,graphMove>* _simulation_astar = NULL;
SFBDSAStar<graphState,graphMove>::QueueNode _simulation_last_expanded_node;
bool _simulation_last_expanded_side;

bool MyClickHandler( unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType ) {
	return false;
};

void MyDisplayHandler( unsigned long windowID, tKeyboardModifier mod, char key ) {
	switch( key ) {
		case 's':
			if( !_simulation_ended ) {
				_simulation_last_expanded_node = _simulation_astar->step(_simulation_last_expanded_side);
				// verbose
				std::cout
					<< "q = (s1=" << _simulation_last_expanded_node.s1
					<< " s2="     << _simulation_last_expanded_node.s2
					<< " fcost="  << _simulation_last_expanded_node.fcost
					<< " gcost1=" << _simulation_last_expanded_node.gcost1
					<< " gcost2="  << _simulation_last_expanded_node.gcost2
					<< ")" 
					<< " expanded " << _simulation_last_expanded_side
					<< std::endl;
				if( _simulation_last_expanded_node.s1 == _simulation_last_expanded_node.s2 )
					_simulation_ended = true;
			}
			break;
		case 'p':
			_simulation_paused = !_simulation_paused;
			break;
		case 'n':
			if( _simulation_astar == NULL )
				std::cout << "simulation not yet started" << std::endl;
			else {
				std::cout << "nodes expanded: " << _simulation_astar->nodesExpanded << std::endl;
			}
			break;
	}
	return;
};

int MyCLHandler( char* argument[], int maxNumArgs ) {
	// if we only get one remaining element return
	if( maxNumArgs <= 1 )
		return 0;

	// parse the input
	if( strcmp( argument[0], "-j" ) == 0 ) {
		_simulation_expandheuristic_param = atoi( argument[1] );
		return 2;
	} else if( strcmp( argument[0], "-h" ) == 0 ) {
		strcpy( _simulation_heuristicname, argument[1] );
		return 2;
	} else if( strcmp( argument[0], "-map" ) == 0 ) {
		_simulation_map = new Map( argument[1] );
		return 2;
	} else if( strcmp( argument[0], "-np" ) == 0 ) {
		_simulation_advanced_pruning = false;
		return 1;
	} else if( strcmp( argument[0], "-s" ) == 0 ) {
		sscanf( argument[1], "%hu,%hu", &_simulation_startloc.x, &_simulation_startloc.y );
		return 2;
	} else if( strcmp( argument[0], "-g" ) == 0 ) {
		sscanf( argument[1], "%hu,%hu", &_simulation_goalloc.x, &_simulation_goalloc.y );
		return 2;
	} else {
		std::cerr << "ERROR: unkown parameter: " << argument[0] << std::endl;
	}
	// go one argument further
	return 1;
};

void CreateSimulation( unsigned long windowID ) {

	// error checks for input
	if( _simulation_map == NULL ) {
		std::cerr << "ERROR: no map submitted." << std::endl;
		exit( 1 );
	}

	if( _simulation_startloc == _simulation_goalloc ) {
		std::cerr << "ERROR: start " << _simulation_startloc << " and goal "
			<< _simulation_goalloc << " are the same." << std::endl;
		exit( 1 );
	}

	// create the simulation
	Graph *g = NULL;
	GraphMapHeuristic *gh = NULL;
	GraphCanonicalHeuristic *gch = NULL;
	MyDiffHeuristic *gdh  = NULL;
	if( strcmp( _simulation_heuristicname, "canonical" ) == 0 ) {
		gch = new GraphCanonicalHeuristic( _simulation_map, 1, 10 );
		g   = gch->GetGraph();
		_simulation_environment = new GraphEnvironment( g, gch );
	} else if( strcmp( _simulation_heuristicname, "octile" ) == 0 ) {
		g  = GraphSearchConstants::GetGraph( _simulation_map );
		gh = new GraphMapHeuristic( _simulation_map, g );
		_simulation_environment = new GraphEnvironment( g, gh );
	} else if( strcmp( _simulation_heuristicname, "differential" ) == 0 ) {
		gdh = new MyDiffHeuristic( _simulation_map, NUMBER_CANONICAL_STATES );
		g   = gdh->GetGraph();
		_simulation_environment = new GraphEnvironment( g, gdh );
	} else {
		std::cerr << "ERROR: unknown heuristic submitted. Exiting." << std::endl;
		exit( 1 );
	}
	_simulation_environment->SetDirected( true );
	graphState start = _simulation_map->GetNodeNum( _simulation_startloc.x, _simulation_startloc.y );
	graphState goal  = _simulation_map->GetNodeNum( _simulation_goalloc.x, _simulation_goalloc.y );
	std::cout << "start: " << start << std::endl;
	std::cout << "goal: " << goal << std::endl;

	_simulation_astar = new SFBDSAStar<graphState,graphMove>( _simulation_environment );
	_simulation_astar->use_pruning( _simulation_advanced_pruning );
	_simulation_astar->initialize( start, goal, _simulation_expandheuristic_param );

	return;
};

// frame handler!!!!
void MyFrameHandler( unsigned long windowID, unsigned int viewport, void* ) {

	_simulation_environment->OpenGLDraw();


	// determine the radius of a ball that will be drawn
	GLdouble rad;
	if( _simulation_map->GetMapHeight() > _simulation_map->GetMapWidth() )
		rad = 1/(double)_simulation_map->GetMapHeight();
	else
		rad = 1/(double)_simulation_map->GetMapWidth();


	// if we are running through the simulation expand one node per frame
	if( !_simulation_paused && !_simulation_ended ) {
		_simulation_last_expanded_node = _simulation_astar->step(_simulation_last_expanded_side);
		// verbose
		std::cout
			<< "q = (s1=" << _simulation_last_expanded_node.s1
			<< " s2="     << _simulation_last_expanded_node.s2
			<< " fcost="  << _simulation_last_expanded_node.fcost
			<< " gcost1=" << _simulation_last_expanded_node.gcost1
			<< " gcost2="  << _simulation_last_expanded_node.gcost2
			<< ")" 
			<< " expanded " << _simulation_last_expanded_side
			<< std::endl;
		if( _simulation_last_expanded_node.s1 == _simulation_last_expanded_node.s2 )
			_simulation_ended = true;
	}


	// draw everything from the astar class
	_simulation_astar->OpenGLDraw( _simulation_last_expanded_node, _simulation_last_expanded_side, rad );
	_simulation_astar->OpenGLDraw( rad );

	return;
};

void MyWindowHandler(unsigned long windowID, tWindowEventType eType) {
	if( eType == kWindowDestroyed ) {
		std::cout << "destroyed window " << windowID << std::endl;
		RemoveFrameHandler(MyFrameHandler, windowID, 0 );
	} else if( eType == kWindowCreated ) {
		std::cout << "created window " << windowID << std::endl;
		SetNumPorts( windowID, 1 );
		InstallFrameHandler( MyFrameHandler, windowID, 0 );
		CreateSimulation( windowID );
	}
	return;
};

void InstallHandlers() {
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "map on which it will be searched");
	InstallCommandLineHandler(MyCLHandler, "-s", "-s x,y", "start position of the search" );
	InstallCommandLineHandler(MyCLHandler, "-g", "-g x,y", "goal position of the search" );
	InstallCommandLineHandler(MyCLHandler, "-j", "-h jumping_policy", "selects the jumping policy as in the sfbds option" );
	InstallCommandLineHandler(MyCLHandler, "-h", "-h heuristic", "selects the heuristic function as in the sfbds option" );
	InstallCommandLineHandler(MyCLHandler, "-np", "-np", "disables advanced pruning" );
	InstallWindowHandler(MyWindowHandler);
	InstallKeyboardHandler(MyDisplayHandler,"step","take a node from the open queue", kAnyModifier, 's' );
	InstallKeyboardHandler(MyDisplayHandler,"pause","pause/unpause simulation", kAnyModifier, 'p' );
	InstallKeyboardHandler(MyDisplayHandler,"number","number nodes expanded", kAnyModifier, 'n' );
	InstallMouseClickHandler(MyClickHandler);
	return;
};

int visualization( int argc, char* argv[] ) {
	InstallHandlers();
	RunHOGGUI( argc, argv );
	return 0;
};



/*------------------------------------------------------------------------------
| Main routine passing the calls to the subroutines
------------------------------------------------------------------------------*/
int main( int argc, char* argv[] ) {

	// Syntax
	if( argc < 2 ) {
		std::cout << "Syntax: " << argv[0] << " <functionality>" << std::endl;
		std::cout << "where functionality is one of:" << std::endl;
		std::cout << "  sfbds         - compute SFBDS for certain heuristics" << std::endl;
		std::cout << "  experiment    - run an experiment with SFBDS" << std::endl;
		std::cout << "  visualize     - run a visualization of SFBDS" << std::endl;
		std::cout << "  testpoints    - compute experiment setups" << std::endl;
		std::cout << "  templatea     - run template A*" << std::endl;
		std::cout << "  templateaex   - run an experiment with Template A*" << std::endl;
		std::cout << "  pancakestates - compute test states for the pancake puzzle" << std::endl;
		std::cout << "  pancakepdb    - builds a PDB for the pancake puzzle" << std::endl;
		std::cout << "  pancakeex     - run an experiment for the pancake puzzle" << std::endl;
		std::cout << "  pancake       - compute for one single instance of the pancake puzzle" << std::endl;
		std::cout << "  pancakeida    - optimal IDA* on a pancake puzzle" << std::endl;
		std::cout << "  pancakeidaex  - experiment with IDA* and jumping policies on the pancakes" << std::endl;
		return(0);
	}

	if( strcmp( argv[1], "sfbds" ) == 0 )
		return compute_sfbds( argc, argv );
	else if( strcmp( argv[1], "experiment" ) == 0 )
		return compute_experiment( argc, argv );
	else if( strcmp( argv[1], "visualize" ) == 0 )
		return visualization( argc, argv );
	else if( strcmp( argv[1], "testpoints" ) == 0 )
		return compute_testpoints( argc, argv );
	else if( strcmp( argv[1], "templatea" ) == 0 )
		return compute_templatea( argc, argv );
	else if( strcmp( argv[1], "templateaex" ) == 0 )
		return compute_experimenttemplatea( argc, argv );
	else if( strcmp( argv[1], "pancakestates" ) == 0 )
		return compute_pancakestates( argc, argv );
	else if( strcmp( argv[1], "pancakepdb" ) == 0 )
		return compute_pancakepdb( argc, argv );
	else if( strcmp( argv[1], "pancakeex" ) == 0 )
		return compute_pancakeex( argc, argv );
	else if( strcmp( argv[1], "pancake" ) == 0 )
		return compute_pancake( argc, argv );
	else if( strcmp( argv[1], "pancakeida" ) == 0 )
		return compute_pancakeida( argc, argv );
	else if( strcmp( argv[1], "pancakeidaex" ) == 0 )
		return compute_pancakeidaex( argc, argv );
	else {
		std::cout << "ERROR: no valid method submitted." << std::endl;
		return(1);
	}

};

// end
