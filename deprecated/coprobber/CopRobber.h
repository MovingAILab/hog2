#include "Common.h"
#include "Map2DEnvironment.h"
#include "Graph.h"

void output_syntax();

// algorithms
void compute_testing( int argc, char* argv[] );
void compute_dijkstra( int argc, char* argv[] );
void compute_rma( int argc, char* argv[] );
void compute_minimax( int argc, char* argv[] );
void compute_minimax_optimized( int argc, char* argv[] );
void compute_tida( int argc, char* argv[] );
void compute_ipn( int argc, char* argv[] );
void compute_ipnttables( int argc, char* argv[] );
void compute_dsidfpn( int argc, char* argv[] );
void compute_tpdijkstra( int argc, char* argv[] );
void compute_dstpdijkstra( int argc, char* argv[] );
void compute_dsdijkstra( int argc, char* argv[] );
void compute_dsdijkstra_memoptim( int argc, char* argv[] );
void compute_dsbestresponse( int argc, char* argv[] );
void compute_dsrma( int argc, char* argv[] );
void compute_dscover( int argc, char* argv[] );
void compute_dscover2( int argc, char* argv[] );
void compute_dsheuristicgreedy( int argc, char* argv[] );
void compute_dsminimax( int argc, char* argv[] );
void compute_dsdam( int argc, char* argv[] );
void compute_dsidam( int argc, char* argv[] );
void compute_dsdatpdijkstra( int argc, char* argv[] );
void compute_dsrandombeacons( int argc, char* argv[] );
void compute_markov( int argc, char* argv[] );
void compute_twocopsdijkstra( int argc, char* argv[] );
// tests
void compute_testpoints( int argc, char* argv[] );
void compute_testpoints_two_cops( int argc, char* argv[] );
void compute_experiment_optimal( int argc, char* argv[] );
void compute_experiment_optimal_two_cops( int argc, char* argv[] );
void compute_experiment_allstate( int argc, char* argv[] );
void compute_experiment_markov( int argc, char* argv[] );
void compute_experiment_graphs( int argc, char* argv[] );
void compute_characterization_2copwin( int argc, char* argv[] );
void compute_website_interface( int argc, char* argv[] );
void compute_experiment_suboptimal( int argc, char* argv[] );


void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyDisplayHandler(unsigned long windowID, tKeyboardModifier, char key);
int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
void InstallHandlers();


void parseCommandLineParameters( int argc, char* argv[], Map* &m, xyLoc &pos_cop, xyLoc &pos_robber, int &max_recursion_level );

Graph *readGraphFromCommandLine( int offset, int argc, char* argv[] );
Graph* readGraph( FILE *fhandler, bool input_with_vertice_coordinates = false );
void writeGraph( FILE *fhandler, Graph *g );
