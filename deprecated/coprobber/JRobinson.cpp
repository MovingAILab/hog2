#include <values.h>
#include <math.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "JRobinson.h"

/*------------------------------------------------------------------------------
| global help variables (s.t. we do not allocate memory all the time)
------------------------------------------------------------------------------*/
gsl_vector *__v_help1, *__v_help2;

/*------------------------------------------------------------------------------
| Help function declarations
------------------------------------------------------------------------------*/
int jrobinson_update( const gsl_matrix *A, gsl_vector *p1, gsl_vector *p2, double epsilon, int &N );

/*------------------------------------------------------------------------------
| Help functions
------------------------------------------------------------------------------*/

// returns the two strategies that should be played:
// player 1 (p1) plays min max
// player 2 (p2) plays max min
int jrobinson( const gsl_matrix *A, double epsilon, gsl_vector *p1, gsl_vector *p2, int &iter ) {
	int ret = 0;

	// failsafe code
	if( p1->size != A->size1 ) {
		printf( "ERROR: in jrobinson size of p1 not compatible with size of A\n" );
		return 1;
	}
	if( p2->size != A->size2 ) {
		printf( "ERROR: in jrobinson size of p2 not compatible with size of A\n" );
		return 1;
	}

	// initialization and allocation
	gsl_vector_set_all( p1, (double)p2->size );
	gsl_vector_set_all( p2, (double)p1->size );
	__v_help1 = gsl_vector_alloc( p1->size );
	__v_help2 = gsl_vector_alloc( p2->size );

	// until we reached the appreciated precision
	iter = 1;
	int N = (p1->size) * (p2->size);
	while( jrobinson_update( A, p1, p2, epsilon, N ) )
		iter++;

	// cleanup of the computation
	// we substract our initial guess from the result (=every move for every
	// player uniform at random)
	N -= (p1->size) * (p2->size);
	if( N != 0 ) {
		ret |= gsl_vector_add_constant( p1, (-1.) * (double)(p2->size) );
		ret |= gsl_vector_add_constant( p2, (-1.) * (double)(p1->size) );
		ret |= gsl_vector_Scale( p1, 1./(double)N );
		ret |= gsl_vector_Scale( p2, 1./(double)N );
	} else {
		// in this case we found an equilibrium without iterating
		// perform all actions uniform at random
		gsl_vector_set_all( p1, 1./(double)(p1->size) );
		gsl_vector_set_all( p2, 1./(double)(p2->size) );
	}

	// cleanup
	gsl_vector_free( __v_help1 );
	gsl_vector_free( __v_help2 );

	return ret;
}

// performs an update on the two strategies of player one and player two
// hereby, player1 tries to min max and player2 tries to max min
int jrobinson_update( const gsl_matrix *A, gsl_vector *p1, gsl_vector *p2, double epsilon, int &N ) {
	double temp, h_max = DBL_MAX;
	size_t indexu, indexv;
	gsl_vector_set_zero( __v_help1 );
	gsl_vector_set_zero( __v_help2 );

	gsl_blas_dgemv( CblasNoTrans, 1.0, A, p2, 1.0, __v_help1 );
	gsl_blas_dgemv( CblasTrans,   1.0, A, p1, 1.0, __v_help2 );

	indexu = gsl_vector_min_index( __v_help1 );
	indexv = gsl_vector_max_index( __v_help2 );

	// if error small enough no more iterations
	if( epsilon > fabs(gsl_vector_get(__v_help1,indexu)-gsl_vector_get(__v_help2,indexv))/N )
		return 0;

	// determine the stepsize we are going to take
	// testing for U_{i_0}-U_i \le h (a_{i,k_0}-a_{i_0,k_0}) where (a_{i,k_0}-a_{i_0,k_0}) < 0
	// where U = __v_help1, indexu = i_0, indexv = k_0
	for( unsigned int i = 0; i < A->size1; i++ ) {
		temp = gsl_matrix_get( A, i, indexv ) - gsl_matrix_get( A, indexu, indexv );
		if( temp < -JROBINSON_EPS ) {
			h_max = GSL_MIN( h_max, (gsl_vector_get(__v_help1,indexu)-gsl_vector_get(__v_help1,i))
			                         /temp );
		}
	}
	// testing for V_{k_0}-V_j \ge h (a_{i_0,j}-a_{i_0,k_0}) where (a_{i_0,j}-a_{i_0,k_0}) > 0
	// where V = __v_help2, i_0 = indexu, k_0 = indexv
	for( unsigned int j = 0; j < A->size2; j++ ) {
		temp = gsl_matrix_get( A, indexu, j ) - gsl_matrix_get( A, indexu, indexv );
		if( temp > JROBINSON_EPS ) {
			h_max = GSL_MIN( h_max, (gsl_vector_get(__v_help2,indexv)-gsl_vector_get(__v_help2,j))
			                        /temp );
		}
	}

	h_max = floor( h_max );

	if( (double)N + h_max >= (double)INT_MAX ) {
		// case 1: there was no restiction on the stepsize
		// -> thus, we have an optimal pure strategy!!!
		// case 2: we add too large of a distance
		// -> then we can set the number of iteration to its maximum and stop
		gsl_vector_set( p1, indexu, gsl_vector_get( p1, indexu ) + (double)(INT_MAX-N) );
		gsl_vector_set( p2, indexv, gsl_vector_get( p2, indexv ) + (double)(INT_MAX-N) );
		N = INT_MAX;
		return 0;

	} else {

		N += (int)h_max + 1;

		gsl_vector_set( p1, indexu, gsl_vector_get( p1, indexu ) + h_max + 1. );
		gsl_vector_set( p2, indexv, gsl_vector_get( p2, indexv ) + h_max + 1. );
	}

	return 1;
}

/*------------------------------------------------------------------------------
| Testing - Main
------------------------------------------------------------------------------*/
//int main( int argc, char** argv ) {
/*
	double bA[2*2] = {
		-1., 1.,
		1., 1.
	};

	gsl_matrix_view A = gsl_matrix_view_array( bA, 2, 2 );

	gsl_vector *p1 = gsl_vector_alloc( 2 );
	gsl_vector *p2 = gsl_vector_alloc( 2 );
*/
/*
	double bA[9*4] = {
		12.9498045569, 12.9498045569, 12.9496772733, 12.9496928083,
		12.9421949857, 12.9421949857, 14.5611261778, 12.9446184024,
		12.9498045569, 12.9498045569, 12.9495363245, 12.9498045569,
		13.7765011050, 13.7765011050, 13.7752708888, 13.7754882879,
		13.7760636294, 13.7760636294, 14.5611261778, 13.7754882879,
		13.7765011050, 13.7765011050, 13.7764877022, 13.7765011050,
		12.0614941173, 12.0614941173, 12.0574655027, 12.0611049086,
		12.0614929677, 12.0614929677, 14.2303793300, 12.0604603271,
		12.0614941173, 12.0614941173, 12.0536482188, 12.0614941173
	};
	gsl_matrix_view A = gsl_matrix_view_array(bA,9,4);
	for( unsigned int i = 0; i < A.matrix.size1; i++ ) {
		for( unsigned int j = 0; j < A.matrix.size2; j++ ) {
			printf( "%f ", gsl_matrix_get( &(A.matrix), i, j ) );
		}
		printf( "\n" );
	}
	gsl_vector *p1 = gsl_vector_alloc( 9 );
	gsl_vector *p2 = gsl_vector_alloc( 4 );

	double epsilon = 0.01;
	int iter;

	printf( "%d\n", jrobinson( &(A.matrix), epsilon, p1, p2, iter ) );

	printf( "iter = %d, p1 = \n", iter );
	gsl_vector_fprintf( stdout, p1, "%f" );
	printf( "p2 = \n" );
	gsl_vector_fprintf( stdout, p2, "%f" );

	// cleanup
	gsl_vector_free( p1 );
	gsl_vector_free( p2 );
	return 0;
}
*/
