#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#ifndef JROBINSON_H
#define JROBINSON_H

#define JROBINSON_EPS (0.0000001)

int jrobinson( const gsl_matrix *A, double epsilon, gsl_vector *p1, gsl_vector *p2, int &iter );

#endif
