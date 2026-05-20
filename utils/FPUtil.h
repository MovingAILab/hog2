/*
 *  $Id: fpUtil.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef fpUtil_H
#define fpUtil_H

#include "float.h"
#include <limits>

// Somehow DBL_MAX is not defined under Linux?
//#ifndef OS_MAC
//#define DBL_MAX std::numeric_limits<double>::max()//1.79769313486231500e+308;    // DBL_MAX for non Mac OS
//#define DBL_MIN std::numeric_limits<double>::min()// DBL_MIN for non Mac OS
//#define MAXFLOAT std::numeric_limits<float>::max()
//#endif

// Floating point comparisons 
static const double TOLERANCE = 0.000001;    // floating point tolerance

inline bool fless(double a, double b, double tolerance=TOLERANCE) { return (a < b - tolerance); }
inline bool fgreater(double a, double b, double tolerance=TOLERANCE) { return (a > b + tolerance); }
inline bool flesseq(double a, double b, double tolerance=TOLERANCE) { return !fgreater(a, b, tolerance); }
inline bool fgreatereq(double a, double b, double tolerance=TOLERANCE) { return !fless(a, b, tolerance); }
inline bool fequal(double a, double b, double tolerance=TOLERANCE)
{ return (a >= b - tolerance) && (a <= b+tolerance); }

inline double min(double a, double b) { return fless(a, b)?a:b; }
inline double max(double a, double b) { return fless(a, b)?b:a; }

#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>


// Code from: https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values/16606128#16606128
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}



#endif
