/*
 *  FunctionApproximator.h
 *  games
 *
 *  Created by Adam White on Mon Apr 04 2005.
 *  Copyright (c) 2005 __MyCompanyName__. All rights reserved.
 *
 */  
 
//#include "LearningPlayer.h"
#include <ctime>
#include <vector>
#include <stdio.h>
//#include "random.h"

#ifndef FUNCTIONAPPROXIMATOR_H
#define FUNCTIONAPPROXIMATOR_H

enum tActivation {
  kLinear,
  kStep,
  kExponential
};

class FunctionApproximator {  
public:
	FunctionApproximator() {};
	virtual ~FunctionApproximator() {};

	//virtual FunctionApproximator *clone() const = 0;
	
	virtual void save(const char *) = 0;
	virtual void save(FILE *) = 0;
	virtual void load(const char *) = 0;
	virtual void load(FILE *) = 0;
	virtual void load(const FunctionApproximator *) = 0;

	// these fucntions are for training with the list of all features and their (arbitrary) values
	virtual double train(std::vector<double> &input, std::vector<double> &output2) = 0;
	virtual double *test(const std::vector<double> &input) = 0;
	// these functions are for training with a list of binary features that are on
	virtual double train(std::vector<unsigned int> &input, std::vector<double> &output2) = 0;
	virtual double *test(const std::vector<unsigned int> &input) = 0;
	// Given a target output (fixed internal weights), return the intput that would
	// lead to the output. (May require more than one call to converge.)
	virtual double GetInput(std::vector<double> &input, const std::vector<double> &target) = 0;

	virtual void setLearnRate(double);
	virtual double getLearnRate();
	
	void setOutputActivation(tActivation t)
	{ outputActivation = t; }
	tActivation getOutputActivation()
	{ return outputActivation; }

	virtual int getNumInputs() { return 0; }
	virtual double getInputWeight(int inp, int outp=0) { return 0; }

	virtual void Print() = 0;
protected:
		double rate;
	tActivation outputActivation;
};

#endif
