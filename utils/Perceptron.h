/*
 *  Perceptron.h
 *  games
 *
 *  Created by Nathan Sturtevant on 3/2/05.
 *  Copyright 2005 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef PERCEPTRON_H
#define PERCEPTRON_H
#include "FunctionApproximator.h"

class perceptron : public FunctionApproximator  {
public:
	perceptron(int inputs, int outputs, double learnrate);
	perceptron(perceptron *);
	perceptron(char *);
	perceptron(FunctionApproximator *);
	~perceptron();
	
	void save(const char *);
	void save(FILE *);
	void load(const char *);
	void load(FILE *);
	void load(const FunctionApproximator *fa) { load((perceptron*)fa); }
	void load(const perceptron *);
	static bool validSaveFile(char *fname);

	double train(std::vector<double> &input, std::vector<double> &output2);
	double *test(const std::vector<double> &input);
	double train(std::vector<unsigned int> &input, std::vector<double> &output2);
	double *test(const std::vector<unsigned int> &input);
	
	void Print();

	int getNumInputs() { return inputs+1; }
	double getInputWeight(int inp, int outp=0) { return weight[outp][inp]; }
private:
		void allocateMemory();
	void freeMemory();

	double g(double a);
	double dg(double a);
	double outputerr(const std::vector<double> &output, const std::vector<double> &expected, int which);
	
	double error(double* output);
	std::vector<std::vector<double> > weight;
	std::vector<double> output;
	//double** weight;
	//double* output;
	int inputs, outputs;
};

#endif
