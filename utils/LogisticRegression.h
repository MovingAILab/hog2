//
//  LogisticRegression.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/19/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__LogisticRegression__
#define __hog2_glut__LogisticRegression__

#include "FPUtil.h"
#include "FunctionApproximator.h"
#include "LinearRegression.h"

class LogisticRegression : public FunctionApproximator  {
public:
	LogisticRegression(int _inputs, int _outputs, double learnrate);
	LogisticRegression(LogisticRegression *);
	LogisticRegression(char *);
	LogisticRegression(FunctionApproximator *);
	~LogisticRegression();
	
	void setUseBinary(bool binary) { useBinary = binary; }
	
	void resizeInputs(int newSize, double newVal);
	void resizeInputs(int newSize);
	
	void save(const char *);
	void save(FILE *);
	void load(const char *);
	void load(FILE *);
	void load(const FunctionApproximator *fa) { load((LogisticRegression*)fa); }
	void load(const LogisticRegression *);
	static bool validSaveFile(char *fname);
	
	double train(std::vector<double> &input, std::vector<double> &output2);
	double *test(const std::vector<double> &input);
	double train(std::vector<unsigned int> &input, std::vector<double> &output2);
	double *test(const std::vector<unsigned int> &input);
	
	void Print();
	
	int getNumInputs() { return inputs; }
	void setInputWeight(double value, unsigned int weightNum, unsigned int whichOutput=0);
	double getInputWeight(int inp, int outp=0) { return weight[outp][inp]; }
	void getWeightUpdateVariance(std::vector<double> &var, unsigned int which=0);
	double getWeightUpdateVariance(unsigned int weightNum, unsigned int whichOutput=0);
	
	void getWeightUpdateAverage(std::vector<double> &var, unsigned int which=0);
	double getWeightUpdateAverage(unsigned int weightNum, unsigned int whichOutput=0);
	
	void getWeightUpdateSum(std::vector<double> &var, unsigned int which=0);
	double getWeightUpdateSum(unsigned int weightNum, unsigned int whichOutput=0);
	void resetWeightVariance(unsigned int weightNum, unsigned int whichOutput=0);
	void resetWeightVariance();
	
	int getWeightFrequency(unsigned int weightNum, unsigned int whichOutput=0);

	double GetInput(std::vector<double> &input, const std::vector<double> &target)
	{ assert(!"Unimplemented"); }
private:
	void allocateMemory();
	void freeMemory();
	
	double g(double a);
	double dg(double a);
	double outputerr(std::vector<double> &output, std::vector<double> &expected, int which);
	
	double error(double* output);
	std::vector<std::vector<double> > weight;
	std::vector<std::vector<updateData> > updates;
	//std::vector<double > weight;
	std::vector<double> output;
	//double** weight;
	//double* output;
	int inputs, outputs;
	bool useBinary;
};

#endif /* defined(__hog2_glut__LogisticRegression__) */
