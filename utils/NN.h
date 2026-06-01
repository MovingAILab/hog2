#ifndef NN_H
#define NN_H

#include "FunctionApproximator.h"	
#include <vector>

class NN : public FunctionApproximator {
public:
	NN(int inputs, int hiddens, int outputs, double learnrate);
	NN(NN *);
	NN(FunctionApproximator *);
	NN(char *);
	~NN();
	void load(const char *);
	void load(FILE *);
	void load(const NN *);
	void load(const FunctionApproximator *fa) { load((NN*)fa); }
	static bool validSaveFile(char *fname);
	void save(const char *);
	void save(FILE *);

	double train(std::vector<double> &input, std::vector<double> &target);
	double *test(const std::vector<double> &input);
	double GetInput(std::vector<double> &input, const std::vector<double> &target);
	double train(std::vector<unsigned int> &input, std::vector<double> &target);
	double *test(const std::vector<unsigned int> &input);

	int getNumInputs() { return inputs; }
	double getInputWeight(int inp, int outp=0) { return weights[0][outp][inp]; }

	void Print();
private:
	void allocateMemory(const NN *nn = 0);
	void freeMemory();
	
	std::vector< std::vector< std::vector<double> > > weights;
	std::vector< std::vector< std::vector<double> > > updatedweights;
	std::vector< std::vector< std::vector<double> > > errors;
	std::vector<double> hidden;
	std::vector<double> output;
	int inputs, hiddens, outputs;
	
	double g(double a);
	double dg(double a);
	double outputerr(const std::vector<double> &output, const std::vector<double> &expected, int which);
	double internalerr(const std::vector<double> &output, const std::vector<double> &expected, int which);
	double internalinputerr(const std::vector<double> &output, const std::vector<double> &expected, int which);
	double error(const std::vector<double> &outputs);
};


#endif
