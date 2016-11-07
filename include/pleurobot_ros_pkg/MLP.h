#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include <fstream>
#include <pthread.h>
#include <vector>
#include <string.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdlib.h>



#define pi          3.141592653589793


#ifndef MLP_HPP
#define MLP_HPP

using namespace Eigen;
using namespace std;



class MLP{


  public:
    //================== public variables ===============================================
    int nX, nH, nY, nParam;
    Matrix<double, Dynamic, 1> W, x, t, y;
    Matrix<double, Dynamic, 1> z2, a2, z3, b1, b2, e, db1, db2;
    Matrix<double, Dynamic, Dynamic> w1, w2, dw1, dw2;
    


    //================== public functions ===============================================
    MLP(int inputDim, int hiddenDim, int outputDim);  // constructor
    void setInput(double *input);
    void setWeights(double *weights);
    void setTarget(double *targets);
    void forwardRun();
    void trainSingleSample(double *targets, double mu);
    MatrixXd sigActFun(MatrixXd v);
    MatrixXd sigActFunPrime(MatrixXd v);
    MatrixXd tanhActFun(MatrixXd v);
    MatrixXd tanhActFunPrime(MatrixXd v);
    MatrixXd linActFun(MatrixXd v);
  private:
    //================== private variables ===============================================
    Matrix<double, Dynamic, 1> linActFunCoeff;
 
	
	
	
	
    
    //================== private functions ===============================================

    void backProp();

};


#endif
