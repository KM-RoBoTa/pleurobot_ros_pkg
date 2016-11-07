#include "pleurobot_ros_pkg/MLP.h"



//================== public functions ===============================================

/* MLP constructor */
MLP :: MLP(int inputDim, int hiddenDim, int outputDim) 
{
    nX=inputDim;
    nH=hiddenDim;
    nY=outputDim;

    // get total number of parameters
    nParam=(nH+1)*nX+(nY+1)*nH;

    // initialize weights
    W=MatrixXd::Random(nParam,1);
    linActFunCoeff=MatrixXd::Ones(nY, 1);
    
    //w1=W.block(0, 0, nX*nH, 1).replicate(nH, nX);
    w1=Map<MatrixXd>(W.block(0, 0, nX*nH, 1).data(),nH,nX);
    b1=W.block(nX*nH, 0, nH, 1);
    w2=Map<MatrixXd>(W.block(nX*nH+nH, 0, nY*nH, 1).data(),nY,nH);
    b2=W.block(nX*nH+nH+nY*nH, 0, nY, 1);  
}

/* Sets inputs */
void
MLP :: setInput(double *input)
{
    x=Map<MatrixXd>(input, nX, 1);
}

/* Sets weights */
void
MLP :: setWeights(double *weights)
{
    W=Map<MatrixXd>(weights, nParam, 1);
}

/* Sets targets */
void
MLP :: setTarget(double *targets)
{
    t=Map<MatrixXd>(targets, nY, 1);
}

/* Runs MLP and calculates output */
void
MLP :: forwardRun()
{
   z2=w1*x+b1;
   a2=tanhActFun(z2);
   z3=w2*a2+b2;
   y=z3;
}

void 
MLP :: trainSingleSample(double *targets, double mu)
{
    t=Map<MatrixXd>(targets, nY, 1);
    e=t-y;
    backProp();
    w1=w1-mu*dw1;
    w2=w2-mu*dw2;
    b1=b1-mu*db1;
    b2=b2-mu*db2;

}


//================== private functions ===============================================

/* Sigmoid activation function */
MatrixXd
MLP :: sigActFun(MatrixXd v){
    v*=-1;
    return (1+v.array().exp()).inverse().matrix();
}

/* Tanh activation function */
MatrixXd
MLP :: tanhActFun(MatrixXd v){
    ArrayXd tmp1(v.rows(), v.cols()), tmp2(v.rows(), v.cols());
    tmp1=v.array().exp();
    v=-v;
    tmp2=v.array().exp();
    return ((tmp1-tmp2)*(tmp1+tmp2).inverse()).matrix();
}

/* Linear activation function */
MatrixXd
MLP :: linActFun(MatrixXd v){
    return v.cwiseProduct(linActFunCoeff);
}

/* Derivative of sigmoid act. function */
MatrixXd
MLP :: sigActFunPrime(MatrixXd v){
    v*=-1;
    return ((1+v.array().exp()).inverse() * (1-(1+v.array().exp()).inverse())).matrix() ;
}

/* Derivative of tanh act. function */
MatrixXd
MLP :: tanhActFunPrime(MatrixXd v){
    ArrayXd tmp1(v.rows(), v.cols()), tmp2(v.rows(), v.cols());
    tmp1=v.array().exp();
    v=-v;
    tmp2=v.array().exp();
    tmp1=(tmp1-tmp2)*(tmp1+tmp2).inverse();
    return (1-tmp1*tmp1).matrix();
}

/* Backpropagation algorithm */
void
MLP :: backProp()
{
    MatrixXd delta3(nH, 1), delta2(nH, 1);

    delta3 = -e;

    delta2 = tanhActFunPrime(z2);
    delta2 = delta2.cwiseProduct(w2.transpose()*delta3);

    dw2=delta3*a2.transpose();
    dw1=delta2*x.transpose();
    db2=delta3;
    db1=delta2;
    

}
