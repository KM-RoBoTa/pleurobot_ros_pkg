#ifndef HILL_MUSCLE_HPP
#define HILL_MUSCLE_HPP

#include <cmath>

#define N_HILL_PROPERTIES 	12
#define N_HILL_STATES 		5


typedef struct{
    double Fmax;
    double vmax;
    double lce_opt;
    double lse_opt;
    double k_se;
}MP;

class hill_muscle{
  
  private:
    /* states */
    double lce;		// length CE (Contraction element)
    double lse;		// length SE (Serial elastic element)
    double lm;		// (total) length muscle-tendon complex
    double vce;		// velocity CE
    double a; 		// activation
    
    /* properties */
    double Fmax;
    double vmax;
    double lce_opt;
    double lse_opt;
    double k_l;
    double Fmax_v, k1_v, k2_v;
    double k1_pe, k2_pe, k3_pe;
    double k_se;
    
    /* lever arm */
    double lever;
    
    /* output */
    double F; 		// force output
    double T; 		// torque output
    
    /* functions */
    double fl();
    double fv();
    double fv_inv(double);
    double fpe();
    double fse();
    
  public:
    /* functions*/
    hill_muscle();		// constructor
    void initialize(double);
    void setMuscleParams(MP);
    void update(double,double,double);
    double getForce();
    double getTorque();
    void getStates(double*);
    void getProperties(double*);
    ~hill_muscle();
};

#endif