#include "pleurobot_ros_pkg/hill_muscle.h"

hill_muscle :: hill_muscle(){
  
  // default parameters
  lce_opt = 1.;
  k_l = 0.2;
//   vmax = 12; //legs 
  vmax = 6; //spine 
  k1_v = 5.;
  k2_v = 7.5;
  Fmax_v = 1.5;
  k1_pe = 2.;
  k2_pe = 0.2;
  k3_pe = 10.;
  lse_opt = 0.01;//0.01
  k_se = 10.;
  Fmax = 10.;
  
  lever = 0.1;
  
  // initial conditions
  lm = lce_opt + lse_opt;
  lce = lce_opt;
  vce = 0.;
  a = 0.;
  F = 0.;
  T = 0.;
  
};

void hill_muscle :: initialize(double l_muscle){
    
    lm = l_muscle;
    
    if(l_muscle <= lce_opt + lse_opt){
	lce = lce_opt;
	lse = lm-lce;
    }
    else{
	lse = sqrt(k1_pe)/(sqrt(k_se)+sqrt(k1_pe))*(lm-lce_opt) + sqrt(k_se)/(sqrt(k_se)+sqrt(k1_pe))*lse_opt;
    }
    
}

void hill_muscle :: setMuscleParams(MP MParams){
    
    vmax = MParams.vmax;
    lce_opt = MParams.lce_opt;
    lse_opt = MParams.lse_opt;
    k_se = MParams.k_se;
    Fmax = MParams.Fmax;
    
};

double hill_muscle :: fl(){
 
  return exp(-pow(fabs(lce-lce_opt),3)/(k_l*k_l));
  
};

double hill_muscle :: fv(){
  
  if(vce < 0){
    return (vmax+vce)/(vmax-k1_v*vce);
  }
  else{
    return Fmax_v-(Fmax_v-1.0)*(vmax+vce)/(vmax+k1_v*k2_v*vce);
  }
  
};

double hill_muscle :: fv_inv(double Fv){

  double v;
  if(Fv > 1.){
    v = (Fv-1.)/((Fmax_v-Fv)*k1_v*k2_v-Fmax_v+1.)*vmax;
    if(isnan(v))
	v = 0;
    if(fabs(v)>vmax || v<0.){
      return vmax;
    }
    else{
      return v;
    }
  }
  else{
    if(Fv < 0.){
      return v = -vmax;
    }
    else{
      return v = (Fv-1.)/(k1_v*Fv+1.)*vmax;
    }
  }
  
  
};
  
double hill_muscle :: fpe(){
  
  if(lce>lce_opt){
    return k1_pe*(lce-lce_opt)*(lce-lce_opt);
  }
  else if(lce<k2_pe*lce_opt){
    return -k3_pe*(lce-k2_pe*lce_opt)*(lce-k2_pe*lce_opt);
  }
  else{
    return 0;
  }
 
};

double hill_muscle :: fse(){

  if(lse>lse_opt){
    return k_se*(lse-lse_opt)*(lse-lse_opt);
  }
  else{
    return 0;
  }
  
};

void hill_muscle :: update(double l_muscle, double activation, double dt){
  
  lm = l_muscle;
  lse = lm - lce;
  F = Fmax * fse();
  T = F *lever;
  a = activation;
  if(a <= 0){
    a = 0.01;
  } // to avoid zero activation --> problems in fv_inv
  
  //vce = fv_inv((fse()-fpe())/(a*fl()));
  vce = fv_inv((fse())/(a*fl()+fpe()));
  lce = lce + vce*dt;
  
};

double hill_muscle :: getForce(){
 
  return F;
  
}

double hill_muscle :: getTorque(){
 
  return T;
  
}

void hill_muscle :: getStates(double *states){
 
  states[0] = lce;
  states[1] = lse;
  states[2] = lm;
  states[3] = vce;
  states[4] = a;
  
}

void hill_muscle :: getProperties(double *properties){
  
  properties[0] = lce_opt;
  properties[1] = k_l;
  properties[2] = vmax; 
  properties[3] = k1_v;
  properties[4] = k2_v;
  properties[5] = Fmax_v;
  properties[6] = k1_pe;
  properties[7] = k2_pe;
  properties[8] = k3_pe;
  properties[9] = lse_opt;
  properties[10] = k_se;
  properties[11] = Fmax;
  
}

hill_muscle :: ~hill_muscle(){
  
}

