 #include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;


void
Controller :: walkingTrajectories()
{
	// initialize starting points for all legs
	static Vector3d swingStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
	static Vector3d stanceStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);

	static Vector3d swingStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
	static Vector3d stanceStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);

	static Vector3d swingStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
	static Vector3d stanceStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);

	static Vector3d swingStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
	static Vector3d stanceStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);



    // ########################## UPDATE STANCE START ################################
    if(legs_stance(0)==0){
        stanceStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
    }
    if(legs_stance(1)==0){
        stanceStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
    }
    if(legs_stance(2)==0){
        stanceStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
    }
    if(legs_stance(3)==0){
        stanceStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
    }


	// ########################## UPDATE SWING START ################################
	if(legs_stance(0)==1){
		swingStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
	}
	if(legs_stance(1)==1){
		swingStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
	}
	if(legs_stance(2)==1){
		swingStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
	}
	if(legs_stance(3)==1){
		swingStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
	}

    



    Vector3d velFL=MatrixXd::Zero(3, 1), velFR=MatrixXd::Zero(3, 1), velHL=MatrixXd::Zero(3, 1), velHR=MatrixXd::Zero(3, 1);
    //legs_stance.setZero();

    if(state==WALKING || state==STANDING || state==POSING){
        z_offset=(joy_l1-joy_r1)/10000.;
    }
    

    trPointsFL(2,0)+=z_offset; trPointsFL(2,2)+=z_offset; 
    trPointsFR(2,0)+=z_offset; trPointsFR(2,2)+=z_offset;
    trPointsHL(2,0)+=z_offset; trPointsHL(2,2)+=z_offset;
    trPointsHR(2,0)+=z_offset; trPointsHR(2,2)+=z_offset;
   

    // GET ADITIONAL POINTS FOR BEZIER INTERPOLATION FOR SWING PHASE
    static MatrixXd B1FL(3, 4), B2FL(3, 4);
    static MatrixXd B1FR(3, 4), B2FR(3, 4);
    static MatrixXd B1HL(3, 4), B2HL(3, 4);
    static MatrixXd B1HR(3, 4), B2HR(3, 4);
    static MatrixXd Bcurve(3, 1);
    Transform<double,3,Affine>  tempRot;
    static double bnorm, tB;



    //FL
    B1FL.block<3, 1>(0,0)=swingStartFL;
    B1FL.block<3, 1>(0,3)=trPointsFL.block<3, 1>(0, 1);
    bnorm=(B1FL.block<3, 1>(0,0)-B1FL.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(trAnglesF0[2], Vector3d::UnitX())*AngleAxisd(trAnglesF0[0], Vector3d::UnitY());
    B1FL.block<3, 1>(0,1) = B1FL.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamF0[1]*bnorm);                  
    B1FL.block<3, 1>(0,2) << B1FL(0,3) - bezierParamF0[0]*(B1FL(0, 3)-B1FL(0,0)), 
                            B1FL(1,3), 
                            B1FL(2,3);
 


    B2FL.block<3, 1>(0,0)=trPointsFL.block<3, 1>(0, 1);
    B2FL.block<3, 1>(0,3)=trPointsFL.block<3, 1>(0, 0);
    bnorm=(B2FL.block<3, 1>(0,0)-B2FL.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(trAnglesF0[3], Vector3d::UnitX())*AngleAxisd(trAnglesF0[1], Vector3d::UnitY());

    B2FL.block<3, 1>(0,2) = B2FL.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamF0[1]*bnorm);                        
    B2FL.block<3, 1>(0,1) << B2FL(0,0) + bezierParamF0[0]*(B2FL(0, 3)-B2FL(0,0)), 
                            B2FL(1,0), 
                            B2FL(2,0);

             
    //FR
    B1FR.block<3, 1>(0,0)=swingStartFR;
    B1FR.block<3, 1>(0,3)=trPointsFR.block<3, 1>(0, 1);
    bnorm=(B1FR.block<3, 1>(0,0)-B1FR.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(-trAnglesF0[2], Vector3d::UnitX())*AngleAxisd(trAnglesF0[0], Vector3d::UnitY());
    B1FR.block<3, 1>(0,1) = B1FR.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamF0[1]*bnorm);                  
    B1FR.block<3, 1>(0,2) << B1FR(0,3) - bezierParamF0[0]*(B1FR(0, 3)-B1FR(0,0)), 
                            B1FR(1,3), 
                            B1FR(2,3);
 


    B2FR.block<3, 1>(0,0)=trPointsFR.block<3, 1>(0, 1);
    B2FR.block<3, 1>(0,3)=trPointsFR.block<3, 1>(0, 0);
    bnorm=(B2FR.block<3, 1>(0,0)-B2FR.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(-trAnglesF0[3], Vector3d::UnitX())*AngleAxisd(trAnglesF0[1], Vector3d::UnitY());

    B2FR.block<3, 1>(0,2) = B2FR.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamF0[1]*bnorm);                        
    B2FR.block<3, 1>(0,1) << B2FR(0,0) + bezierParamF0[0]*(B2FR(0, 3)-B2FR(0,0)), 
                            B2FR(1,0), 
                            B2FR(2,0);
    //HL                        
    B1HL.block<3, 1>(0,0)=swingStartHL;
    B1HL.block<3, 1>(0,3)=trPointsHL.block<3, 1>(0, 1);
    bnorm=(B1HL.block<3, 1>(0,0)-B1HL.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(trAnglesH0[2], Vector3d::UnitX())*AngleAxisd(trAnglesH0[0], Vector3d::UnitY());
    B1HL.block<3, 1>(0,1) = B1HL.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamH0[1]*bnorm);                  
    B1HL.block<3, 1>(0,2) << B1HL(0,3) - bezierParamH0[0]*(B1HL(0, 3)-B1HL(0,0)), 
                            B1HL(1,3), 
                            B1HL(2,3);
 


    B2HL.block<3, 1>(0,0)=trPointsHL.block<3, 1>(0, 1);
    B2HL.block<3, 1>(0,3)=trPointsHL.block<3, 1>(0, 0);
    bnorm=(B2HL.block<3, 1>(0,0)-B2HL.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(trAnglesH0[3], Vector3d::UnitX())*AngleAxisd(trAnglesH0[1], Vector3d::UnitY());

    B2HL.block<3, 1>(0,2) = B2HL.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamH0[1]*bnorm);                        
    B2HL.block<3, 1>(0,1) << B2HL(0,0) + bezierParamH0[0]*(B2HL(0, 3)-B2HL(0,0)), 
                            B2HL(1,0), 
                            B2HL(2,0);

    //HR
    B1HR.block<3, 1>(0,0)=swingStartHR;
    B1HR.block<3, 1>(0,3)=trPointsHR.block<3, 1>(0, 1);
    bnorm=(B1HR.block<3, 1>(0,0)-B1HR.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(-trAnglesH0[2], Vector3d::UnitX())*AngleAxisd(trAnglesH0[0], Vector3d::UnitY());
    B1HR.block<3, 1>(0,1) = B1HR.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamH0[1]*bnorm);                  
    B1HR.block<3, 1>(0,2) << B1HR(0,3) - bezierParamH0[0]*(B1HR(0, 3)-B1HR(0,0)), 
                            B1HR(1,3), 
                            B1HR(2,3);
 


    B2HR.block<3, 1>(0,0)=trPointsHR.block<3, 1>(0, 1);
    B2HR.block<3, 1>(0,3)=trPointsHR.block<3, 1>(0, 0);
    bnorm=(B2HR.block<3, 1>(0,0)-B2HR.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(-trAnglesH0[3], Vector3d::UnitX())*AngleAxisd(trAnglesH0[1], Vector3d::UnitY());

    B2HR.block<3, 1>(0,2) = B2HR.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParamH0[1]*bnorm);                        
    B2HR.block<3, 1>(0,1) << B2HR(0,0) + bezierParamH0[0]*(B2HR(0, 3)-B2HR(0,0)), 
                            B2HR(1,0), 
                            B2HR(2,0);

                


    // GENERATE TRAJECTORIES                
    if(state==WALKING){
    	legPhaseDynamics(dt);

        // FL
        if(legPhase(0)<Duty(0)) {
            stancePhase(0)=legPhase(0)/Duty(0);
            if(stancePhase(0)<0.99){
                velFL=(trPointsFL.block<3,1>(0,2)-rFL)/(1-stancePhase(0));
                rFL=rFL + velFL*dt/(Duty(0))*freq_walk;
            }
            else{
                rFL=rFL + velFL*dt/(Duty(0))*freq_walk;
            }

            legs_stance(0)=1;

            


        }

        else{
            swingPhase(0)=(legPhase(0)-Duty(0))/(1-Duty(0));


            if(swingPhase(0)<0.5){
                tB=swingPhase(0)*2;

                Bcurve= pow(1-tB, 3)*B1FL.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B1FL.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B1FL.block<3, 1>(0,2)  +
                        pow(tB, 3)*B1FL.block<3, 1>(0,3); 
            }
            else{
                tB=swingPhase(0)*2-1;

                Bcurve= pow(1-tB, 3)*B2FL.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B2FL.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B2FL.block<3, 1>(0,2)  +
                        pow(tB, 3)*B2FL.block<3, 1>(0,3);
            }
            velFL=(Bcurve - rFL)/dt;
            rFL=rFL + velFL*dt;

            legs_stance(0)=0;

        }


        // FR
        if(legPhase(1)<Duty(0)) {
            stancePhase(1)=legPhase(1)/Duty(0);
            if(stancePhase(1)<0.99){
                velFR=(trPointsFR.block<3,1>(0,2)-rFR)/(1-stancePhase(1));
                rFR=rFR + velFR*dt/(Duty(0))*freq_walk;
            }
            else{
                rFR=rFR + velFR*dt/(Duty(0))*freq_walk;
            }

            legs_stance(1)=1;
        }

        else{
            swingPhase(1)=(legPhase(1)-Duty(0))/(1-Duty(0));


            if(swingPhase(1)<0.5){
                tB=swingPhase(1)*2;

                Bcurve= pow(1-tB, 3)*B1FR.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B1FR.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B1FR.block<3, 1>(0,2)  +
                        pow(tB, 3)*B1FR.block<3, 1>(0,3); 
            }
            else{
                tB=swingPhase(1)*2-1;

                Bcurve= pow(1-tB, 3)*B2FR.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B2FR.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B2FR.block<3, 1>(0,2)  +
                        pow(tB, 3)*B2FR.block<3, 1>(0,3);
            }
            velFR=(Bcurve - rFR)/dt;
            rFR=rFR + velFR*dt;

            legs_stance(1)=0;
        }


        // HL
        if(legPhase(2)<Duty(0)) {
            stancePhase(2)=legPhase(2)/Duty(0);
            if(stancePhase(2)<0.99){
                velHL=(trPointsHL.block<3,1>(0,2)-rHL)/(1-stancePhase(2));
                rHL=rHL + velHL*dt/(Duty(1))*freq_walk;
            }
            else{
                rHL=rHL + velHL*dt/(Duty(1))*freq_walk;
            }
            legs_stance(2)=1;

        }

        else{
            swingPhase(2)=(legPhase(2)-Duty(0))/(1-Duty(0));
            if(swingPhase(2)<0.5){
                tB=swingPhase(2)*2;

                Bcurve= pow(1-tB, 3)*B1HL.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B1HL.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B1HL.block<3, 1>(0,2)  +
                        pow(tB, 3)*B1HL.block<3, 1>(0,3); 
            }
            else{
                tB=swingPhase(2)*2-1;

                Bcurve= pow(1-tB, 3)*B2HL.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B2HL.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B2HL.block<3, 1>(0,2)  +
                        pow(tB, 3)*B2HL.block<3, 1>(0,3);
            }
            velHL=(Bcurve - rHL)/dt;
            rHL=rHL + velHL*dt;

            legs_stance(2)=0;
            
        }

        // HR
        if(legPhase(3)<Duty(0)) {
            stancePhase(3)=legPhase(3)/Duty(0);
            if(stancePhase(3)<0.99){
                velHR=(trPointsHR.block<3,1>(0,2)-rHR)/(1-stancePhase(3));
                rHR=rHR + velHR*dt/(Duty(1))*freq_walk;
            }
            else{
                rHR=rHR + velHR*dt/(Duty(1))*freq_walk;
            }
            legs_stance(3)=1;

        }

        else{
            swingPhase(3)=(legPhase(3)-Duty(0))/(1-Duty(0));
            if(swingPhase(3)<0.5){
                tB=swingPhase(3)*2;

                Bcurve= pow(1-tB, 3)*B1HR.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B1HR.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B1HR.block<3, 1>(0,2)  +
                        pow(tB, 3)*B1HR.block<3, 1>(0,3); 
            }
            else{
                tB=swingPhase(3)*2-1;

                Bcurve= pow(1-tB, 3)*B2HR.block<3, 1>(0,0) +
                        3*pow(1-tB, 2)*tB* B2HR.block<3, 1>(0,1)  +
                        3*pow(tB, 2)*(1-tB)* B2HR.block<3, 1>(0,2)  +
                        pow(tB, 3)*B2HR.block<3, 1>(0,3);
            }
            velHR=(Bcurve - rHR)/dt;
            rHR=rHR + velHR*dt;

            legs_stance(3)=0;
        }
    }

    if(state==STANDING){
        for(int i=0; i<4; legs_stance(i++)=1);
    }
    pFL=rFL;
    pFR=rFR;
    pHL=rHL;
    pHR=rHR;


   

    //############################ STEERING ########################################################

    static Vector2d tmp21(0, 1000), tmp22(0, 1000), tmp23(0, 1000), tmp24(0, 1000), tmp2;


    R=-steeringRadius;

    tmp2(0)=0;
    tmp2(1)=R;


    if(legs_stance(0)==1)
        tmp21=tmp2;
    if(legs_stance(1)==1)
        tmp22=tmp2;
    if(legs_stance(2)==1)
        tmp23=tmp2;
    if(legs_stance(3)==1)
        tmp24=tmp2;


    pFL=steerFun(pFL, tmp21, phiScl);
    pFR=steerFun(pFR, tmp22, phiScl);
    pHL=steerFun(pHL, tmp23, phiScl);
    pHR=steerFun(pHR, tmp24, phiScl);




    // detect beginning of a swing phase
    for(int i=0; i<4; i++){
        if(legs_stance_old(i)==1 && legs_stance(i)==0)
            phaseTransitionStanceSwing(i)=1;
        else
            phaseTransitionStanceSwing(i)=0;
        if(legs_stance_old(i)==0 && legs_stance(i)==1)
            phaseTransitionSwingStance(i)=1;
        else
            phaseTransitionSwingStance(i)=0;
    }
    legs_stance_old=legs_stance;
}

/* Modifies trajectories for steering */
Vector3d
Controller :: steerFun(Vector3d p, Vector2d R, double phiScl)
{
    if(abs(R(1)>100)|| R(1)!=R(1))
        return p;
    Vector3d pc;
    double phi;
    pc=p;
    pc(0)=p(0)-R(0);
    pc(1)=p(1)-R(1);

    if(R(1)<-0.1) {
        phi=atan2(-R(1)+phiScl, p(0));


    } else if (R(1)>0.1) {
        phi=atan2(R(1)+phiScl, -p(0));

    } else
        return p;

    p(0)=pc(1)*cos(phi)+R(0);
    p(1)=pc(1)*sin(phi)+R(1);

    return p;
}

/* Steering by rotating trajectories and without spine bending */
void
Controller :: steeringSpineless(double heading_omega){
	static double r[4], phi[4], x[4], y[4], x2[4], y2[4], xc, yc, xsum[4], ysum[4];
	xc=0.5*Fgird0(0, 3)+0.5*Hgird0(0, 3);
	yc=0.5*Fgird0(1, 3)+0.5*Hgird0(1, 3);
	x[0]=HJfl_g[4](0, 3);
	x[1]=HJfr_g[4](0, 3);
	x[2]=HJhl_g[4](0, 3);
	x[3]=HJhr_g[4](0, 3);
	y[0]=HJfl_g[4](1, 3);
	y[1]=HJfr_g[4](1, 3);
	y[2]=HJhl_g[4](1, 3);
	y[3]=HJhr_g[4](1, 3);

	for(int i=0; i<4; i++){
		r[i]=sqrt((x[i]-xc)*(x[i]-xc) + (y[i]-yc)*(y[i]-yc));
		phi[i]=atan2(y[i]-yc, x[i]-xc);
		phi[i]+=heading_omega*dt;
		x2[i]=r[i]*cos(phi[i])+xc;
		y2[i]=r[i]*sin(phi[i])+yc;
		xsum[i]+=x2[i]-x[i];
		ysum[i]+=y2[i]-y[i];
		if(legs_stance(i)==0){
			xsum[i]=0;
			ysum[i]=0;
		}
	}

	pFL(0)+=xsum[0];
	pFR(0)+=xsum[1];
	pHL(0)+=xsum[2];
	pHR(0)+=xsum[3];

	pFL(1)+=ysum[0];
	pFR(1)+=ysum[1];
	pHL(1)+=ysum[2];
	pHR(1)+=ysum[3];
}

/* Steering by rotating trajectories and without spine bending */
void
Controller :: headingController(double heading_angle){

	double err=-heading_angle+atan2(rotMatFgird(1,0), rotMatFgird(0,0));
	static double err_int=0;
	err_int+=err*dt;


	steeringSpineless(steeringSpinelessGains[0]*err + steeringSpinelessGains[1]*err_int);


}

void
Controller :: simpleTrajectoryFollower(){
	static int footstep_indx[4]={0};
	static Vector4d phaseTransitionStanceSwing, legs_stance_old=Vector4d::Zero();
	static MatrixXd tmpVec(3,4);
	static Vector3d gpsPosTmp;
	static Vector3d hgirdPosTmp;
	static Vector4d is_updated=Vector4d::Zero();
	// detect beginning of a swing phase
	for(int i=0; i<4; i++){
		if(legs_stance_old(i)==1 && legs_stance(i)==0)
			phaseTransitionStanceSwing(i)=1;
		else
			phaseTransitionStanceSwing(i)=0;
	}
	legs_stance_old=legs_stance;

	static ofstream footstepsLog("./data/footstepsLog.txt");
	static ofstream rotmats("./data/rotmatsLog.txt");
	gpsPosTmp=gpsPos;
	gpsPosTmp(2)=0;

	for(int i=0; i<4; i++){

		
		if(phaseTransitionStanceSwing(i)){
			is_updated(i)=1;
			gpsPosTmp=gpsPos;
			gpsPosTmp(2)=0;

			// get corresponding footstep (in global frame)
			tmpVec.block<3,1>(0,i) << footsteps[footstep_indx[i]][3*i], footsteps[footstep_indx[i]][3*i+1], footsteps[footstep_indx[i]][3*i+2];
			footstep_indx[i]+=1;

			// get it to the local frame of a corresponding girdle

			//front girdle (just substract gps and rotate)
			if(i==0){
				tmpVec.block<3,1>(0,i)=gpsPosTmp-tmpVec.block<3,1>(0,i);
				tmpVec.block<3,1>(0,i)=rotMatFgird*tmpVec.block<3,1>(0,i);
			}
			// get it in the girdle ref frame 
			if(i<2){
				tmpVec.block<3,1>(0,i)=-gpsPosTmp+tmpVec.block<3,1>(0,i); 
				//tmpVec.block<3,1>(0,i)=Fgird.block<3,3>(0,0)*rotMatFgird*tmpVec.block<3,1>(0,i);
			}
			else{				
				tmpVec.block<3,1>(0,i)=-gpsPosTmp+tmpVec.block<3,1>(0,i)-Fgird.block<3,3>(0,0)*rotMatFgird*Hgird0.block<3,1>(0,3);
				tmpVec.block<3,1>(0,i)=Fgird.block<3,3>(0,0)*rotMatFgird*tmpVec.block<3,1>(0,i);
			}

		}

		double trajGain=0.0;

		if(legs_stance(0)==0 && is_updated(0))
			trPointsFL.block<2,1>(0,0)+=(tmpVec.block<2,1>(0,0)-trPointsFL.block<2,1>(0,0))*trajGain;
		else if(legs_stance(1)==0 && is_updated(1))
			trPointsFR.block<2,1>(0,0)+=(tmpVec.block<2,1>(0,1)-trPointsFR.block<2,1>(0,0))*trajGain;
		else if(legs_stance(2)==0 && is_updated(2))
			trPointsHL.block<2,1>(0,0)+=(tmpVec.block<2,1>(0,2)-trPointsHL.block<2,1>(0,0))*trajGain;
		else if(legs_stance(3)==0 && is_updated(3))
			trPointsHR.block<2,1>(0,0)+=(tmpVec.block<2,1>(0,3)-trPointsHR.block<2,1>(0,0))*trajGain;

	}

	
	hgirdPosTmp=-gpsPosTmp+Fgird.block<3,3>(0,0)*rotMatFgird*Hgird0.block<3,1>(0,3);

	for(int i=0; i<2; i++){
		footstepsLog << tmpVec.block<3,1>(0,i).transpose() << "\t";
	}
	footstepsLog << gpsPosTmp.transpose() << "\t";
	footstepsLog << Hgird0.block<3,1>(0,3).transpose() << "\t";
	footstepsLog<<endl;
}


void
Controller :: GetCompass(double data[3])
{	
	compassData[0]=data[0];  
	compassData[1]=data[1];
	compassData[2]=data[2];

}



/* Updates phase of the legs */
void
Controller :: legPhaseDynamics(double dt)
{
	static bool initLegphase=0;
	if(!initLegphase){
	//	legPhase << 0, 0, 0, 0;
		initLegphase=true;
	}

	static MatrixXd dphi(4,1), phi(4,1);
	double omega=2*pi*freq_walk;

	if(state==WALKING){
		for(int i=0; i<4; i++){
			phi(i)=(legPhase(i)-Duty(floor(0.5*i)))  *2*pi;
			//===================== Akio feedback rule =====================

			dphi(i)=omega; //- akio_legs_sigma_v*feet_force.block<1,1>(2,i).norm()*cos(phi(i));

			phi(i)+=dphi(i)*dt;

	    	//==============================================================
	    	legPhase(i)=phi(i)/2/pi + Duty(floor(0.5*i));
	    	legPhase(i)=legPhase(i)>1?0:legPhase(i);
	    }
	}
}



/* Get spine angles from fgird and hgird oscillators */
void
Controller :: spineFromGirdleOsc(double dt){
	static bool init=false;
	static Vector2d girdleCPGphase=Vector2d::Zero(2,1);
	static Vector2d dgirdleCPGphase=Vector2d::Zero(2,1);
	double omega=2*pi*freq_walk;
	static double segment_phi;
/*
	if(!init){
		girdleCPGs
	}
	*/

	dgirdleCPGphase(0) = omega - akio_spine_sigma(0)* (feet_force.block<1,1>(2,0).norm() - feet_force.block<1,1>(2,1).norm()   )*cos(girdleCPGphase(0) + girdleOscOffset(0));
	dgirdleCPGphase(1) = omega - akio_spine_sigma(1)* (feet_force.block<1,1>(2,2).norm() - feet_force.block<1,1>(2,3).norm()   )*cos(girdleCPGphase(1) + girdleOscOffset(1));

	girdleCPGphase+=dt*dgirdleCPGphase;

	segment_phi= (girdleOscAmp(1)*sin(girdleCPGphase(1) + girdleOscOffset(1)) - girdleOscAmp(0)*sin(girdleCPGphase(0) + girdleOscOffset(0))) / spineGirdleCPGsWeights.sum();

	qsr.block<5,1>(1,0)=spineGirdleCPGsWeights*segment_phi;



}


/* Get spine angles from fgird and hgird oscillators */
void
Controller :: spineFromGirdle(double dt){
    static bool init=false;
    static Vector2d girdleCPGphase=Vector2d::Zero(2,1);
    static Vector2d dgirdleCPGphase=Vector2d::Zero(2,1);
    double omega=2*pi*freq_walk;
    static double segment_phi;
  

    // girdle phases
    dgirdleCPGphase(0) = omega;
    dgirdleCPGphase(1) = omega;

    girdleCPGphase+=dt*dgirdleCPGphase;

    // interpolate for the spine
    if(IS_PLEUROBOT){
        segment_phi= (girdleOscAmp(1)*sin(girdleCPGphase(1) + girdleOscOffset(1)) - girdleOscAmp(0)*sin(girdleCPGphase(0) + girdleOscOffset(0))) / spineGirdleCPGsWeights.sum();
        qsr.block<5,1>(1,0)=spineGirdleCPGsWeights*segment_phi;
    }
    else{
        segment_phi= (girdleOscAmp(1)*sin(girdleCPGphase(1) + girdleOscOffset(1)) - girdleOscAmp(0)*sin(girdleCPGphase(0) + girdleOscOffset(0))) / spineGirdleCPGsWeights.block<4,1>(1,0).sum();
        qsr.block<4,1>(2,0)=spineGirdleCPGsWeights.block<4,1>(1,0)*segment_phi;
    }
    
    //qsr.block<5,1>(1,0);

    // get angles for the tail

}


void
Controller :: gpsCheatedTrajectoryFollower(){
    static int footstep_indx[4]={0};
    
    static MatrixXd tmpFootstep(3,4);
    static Vector3d gpsPosTmp;
    static Vector3d hgirdPosTmp;
    static Vector4d is_updated=Vector4d::Zero();


    static ofstream footstepsLog("./data/footstepsLog.txt");
    static ofstream rotmats("./data/rotmatsLog.txt");
    gpsPosTmp=gpsPos;
    gpsPosTmp(2)=0;




    for(int i=0; i<4; i++){
        // get corresponding footstep (in global frame)
        tmpFootstep.block<3,1>(0,i) << footsteps[footstep_indx[i]][3*i], footsteps[footstep_indx[i]][3*i+1], footsteps[footstep_indx[i]][3*i+2];
        if(phaseTransitionStanceSwing(i)){
            is_updated(i)=1;
            footstep_indx[i]+=1;
        }

        tmpFootstep.block<3,1>(0,i)=rotMatFgird.inverse()*tmpFootstep.block<3,1>(0,i);   //rotMatFgird.inverse()*
        feetGPS.block<3,1>(0,i)=rotMatFgird.inverse()*feetGPS.block<3,1>(0,i);   //rotMatFgird.inverse()*
    }




    // estimate velocity  estVelFromGps  (already rotated)
    GetVelocityFromGPS();

    // capture take off points
    static MatrixXd feetTakeOffPosition=feetGPS;
    for(int i=0; i<4; i++){
        if(phaseTransitionStanceSwing(i)){
            feetTakeOffPosition=feetGPS;
        }
    }

    // get a duration of a swing phase 
    static Vector4d swingDuration;

    static Vector4d swingPhase;
    swingPhase(0)=(legPhase(0)-Duty(0))/(1-Duty(0));
    swingPhase(1)=(legPhase(1)-Duty(0))/(1-Duty(0));
    swingPhase(2)=(legPhase(2)-Duty(1))/(1-Duty(1));
    swingPhase(3)=(legPhase(3)-Duty(1))/(1-Duty(1));


    swingDuration(0)=(1-Duty(0))/freq_walk;  // * (1-swingPhase(0))
    swingDuration(1)=(1-Duty(0))/freq_walk;  // * (1-swingPhase(1))
    swingDuration(2)=(1-Duty(1))/freq_walk;  // * (1-swingPhase(2))
    swingDuration(3)=(1-Duty(1))/freq_walk;  // * (1-swingPhase(3))

    // estimate future point based on estimated velocity
    static MatrixXd feetTakeOffPositionAfterSwingPhase(3,4);
    for(int i=0; i<4; i++){
        //feetTakeOffPositionAfterSwingPhase(0,i)=feetTakeOffPosition(0,i) + estVelFromGps(0)*swingDuration(i);
        feetTakeOffPositionAfterSwingPhase(0,i)=feetTakeOffPosition(0,i) + desiredSpeed*0.1*swingDuration(i);
        feetTakeOffPositionAfterSwingPhase(1,i)=feetTakeOffPosition(1,i); 
    }


    static MatrixXd newLandingPoint(3,4), fixedTakeoffPoint(3,4);
    fixedTakeoffPoint.block<3,1>(0,0)=trPointsFL.block<3,1>(0,0);
    fixedTakeoffPoint.block<3,1>(0,1)=trPointsFR.block<3,1>(0,0);
    fixedTakeoffPoint.block<3,1>(0,2)=trPointsHL.block<3,1>(0,0);
    fixedTakeoffPoint.block<3,1>(0,3)=trPointsHR.block<3,1>(0,0);

    // get new landing points

    for(int i=0; i<4; i++){
        if(legs_stance(i)==0 && phaseTransitionStanceSwing(i)){
            newLandingPoint(0,i)=fixedTakeoffPoint(0,i) + (tmpFootstep(0,i)-feetTakeOffPositionAfterSwingPhase(0,i));
            newLandingPoint(1,i)=fixedTakeoffPoint(1,i) + (tmpFootstep(1,i)-feetTakeOffPositionAfterSwingPhase(1,i)); 
        }
        //cout<<tmpFootstep.block<2,1>(0,0).transpose()<<"\t\t"<<feetGPS.block<2,1>(0,0).transpose()<<endl;
        //cout<<trPointsFL<<endl;
    
    }
 //cout<<tmpFootstep(1,1)<<"\t"<<feetTakeOffPositionAfterSwingPhase(1,1)<<endl;

    static MatrixXd  fixedTakeoffPoint_old(3,4);
    
    if(legs_stance(0)==0 && is_updated(0)){
        trPointsFL(0,0)+=   (newLandingPoint(0,0) - fixedTakeoffPoint_old(0,0) )                  *trajFollowerParam(2)*dt;
        trPointsFL(1,0)+=   (newLandingPoint(1,0) - fixedTakeoffPoint_old(1,0) )                  *trajFollowerParam(3)*dt;
    }

    if(legs_stance(1)==0 && is_updated(1)){
        trPointsFR(0,0)+=     (newLandingPoint(0,1) - fixedTakeoffPoint_old(0,1) )                         *trajFollowerParam(2)*dt;
        trPointsFR(1,0)+=     (newLandingPoint(1,1) - fixedTakeoffPoint_old(1,1) )                         *trajFollowerParam(3)*dt;
    }

    if(legs_stance(2)==0 && is_updated(2)){
        trPointsHL(0,0)+=     (newLandingPoint(0,2) - fixedTakeoffPoint_old(0,2) )                         *trajFollowerParam(2)*dt;
        trPointsHL(1,0)+=     (newLandingPoint(1,2) - fixedTakeoffPoint_old(1,2) )                         *trajFollowerParam(3)*dt;
    }

    if(legs_stance(3)==0 && is_updated(3)){
        trPointsHR(0,0)+=      (newLandingPoint(0,3) - fixedTakeoffPoint_old(0,3) )                        *trajFollowerParam(2)*dt;
        trPointsHR(1,0)+=      (newLandingPoint(1,3) - fixedTakeoffPoint_old(1,3) )                        *trajFollowerParam(3)*dt;
    }


    fixedTakeoffPoint_old.block<3,1>(0,0)=trPointsFL.block<3,1>(0,0);
    fixedTakeoffPoint_old.block<3,1>(0,1)=trPointsFR.block<3,1>(0,0);
    fixedTakeoffPoint_old.block<3,1>(0,2)=trPointsHL.block<3,1>(0,0);
    fixedTakeoffPoint_old.block<3,1>(0,3)=trPointsHR.block<3,1>(0,0);
        

      /*  
    if(legs_stance(0)==0 && is_updated(0)){
        trPointsFL(0,0)+=(tmpFootstep(0,0)-feetGPS(0,0))*trajFollowerParam(0)*swingPhase(0)*dt;
        trPointsFL(1,0)+=(tmpFootstep(1,0)-feetGPS(1,0))*trajFollowerParam(1)*swingPhase(0)*dt;
        //cout<<tmpFootstep.block<2,1>(0,0).transpose()<<"\t\t"<<feetGPS.block<2,1>(0,0).transpose()<<endl;
    }

    if(legs_stance(1)==0 && is_updated(1)){
        trPointsFR(0,0)+=(tmpFootstep(0,1)-feetGPS(0,1))*trajFollowerParam(0)*swingPhase(1)*dt;
        trPointsFR(1,0)+=(tmpFootstep(1,1)-feetGPS(1,1))*trajFollowerParam(1)*swingPhase(1)*dt;
    }

    if(legs_stance(2)==0 && is_updated(2)){
        trPointsHL(0,0)+=(tmpFootstep(0,2)-feetGPS(0,2))*trajFollowerParam(0)*swingPhase(2)*dt;
        trPointsHL(1,0)+=(tmpFootstep(1,2)-feetGPS(1,2))*trajFollowerParam(1)*swingPhase(2)*dt;
    }

    if(legs_stance(3)==0 && is_updated(3)){
        trPointsHR(0,0)+=(tmpFootstep(0,3)-feetGPS(0,3))*trajFollowerParam(0)*swingPhase(3)*dt;
        trPointsHR(1,0)+=(tmpFootstep(1,3)-feetGPS(1,3))*trajFollowerParam(1)*swingPhase(3)*dt;
    }
    */
    
   // cout<<trPointsFL(0,0)<<endl;
}


void
Controller :: GetFeetGPS(double FL_feet_gpos[3], double FR_feet_gpos[3], double HL_feet_gpos[3], double HR_feet_gpos[3]){

	feetGPS(0, 0)=FL_feet_gpos[0];  feetGPS(1, 0)=-FL_feet_gpos[2];  feetGPS(2, 0)=FL_feet_gpos[1];
	feetGPS(0, 1)=FR_feet_gpos[0];  feetGPS(1, 1)=-FR_feet_gpos[2];  feetGPS(2, 1)=FR_feet_gpos[1];
	feetGPS(0, 2)=HL_feet_gpos[0];  feetGPS(1, 2)=-HL_feet_gpos[2];  feetGPS(2, 2)=HL_feet_gpos[1];
	feetGPS(0, 3)=HR_feet_gpos[0];  feetGPS(1, 3)=-HR_feet_gpos[2];  feetGPS(2, 3)=HR_feet_gpos[1];
}

void
Controller :: maintainDesiredSpeed(){
    static bool isInit=false;
    if(speedPriority){

        // get stance duration
        static Vector2d stanceDuration;
        stanceDuration(0)=Duty(0)/freq_walk;
        stanceDuration(1)=Duty(1)/freq_walk;

        // adjust end point of stance to maintain desired speed for all the legs
        static Vector4d tmpEndPoints;

        if(!isInit){
            tmpEndPoints << trPointsFL(0,2), trPointsFR(0,2), trPointsHL(0,2), trPointsHR(0,2);
            isInit=true;
        }

        if(phaseTransitionSwingStance(0)){
            tmpEndPoints(0)=trPointsFL(0,0)-desiredSpeed*stanceDuration(0);
        }
        if(phaseTransitionSwingStance(0)){
            tmpEndPoints(1)=trPointsFR(0,0)-desiredSpeed*stanceDuration(0);
        }
        if(phaseTransitionSwingStance(0)){
            tmpEndPoints(2)=trPointsHL(0,0)-desiredSpeed*stanceDuration(1);
        }
        if(phaseTransitionSwingStance(0)){
            tmpEndPoints(3)=trPointsHR(0,0)-desiredSpeed*stanceDuration(1);
        }
            
        trPointsFL(0,2)=tmpEndPoints(0);
        trPointsFR(0,2)=tmpEndPoints(1);
        trPointsHL(0,2)=tmpEndPoints(2);
        trPointsHR(0,2)=tmpEndPoints(3);
        //cout<<tmpEndPoints.transpose()<<endl;
    }

}
