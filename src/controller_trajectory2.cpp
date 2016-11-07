 #include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;

void
Controller :: girdleTrajectories(double v, double w)
{

    girdleTraj.block<6,1>(0,0)=girdleTraj.block<6,1>(0,1);


    girdleTraj(0,1)=girdleTraj(0,0)+v*cos(girdleTraj(2,0))*dt;
    girdleTraj(1,1)=girdleTraj(1,0)+v*sin(girdleTraj(2,0))*dt;
    girdleTraj(2,1)=girdleTraj(2,0)+w*dt;

    static double delta;
    static MatrixXd IGdist(2,1);
    IGdist=girdleTraj.block<2,1>(0,0)-girdleTraj.block<2,1>(3,0);
    delta=IGdist.norm()-IG;

    girdleTraj(5,0)=girdleTraj(5,1);

    girdleTraj(5,1)=atan2(girdleTraj(1,0)-girdleTraj(4,0), girdleTraj(0,0)-girdleTraj(3,0));

    girdleTraj(5,1)=girdleTraj(5,1)-floor(abs((girdleTraj(5,1)-girdleTraj(2,1)))/pi)*pi*(((girdleTraj(5,1)-girdleTraj(2,1)) > 0) - ((girdleTraj(5,1)-girdleTraj(2,1)) < 0));

    

    girdleTraj(3,1)=girdleTraj(3,0) + delta*cos(girdleTraj(5,1));
    girdleTraj(4,1)=girdleTraj(4,0) + delta*sin(girdleTraj(5,1));


    static ofstream girdleLog("./data/girdleLog.txt");
    girdleLog<<girdleTraj.block<6,1>(0,0).transpose()<<"\t"<<girdleTraj.block<6,1>(0,1).transpose()<<"\t"<<dt<<endl;

    Fgird_mean;


}

void
Controller :: legTrajFromGirdleTraj()
{

    

    Rotation2D<double> rotFgird(-(girdleTraj(2,1)-girdleTraj(2,0))), rotHgird(-(girdleTraj(5,1)-girdleTraj(5,0)));




    Transform<double,2,Affine> transformFgird, transformHgird;

    transformFgird=Rotation2D<double>(-(girdleTraj(2,0))) * 
                    Translation2d( -(girdleTraj(0,1)-girdleTraj(0,0)), -(girdleTraj(1,1)-girdleTraj(1,0)));

    transformHgird=Rotation2D<double>(-(girdleTraj(5,0))) * 
                    Translation2d( -(girdleTraj(3,1)-girdleTraj(3,0)), -(girdleTraj(2,1)-girdleTraj(2,0)));

                    Rotation2D<double>(-(girdleTraj(2,0)))*(girdleTraj.block<2,1>(0,1)-girdleTraj.block<2,1>(0,0));

    // translation
    static MatrixXd deltaFgird(2,1), deltaHgird(2,1);
    deltaFgird=Rotation2D<double>(-(girdleTraj(2,0)))*(girdleTraj.block<2,1>(0,1)-girdleTraj.block<2,1>(0,0));
    deltaHgird=Rotation2D<double>(-(girdleTraj(5,0)))*(girdleTraj.block<2,1>(3,1)-girdleTraj.block<2,1>(3,0));

    // rotation




    static Vector3d relativePosFL, relativePosFR, relativePosHL, relativePosHR;

    relativePosFL=Fgird.block<3, 3>(0,0).inverse()* (HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3));
    relativePosFR=Fgird.block<3, 3>(0,0).inverse()* (HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3));
    relativePosHL=Hgird.block<3, 3>(0,0).inverse()* (HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3));
    relativePosHR=Hgird.block<3, 3>(0,0).inverse()* (HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3));



/*
    legStanceTraj.block<2,1>(0,0)=-deltaFgird;
    legStanceTraj.block<2,1>(0,1)=-deltaFgird; //legStanceTraj(1,1)*=-1;
    legStanceTraj.block<2,1>(0,2)=-deltaHgird;
    legStanceTraj.block<2,1>(0,3)=-deltaHgird; //legStanceTraj(1,3)*=-1;
*/    

    /*
    legStanceTraj.block<2,1>(0,0)=transformFgird*relativePosFL.block<2,1>(0,0)-relativePosFL.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,1)=transformFgird*relativePosFR.block<2,1>(0,0)-relativePosFR.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,2)=transformHgird*relativePosHL.block<2,1>(0,0)-relativePosHL.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,3)=transformHgird*relativePosHR.block<2,1>(0,0)-relativePosHR.block<2,1>(0,0);
    */

    legStanceTraj.block<2,1>(0,0)= rotFgird*(relativePosFL.block<2,1>(0,0)-deltaFgird)   -relativePosFL.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,1)= rotFgird*(relativePosFR.block<2,1>(0,0)-deltaFgird)   -relativePosFR.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,2)= rotHgird*(relativePosHL.block<2,1>(0,0)-deltaHgird)   -relativePosHL.block<2,1>(0,0);
    legStanceTraj.block<2,1>(0,3)= rotHgird*(relativePosHR.block<2,1>(0,0)-deltaHgird)   -relativePosHR.block<2,1>(0,0);
    

    //legStanceTraj(1,1)*=-1;
    //legStanceTraj(1,3)*=-1;


}


MatrixXd
Controller :: legSwingTraj(MatrixXd initPoint, MatrixXd currPoint, MatrixXd trPoints, double phase, double trAngles[4], double bezierParam[2])
{
    static MatrixXd B1(3, 4), B2(3, 4);
    static double bnorm, tB;
    static MatrixXd Bcurve(3, 1);
    Transform<double,3,Affine>  tempRot;

    B1.block<3, 1>(0,0)=initPoint;
    //B1.block<3, 1>(0,0)=trPoints.block<3, 1>(0, 2);
    B1.block<3, 1>(0,3)=trPoints.block<3, 1>(0, 1);
    bnorm=(B1.block<3, 1>(0,0)-B1.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(trAngles[2], Vector3d::UnitX())*AngleAxisd(trAngles[0], Vector3d::UnitY());
    B1.block<3, 1>(0,1) = B1.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParam[1]*bnorm);                  
    B1.block<3, 1>(0,2) << B1(0,3) - bezierParam[0]*(B1(0, 3)-B1(0,0)), 
                            B1(1,3), 
                            B1(2,3);
 
    B2.block<3, 1>(0,0)=trPoints.block<3, 1>(0, 1);
    B2.block<3, 1>(0,3)=trPoints.block<3, 1>(0, 0);
    bnorm=(B2.block<3, 1>(0,0)-B2.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(trAngles[3], Vector3d::UnitX())*AngleAxisd(trAngles[1], Vector3d::UnitY());

    B2.block<3, 1>(0,2) = B2.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParam[1]*bnorm);                        
    B2.block<3, 1>(0,1) << B2(0,0) + bezierParam[0]*(B2(0, 3)-B2(0,0)), 
                            B2(1,0), 
                            B2(2,0);


    if(phase<0.5){
        tB=phase*2;

        Bcurve= pow(1-tB, 3)*B1.block<3, 1>(0,0) +
                3*pow(1-tB, 2)*tB* B1.block<3, 1>(0,1)  +
                3*pow(tB, 2)*(1-tB)* B1.block<3, 1>(0,2)  +
                pow(tB, 3)*B1.block<3, 1>(0,3); 
    }
    else{
        tB=phase*2-1;

        Bcurve= pow(1-tB, 3)*B2.block<3, 1>(0,0) +
                3*pow(1-tB, 2)*tB* B2.block<3, 1>(0,1)  +
                3*pow(tB, 2)*(1-tB)* B2.block<3, 1>(0,2)  +
                pow(tB, 3)*B2.block<3, 1>(0,3);
    }




    // HALF SINE
    Bcurve(0)=initPoint(0)*(1-phase)+trPoints(0, 0)*phase;
    Bcurve(1)=initPoint(1)*(1-phase)+trPoints(1, 0)*phase;
    Bcurve(2)=(trPoints(2, 1) - trPoints(2, 0))*sin(pi*phase) + initPoint(2)*(1-phase)+trPoints(2, 0)*phase;

    static MatrixXd newPoint(3,1);
    newPoint=currPoint + (Bcurve - currPoint);
    return newPoint;
}

void
Controller :: walkingTrajectories2()
{


    girdleTrajectories(0.25, -0.5); 
    


    legTrajFromGirdleTraj();



    legPhaseDynamics(dt);




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
        //stanceStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
        stanceStartFL=HJfl[4].block<3, 1>(0, 3);
    }
    if(legs_stance(1)==0){
        //stanceStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
        stanceStartFR=HJfr[4].block<3, 1>(0, 3);
    }
    if(legs_stance(2)==0){
        //stanceStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
        stanceStartHL=HJhl[4].block<3, 1>(0, 3);
    }
    if(legs_stance(3)==0){
        //stanceStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
        stanceStartHR=HJhr[4].block<3, 1>(0, 3);
    }


    // ########################## UPDATE SWING START ################################
    if(legs_stance(0)==1){
        swingStartFL=HJfl_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
        //swingStartFL=HJfl[4].block<3, 1>(0, 3);
    }
    if(legs_stance(1)==1){
        swingStartFR=HJfr_g[4].block<3, 1>(0, 3)-Fgird.block<3, 1>(0, 3);
        //swingStartFR=HJfr[4].block<3, 1>(0, 3);
    }
    if(legs_stance(2)==1){
        swingStartHL=HJhl_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
        //swingStartHL=HJhl[4].block<3, 1>(0, 3);
    }
    if(legs_stance(3)==1){
        swingStartHR=HJhr_g[4].block<3, 1>(0, 3)-Hgird.block<3, 1>(0, 3);
        //swingStartHR=HJhr[4].block<3, 1>(0, 3);
    }





    // ########################## CALCULATE NEW REFERENCE POINTS ################################
    if(state==WALKING){
        // FL
        if(legPhase(0)<Duty(0)){
            stancePhase(0)=legPhase(0)/Duty(0);
            rFL.block<2,1>(0,0) = rFL.block<2,1>(0,0) + legStanceTraj.block<2,1>(0,0);
            rFL(2)=trPointsFL(2,0);
            legs_stance(0)=1;
        }
        else{
            swingPhase(0)=(legPhase(0)-Duty(0))/(1-Duty(0));
            rFL=legSwingTraj(swingStartFL, rFL, trPointsFL, swingPhase(0), trAnglesF0, bezierParamF0);
            legs_stance(0)=0;
        }

        // FR
        if(legPhase(1)<Duty(0)){
            stancePhase(1)=legPhase(1)/Duty(0);
            rFR.block<2,1>(0,0) = rFR.block<2,1>(0,0) + legStanceTraj.block<2,1>(0,1);
            rFR(2)=trPointsFR(2,0);
            legs_stance(1)=1;
        }
        else{
            swingPhase(1)=(legPhase(1)-Duty(0))/(1-Duty(0));
            rFR=legSwingTraj(swingStartFR, rFR, trPointsFR, swingPhase(1), trAnglesF0, bezierParamF0);
            legs_stance(1)=0;
        }

        // HL
        if(legPhase(2)<Duty(1)){
            stancePhase(2)=legPhase(2)/Duty(1);
            rHL.block<2,1>(0,0) = rHL.block<2,1>(0,0) + legStanceTraj.block<2,1>(0,2);
            rHL(2)=trPointsHL(2,0);
            legs_stance(2)=1;
        }
        else{
            swingPhase(2)=(legPhase(2)-Duty(1))/(1-Duty(1));
            rHL=legSwingTraj(swingStartHL, rHL, trPointsHL, swingPhase(2), trAnglesH0, bezierParamH0);
            legs_stance(2)=0;
        }

        // HR
        if(legPhase(3)<Duty(1)){
            stancePhase(3)=legPhase(3)/Duty(1);
            rHR.block<2,1>(0,0) = rHR.block<2,1>(0,0) + legStanceTraj.block<2,1>(0,3);
            rHR(2)=trPointsHR(2,0);
            legs_stance(3)=1;
        }
        else{
            swingPhase(3)=(legPhase(3)-Duty(1))/(1-Duty(1));
            rHR=legSwingTraj(swingStartHR, rHR, trPointsHR, swingPhase(3), trAnglesH0, bezierParamH0);
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
