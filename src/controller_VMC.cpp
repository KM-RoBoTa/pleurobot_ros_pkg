#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;


/* Calculates angles from VMC */
void
Controller :: VmcController(){

    static MatrixXd JFL(3,4), JFR(3,4), JHL(3,4), JHR(3,4);
    static MatrixXd vmc_FFL(3,1), vmc_FFR(3,1), vmc_FHL(3,1), vmc_FHR(3,1);
    static MatrixXd vmc_TFL(4,1), vmc_TFR(4,1), vmc_THL(4,1), vmc_THR(4,1), vmc_TS(11,1);
    static MatrixXd fbck_position_old(27,1), fbck_position_int(27,1);
    fbck_position_int+=fbck_position*dt;



    // ========================== GRANNY WALKER ================================================//
    static MatrixXd GW_Z(4,1), GW_Zref(4,1);
    static MatrixXd GW_pos(3,4), GW_posRef(3,4), GW_posErr(3,4), GW_posErrOld=MatrixXd::Zero(3,4);

    // get ground plane from legs
    static MatrixXd feetCoordinates(3, 4), tmp3x3, planeParam(3,1);
    feetCoordinates.block<2,4>(0,0)=feet_position_global.block<2,4>(0,0);
    feetCoordinates.block<1,4>(2,0)=MatrixXd::Ones(1,4);

    tmp3x3=feetCoordinates*feetCoordinates.transpose();
    planeParam=tmp3x3.inverse()*feetCoordinates*feet_position_global.block<1,4>(2,0).transpose();



    GW_pos.block<2,4>(0,0) = feet_position_global.block<2,4>(0,0);
    GW_pos.block<1,4>(2,0) = planeParam.transpose()*(MatrixXd(3,4) << 0, 0, GW_x_offset, GW_x_offset, GW_y_offset, -GW_y_offset, GW_y_offset, -GW_y_offset, 1, 1, 1, 1).finished();

    GW_posRef.block<1,4>(0,0) << 0, 0, GW_x_offset, GW_x_offset;
    GW_posRef.block<1,4>(1,0) << GW_y_offset, -GW_y_offset, GW_y_offset, -GW_y_offset;
    GW_posRef.block<1,4>(2,0) << GW_Zref_initial, GW_Zref_initial, GW_Zref_initial, GW_Zref_initial;


    // get forces from springs acting on the virtual plane on the anchor points
    static MatrixXd GW_forces(3,4);

    GW_posErr=GW_posRef - GW_pos;

    GW_forces.block<2, 4>(0,0) = pt1_vec(GW_spring_stiffness_xy * GW_posErr.block<2,4>(0,0) + GW_spring_damping_xy * (GW_posErr.block<2,4>(0,0)  - GW_posErrOld.block<2,4>(0,0) ) / dt, GW_forces.block<2, 4>(0,0), GW_Tfilt, dt);
    GW_forces.block<1, 4>(2,0) = pt1_vec(GW_spring_stiffness_z * GW_posErr.block<1,4>(2,0) + GW_spring_damping_z * (GW_posErr.block<1,4>(2,0)  - GW_posErrOld.block<1,4>(2,0) ) / dt, GW_forces.block<1, 4>(2,0), GW_Tfilt, dt);





    GW_posErrOld=GW_posErr;

    //cout<<GW_posErr <<endl<<endl<<GW_forces <<endl<<endl<<endl;
    // ========================== DISTRIBUTE FORCES ================================================//
    vmc_FFL = GW_forces.block<3,1>(0,0);
    vmc_FFR = GW_forces.block<3,1>(0,1);
    vmc_FHL = GW_forces.block<3,1>(0,2);
    vmc_FHR = GW_forces.block<3,1>(0,3);

    vmc_FFL(0)=vmc_FFL(0);
    vmc_FFL(1)=vmc_FFL(1);
    vmc_FFL(2)=-vmc_FFL(2);

    vmc_FFR(0)=vmc_FFR(0);
    vmc_FFR(1)=-vmc_FFR(1);
    vmc_FFR(2)=-vmc_FFR(2);

    vmc_FHL(0)=vmc_FHL(0);
    vmc_FHL(1)=vmc_FHL(1);
    vmc_FHL(2)=-vmc_FHL(2);

    vmc_FHR(0)=vmc_FHR(0);
    vmc_FHR(1)=-vmc_FHR(1);
    vmc_FHR(2)=-vmc_FHR(2);

    // ========================== LEGS CONTROLLER ================================================//














    // ========================== SPINE CONTROLLER ================================================//
    vmc_TS=pt1_vec(-10*fbck_position.block<11,1>(0,0) - 5*(fbck_position.block<11,1>(0,0)-fbck_position_old.block<11,1>(0,0))/dt - 30*fbck_position_int.block<11,1>(0,0), vmc_TS, 0.3, dt);




    // ========================== MAP FORCES TO TORQUES ================================================//

    JFL=Jacob(fbck_position.block<4,1>(11, 0), 0);
    JFR=Jacob(fbck_position.block<4,1>(15, 0), 0);
    JHL=Jacob(fbck_position.block<4,1>(19, 0), 1);
    JHR=Jacob(fbck_position.block<4,1>(23, 0), 1);

    vmc_TFL=JFL.transpose()*vmc_FFL;
    vmc_TFR=JFR.transpose()*vmc_FFR;
    vmc_THL=JHL.transpose()*vmc_FHL;
    vmc_THR=JHR.transpose()*vmc_FHR;





    // ========================== ROTATIONAL SPRING for ROLL ================================================//
    static double rollErrFL, rollErrFR, rollErrHL, rollErrHR, rollErrOldFL=0, rollErrOldFR=0, rollErrOldHL=0, rollErrOldHR=0;
    static double rollTorqueFL, rollTorqueFR, rollTorqueHL, rollTorqueHR;

    rollErrFL = VMC_roll_ref_front - fbck_position(13);
    rollErrFR = VMC_roll_ref_front - fbck_position(17);
    rollErrHL = VMC_roll_ref_hind - fbck_position(21);
    rollErrHR = VMC_roll_ref_hind - fbck_position(25);

    rollTorqueFL=pt1(VMC_roll_spring_stiffness_front * rollErrFL +  VMC_roll_damping_front * (rollErrFL - rollErrOldFL)/dt, rollTorqueFL, GW_Tfilt, dt);
    rollTorqueFR=pt1(VMC_roll_spring_stiffness_front * rollErrFR +  VMC_roll_damping_front * (rollErrFR - rollErrOldFR)/dt, rollTorqueFR, GW_Tfilt, dt);
    rollTorqueHL=pt1(VMC_roll_spring_stiffness_hind * rollErrHL +  VMC_roll_damping_hind * (rollErrHL - rollErrOldHL)/dt, rollTorqueHL, GW_Tfilt, dt);
    rollTorqueHR=pt1(VMC_roll_spring_stiffness_hind * rollErrHR +  VMC_roll_damping_hind * (rollErrHR - rollErrOldHR)/dt, rollTorqueHR, GW_Tfilt, dt);


    torques.block<4,1>(11,0)=vmc_TFL;
    torques.block<4,1>(15,0)=vmc_TFR;
    torques.block<4,1>(19,0)=vmc_THL;
    torques.block<4,1>(23,0)=vmc_THR;
    torques.block<11,1>(0,0)=vmc_TS;


    torques(13)+=rollTorqueFL;
    torques(17)+=rollTorqueFR;
    torques(21)+=rollTorqueHL;
    torques(25)+=rollTorqueHR;

   // cout<<fbck_position(21)<<endl<<endl;



    rollErrOldFL=rollErrFL;
    rollErrOldFR=rollErrFR;
    rollErrOldHL=rollErrHL;
    rollErrOldHR=rollErrHR;

    fbck_position_old=fbck_position;

}
