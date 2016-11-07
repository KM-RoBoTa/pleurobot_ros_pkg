#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;

/* Returns transformation matrix for given DH matrix */
void
Controller :: Hmat(MatrixXd *H, MatrixXd DH, Vector4d q)
{
    H->block<4,4>(0,0)=MatrixXd::Identity(4,4);
    static MatrixXd T(4,4);
    for(int k=0; k<DH.rows(); k++) {

        T<< cos(q(k)),       -cos(DH(k, 3))*sin(q(k)),        sin(DH(k, 3))*sin(q(k)),         DH(k, 2)*cos(q(k)),
        sin(q(k)),       cos(DH(k, 3))*cos(q(k)),         -sin(DH(k, 3))*cos(q(k)),        DH(k, 2)*sin(q(k)),
        0,               sin(DH(k, 3)),                   cos(DH(k, 3)),                   DH(k, 1),
        0,               0,                               0,                               1;



        if (k!=0)
            H->block<4,4>(4*(k+1),0)=H->block<4,4>(4*k,0)*T;
        else
            H->block<4,4>(4,0)=T;

    }

    return;
}

/* Calculates forward kinematics for a single leg */
MatrixXd
Controller :: legKinematics(Vector4d q, int leg)
{

    static vector<Matrix4d> H(5);

    if(leg==0){     //FL
        H[0] <<  cos(q(0)), -sin(q(0)), 0, FL_kin(0,0),
                sin(q(0)), cos(q(0)), 0, FL_kin(0,1),
                0, 0, 1, FL_kin(0,2),
                0, 0, 0, 1;
        H[1] <<  1, 0, 0, FL_kin(1,0),
                0, cos(q(1)), -sin(q(1)), FL_kin(1,1),
                0, sin(q(1)), cos(q(1)), FL_kin(1,2),
                0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), FL_kin(2,0),
                0, 1, 0, FL_kin(2,1),
                -sin(q(2)), 0, cos(q(2)), FL_kin(2,2),
                0, 0, 0, 1;
        H[3] <<  cos(q(3)), -sin(q(3)), 0, FL_kin(3,0),
                sin(q(3)), cos(q(3)), 0, FL_kin(3,1),
                0, 0, 1, FL_kin(3,2),
                0, 0, 0, 1;
        H[4] <<  1, 0, 0, FL_kin(4,0),
                0, 1, 0, FL_kin(4,1),
                0, 0, 1, FL_kin(4,2),
                0, 0, 0, 1;
    }
    else if (leg==1){       //FR
        H[0] <<  cos(q(0)), -sin(q(0)), 0, FR_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, FR_kin(0,1),
                    0, 0, 1, FR_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, FR_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), FR_kin(1,1),
                    0, sin(q(1)), cos(q(1)), FR_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), FR_kin(2,0),
                    0, 1, 0, FR_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), FR_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  cos(q(3)), -sin(q(3)), 0, FR_kin(3,0),
                    sin(q(3)), cos(q(3)), 0, FR_kin(3,1),
                    0, 0, 1, FR_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, FR_kin(4,0),
                    0, 1, 0, FR_kin(4,1),
                    0, 0, 1, FR_kin(4,2),
                    0, 0, 0, 1;
    }
    else if (leg==2){       //HL
        H[0] <<  cos(q(0)), -sin(q(0)), 0, HL_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, HL_kin(0,1),
                    0, 0, 1, HL_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, HL_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), HL_kin(1,1),
                    0, sin(q(1)), cos(q(1)), HL_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), HL_kin(2,0),
                    0, 1, 0, HL_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), HL_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  1, 0, 0, HL_kin(3,0),
                    0, cos(q(3)), -sin(q(3)), HL_kin(3,1),
                    0, sin(q(3)), cos(q(3)), HL_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, HL_kin(4,0),
                    0, 1, 0, HL_kin(4,1),
                    0, 0, 1, HL_kin(4,2),
                    0, 0, 0, 1;
    }
    else if (leg==3){       //HR
        H[0] <<  cos(q(0)), -sin(q(0)), 0, HR_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, HR_kin(0,1),
                    0, 0, 1, HR_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, HR_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), HR_kin(1,1),
                    0, sin(q(1)), cos(q(1)), HR_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), HR_kin(2,0),
                    0, 1, 0, HR_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), HR_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  1, 0, 0, HR_kin(3,0),
                    0, cos(q(3)), -sin(q(3)), HR_kin(3,1),
                    0, sin(q(3)), cos(q(3)), HR_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, HR_kin(4,0),
                    0, 1, 0, HR_kin(4,1),
                    0, 0, 1, HR_kin(4,2),
                    0, 0, 0, 1;
    }


    return H[0]*H[1]*H[2]*H[3]*H[4];
}

/* Returns jacobian matrix of one leg */
Matrix<double, 3, 4>
Controller :: Jacob(Vector4d q, int leg)
{
    static MatrixXd J(3, 4);
    static double S1, S2, S3, S4, C1, C2, C3, C4;
    S1 = sin(q(0));
    S2 = sin(q(1));
    S3 = sin(q(2));
    S4 = sin(q(3));
    C1 = cos(q(0));
    C2 = cos(q(1));
    C3 = cos(q(2));
    C4 = cos(q(3));

    if(IS_PLEUROBOT){
        if(leg==0){     //FL
            J(0) = - 0.143*C1*C2 - 0.1*C4*(C3*S1 + C1*S2*S3) - 0.1*C1*C2*S4;
            J(1) = 0.1*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.143*C2*S1 - 0.1*C2*S1*S4;
            J(2) = 0;
            J(3) = 0.001*S1*(143.0*S2 + 100.0*S2*S4 - 100.0*C2*C4*S3);
            J(4) = -0.001*C1*(143.0*S2 + 100.0*S2*S4 - 100.0*C2*C4*S3);
            J(5) = 0.143*C2 + 0.1*C2*S4 + 0.1*C4*S2*S3;
            J(6) = -0.1*C4*(C1*S3 + C3*S1*S2);
            J(7) = -0.1*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(8) = -0.1*C2*C3*C4;
            J(9) = - 0.1*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.1*C2*C4*S1;
            J(10) = 0.1*C1*C2*C4 - 0.1*S4*(C3*S1 + C1*S2*S3);
            J(11) = 0.1*C4*S2 + 0.1*C2*S3*S4;
        }
        else if (leg==1){       //FR
            J(0) = 0.143*C1*C2 - 0.1*C4*(C3*S1 + C1*S2*S3) - 0.1*C1*C2*S4;
            J(1) = 0.143*C2*S1 + 0.1*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.1*C2*S1*S4;
            J(2) = 0;
            J(3) = -0.001*S1*(143.0*S2 - 100.0*S2*S4 + 100.0*C2*C4*S3);
            J(4) = 0.001*C1*(143.0*S2 - 100.0*S2*S4 + 100.0*C2*C4*S3);
            J(5) = 0.1*C2*S4 - 0.143*C2 + 0.1*C4*S2*S3;
            J(6) = -0.1*C4*(C1*S3 + C3*S1*S2);
            J(7) = -0.1*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(8) = -0.1*C2*C3*C4;
            J(9) = - 0.1*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.1*C2*C4*S1;
            J(10) = 0.1*C1*C2*C4 - 0.1*S4*(C3*S1 + C1*S2*S3);
            J(11) = 0.1*C4*S2 + 0.1*C2*S3*S4;
        }
        else if (leg==2){       //HL
            J(0) = - 0.118*C1*C2 - 0.1*S4*(S1*S3 - 1.0*C1*C3*S2) - 0.1*C1*C2*C4;
            J(1) = 0.1*S4*(C1*S3 + C3*S1*S2) - 0.118*C2*S1 - 0.1*C2*C4*S1;
            J(2) = 0;
            J(3) = 0.002*S1*(59.0*S2 + 50.0*C4*S2 + 50.0*C2*C3*S4);
            J(4) = -0.002*C1*(59.0*S2 + 50.0*C4*S2 + 50.0*C2*C3*S4);
            J(5) = 0.118*C2 + 0.1*C2*C4 - 0.1*C3*S2*S4;
            J(6) = 0.1*S4*(C1*C3 - 1.0*S1*S2*S3);
            J(7) = 0.1*S4*(C3*S1 + C1*S2*S3);
            J(8) = -0.1*C2*S3*S4;
            J(9) = 0.1*C4*(C1*S3 + C3*S1*S2) + 0.1*C2*S1*S4;
            J(10) = 0.1*C4*(S1*S3 - 1.0*C1*C3*S2) - 0.1*C1*C2*S4;
            J(11) = 0.1*C2*C3*C4 - 0.1*S2*S4;
        }
        else if (leg==3){       //HR
            J(0) = 0.118*C1*C2 + 0.1*S4*(S1*S3 - 1.0*C1*C3*S2) + 0.1*C1*C2*C4;
            J(1) = 0.118*C2*S1 - 0.1*S4*(C1*S3 + C3*S1*S2) + 0.1*C2*C4*S1;
            J(2) = 0;
            J(3) = -0.002*S1*(59.0*S2 + 50.0*C4*S2 + 50.0*C2*C3*S4);
            J(4) = 0.002*C1*(59.0*S2 + 50.0*C4*S2 + 50.0*C2*C3*S4);
            J(5) = 0.1*C3*S2*S4 - 0.1*C2*C4 - 0.118*C2;
            J(6) = -0.1*S4*(C1*C3 - 1.0*S1*S2*S3);
            J(7) = -0.1*S4*(C3*S1 + C1*S2*S3);
            J(8) = 0.1*C2*S3*S4;
            J(9) = - 0.1*C4*(C1*S3 + C3*S1*S2) - 0.1*C2*S1*S4;
            J(10) = 0.1*C1*C2*S4 - 0.1*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(11) = 0.1*S2*S4 - 0.1*C2*C3*C4;
        }
    }
    else{
        if(leg==0){     //FL
            J(0) = - 0.1168*C1*C2 - 0.1048*C4*(C3*S1 + C1*S2*S3) - 0.1048*C1*C2*S4;
            J(1) = 0.1048*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.1168*C2*S1 - 0.1048*C2*S1*S4;
            J(2) = 0;
            J(3) = 0.0008*S1*(146.0*S2 + 131.0*S2*S4 - 131.0*C2*C4*S3);
            J(4) = -0.0008*C1*(146.0*S2 + 131.0*S2*S4 - 131.0*C2*C4*S3);
            J(5) = 0.1168*C2 + 0.1048*C2*S4 + 0.1048*C4*S2*S3;
            J(6) = -0.1048*C4*(C1*S3 + C3*S1*S2);
            J(7) = -0.1048*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(8) = -0.1048*C2*C3*C4;
            J(9) = - 0.1048*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.1048*C2*C4*S1;
            J(10) = 0.1048*C1*C2*C4 - 0.1048*S4*(C3*S1 + C1*S2*S3);
            J(11) = 0.1048*C4*S2 + 0.1048*C2*S3*S4;
        }
        else if (leg==1){       //FR
            J(0) = 0.1168*C1*C2 - 0.1048*C4*(C3*S1 + C1*S2*S3) - 0.1048*C1*C2*S4;
            J(1) = 0.1168*C2*S1 + 0.1048*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.1048*C2*S1*S4;
            J(2) = 0;
            J(3) = -0.0008*S1*(146.0*S2 - 131.0*S2*S4 + 131.0*C2*C4*S3);
            J(4) = 0.0008*C1*(146.0*S2 - 131.0*S2*S4 + 131.0*C2*C4*S3);
            J(5) = 0.1048*C2*S4 - 0.1168*C2 + 0.1048*C4*S2*S3;
            J(6) = -0.1048*C4*(C1*S3 + C3*S1*S2);
            J(7) = -0.1048*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(8) = -0.1048*C2*C3*C4;
            J(9) = - 0.1048*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.1048*C2*C4*S1;
            J(10) = 0.1048*C1*C2*C4 - 0.1048*S4*(C3*S1 + C1*S2*S3);
            J(11) = 0.1048*C4*S2 + 0.1048*C2*S3*S4;
        }
        else if (leg==2){       //HL
            J(0) = - 0.1272*C1*C2 - 0.154*S4*(S1*S3 - 1.0*C1*C3*S2) - 0.154*C1*C2*C4;
            J(1) = 0.154*S4*(C1*S3 + C3*S1*S2) - 0.1272*C2*S1 - 0.154*C2*C4*S1;
            J(2) = 0;
            J(3) = 0.0004*S1*(318.0*S2 + 385.0*C4*S2 + 385.0*C2*C3*S4);
            J(4) = -0.0004*C1*(318.0*S2 + 385.0*C4*S2 + 385.0*C2*C3*S4);
            J(5) = 0.1272*C2 + 0.154*C2*C4 - 0.154*C3*S2*S4;
            J(6) = 0.154*S4*(C1*C3 - 1.0*S1*S2*S3);
            J(7) = 0.154*S4*(C3*S1 + C1*S2*S3);
            J(8) = -0.154*C2*S3*S4;
            J(9) = 0.154*C4*(C1*S3 + C3*S1*S2) + 0.154*C2*S1*S4;
            J(10) = 0.154*C4*(S1*S3 - 1.0*C1*C3*S2) - 0.154*C1*C2*S4;
            J(11) = 0.154*C2*C3*C4 - 0.154*S2*S4;
        }
        else if (leg==3){       //HR
            J(0) = 0.1272*C1*C2 + 0.154*S4*(S1*S3 - 1.0*C1*C3*S2) + 0.154*C1*C2*C4;
            J(1) = 0.1272*C2*S1 - 0.154*S4*(C1*S3 + C3*S1*S2) + 0.154*C2*C4*S1;
            J(2) = 0;
            J(3) = -0.0004*S1*(318.0*S2 + 385.0*C4*S2 + 385.0*C2*C3*S4);
            J(4) = 0.0004*C1*(318.0*S2 + 385.0*C4*S2 + 385.0*C2*C3*S4);
            J(5) = 0.154*C3*S2*S4 - 0.154*C2*C4 - 0.1272*C2;
            J(6) = -0.154*S4*(C1*C3 - 1.0*S1*S2*S3);
            J(7) = -0.154*S4*(C3*S1 + C1*S2*S3);
            J(8) = 0.154*C2*S3*S4;
            J(9) = - 0.154*C4*(C1*S3 + C3*S1*S2) - 0.154*C2*S1*S4;
            J(10) = 0.154*C1*C2*S4 - 0.154*C4*(S1*S3 - 1.0*C1*C3*S2);
            J(11) = 0.154*S2*S4 - 0.154*C2*C3*C4;
        }  
    }

    return J;
}

/* Solves inverse kinematics for one leg by using Cyclic Coordinate Descend method */
Vector4d
Controller :: iKinCCD(Matrix4d DH, Vector3d p, Vector4d q0, double tol, int iter, double *w)
{

    MatrixXd H(20, 4);
    Vector3d ax, Pg1, Pt1, Pg, Pt, tmp, r;
    double theta;
    Vector4d q;
    q=q0;
    for(int i=0; i<iter; i++) {
        for(int j=3; j>=0; j--) {

            Hmat(&H, DH, q);
            ax=H.block<3,1>(j*4, 2);

            Pg1=p-H.block<3,1>(j*4, 3);
            Pt1=H.block<3,1>(16, 3)-H.block<3,1>(j*4, 3);

            tmp=p-H.block<3,1>(16, 3);
            if(tmp.norm()<tol)
                return q;

            Pg=Pg1-Pg1.dot(ax)*ax;
            Pt=Pt1-Pt1.dot(ax)*ax;

            theta=Pg.dot(Pt)/Pt.norm()/Pg.norm();
            if(theta<-1)
                theta=-1;
            else if (theta>1)
                theta=1;

            theta=acos(theta);
            theta*=w[j];
            r<< Pg(1)*Pt(2)-Pg(2)*Pt(1),
            -Pg(0)*Pt(2)+Pg(2)*Pt(0),
            Pg(0)*Pt(1)-Pg(1)*Pt(0);

            if(r.dot(ax)>0)
                q(j)=q(j)-theta;
            else
                q(j)=q(j)+theta;
        }

    }
    return q;
}

/* Solves inverse kinematics for one leg by using Iterative Damped Inverse Jacobian method */
Vector4d
Controller :: iKinIDLS(Matrix4d DH, Vector3d pref, Vector4d qref, Vector4d q0, Matrix4d Cinv, Matrix4d M, double max_dist, int leg, int maxIter, double tol)
{


    MatrixXd J(3,4), tmp(3,3);
    MatrixXd H(20, 4);
    Vector4d dqref, q, dq;
    Vector3d dp, p0;
    double norm_dp;

    // get current position
    for(int i=0; i<maxIter; i++){

        Hmat(&H, DH, q0);
        p0=H.block<3, 1>(16, 3);
        dp=pref-p0;
        norm_dp=sqrt(dp(0)*dp(0)+dp(1)*dp(1)+dp(2)*dp(2));
        if(norm_dp<tol && i>0){
            break;
        }

        J=Jacob(q0, leg);

        dqref=qref-q0;




        if(norm_dp>max_dist){
            dp=max_dist/norm_dp*dp;
        }

        tmp=Matrix3d::Identity() + J * Cinv * J.transpose();
        dq=(Cinv - Cinv * J.transpose() * tmp.inverse() * J * Cinv) * (J.transpose() * dp + M.transpose() * dqref);

        q=q0+dq;
        q0=q;

    }

    return q;
}


/* Gets entire forward kinematics */
void 
Controller :: forwardKinematics()
{

    static vector<Matrix4d> HJs_g0(12), HJs0(12);

    


    // SPINE
    HJs[0]<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
    HJs_g[0]=HJs[0];  
    HJs0[0]=HJs[0];
    HJs_g0[0]=HJs_g[0];


        


    for(int i=1; i<12; i++){
        HJs[i]<<cos(joint_angles(i)), -sin(joint_angles(i)), 0, -spine_kin(i-1),
                sin(joint_angles(i)), cos(joint_angles(i)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        HJs0[i]<<cos(offsets(i-1)), -sin(offsets(i-1)), 0, -spine_kin(i-1),
                sin(offsets(i-1)), cos(offsets(i-1)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        HJs_g[i]=HJs_g[i-1]*HJs[i];
        HJs_g0[i]=HJs_g0[i-1]*HJs0[i];
    }


    
    // GIRDLES
    //PLEUROBOT
    if(IS_PLEUROBOT){
        //Fgird0 << 1, 0, 0, 0,     0, 1, 0, 0,     0, 0, 1, 0,      0, 0, 0, 1;
        //Hgird0 << 1, 0, 0, -(spine_kin(0)+spine_kin(1)+spine_kin(2)+spine_kin(3)+spine_kin(4)+0.5*spine_kin(5))+0.5*(spine_kin(6)),     0, 1, 0, 0,     0, 0, 1, 0,      0, 0, 0, 1;
        

        Fgird0.block<3,3>(0, 0)=HJs_g0[0].block<3,3>(0, 0);
        Hgird0.block<3,3>(0, 0)=HJs_g0[5].block<3,3>(0, 0);
        Fgird0.block<3,1>(0, 3) = 0.5*(HJs_g0[0].block<3,1>(0, 3) + HJs_g0[1].block<3,1>(0, 3));
        Hgird0.block<3,1>(0, 3) = 0.5*(HJs_g0[5].block<3,1>(0, 3) + HJs_g0[6].block<3,1>(0, 3));
        Fgird0.block<3,3>(0, 0)=HJs_g0[0].block<3,3>(0, 0);
        Hgird0.block<3,3>(0, 0)=HJs_g0[5].block<3,3>(0, 0);
        Fgird0.block<1,4>(3, 0) << 0, 0, 0, 1;
        Hgird0.block<1,4>(3, 0) << 0, 0, 0, 1;


        Fgird.block<3,3>(0, 0)=HJs_g[0].block<3,3>(0, 0);
        Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
        Fgird.block<3,1>(0, 3) = 0.5*(HJs_g[0].block<3,1>(0, 3) + HJs_g[1].block<3,1>(0, 3));
        Hgird.block<3,1>(0, 3) = 0.5*(HJs_g[5].block<3,1>(0, 3) + HJs_g[6].block<3,1>(0, 3));
        Fgird.block<3,3>(0, 0)=HJs_g[0].block<3,3>(0, 0);
        Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
        Fgird.block<1,4>(3, 0) << 0, 0, 0, 1;
        Hgird.block<1,4>(3, 0) << 0, 0, 0, 1;
    }
    //OROBOT
    else{
        //Fgird0 << 1, 0, 0, 0,     0, 1, 0, 0,     0, 0, 1, 0,      0, 0, 0, 1;
        //Hgird0 << 1, 0, 0, -0.4368,     0, 1, 0, 0,     0, 0, 1, 0,      0, 0, 0, 1;

        Fgird0.block<3,3>(0, 0)=HJs_g0[1].block<3,3>(0, 0);
        Hgird0.block<3,3>(0, 0)=HJs_g0[5].block<3,3>(0, 0);
        Fgird0.block<3,1>(0, 3) = 0.7464*HJs_g0[1].block<3,1>(0, 3) + 0.2536*HJs_g0[2].block<3,1>(0, 3);
        Hgird0.block<3,1>(0, 3) = 0.6363*HJs_g0[5].block<3,1>(0, 3) + 0.3637*HJs_g0[6].block<3,1>(0, 3);
        Fgird0.block<3,3>(0, 0)=HJs_g0[1].block<3,3>(0, 0);
        Hgird0.block<3,3>(0, 0)=HJs_g0[5].block<3,3>(0, 0);
        Fgird0.block<1,4>(3, 0) << 0, 0, 0, 1;
        Hgird0.block<1,4>(3, 0) << 0, 0, 0, 1;



        Fgird.block<3,3>(0, 0)=HJs_g[1].block<3,3>(0, 0);
        Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
        Fgird.block<3,1>(0, 3) = 0.7464*HJs_g[1].block<3,1>(0, 3) + 0.2536*HJs_g[2].block<3,1>(0, 3);
        Hgird.block<3,1>(0, 3) = 0.6363*HJs_g[5].block<3,1>(0, 3) + 0.3637*HJs_g[6].block<3,1>(0, 3);
        Fgird.block<3,3>(0, 0)=HJs_g[1].block<3,3>(0, 0);
        Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
        Fgird.block<1,4>(3, 0) << 0, 0, 0, 1;
        Hgird.block<1,4>(3, 0) << 0, 0, 0, 1;
    }



    // FRONT LEFT LEG
    
    HJfl[0] <<  cos(joint_angles(11)), -sin(joint_angles(11)), 0, FL_kin(0,0),
                sin(joint_angles(11)), cos(joint_angles(11)), 0, FL_kin(0,1),
                0, 0, 1, FL_kin(0,2),
                0, 0, 0, 1;
    HJfl[1] <<  1, 0, 0, FL_kin(1,0),
                0, cos(joint_angles(12)), -sin(joint_angles(12)), FL_kin(1,1),
                0, sin(joint_angles(12)), cos(joint_angles(12)), FL_kin(1,2),
                0, 0, 0, 1;
    HJfl[2] <<  cos(joint_angles(13)), 0, sin(joint_angles(13)), FL_kin(2,0),
                0, 1, 0, FL_kin(2,1),
                -sin(joint_angles(13)), 0, cos(joint_angles(13)), FL_kin(2,2),
                0, 0, 0, 1;
    HJfl[3] <<  cos(joint_angles(14)), -sin(joint_angles(14)), 0, FL_kin(3,0),
                sin(joint_angles(14)), cos(joint_angles(14)), 0, FL_kin(3,1),
                0, 0, 1, FL_kin(3,2),
                0, 0, 0, 1;
    HJfl[4] <<  1, 0, 0, FL_kin(4,0),
                0, 1, 0, FL_kin(4,1),
                0, 0, 1, FL_kin(4,2),
                0, 0, 0, 1;
    HJfl_g[0]=Fgird*HJfl[0];
    for(int i=1;i<5;i++){
        HJfl_g[i]=HJfl_g[i-1]*HJfl[i];
    }
    // FRONT RIGHT LEG
    
    HJfr[0] <<  cos(joint_angles(15)), -sin(joint_angles(15)), 0, FR_kin(0,0),
                sin(joint_angles(15)), cos(joint_angles(15)), 0, FR_kin(0,1),
                0, 0, 1, FR_kin(0,2),
                0, 0, 0, 1;
    HJfr[1] <<  1, 0, 0, FR_kin(1,0),
                0, cos(joint_angles(16)), -sin(joint_angles(16)), FR_kin(1,1),
                0, sin(joint_angles(16)), cos(joint_angles(16)), FR_kin(1,2),
                0, 0, 0, 1;
    HJfr[2] <<  cos(joint_angles(17)), 0, sin(joint_angles(17)), FR_kin(2,0),
                0, 1, 0, FR_kin(2,1),
                -sin(joint_angles(17)), 0, cos(joint_angles(17)), FR_kin(2,2),
                0, 0, 0, 1;
    HJfr[3] <<  cos(joint_angles(18)), -sin(joint_angles(18)), 0, FR_kin(3,0),
                sin(joint_angles(18)), cos(joint_angles(18)), 0, FR_kin(3,1),
                0, 0, 1, FR_kin(3,2),
                0, 0, 0, 1;
    HJfr[4] <<  1, 0, 0, FR_kin(4,0),
                0, 1, 0, FR_kin(4,1),
                0, 0, 1, FR_kin(4,2),
                0, 0, 0, 1;
    HJfr_g[0]=Fgird*HJfr[0];
    for(int i=1;i<5;i++){
        HJfr_g[i]=HJfr_g[i-1]*HJfr[i];
    }

    // HIND LEFT LEG
    
    HJhl[0] <<  cos(joint_angles(19)), -sin(joint_angles(19)), 0, HL_kin(0,0),
                sin(joint_angles(19)), cos(joint_angles(19)), 0, HL_kin(0,1),
                0, 0, 1, HL_kin(0,2),
                0, 0, 0, 1;
    HJhl[1] <<  1, 0, 0, HL_kin(1,0),
                0, cos(joint_angles(20)), -sin(joint_angles(20)), HL_kin(1,1),
                0, sin(joint_angles(20)), cos(joint_angles(20)), HL_kin(1,2),
                0, 0, 0, 1;
    HJhl[2] <<  cos(joint_angles(21)), 0, sin(joint_angles(21)), HL_kin(2,0),
                0, 1, 0, HL_kin(2,1),
                -sin(joint_angles(21)), 0, cos(joint_angles(21)), HL_kin(2,2),
                0, 0, 0, 1;
    HJhl[3] <<  1, 0, 0, HL_kin(3,0),
                0, cos(joint_angles(22)), -sin(joint_angles(22)), HL_kin(3,1),
                0, sin(joint_angles(22)), cos(joint_angles(22)), HL_kin(3,2),
                0, 0, 0, 1;
    HJhl[4] <<  1, 0, 0, HL_kin(4,0),
                0, 1, 0, HL_kin(4,1),
                0, 0, 1, HL_kin(4,2),
                0, 0, 0, 1;

    HJhl_g[0]=Hgird*HJhl[0];
    for(int i=1;i<5;i++){
        HJhl_g[i]=HJhl_g[i-1]*HJhl[i];
    }

    // HIND RIGHT LEG
    
    HJhr[0] <<  cos(joint_angles(23)), -sin(joint_angles(23)), 0, HR_kin(0,0),
                sin(joint_angles(23)), cos(joint_angles(23)), 0, HR_kin(0,1),
                0, 0, 1, HR_kin(0,2),
                0, 0, 0, 1;
    HJhr[1] <<  1, 0, 0, HR_kin(1,0),
                0, cos(joint_angles(24)), -sin(joint_angles(24)), HR_kin(1,1),
                0, sin(joint_angles(24)), cos(joint_angles(24)), HR_kin(1,2),
                0, 0, 0, 1;
    HJhr[2] <<  cos(joint_angles(25)), 0, sin(joint_angles(25)), HR_kin(2,0),
                0, 1, 0, HR_kin(2,1),
                -sin(joint_angles(25)), 0, cos(joint_angles(25)), HR_kin(2,2),
                0, 0, 0, 1;
    HJhr[3] <<  1, 0, 0, HR_kin(3,0),
                0, cos(joint_angles(26)), -sin(joint_angles(26)), HR_kin(3,1),
                0, sin(joint_angles(26)), cos(joint_angles(26)), HR_kin(3,2),
                0, 0, 0, 1;
    HJhr[4] <<  1, 0, 0, HR_kin(4,0),
                0, 1, 0, HR_kin(4,1),
                0, 0, 1, HR_kin(4,2),
                0, 0, 0, 1;

    HJhr_g[0]=Hgird*HJhr[0];
    for(int i=1;i<5;i++){
        HJhr_g[i]=HJhr_g[i-1]*HJhr[i];
    }

    // TRANSFORM EVERYTHING INTO ROBOT's COO FRAME (passing through both girdles)
    static Matrix4d HTransform, HTranslate, HRotate;
   

    hgirdlePhi = atan2(Hgird(1, 3)+hgirdsway-Fgird(1, 3), -Hgird(0, 3)+Fgird(0, 3));
    headq=hgirdlePhi;
    HTransform.block(0,0,3,3) << cos(hgirdlePhi), -sin(hgirdlePhi), 0,    sin(hgirdlePhi), cos(hgirdlePhi), 0,    0, 0, 1;
    HTransform.block(0,3,3,1) = -Fgird.block<3,1>(0,3);
    HTransform.block(3,0,1,4) << 0, 0, 0, 1;

    HTranslate=Matrix4d::Identity();
    HTranslate.block<3,1>(0,3) = -Fgird.block<3,1>(0,3);
    HRotate=Matrix4d::Identity();
    HRotate.block<3,3>(0,0) << cos(hgirdlePhi), -sin(hgirdlePhi), 0,    sin(hgirdlePhi), cos(hgirdlePhi), 0,    0, 0, 1;


    HTransform=HTransform.inverse().eval();
    HTransform=HRotate*HTranslate;
    for(int i=0;i<5;i++){
        HJfl_g[i]=HTransform*HJfl_g[i];
        HJfr_g[i]=HTransform*HJfr_g[i];
        HJhl_g[i]=HTransform*HJhl_g[i];
        HJhr_g[i]=HTransform*HJhr_g[i];
    }

    for(int i=0;i<11;i++){
        HJs_g[i]=HTransform*HJs_g[i];
    }

    Hgird=HTransform*Hgird;
    Fgird=HTransform*Fgird;

    // HEAD
    HJh<<cos(-headq+pi), -sin(-headq+pi), 0, 0,
                sin(-headq+pi), cos(-headq+pi), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;


    HJh_g=HJs_g[0]*HJh;



    // get fgird rotation in respect to the frame of reference
    fgird2ForRotMat=AngleAxisd(hgirdlePhi, Vector3d::UnitZ());

/*
    attData[0]=atan2(bodyRotMat(2, 1), bodyRotMat(2, 2));
    attData[1]=atan2(-bodyRotMat(2, 0), sqrt(bodyRotMat(0,0)*bodyRotMat(0,0) + bodyRotMat(1,0)*bodyRotMat(1,0)));
*/
    FoROrientation=hgirdlePhi;

    // compensate HGird0 position for steering 
    hgirdlePhi = atan2(Hgird0(1, 3)-Fgird0(1, 3), -Hgird0(0, 3)+Fgird0(0, 3));
    HTranslate=Matrix4d::Identity();
    HTranslate.block<3,1>(0,3) = -Fgird0.block<3,1>(0,3);
    HRotate=Matrix4d::Identity();
    HRotate.block<3,3>(0,0) << cos(hgirdlePhi), -sin(hgirdlePhi), 0,    sin(hgirdlePhi), cos(hgirdlePhi), 0,    0, 0, 1;
    HTransform=HRotate*HTranslate;
    Hgird0=HTransform*Hgird0;
    Fgird0=HTransform*Fgird0;


    // find steering radius
    static double alpha;
    alpha=0;
    if(IS_PLEUROBOT){
        for(int i=1; i<6; i++)
            alpha+=offsets(i);
    }
    else 
        for(int i=2; i<6; i++)
            alpha+=offsets(i);
    if(abs(alpha)>0.001){
        steeringRadius=-Hgird0(0, 3)/2./asin(alpha/2.);
    }
    else 
        steeringRadius=1000;
    if(isnan(steeringRadius))
        steeringRadius=1000;


    //static ofstream legdifflog("/data/Dropbox/MATLAB/Orobot/legdifflog.txt");


    //legdifflog<<HJfl_g[4](0, 3)<<"\t"<<HJhl_g[4](0, 3)<<"\t"<<endl;
    //legdifflog<<"diff: "<<HJfl_g[4](0, 3)-HJhl_g[4](0, 3)<<endl;
 

}

/* Calculates floating frame of reference and all joint positions in respect to it */
void
Controller :: getFloatingFrame(){

    static MatrixXd Xpoints(11, 2), Ypoints(11, 1), tmp22(2, 2);

    for(int i=0; i<11; i++){
        Xpoints(i, 0)=HJs_g[i](0, 3);
        Xpoints(i, 1)=1;
        Ypoints(i, 0)=HJs_g[i](1, 3);
    }


    tmp22=Xpoints.transpose()*Xpoints;

    FFParam = tmp22.inverse()*Xpoints.transpose()*Ypoints;

}


/* Calculates global position of all the joints for a given GPS position and front girdle orientation */
void 
Controller :: globalKinematics(double gpsPosition[3], double rotMat[9])
{
    gpsPos(0)=gpsPosition[0];
    gpsPos(1)=-gpsPosition[2];
    gpsPos(2)=gpsPosition[1];

   rotMatFgird = fgird2ForRotMat.inverse()*compassRotMat;    

    for(int i=0; i<11; i++){
        global_joint_pos.block<3,1>(0,i) =rotMatFgird*HJs_g[i].block<3,1>(0, 3) + gpsPos;
    }
    for(int i=0; i<4; i++){
        global_joint_pos.block<3,1>(0,11+i) = rotMatFgird*HJfl_g[i].block<3,1>(0, 3) + gpsPos;
        global_joint_pos.block<3,1>(0,15+i) = rotMatFgird*HJfr_g[i].block<3,1>(0, 3) + gpsPos;
        global_joint_pos.block<3,1>(0,19+i) = rotMatFgird*HJhl_g[i].block<3,1>(0, 3) + gpsPos;
        global_joint_pos.block<3,1>(0,23+i) = rotMatFgird*HJhr_g[i].block<3,1>(0, 3) + gpsPos;
    }
    global_feet_pos.block<3,1>(0,0) = rotMatFgird*HJfl_g[4].block<3,1>(0, 3) + gpsPos;
    global_feet_pos.block<3,1>(0,1) = rotMatFgird*HJfr_g[4].block<3,1>(0, 3) + gpsPos;
    global_feet_pos.block<3,1>(0,2) = rotMatFgird*HJhl_g[4].block<3,1>(0, 3) + gpsPos;
    global_feet_pos.block<3,1>(0,3) = rotMatFgird*HJhr_g[4].block<3,1>(0, 3) + gpsPos;

    //cout<<Fgird.block<3,3>(0,0)*rotMatFgird<<endl<<endl;
    static Matrix3d tmp33;
    tmp33=rotMatFgird;
    //cout<<Fgird.block<3,3>(0,0)<<endl<<endl;
    //FoROrientation=atan2(tmp33(1,0), tmp33(0,0));



} 

/* get global feet position from webots */
void
Controller :: getFeetPosition(double *fl, double *fr, double *hl, double *hr){
    globalPosFL(0)=fl[0];
    globalPosFL(1)=-fl[2];
    globalPosFL(2)=fl[1];

    globalPosFR(0)=fr[0];
    globalPosFR(1)=-fr[2];
    globalPosFR(2)=fr[1];

    globalPosHL(0)=hl[0];
    globalPosHL(1)=-hl[2];
    globalPosHL(2)=hl[1];

    globalPosHR(0)=hr[0];
    globalPosHR(1)=-hr[2];
    globalPosHR(2)=hr[1];

}

/* Calculates angles from IKIN and compensates for spine_kin */
void
Controller :: inverseKinematicsController()
{


    //############################ spine_kin COMPENSATION ##############################################
   

    HFL = AngleAxisd(-atan2(-Fgird(0, 1), Fgird(0, 0)), Vector3d::UnitZ()) *    Translation3d( -(Fgird(0, 3)-Fgird0(0, 3)), -(Fgird(1, 3)-Fgird0(1, 3)) ,0);
    HFR = AngleAxisd(-atan2(-Fgird(0, 1), Fgird(0, 0)), Vector3d::UnitZ()) *    Translation3d( -(Fgird(0, 3)-Fgird0(0, 3)), -(Fgird(1, 3)-Fgird0(1, 3)) ,0);
    HHL = AngleAxisd(-atan2(-Hgird(0, 1), Hgird(0, 0)), Vector3d::UnitZ()) *    Translation3d( -(Hgird(0, 3)-Hgird0(0, 3)), -(Hgird(1, 3)-Hgird0(1, 3)) ,0);
    HHR = AngleAxisd(-atan2(-Hgird(0, 1), Hgird(0, 0)), Vector3d::UnitZ()) *    Translation3d( -(Hgird(0, 3)-Hgird0(0, 3)), -(Hgird(1, 3)-Hgird0(1, 3)) ,0);

    pFL=HFL*pFL;
    pFR=HFR*pFR;
    pHL=HHL*pHL;
    pHR=HHR*pHR;





    //############################ USE ANIMAL DATA AS PREFERED JOINT ANGLES #######################
    if(useAnDF){
        for(int i=0; i<4; i++){
            if(legs_stance(0))
                q0FL(i)=animalStance[(int)floor(stancePhase(0)*animalStanceLen[0])][i];     
            else
                q0FL(i)=animalSwing[(int)floor(swingPhase(0)*animalSwingLen[0])][i]; 


            if(legs_stance(1))
                q0FR(i)=animalStance[(int)floor(stancePhase(1)*animalStanceLen[1])][i+4];     
            else
                q0FR(i)=animalSwing[(int)floor(swingPhase(1)*animalSwingLen[1])][i+4]; 
        }
    }

    if(useAnDH){
        for(int i=0; i<4; i++){
            if(legs_stance(2))
                q0HL(i)=animalStance[(int)floor(stancePhase(2)*animalStanceLen[2])][i+8];     
            else
                q0HL(i)=animalSwing[(int)floor(swingPhase(2)*animalSwingLen[2])][i+8]; 


            if(legs_stance(3))
                q0HR(i)=animalStance[(int)floor(stancePhase(3)*animalStanceLen[3])][i+12];     
            else
                q0HR(i)=animalSwing[(int)floor(swingPhase(3)*animalSwingLen[3])][i+12]; 
        }
    }

    //############################ IKIN ###########################################################

    // DLS
    /*
    qFL=iKinDLS(HJfl[0]*HJfl[1]*HJfl[2]*HJfl[3]*HJfl[4], pFL, q0FL, qFL, CinvF, MF, max_dist, 0);
    qFR=iKinDLS(HJfr[0]*HJfr[1]*HJfr[2]*HJfr[3]*HJfr[4], pFR, q0FR, qFR, CinvF, MF, max_dist, 1);
    qHL=iKinDLS(HJhl[0]*HJhl[1]*HJhl[2]*HJhl[3]*HJhl[4], pHL, q0HL, qHL, CinvH, MH, max_dist, 2);
    qHR=iKinDLS(HJhr[0]*HJhr[1]*HJhr[2]*HJhr[3]*HJhr[4], pHR, q0HR, qHR, CinvH, MH, max_dist, 3);
    */
    //cout<<pFL.transpose()<<endl;
    // NullIDLS
    qFL=iKinNullIDLS(0, pFL, q0FL, qFL, lamF, MF, max_dist, ikin_maxIter, ikin_tol, constrFL, ikin_constr_penalty);
    qFR=iKinNullIDLS(1, pFR, q0FR, qFR, lamF, MF, max_dist, ikin_maxIter, ikin_tol, constrFR, ikin_constr_penalty);
    qHL=iKinNullIDLS(2, pHL, q0HL, qHL, lamH, MH, max_dist, ikin_maxIter, ikin_tol, constrHL, ikin_constr_penalty);
    qHR=iKinNullIDLS(3, pHR, q0HR, qHR, lamH, MH, max_dist, ikin_maxIter, ikin_tol, constrHR, ikin_constr_penalty);


    // LOGGING
    /*
    angles.block<11,1>(0,0)=qs;
    angles.block<4,1>(11,0)=qFL;
    angles.block<4,1>(15,0)=qFR;
    angles.block<4,1>(19,0)=qHL;
    angles.block<4,1>(23,0)=qHR;

    joint_angles=angles;


    //forwardKinematics();
    
    static ofstream ikinLog("ikinLog.txt");
    Matrix4d tmpFL, tmpFR, tmpHL, tmpHR;

    tmpFL=HJfl[0]*HJfl[1]*HJfl[2]*HJfl[3]*HJfl[4];
    tmpFR=HJfr[0]*HJfr[1]*HJfr[2]*HJfr[3]*HJfr[4];
    tmpHL=HJhl[0]*HJhl[1]*HJhl[2]*HJhl[3]*HJhl[4];
    tmpHR=HJhr[0]*HJhr[1]*HJhr[2]*HJhr[3]*HJhr[4];

    for(int i=0; i<3; i++){
        ikinLog << pFL(i)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << pFR(i)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << pHL(i)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << pHR(i)<<"\t";
    }

    for(int i=0; i<3; i++){
        ikinLog << tmpFL(i, 3)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << tmpFR(i, 3)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << tmpHL(i, 3)<<"\t";
    }
    for(int i=0; i<3; i++){
        ikinLog << tmpHR(i, 3)<<"\t";
    }
    ikinLog << endl;*/
}

/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
Vector4d
Controller :: iKinDLS(Matrix4d H_leg, Vector3d pref, Vector4d qref, Vector4d q0, Matrix4d Cinv, Matrix4d M, double max_dist, int leg)
{



    MatrixXd J(3,4), tmp(3,3);
    Vector4d dqref, q, dq;
    Vector3d dp, p0;
    double norm_dp;

    // get current position
    p0=H_leg.block<3, 1>(0, 3);


    J=Jacob(q0, leg);

    dqref=qref-q0;

    dp=pref-p0;

    norm_dp=sqrt(dp(0)*dp(0)+dp(1)*dp(1)+dp(2)*dp(2));
    if(norm_dp>max_dist){
        dp=max_dist/norm_dp*dp;
    }

    tmp=Matrix3d::Identity() + J * Cinv * J.transpose();
    dq=(Cinv - Cinv * J.transpose() * tmp.inverse() * J * Cinv) * (J.transpose() * dp + M.transpose() * dqref);

    q=q0+dq;
    return q;
}

/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
Vector4d
Controller :: iKinNullIDLS(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, Vector3d constr_penalty)
{
    static MatrixXd J(3,4), tmp33(3,3), tmp44(4,4), H(4,4), D(4,4), U(3,3), V(4,4), S(3,1), Jnull(4,4), C(4,4);
    static Vector4d dqref, q, dq;
    static Vector3d dp, p0;
    static double norm_dp;
    static bool constr_violated;
    static Matrix4d P=MatrixXd::Constant(4,4,constr_penalty(0)/constr_penalty(1));

    constr_penalty(0)/=constr_penalty(1);
    //P=MatrixXd::Zero(1,4);
    C=MatrixXd::Zero(4,4);
  /*  for(int i=0; i<4;i++){
        P(leg,i)=constr_penalty(0);
    }*/
    // BASE CONTROLLER
    q=q0;
    for(int k=0; k<maxIter; k++){
        
        // get current position
        H=legKinematics(q, leg);
        p0=H.block<3, 1>(0, 3);


        J=Jacob(q, leg);

        dqref=qref-q;

        dp=pref-p0;

        norm_dp=sqrt(dp(0)*dp(0)+dp(1)*dp(1)+dp(2)*dp(2));
        if(norm_dp>max_dist){
            dp=max_dist/norm_dp*dp;
        }
        
        if(norm_dp<tol &&constr_violated==false){
            break;
        }
        
        
        tmp44 = lam(0)*Matrix4d::Identity() + J.transpose()*J;
        dq = tmp44.inverse() * (J.transpose()*dp  - C*P.block<1,4>(leg, 0).transpose());
        q+=dq;

        // check and penalize limits
        constr_violated=false;
        C=MatrixXd::Zero(4,4);
        for(int i=0; i<4; i++){
            if(q(i)<constr(0, i)){
                C(i,i)=-1;
                constr_violated=true;

            }
            else if(q(i)>constr(1, i)){
                C(i,i)=1;
                constr_violated=true;

            }
            if(C(i,i)){
                P(leg, i)*=constr_penalty(1);
            }
            else if (P(leg, i)>constr_penalty(0)){
                P(leg, i)/=constr_penalty(1);
            }
        }
        if(!C(0,0) && !C(1,1) && !C(2,2) && !C(3,3)){
            constr_violated==false;
            tol*=constr_penalty(2);
            

        }


    }

    // NULL SPACE POSTURE CONTROLLER
    // compute SVD decomposition of Jacobian
    J=Jacob(q, leg);
    JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
    S=svd.singularValues();
    U=svd.matrixU();
    V=svd.matrixV();

    D=Matrix4d::Zero();
    D(3,3)=1;
    for(int i=2; i<0;i++){
        if(abs(S(i))<0.001)
            D(i, i)=1;
    }
    Jnull=V*D*V.transpose();


    tmp44=Jnull.transpose()*M.transpose()*Jnull+lam(1)*Matrix4d::Identity();
    dq=tmp44.inverse()*Jnull.transpose()*M.transpose()*dqref;

    q+=dq;
    return q;
}




Vector3d
Controller :: getCoM()
{

    static Matrix4d mvec=MatrixXd::Identity(4,4);
    static double total_mass;

    CoM<<0,0,0;
    total_mass=0;


    //head
    mvec.block<3,1>(0,3)=masses.block<3,1>(1,0);
    CoM+=masses(0,0)*(HJh_g*mvec).block<3,1>(0,3);
    total_mass+=masses(0,0);
    

    //spine
    for(int i=0; i<6; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,1+i);
        CoM+=masses(0,1+i)*(HJs_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,1+i);
    }

    //FL
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i);
        CoM+=masses(0,7+i)*(HJfl_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i);
    }

    //FR
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+4);
        CoM+=masses(0,7+i+4)*(HJfr_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+4);
    }

    //HL
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+8);
        CoM+=masses(0,7+i+8)*(HJhl_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+8);
    }

    //HR
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+12);
        CoM+=masses(0,7+i+12)*(HJhr_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+12);
    }


    CoM/=total_mass;

    return CoM;
}


void
Controller :: getLegJacobians(){
    legJacob[0]=Jacob(fbck_position.block<4,1>(11,0), 0);
    legJacob[1]=Jacob(fbck_position.block<4,1>(15,0), 1);
    legJacob[2]=Jacob(fbck_position.block<4,1>(19,0), 2);
    legJacob[3]=Jacob(fbck_position.block<4,1>(23,0), 3);

}


























/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
/*Vector4d
Controller :: iKinNullIDLS(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, Vector2d constr_penalty)
{
    static MatrixXd J(3,4), tmp33(3,3), tmp44(4,4), H(4,4), D(4,4), U(3,3), V(4,4), S(3,1), Jnull(4,4), P(1,4), C(4,4);
    static Vector4d dqref, q, dq;
    static Vector3d dp, p0;
    static double norm_dp;
    static bool constr_violated;


    constr_penalty(0)/=constr_penalty(1);
    P=MatrixXd::Zero(1,4);
    C=MatrixXd::Zero(4,4);
    for(int i=0; i<4;i++){
        P(i)=constr_penalty(0);
    }
    // BASE CONTROLLER
    q=q0;
    for(int k=0; k<maxIter; k++){
        
        // get current position
        H=legKinematics(q, leg);
        p0=H.block<3, 1>(0, 3);


        J=Jacob(q, leg);

        dqref=qref-q;

        dp=pref-p0;

        norm_dp=sqrt(dp(0)*dp(0)+dp(1)*dp(1)+dp(2)*dp(2));
        if(norm_dp>max_dist){
            dp=max_dist/norm_dp*dp;
        }
        
        
        if(norm_dp<tol && constr_violated==false){
            break;
        }
        
        tmp44 = lam(0)*Matrix4d::Identity() + J.transpose()*J;
        dq = tmp44.inverse() * (J.transpose()*dp  - C*P.transpose());
        q+=dq;

        // check and penalize limits
        constr_violated=false;
        C=MatrixXd::Zero(4,4);
        for(int i=0; i<4; i++){
            if(q(i)<constr(0, i)){
                C(i,i)=-1;
                constr_violated=true;

            }
            else if(q(i)>constr(1, i)){
                C(i,i)=1;
                constr_violated=true;

            }
            if(C(i,i)){
                P(i)*=constr_penalty(1);
            }
            else{
                //P(i)=constr_penalty(0);
            }
        }
        if(!C(0,0) && !C(1,1) && !C(2,2) && !C(3,3)){
            constr_violated==false;

        }
        else{
            q-=dq;
        }

    }

    // NULL SPACE POSTURE CONTROLLER
    // compute SVD decomposition of Jacobian
    J=Jacob(q0+dq, leg);
    JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
    S=svd.singularValues();
    U=svd.matrixU();
    V=svd.matrixV();

    D=Matrix4d::Zero();
    D(3,3)=1;
    for(int i=2; i<0;i++){
        if(abs(S(i))<0.001)
            D(i, i)=1;
    }
    Jnull=V*D*V.transpose();


    tmp44=Jnull.transpose()*M.transpose()*Jnull+lam(1)*Matrix4d::Identity();
    dq=tmp44.inverse()*Jnull.transpose()*M.transpose()*dqref;

    q+=dq;
    return q;
}*/
