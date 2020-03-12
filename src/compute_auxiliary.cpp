#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "../inc/cxz_robot_arm.h"

void cxz::ComputePostureAngleByMatrixXYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3])
{
     double x,y,z;
    y=atan2(-posture_matrix[2][0],pow(posture_matrix[0][0]*posture_matrix[0][0]+posture_matrix[1][0]*posture_matrix[1][0],0.5));
    z=atan2(posture_matrix[1][0]/cos(y),posture_matrix[0][0]/cos(y));
    x=atan2(posture_matrix[2][1]/cos(y),posture_matrix[2][2]/cos(y));

    posture_angle[0]=x;   
    posture_angle[1]=y;
    posture_angle[2]=z;
}

void cxz::ComputePostureAngleByMatrixZYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3])
{
	double z1,y,z2;
	y = atan2(pow(posture_matrix[2][0]* posture_matrix[2][0]+ posture_matrix[2][1] * posture_matrix[2][1],0.5), posture_matrix[2][2]);
	z1 = atan2(posture_matrix[1][2]/sin(y), posture_matrix[0][2]/sin(y));
	z2 = atan2(posture_matrix[2][1]/sin(y),-posture_matrix[2][0]/sin(y));

	posture_angle[0] = z1;
	posture_angle[1] = y;
	posture_angle[2] = z2;
}

void cxz::ComputePostureMatrixByAngle(double (&posture_matrix)[3][3],const double  (&posture_angle)[3])
{
        using namespace Eigen;
        Matrix3f X,Y,Z,R;
        X<< 1,0,0,
                 0,cos(posture_angle[0]),-sin(posture_angle[0]),
                 0,sin(posture_angle[0]),cos(posture_angle[0]);
        Y<< cos(posture_angle[1]),0,sin(posture_angle[1]),
                 0,1,0,
                -sin(posture_angle[1]),0,cos(posture_angle[1]);
        Z<< cos(posture_angle[2]),-sin(posture_angle[2]),0,
                 sin(posture_angle[2]),cos(posture_angle[2]),0,
                 0,0,1;
        R=Z*Y*X;

        for(size_t i=0;i<3;i++){
            for(size_t j=0;j<3;j++){
                posture_matrix[i][j]=R(i,j);
            }
        }
}

void cxz::ComputeQuaternion(const double (&posture_matrix)[3][3],double  (&quaternion_angle)[4])
{
    quaternion_angle[3]=0.5*pow(1+posture_matrix[0][0]+posture_matrix[1][1]+posture_matrix[2][2],0.5);
    quaternion_angle[0]=(posture_matrix[2][1]-posture_matrix[1][2])/(4*quaternion_angle[3]);
    quaternion_angle[1]=(posture_matrix[0][2]-posture_matrix[2][0])/(4*quaternion_angle[3]);
    quaternion_angle[2]=(posture_matrix[1][0]-posture_matrix[0][1])/(4*quaternion_angle[3]);
}
