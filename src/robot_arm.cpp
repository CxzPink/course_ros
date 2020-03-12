#include <eigen3/Eigen/Core>
#include <math.h>
#include <iostream>
#include "../inc/cxz_robot_arm.h"
using namespace Eigen;
using namespace cxz;
double  INITIAL_PARAMETERS_MDH[6][4] = { {0, 0, 0.284, 0}, {PI/2, 0.0, 0.0, PI/2}, {0,0.225,0,0},{PI/2,0,0.2289,0},{-PI/2,0,0,-PI/2},{PI/2,0,0.055,0} };

void cxz::ComputePostureAngleByMatrixXYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3]);
void cxz::ComputeInverseKinematics(const double(&target_pose)[6], double(&angle_solution_temp)[2][6]);
int cxz::ChoseSolution(const double(&now_joint_angle)[6],const double(&angle_solution_temp)[2][6], std::vector<double *> &angle_solution);

cxz::RobotArm::RobotArm()
{
    double *p;
    joint_number=6;

    for(size_t i=0;i<joint_number;i++){
        p=new double[4];
        for(size_t j=0;j<4;j++){
            p[j]=INITIAL_PARAMETERS_MDH[i][j];
        }
        mdh_parameter.push_back(p);
    }

    for(size_t i=0;i<joint_number;i++)
        joint_angle.push_back(0);

    ComputePositionAndPosture();
}

int cxz::RobotArm::UpdateAngle(const double (&new_joint_angle)[6])
{
    if(joint_number!=6)
        return 1;
    for(size_t i=0;i<joint_number;i++){
        joint_angle[i]=new_joint_angle[i];
    }
    UpdateMDH();
    ComputePositionAndPosture();
    TailPostureMatrixToAngle();
}

int cxz::RobotArm::SolveTheTargetPoint(const double (&target_pose)[6])
{
    double angle_solution_temp[2][6],now_joint_angle[6];
    std::vector<double *> angle_solution;
    for(size_t i=0;i<6;i++)
        now_joint_angle[i]=joint_angle[i];
    
    ComputeInverseKinematics(target_pose,angle_solution_temp);
    if(ChoseSolution(now_joint_angle,angle_solution_temp,angle_solution)==1)
       return 1;
    for(size_t i=0;i<6;i++)
        now_joint_angle[i]=angle_solution[0][i];
    UpdateAngle(now_joint_angle);
    std::cout<<"--------------------------solution joint angle--------------------------"<<std::endl;
    PrintJointAngle();
    std::cout<<"-----------------tail position and posture in these joint angle-----------------"<<std::endl;
    PrintTailPositionAndPosture();
    return 0;  
}

void cxz::RobotArm::UpdateMDH(void)
{
    for(size_t i=0; i<joint_number; i++){       
         mdh_parameter[i][3]=joint_angle[i]+INITIAL_PARAMETERS_MDH[i][3];
    }
}

void cxz::RobotArm::ComputePositionAndPosture(void)
{
     Matrix4f M = MatrixXf::Identity(4,4);
     Matrix4f T;
     Position position;
     joint_position.clear();
     
     for(size_t i=0; i<joint_number; i++){
         T<<  cos(mdh_parameter[i][3]), -sin(mdh_parameter[i][3]), 0, mdh_parameter[i][1],
                   sin(mdh_parameter[i][3])*cos(mdh_parameter[i][0]),  cos(mdh_parameter[i][3])*cos(mdh_parameter[i][0]), -sin(mdh_parameter[i][0]), -sin(mdh_parameter[i][0])*mdh_parameter[i][2],
                   sin(mdh_parameter[i][3])*sin(mdh_parameter[i][0]),  cos(mdh_parameter[i][3])*sin(mdh_parameter[i][0]), cos(mdh_parameter[i][0]),  cos(mdh_parameter[i][0])*mdh_parameter[i][2],
                   0, 0, 0, 1;
        M=M*T;

        for(size_t j=0; j<3; j++)
            position.coord[j]=M(j,3);
        joint_position.push_back(position);
     }
    for(size_t j=0; j<3;j++)
        for(size_t k=0; k<3 ;k++)
            tail_posture.posture_maxtrix[j][k]=M(j,k);
}

void cxz::RobotArm::TailPostureMatrixToAngle(void)
{
    double x,y,z;
    size_t end=joint_number-1;

    ComputePostureAngleByMatrixXYZ(tail_posture.posture_maxtrix,tail_posture.posture_angle);
    if(tail_posture.posture_angle[0]<(-PI/2))
        tail_posture.posture_angle[0]=tail_posture.posture_angle[0]+3*PI/2;
    else
        tail_posture.posture_angle[0]=tail_posture.posture_angle[0]-PI/2;        

}

void cxz::RobotArm::PrintTailPositionAndPosture(void)
{
    size_t end=joint_number-1;
    std::cout<<"Tail Position And Posture:  "
        <<joint_position[end].coord[0]<<"  "
        <<joint_position[end].coord[1]<<"  "
        <<joint_position[end].coord[2]<<"  "
        <<tail_posture.posture_angle[0]<<"  "
        <<tail_posture.posture_angle[1]<<"  "
        <<tail_posture.posture_angle[2]<<"  "
    <<std::endl;
}

void cxz::RobotArm::PrintJointAngle(void)
{
    std::cout<<"Joint Angle: ";
    for(size_t i=0;i<joint_number;i++)
        std::cout<<joint_angle[i]<<"  ";
    std::cout<<std::endl;
}