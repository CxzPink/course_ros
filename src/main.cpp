#include"../inc/cxz_robot_arm.h"

int main(int argc,char *argv[])
{
    using namespace cxz;

//    double new_angle[6]={0.927,-0.687,-0.396,0,1.083,0.927};
//    double new_angle[6]={0.322,-0.855,-0.021,0,0.877,0.322};
//    double new_angle[6]={-0.322,0.636,-0.011,0,0.647,-0.322};
//    double target_pose[6]={0.2,0.2,0.2007,1.57,-1.57,0}; 
//    double target_pose[6]={0.15,0.2,0.2007,0,0,0};
    double target_pose[6]={0.3,0,0.122,1.57,0,0};
    RobotArm robotarm;
    robotarm.SolveTheTargetPoint(target_pose);
    return 0;
}