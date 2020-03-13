#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "../inc/cxz_robot_arm.h"
#define MAX_V 1
#define MAX_V_LIMIT 0.9
#define MAX_A 1
#define MAX_A_LIMIT 0.9
#define STEP 1
#define ADJUST_NUMBER 10
using namespace Eigen;

void cxz::TrackSolution(const std::vector<double> point_value,std::vector<double *> & solution)
{
    TrackInitialTime(point_value,solution);
    CubicSpline(point_value,solution);
    for(size_t i=0;i<ADJUST_NUMBER;i++){
        while(NegativeAdjustTime(solution)){
            CubicSpline(point_value,solution);
        }
        while(PositiveAdjustTime(solution)){
            CubicSpline(point_value,solution);
        }
    }
}
void cxz::TrackInitialTime(const std::vector<double> point_value,std::vector<double *> & solution)
{
    double *p=NULL;
    size_t m_size=point_value.size()-1;
    for(size_t i=0;i<m_size;i++){
        p=new double[5];
        solution.push_back(p);
    }
    for(size_t i=0;i<m_size;i++){
        solution[i][4]=(point_value[i+1]-point_value[i])/(MAX_V);
    }
}
void cxz::CubicSpline(const std::vector<double> point_value,std::vector<double *> & solution)
{
    size_t n=point_value.size()-1;
    MatrixXf A(n+1);
    VectorXf B(n+1),X(n+1);

    A(0,0)=2*solution[0][4];
    A(0,1)=solution[0][4];
    A(n,n)=2*solution[n-1][4];
    A(n,n-1)=solution[n-1][4];
    for(auto i=1;i<n;i++){
        A(i,i-1)=solution[i-1][4];
        A(i,i)=2*(solution[i-1][4]+solution[i][4]);
        A(i,i+1)=solution[i][4];
    }
    B[0]=3*(point_value[1]-point_value[0])/solution[0][4];
    B[n]=(-3)*(point_value[n]-point_value[n-1])/solution[n-1][4];
    for(auto i=1;i<n;i++){
        B[i]=3*(point_value[i+1]-point_value[i])/solution[i][4]-3*(point_value[i]-point_value[i-1])/solution[i-1][4];
    }
    X=(A.inverse())*B;

    for(auto i=0;i<n;i++){
        solution[i][0]=point_value[i];
        solution[i][1]=(point_value[i+1]-point_value[i])/solution[i][4]-solution[i][4]*(X[i+1]+2*X[i])/3;
        solution[i][2]=X[i];
        solution[i][3]=(X[i+1]-X[i])/(3*solution[i][4]);
    }
}
int cxz::IsTimeTooShort(const double * s)
{
    double temp;
    temp=-s[2]/(3*s[3]);
    if(temp>=0 && temp<=s[4]){
        if(fabs(3*s[3]*pow(temp,2)+2*s[2]*temp+s[1])<=MAX_V 
           && fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V
           && fabs(2*s[2])<=MAX_A && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A)
           return 0;
        else
           return 1;
    }
    else{
        if(fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V
           && fabs(2*s[2])<=MAX_A && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A)
           return 0;
        else
           return 1;
    }
}
int cxz::IsTimeTooLong(const double * s)
{
    double temp;
    temp=-s[2]/(3*s[3]);
    if(temp>=0 && temp<=s[4]){
        if(fabs(3*s[3]*pow(temp,2)+2*s[2]*temp+s[1])<=MAX_V_LIMIT 
           && fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_LIMIT 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_LIMIT
           && fabs(2*s[2])<=MAX_A && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_LIMIT)
           return 1;
        else
           return 0;
    }
    else{
        if(fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_LIMIT 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_LIMIT
           && fabs(2*s[2])<=MAX_A && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_LIMIT)
           return 1;
        else
           return 0;
    }
}
int cxz::PositiveAdjustTime(std::vector<double *> & solution)
{
    size_t solution_size=solution.size();
    int flag=0;
    for(size_t i=0;i<solution_size;i++){
        if(IsTimeTooShort(solution[i])){
            solution[i][4]=solution[i][4]+STEP;
            flag=1;
        }
    }
    if(flag==0)
       return 0;
    else
       return 1;
     
}
int cxz::NegativeAdjustTime(std::vector<double *> & solution)
{
    size_t solution_size=solution.size();
    int flag=0;
    for(size_t i=0;i<solution_size;i++){
        if(IsTimeTooLong(solution[i])){
            solution[i][4]=solution[i][4]-STEP;
            flag=1;
        }
    }
    if(flag==0)
       return 0;
    else
       return 1;
     
}
double cxz::ReadVelocityFromSolution(const std::vector<double *> & solution,const double & t)
{
    double sum_time=0;
    size_t size_solution=solution.size();
    for(size_t i=0;i<size_solution;i++){
        if(sum_time<=t && (sum_time+solution[i][4])>=t)
            return (solution[i][0]+solution[i][1]*pow((t-sum_time),1)+solution[i][2]*pow((t-sum_time),2)+solution[i][3]*pow((t-sum_time),3));
        else
            sum_time=sum_time+solution[i][4];        
    }
    return 0;
}