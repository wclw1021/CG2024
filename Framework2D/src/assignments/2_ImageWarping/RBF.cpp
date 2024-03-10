#include "RBF.h"
#include "comp_warping.h"
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
namespace USTC_CG
{
std::pair<int, int>
RBF::warp(int x, int y, int width, int height)
{
    int n = static_cast<int>(start_points_.size());
    std::pair<int, int> p = std::make_pair(x,y);
    float sum_x = 0, sum_y = 0;
    for (int i = 0; i < n; i++)
    {
        sum_x = sum_x + X_(i) * D(p,startpoints[i]);
        sum_y = sum_y + X_(i+n) * D(p,startpoints[i]);
    }
    sum_x = sum_x + X_(2*n)*x + X_(2*n+1)*y + X_(2*n+4);
    sum_y = sum_y + X_(2*n+2)*x + X_(2*n+3)*y + X_(2*n+5);
    int new_x = static_cast<int>(sum_x);
    int new_y = static_cast<int>(sum_y);    
    return std::make_pair(new_x, new_y);
}
// compute the R_distance
float RBF::D(std::pair<float,float> p_1, std::pair<float,float> p_2)
{
    float Distance = distance(p_1,p_2);
    float d = static_cast<float>(pow((pow(Distance,2)+pow(r,2)),mu));
    return d;
}

// compute the solution matrix 
Eigen::MatrixXd RBF::Solution(std::vector<std::pair<float,float>> P_i, std::vector<std::pair<float,float>> Q_i)
{
    int n = static_cast<int>(P_i.size());
    Eigen::MatrixXd A((2*n+6),(2*n+6));
    A = Eigen::MatrixXd::Constant(2 * n + 6, 2 * n + 6, 0.f);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            A(i,j) = D(P_i[i],P_i[j]);
        }
    }
    for (int i = 0; i < n; i++)
    {
        A(i,2*n) = P_i[i].first;
        A(i,2*n+1) = P_i[i].second;
        A(i,2*n+4) = 1;
    }
    for (int i = n; i < 2*n; i++)
    {
        for (int j = n; j < 2*n; j++)
        {
            A(i,j) = D(P_i[i-n],P_i[j-n]);
        }
    }
    for (int i = n; i < 2*n; i++)
    {
        A(i,2*n+2) = P_i[i-n].first;
        A(i,2*n+3) = P_i[i-n].second;
        A(i,2*n+5) = 1;
    }
    for (int j = 0; j < n; j++)
    {
        A(2*n,j) = P_i[j].first;
        A(2*n+1,j) = P_i[j].second;
        A(2*n+2,j) = 1;
    }
    for (int j = n; j < 2*n; j++)
    {
        A(2*n+3,j) = P_i[j-n].first;
        A(2*n+4,j) = P_i[j-n].second;
        A(2*n+5,j) = 1;
    }
    Eigen::MatrixXd B((2*n+6),1);
    B.fill(0.f);
    for (int i = 0; i < n; i++)
    {
        B(i) = Q_i[i].first;
    }
    for (int i = n; i < 2*n; i++)
    {
        B(i) = Q_i[i-n].second;
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu_solver(A);
    Eigen::VectorXd X = lu_solver.solve(B);
    return X;
}
}