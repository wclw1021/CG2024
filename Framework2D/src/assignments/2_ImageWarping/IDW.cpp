#include "IDW.h"
#include "comp_warping.h"
#include "warping.h"
#include <cmath>
#include <Eigen/Dense>
namespace USTC_CG
{
std::pair<int, int>
IDW::warp(int x, int y, int width, int height)
{
    std::pair<int, int> p = std::make_pair(x,y);
    float sum_x = 0, sum_y = 0;
    std::vector <float> Omega_vec = omega_vec(p,startpoints);
    for (int i = 0; i < number; i++)
    {
        Eigen::Matrix<float, 2, 2> D = D_[i];
        if (distance(p,startpoints[i]) < 1e-6)
        {
            return endpoints[i];
        }
        else 
        {
            sum_x = sum_x + Omega_vec[i] * (endpoints[i].first + D(0,0)*(p.first - startpoints[i].first)+D(0,1)*(p.second - startpoints[i].second));
            sum_y = sum_y + Omega_vec[i] * (endpoints[i].second + D(1,0)*(p.first - startpoints[i].first)+D(1,1)*(p.second - startpoints[i].second));
        }
    }
    int new_x = static_cast<int>(sum_x);
    int new_y = static_cast<int>(sum_y);
    return std::make_pair(new_x, new_y);
}

// compute the correctness
std::vector<Eigen::Matrix<float, 2, 2>> IDW::Plus(std::vector<std::pair<float,float>> P_i, std::vector<std::pair<float,float>> Q_i)
{
    int n = static_cast<int>(P_i.size());
    std::vector<Eigen::Matrix<float, 2, 2>> plus(n);
    for (int i = 0 ; i < n ;i++)
    {
        std::vector<float> P_ij(n);
        P_ij[i] = 0;
        for (int j = 0; j < n; j++)
        {
            if(j != i)
            {
                P_ij[j] = sigma(P_i[i],P_i[j]);
            }   
        }
        Eigen::Matrix<float, 2, 2> Q;
        Q << 0,0,0,0;
        std::vector<Eigen::Matrix<float, 2, 1>> Q_(n);
        for (int j = 0; j < n; j++)
        {
            Q_[j](0) = Q_i[j].first - Q_i[i].first;
            Q_[j](1) = Q_i[j].second - Q_i[i].second;
        }
        Eigen::Matrix<float, 2, 2> P;
        std::vector<Eigen::Matrix<float, 2, 1>> P_(n);
        P << 0,0,0,0;
        for (int j = 0; j < n; j++)
        {
            P_[j](0) = P_i[j].first - P_i[i].first;
            P_[j](1) = P_i[j].second - P_i[i].second;
            Q = Q + P_ij[j]* Q_[j] * P_[j].transpose();
            P = P + P_ij[j]* P_[j] * P_[j].transpose();
        }
        Eigen::Matrix<float, 2, 2> D;
        D = Q * P.inverse();
        plus[i] = D;
    }
    return plus;
}
}