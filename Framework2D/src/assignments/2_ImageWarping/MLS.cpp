#include "MLS.h"
#include "comp_warping.h"
#include <cmath>
#include <Eigen/Dense>
namespace USTC_CG
{
std::pair<int, int>
MLS::warp(int x, int y, int width, int height)
{
    int n = static_cast<int>(start_points_.size());
    std::pair<int, int> p = std::make_pair(x,y);
    std::vector<float> Sigma_vec = sigma_vec(p,startpoints);
    std::vector<Eigen::Matrix<float, 2, 2>> A = A_i( p, Sigma_vec, startpoints);
    std::pair<float,float> p_star = points_star(Sigma_vec, startpoints);
    std::pair<float,float> q_star = points_star(Sigma_vec, endpoints);
    float sum_x = 0, sum_y = 0;
    for (int i = 0; i < n; i++)
    {
        sum_x = sum_x + (endpoints[i].first - q_star.first)*A[i](0,0) + (endpoints[i].second - q_star.second)*A[i](1,0);
        sum_y = sum_y + (endpoints[i].first - q_star.first)*A[i](0,1) + (endpoints[i].second - q_star.second)*A[i](1,1);
    }
    float norm = std::sqrt(sum_x*sum_x+sum_y*sum_y);
    float norm_ = std::sqrt((p.first-p_star.first)*(p.first-p_star.first)+(p.second-p_star.second)*(p.second-p_star.second));
    sum_x = sum_x/norm * norm_ + q_star.first;
    sum_y = sum_y/norm * norm_ + q_star.second;
    int new_x = static_cast<int>(sum_x);
    int new_y = static_cast<int>(sum_y);    
    return std::make_pair(new_x, new_y);
}

// compute the p_star
std::pair<float,float> MLS::points_star(std::vector<float> sigma_vec, std::vector<std::pair<float,float>> P_i)
{
    int n = static_cast<int>(P_i.size());
    float sum_x = 0, sum_y = 0, sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum = sum + sigma_vec[i];
    }
    for (int i = 0; i < n; i++)
    {
        sum_x = sum_x + sigma_vec[i] * P_i[i].first;
        sum_y = sum_y + sigma_vec[i] * P_i[i].second;
    }
    sum_x = sum_x/sum;
    sum_y = sum_y/sum;
    return std::make_pair(sum_x,sum_y);
}

// compute the A_i
std::vector<Eigen::Matrix<float, 2, 2>> MLS::A_i(std::pair<int,int> p, std::vector<float> sigma_vec, std::vector<std::pair<float,float>> P_i)
{
    int n = static_cast<int>(P_i.size());
    std::pair<float,float> p_star = points_star(sigma_vec, P_i);
    std::vector<Eigen::Matrix<float, 2, 2>> A(n);
    for (int i = 0; i < n; i++)
    {
        Eigen::Matrix<float, 2, 2> A_1;
        A_1(0,0) = P_i[i].first - p_star.first;
        A_1(0,1)= P_i[i].second - p_star.second;
        A_1(1,0) = P_i[i].second - p_star.second;
        A_1(1,1) = p_star.first-P_i[i].first;
        Eigen::Matrix<float, 2, 2> A_2;
        A_2(0,0) = p.first-p_star.first;
        A_2(0,1)= p_star.second - p.second;
        A_2(1,0) =  p.second - p_star.second;
        A_2(1,1) = p.first-p_star.first;
        A[i] = sigma_vec[i]*A_1 * A_2;
    }
    return A;
}
}