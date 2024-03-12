#include "warping.h"
#include "comp_warping.h"
#include "view/comp_image.h"
#include "warping.h"
#include "fish.h"
#include "IDW.h"
#include "RBF.h"
#include <cmath>

namespace USTC_CG
{
// compute the distance
float Warping::distance(std::pair<float,float> p_1,std::pair<float,float> p_2)
{
    float Distance = std::sqrt((p_1.first-p_2.first) * (p_1.first-p_2.first) + (p_1.second-p_2.second) * (p_1.second-p_2.second));
    return Distance;
}
// change data format
std::vector<std::pair<float,float>> Warping::setP(std::vector<ImVec2> P)
{
    int n = static_cast<int>(P.size());
    std::vector<std::pair<float,float>> P_(n);
    for (int i = 0; i < n; i++)
    {
        P_[i].first = P[i].x;
        P_[i].second = P[i].y;
    }
    return P_;
}
// compute the weight
float Warping::sigma(std::pair<float,float> p_1,std::pair<float,float> p_2)
{
    float Distance = distance(p_1,p_2);
    float Sigma = 1/ pow(Distance,mu);
    return Sigma;
}

//compute the weight vector
std::vector<float> Warping::sigma_vec(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i)
{
    int n = static_cast<int>(P_i.size());
    std::vector<float> Sigma_vec(n);
    for (int i = 0; i < n; i++)
    {
        Sigma_vec[i] = sigma(p,P_i[i]);
    }
    return Sigma_vec;
}

//compute the sum
float Warping::sigma_sum(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i)
{
    int n = static_cast<int>(P_i.size());
    float sum = 0;
    for(int i = 0; i < n; i++)
    {
        sum = sum + sigma(p,P_i[i]);
    }
    return sum;
}

// compute the weight vector
std::vector<float> Warping::omega_vec(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i)
{
    float sum = sigma_sum(p,P_i);
    int n = static_cast<int>(P_i.size()); 
    std::vector<float> Omega_vec(n);
    for (int i = 0; i < n; i++)
    {
        Omega_vec[i] = sigma(p,P_i[i])/sum;
    }
    return Omega_vec;
}
}