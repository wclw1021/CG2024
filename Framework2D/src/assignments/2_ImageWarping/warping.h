#include <vector>
#include "imgui.h"
#include "view/comp_image.h"
#pragma once 

namespace USTC_CG
{

class Warping
{
   public:
    virtual std::pair<int, int> warp(int x, int y, int width, int height) = 0;

    // to tranform the data format
    std::vector<std::pair<float,float>> setP(std::vector<ImVec2> P);

    float distance(std::pair<float,float> p_1,std::pair<float,float> p_2);

    // used in IDW and MLS
    float sigma(std::pair<float,float> p_1,std::pair<float,float> p_2);
    std::vector<float> sigma_vec(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i);
    float sigma_sum(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i);
    std::vector<float> omega_vec(std::pair<float,float> p, std::vector<std::pair<float,float>> P_i);

    // parameter but need to be called on
    float mu = 2.0;
   private:
    
    
};
} //namespace USTC_CG