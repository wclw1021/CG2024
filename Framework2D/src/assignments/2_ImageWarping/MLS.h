#pragma once
#include "warping.h"
#include "comp_warping.h"
#include <Eigen/Dense>
namespace USTC_CG
{
class MLS : public Warping
{
   public: 
   MLS(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points) : start_points_(start_points), end_points_(end_points) 
   { 
      startpoints = setP(start_points_);
      endpoints = setP(end_points_);
   }
   std::pair<int, int> warp(int x, int y, int width, int height) override;

   private:
   std::pair<float,float> points_star(std::vector<float> sigma_vec, std::vector<std::pair<float,float>> P_i);
   std::vector<Eigen::Matrix<float, 2, 2>> A_i(std::pair<int,int> p, std::vector<float> sigma_vec, std::vector<std::pair<float,float>> P_i);

    // some basic information
   std::vector<ImVec2> start_points_, end_points_;
   std::vector<std::pair<float,float>> startpoints, endpoints;
};
} //namespace USTC_CG