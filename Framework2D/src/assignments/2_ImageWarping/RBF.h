#pragma once
#include "warping.h"
#include "comp_warping.h"
#include <Eigen/Dense>
namespace USTC_CG
{
class RBF : public Warping
{
   public: 
   RBF(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points) : start_points_(start_points), end_points_(end_points) 
   { 
      X_ = Solution(setP(start_points), setP(end_points));
      startpoints = setP(start_points_);
      endpoints = setP(end_points_);
   }
   std::pair<int, int> warp(int x, int y, int width, int height) override;

   private:
   float D(std::pair<float,float> p_1, std::pair<float,float> p_2);
   Eigen::MatrixXd Solution(std::vector<std::pair<float,float>> P_i, std::vector<std::pair<float,float>> Q_i);


   std::vector<ImVec2> start_points_, end_points_;
   float r = 10;
   Eigen::MatrixXd X_;
   std::vector<std::pair<float,float>> startpoints, endpoints;
};
} //namespace USTC_CG