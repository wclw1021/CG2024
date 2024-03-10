#pragma once
#include "warping.h"
#include "comp_warping.h"
#include <Eigen/Dense>
namespace USTC_CG
{
class IDW : public Warping
{
   public: 
   IDW(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points) : start_points_(start_points), end_points_(end_points) 
   { 
      startpoints = setP(start_points_);
      endpoints = setP(end_points_);
      D_ = Plus(startpoints,endpoints);
   }
   std::pair<int, int> warp(int x, int y, int width, int height) override;
   
   private:
   std::vector<Eigen::Matrix<float, 2, 2>> Plus(std::vector<std::pair<float,float>> P_i, std::vector<std::pair<float,float>> Q_i);
   
   // the correctness to lower the energy
   std::vector<Eigen::Matrix<float, 2, 2>> D_;

   // some basic input and input change
   std::vector<ImVec2> start_points_, end_points_;
   int number = static_cast<int>(start_points_.size());
   std::vector<std::pair<float,float>> startpoints, endpoints;
};
} //namespace USTC_CG