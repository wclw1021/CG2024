#pragma once
#include "warping.h"
namespace USTC_CG
{
class Fish : public Warping
{
   public: 
   std::pair<int, int> warp(int x, int y, int width, int height) override;

   private:

};
} //namespace USTC_CG