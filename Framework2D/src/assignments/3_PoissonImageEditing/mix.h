#pragma once
#include "comp_target_image.h"
#include "cloning.h"
namespace USTC_CG
{
class Mix : public Cloning
{
    public:
    using Cloning::Cloning;
    
    void depict(std::shared_ptr<USTC_CG::Image> mask) override;

    private:

};
}