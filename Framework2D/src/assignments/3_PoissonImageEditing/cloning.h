#pragma once
#include "comp_target_image.h"

namespace USTC_CG
{
class Cloning
{
    public:
    Cloning(ImVec2 mouse_position ,
    std::shared_ptr<CompSourceImage> source_image, 
    int image_width,
    int image_height,
    std::shared_ptr<Image> data)
    : mouse_position_(mouse_position),
    source_image_(source_image),
    image_width_(image_width),
    image_height_(image_height),
    data_(data){}

    virtual void depict(std::shared_ptr<USTC_CG::Image> mask) = 0;

    // 
    ImVec2 mouse_position_;
    std::shared_ptr<CompSourceImage> source_image_;
    int image_width_;
    int image_height_;
    std::shared_ptr<Image> data_;
    private:
};
}