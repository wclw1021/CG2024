#include "comp_target_image.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include "cloning.h"
#include "paste.h"
#include "seamless.h"
#include "mix.h"
#include <iostream>

namespace USTC_CG
{
using uchar = unsigned char;

CompTargetImage::CompTargetImage(
    const std::string& label,
    const std::string& filename)
    : ImageEditor(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void CompTargetImage::draw()
{
    // Draw the image
    ImageEditor::draw();
    // Invisible button for interactions
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    bool is_hovered_ = ImGui::IsItemHovered();
    // When the mouse is clicked or moving, we would adapt clone function to
    // copy the selected region to the target.
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        edit_status_ = true;
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        clone();
    }
    if (edit_status_)
    {
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (flag_realtime_updating)
            clone();
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            edit_status_ = false;
        }
    }
}

void CompTargetImage::set_source(std::shared_ptr<CompSourceImage> source)
{
    source_image_ = source;
}

void CompTargetImage::set_realtime(bool flag)
{
    flag_realtime_updating = flag;
}

void CompTargetImage::restore()
{
    *data_ = *back_up_;
    update();
}

void CompTargetImage::set_paste()
{
    clone_type_ = kPaste;
}

void CompTargetImage::set_seamless()
{
    clone_type_ = kSeamless;
}

void CompTargetImage::set_mix()
{
    clone_type_ = kMix;
}

void CompTargetImage::clone()
{
    // The implementation of different types of cloning
    // HW3_TODO: In this function, you should at least implement the "seamless"
    // cloning labeled by `clone_type_ ==kSeamless`.
    //
    // The realtime updating (update when the mouse is moving) is only available
    // when the checkboard is selected. It is required to improve the efficiency
    // of your seamless cloning to achieve realtime editing. (Use decomposition
    // of sparse matrix before solve the linear system)
    if (data_ == nullptr || source_image_ == nullptr ||
        source_image_->get_region() == nullptr)
        return;
    std::shared_ptr<Image> mask = source_image_->get_region();

    switch (clone_type_)
    {
        case USTC_CG::CompTargetImage::kDefault: break;
        case USTC_CG::CompTargetImage::kPaste:
        {
            restore();
            Paste paste(mouse_position_,source_image_,image_width_,image_height_,data_);
            paste.depict(mask);
            break;
        }
        // poisson editing
        case USTC_CG::CompTargetImage::kSeamless:
        {
            restore();
            Seamless seamless(mouse_position_,source_image_,image_width_,image_height_,data_);
            seamless.depict(mask);
            break;
        }
        // mix method, like the formal one
        case USTC_CG::CompTargetImage::kMix:
        {
            restore();
            Mix mix(mouse_position_,source_image_,image_width_,image_height_,data_);
            mix.depict(mask);
            break;
        }
        default: break;
    }

    update();
}

}  // namespace USTC_CG