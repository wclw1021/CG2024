#include "view/shapes/elli.h"

#include <imgui.h>
#include <cmath>
namespace USTC_CG
{
// Draw the ellipse using ImGui
void Elli::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddEllipse(
        ImVec2(
        config.bias[0] + (start_point_x_+end_point_x_)/2, config.bias[1] + (start_point_y_+end_point_y_)/2),
        std::abs(end_point_x_-start_point_x_)/2,
        std::abs(end_point_y_-start_point_y_)/2,
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        angle_,  // No rounding of corners
        ImDrawFlags_None,
        width_);
}

void Elli::update(float x, float y)
{
    end_point_x_ = x;
    end_point_y_ = y;
}
}