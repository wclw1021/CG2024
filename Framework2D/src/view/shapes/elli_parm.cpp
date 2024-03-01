#include "view/shapes/elli_parm.h"

#include <imgui.h>

namespace USTC_CG
{
// Draw the ellipse using ImGui
void Elli_parm::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddEllipse(  
        ImVec2(
        config.bias[0] + center_point_x_, config.bias[1] + center_point_y_),
        radius_x_,
        radius_y_,
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        angle_,  // No rounding of corners
        ImDrawFlags_None,
        width_);
}

void Elli_parm::update(float x, float y) {}

}