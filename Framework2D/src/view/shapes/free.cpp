#include "view/shapes/free.h"
#include <imgui.h>
#include <vector>

namespace USTC_CG
{
// Draw the line using ImGui
void Free::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    std::vector<ImVec2> bias_points_ (points_);
    
    for (auto& p : bias_points_) {
        p.x += config.bias[0];
        p.y += config.bias[1];
    }

    draw_list->AddPolyline(
        bias_points_.data(),
        (int) bias_points_.size(),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        0,
        width_);
}

void Free::update(float x, float y)
{
    points_.push_back(ImVec2(x,y));
}
}  // namespace USTC_CG