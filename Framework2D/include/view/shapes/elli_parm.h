#pragma once
#include <imgui.h>
#include "shape.h"

namespace USTC_CG
{
class Elli_parm : public Shape {
   public:
    Elli_parm() = default;

    // Initialize a ellipse with start and end points
    Elli_parm(
        float center_point_x,
        float center_point_y,
        float radius_x,
        float radius_y,
        float angle,
        float width)
        : center_point_x_(center_point_x),
          center_point_y_(center_point_y),
          radius_x_(radius_x),
          radius_y_(radius_y),
          angle_(angle),
          width_(width)
    {
    }

    ~Elli_parm() = default;

    // Draws the ellipse on the screen
    // Overrides draw function to implement ellipse-specific drawing logic
    void draw(const Config& config) const override;

    void update(float x, float y) override;

   private:
    // Coordinates of the center, radius of x and y direction
    float center_point_x_ = 0.0f, center_point_y_ = 0.0f;
    float radius_x_ = 0.0f, radius_y_ = 0.0f, angle_ = 0.0f, width_;
};
}  // namespace USTC_CG