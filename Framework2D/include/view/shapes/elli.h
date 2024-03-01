#pragma once
#include <imgui.h>
#include "shape.h"

namespace USTC_CG
{
class Elli : public Shape
{
   public:
    Elli() = default;

    // Initialize a ellipse with start and end points
    Elli(
        float start_point_x,
        float start_point_y,
        float end_point_x,
        float end_point_y,
        float angle,
        float width)
        : start_point_x_(start_point_x),
          start_point_y_(start_point_y),
          end_point_x_(end_point_x),
          end_point_y_(end_point_y),
          angle_(angle),
          width_(width)
    {
    }

    virtual ~Elli() = default;

    // Draws the ellipse on the screen
    // Overrides draw function to implement ellipse-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the ellipse size during
    // interaction
    void update(float x, float y) override;

   private:
    // Coordinates of the center, radius of x and y direction
    float start_point_x_ = 0.0f, start_point_y_ = 0.0f;
    float end_point_x_ = 0.0f, end_point_y_ = 0.0f, angle_,width_;
};
}  // namespace USTC_CG