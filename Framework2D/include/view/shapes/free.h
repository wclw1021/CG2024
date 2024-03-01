#pragma once
#include <imgui.h>
#include "shape.h"
#include <vector>

namespace USTC_CG
{
class Free : public Shape
{
   public:
    Free() = default;

    // Initialize a ellipse with start and end points
    Free(std::vector<ImVec2> points, float width)
    : points_(points),
    width_(width)
    {
    }

    virtual ~Free() = default;

    // Draws the ellipse on the screen
    // Overrides draw function to implement ellipse-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the ellipse size during
    // interaction
    void update(float x, float y) override;

   private:
    // Coordinates of the center, radius of x and y direction
    std::vector<ImVec2> points_;
    float width_;
};
}  // namespace USTC_CG