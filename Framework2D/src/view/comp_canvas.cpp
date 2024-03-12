#include "view/comp_canvas.h"

#include <cmath>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "imgui.h"
#include "view/shapes/line.h"
#include "view/shapes/rect.h"
#include "view/shapes/elli.h"
#include "view/shapes/elli_parm.h"
#include "view/shapes/poly.h"
#include "view/shapes/free.h"

namespace USTC_CG
{
void Canvas::draw()
{
    draw_background();

    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        mouse_click_event();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        mouse_right_click_event();
    mouse_move_event();
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        mouse_release_event();

    draw_shapes();
}

void Canvas::set_attributes(const ImVec2& min, const ImVec2& size)
{
    canvas_min_ = min;
    canvas_size_ = size;
    canvas_minimal_size_ = size;
    canvas_max_ =
        ImVec2(canvas_min_.x + canvas_size_.x, canvas_min_.y + canvas_size_.y);
}

void Canvas::show_background(bool flag)
{
    show_background_ = flag;
}

void Canvas::set_default()
{
    draw_status_ = false;
    shape_type_ = kDefault;
}

void Canvas::set_line()
{
    draw_status_ = false;
    shape_type_ = kLine;
}

void Canvas::set_rect()
{
    draw_status_ = false;
    shape_type_ = kRect;
}

void Canvas::set_elli()
{
    draw_status_ = false;
    shape_type_ = kEllipse;
}

void Canvas::set_elli_std()
{
    draw_status_ = false;
    shape_type_ = kEllipse_parm;
}

void Canvas::set_poly()
{
    draw_status_ = false;
    shape_type_ = kPolygon;
}

void Canvas::set_free()
{
    draw_status_ = false;
    shape_type_ = kFree;
}

void Canvas::set_width()
{
    draw_status_ = false;
}

void Canvas::clear_shape_list()
{
    shape_list_.clear();
}

void Canvas::draw_background()
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    if (show_background_)
    {
        // Draw background recrangle
        draw_list->AddRectFilled(canvas_min_, canvas_max_, background_color_);
        // Draw background border
        draw_list->AddRect(canvas_min_, canvas_max_, border_color_);
    }
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(canvas_min_);
    ImGui::InvisibleButton(
        label_.c_str(), canvas_size_, ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    is_hovered_ = ImGui::IsItemHovered();
    is_active_ = ImGui::IsItemActive(); 
}

void Canvas::draw_shapes()
{
    Shape::Config s = { .bias = { canvas_min_.x, canvas_min_.y } };
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // ClipRect can hide the drawing content outside of the rectangular area
    draw_list->PushClipRect(canvas_min_, canvas_max_, true);
    for (const auto& shape : shape_list_)
    {
        shape->draw(s);
    }
    if (draw_status_ && current_shape_)
    {
        current_shape_->draw(s);
    }
    draw_list->PopClipRect();
}

void Canvas::mouse_right_click_event() {
    if (shape_type_ == kPolygon && !points_.empty()) {
        draw_status_ = false;
        points_.push_back(points_[0]);
        current_shape_ = std::make_shared<Poly>(points_, width_);
        shape_list_.push_back(current_shape_);
        current_shape_.reset();
        points_.clear();
    }
}

void Canvas::mouse_click_event()
{
    if (shape_type_ == kEllipse_parm)
    {
        draw_status_ = true;
        center_point_ = mouse_pos_in_canvas();
        std::cout << "Set shape to Ellipse by parameter" << std::endl;
        current_shape_ = std::make_shared<Elli_parm>(
            center_point_.x, center_point_.y, radius_x_, radius_y_, angle_,width_);
    }
    else if (shape_type_ == kPolygon)
    {
        draw_status_ = true;
        points_.push_back(mouse_pos_in_canvas());
        current_shape_ = std::make_shared<Poly>(points_, width_);
    } 
    else if (shape_type_ == kFree)
    {
        draw_status_ = true;
        points_.push_back(mouse_pos_in_canvas());
        current_shape_ = std::make_shared<Free>(points_, width_);
    }
    else if (!draw_status_)
    {
        draw_status_ = true;
        start_point_ = end_point_ = mouse_pos_in_canvas();
        switch (shape_type_)
        {
            case USTC_CG::Canvas::kDefault:
            {
                break;
            }
            case USTC_CG::Canvas::kLine:
            {
                current_shape_ = std::make_shared<Line>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y,width_);
                break;
            }
            case USTC_CG::Canvas::kRect:
            {
                current_shape_ = std::make_shared<Rect>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y,rounding_,width_);
                break;
            }
            case USTC_CG::Canvas::kEllipse:
            {
                current_shape_ = std::make_shared<Elli>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y,angle_,width_);
                break;
            } 
            default: break;
        }
    }
    else
    {
        draw_status_ = false;
        if (current_shape_)
        {
            shape_list_.push_back(current_shape_);
            current_shape_.reset();
        }
    }
}

void Canvas::mouse_move_event()
{
    if (draw_status_)
    {
        end_point_ = mouse_pos_in_canvas();
        if (current_shape_)
        {
            current_shape_->update(end_point_.x, end_point_.y);
        }
    }
}

void Canvas::mouse_release_event()
{
    if (shape_type_ == kFree)
    {
        draw_status_ = false;
        if (current_shape_) {
            shape_list_.push_back(current_shape_);
            current_shape_.reset();
        }
        points_.clear();
    }
}

ImVec2 Canvas::mouse_pos_in_canvas() const
{
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 mouse_pos_in_canvas(
        io.MousePos.x - canvas_min_.x, io.MousePos.y - canvas_min_.y);
    return mouse_pos_in_canvas;
}
}  // namespace USTC_CG