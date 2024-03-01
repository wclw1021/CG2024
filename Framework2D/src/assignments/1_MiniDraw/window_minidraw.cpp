#include "window_minidraw.h"
#include "view/shapes/elli_parm.h"
#include <ImGuiFileDialog.h>
#include "view/comp_canvas.h"
#include <iostream>

namespace USTC_CG
{
MiniDraw::MiniDraw(const std::string& window_name) : Window(window_name)
{
    p_canvas_ = std::make_shared<Canvas>("Cmpt.Canvas");
    p_canvas_->set_line();
}

MiniDraw::~MiniDraw()
{
}

void MiniDraw::draw()
{    
    draw_toolbar();
    if (flag_open_file_dialog_)
        draw_open_image_file_dialog();
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    if (ImGui::Begin(
            "Canvas",
            &flag_show_canvas_view_,
            ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBackground))
    {
        if (ImGui::Button("Line"))
        {
            std::cout << "Set shape to Line" << std::endl;
            p_canvas_->set_line();
        }
        ImGui::SameLine();
        if (ImGui::Button("Rect"))
        {
            ImGui::OpenPopup("Rectangle Parameters");
            std::cout << "Set shape to Rectangle" << std::endl;
            p_canvas_->set_rect();
        }
        if (ImGui::BeginPopup("Rectangle Parameters"))
        {
            ImVec2 center = ImVec2(ImGui::GetIO().DisplaySize.x / 2, ImGui::GetIO().DisplaySize.y / 2);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Text("rounding");
            ImGui::InputFloat("##rounding", &p_canvas_->rounding_);
            if (ImGui::Button("OK"))
            {
                std::cout << " rounding: " << p_canvas_->rounding_ << std::endl;
                ImGui::CloseCurrentPopup();           
                p_canvas_->set_rect();
            }
            ImGui::EndPopup();         
        }
        ImGui::SameLine();
        if (ImGui::Button("Elli"))
        {
            ImGui::OpenPopup("Ellipse Angle");
            std::cout << "Set shape to Ellipse" << std::endl;
            p_canvas_->set_elli();
        }
        if (ImGui::BeginPopup("Ellipse Angle"))
        {
            ImVec2 center = ImVec2(ImGui::GetIO().DisplaySize.x / 2, ImGui::GetIO().DisplaySize.y / 2);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Text("angle");
            ImGui::InputFloat("##angle", &p_canvas_->angle_);
            if (ImGui::Button("OK"))
            {
                std::cout << " angle: " << p_canvas_->angle_ << std::endl;
                ImGui::CloseCurrentPopup();           
                p_canvas_->set_elli();
            }
            ImGui::EndPopup();         
        }
        ImGui::SameLine();
        if (ImGui::Button("Elli_parm"))
        {
            ImGui::OpenPopup("Ellipse Parameters");
        }
        if (ImGui::BeginPopup("Ellipse Parameters"))
        {
            ImVec2 center = ImVec2(ImGui::GetIO().DisplaySize.x / 2, ImGui::GetIO().DisplaySize.y / 2);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Text("radius_x");
            ImGui::InputFloat("##radius_x", &p_canvas_->radius_x_);
            ImGui::Text("radius_y");
            ImGui::InputFloat("##radius_y", &p_canvas_->radius_y_);
            ImGui::Text("angle");
            ImGui::InputFloat("##angle", &p_canvas_->angle_);
            if (ImGui::Button("OK"))
            {
                std::cout << "radius_x: " << p_canvas_->radius_x_ << ", radius_y: " << p_canvas_->radius_y_ << ", angle: " << p_canvas_->angle_ << std::endl;
                ImGui::CloseCurrentPopup();           
                p_canvas_->set_elli_std();
            }
            ImGui::EndPopup();         
        }
        ImGui::SameLine();
        if (ImGui::Button("Poli"))
        {
            std::cout << "Set shape to Poligon" << std::endl;
            p_canvas_->set_poly();
        }
        ImGui::SameLine();
        if (ImGui::Button("Free"))
        {
            std::cout << "Set shape to Freehand" << std::endl;
            p_canvas_->set_free();
        }
        ImGui::SameLine();
        if (ImGui::Button("Width"))
        {
            std::cout << "Set width to Shapes" << std::endl;
            p_canvas_->set_width();
            ImGui::OpenPopup("Width");
        }
        if (ImGui::BeginPopup("Width"))
        {
            ImVec2 center = ImVec2(ImGui::GetIO().DisplaySize.x / 2, ImGui::GetIO().DisplaySize.y / 2);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Text("width");
            ImGui::InputFloat("##width", &p_canvas_->width_);
            if (ImGui::Button("OK"))
            {
                std::cout << " width: " << p_canvas_->width_ << std::endl;
                ImGui::CloseCurrentPopup();           
            }
            ImGui::EndPopup();         
        }
        ImGui::Text("Press left mouse to add shapes.");
        if (p_image_)
            draw_image();
        if (flag_enable_canvas_)
            draw_canvas();
        ImGui::End();     
    }
}

void MiniDraw::draw_toolbar()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open Image File.."))
            {
                flag_open_file_dialog_ = true;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit"))
        {
            ImGui::Checkbox("Enable Canvas", &flag_enable_canvas_);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void MiniDraw::draw_canvas()
{
    const auto& canvas_min = ImGui::GetCursorScreenPos();
    const auto& canvas_size = ImGui::GetContentRegionAvail();

    if (p_image_)
    {
        // Resize the canvas to fit the image
        const auto& image_size = p_image_->get_image_size();
        ImVec2 pos = ImVec2(
            canvas_min.x + canvas_size.x / 2 - image_size.x / 2,
            canvas_min.y + canvas_size.y / 2 - image_size.y / 2);
        p_canvas_->set_attributes(pos, image_size);
        p_canvas_->show_background(false);
    }
    else
    {
        // Fill the window
        const auto& canvas_size = ImGui::GetContentRegionAvail();
        p_canvas_->set_attributes(canvas_min, canvas_size);
        p_canvas_->show_background(true);
    }

    p_canvas_->draw();
}

void MiniDraw::draw_image()
{
    const auto& canvas_min = ImGui::GetCursorScreenPos();
    const auto& canvas_size = ImGui::GetContentRegionAvail();
    const auto& image_size = p_image_->get_image_size();
    // Center the image in the window
    ImVec2 pos = ImVec2(
        canvas_min.x + canvas_size.x / 2 - image_size.x / 2,
        canvas_min.y + canvas_size.y / 2 - image_size.y / 2);
    p_image_->set_position(pos);
    p_image_->draw();
}

void MiniDraw::draw_open_image_file_dialog()
{
    IGFD::FileDialogConfig config; config.path = ".";
    ImGuiFileDialog::Instance()->OpenDialog(
        "ChooseImageOpenFileDlg", "Choose Image File", ".png,.jpg", config);
    if (ImGuiFileDialog::Instance()->Display("ChooseImageOpenFileDlg"))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string filePathName =
                ImGuiFileDialog::Instance()->GetFilePathName();
            std::string label = filePathName;
            p_image_ = std::make_shared<Image>(label, filePathName);
            p_canvas_->clear_shape_list();
        }
        ImGuiFileDialog::Instance()->Close();
        flag_open_file_dialog_ = false;
    }
}
}  // namespace USTC_CG