/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of Easy3D. If it is useful in your research/work,
*   I would be grateful if you show your appreciation by citing it:
*   ------------------------------------------------------------------
*           Liangliang Nan.
*           Easy3D: a lightweight, easy-to-use, and efficient C++
*           library for processing and rendering 3D data. 2018.
*   ------------------------------------------------------------------
*
*	Easy3D is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	Easy3D is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "viewer_imgui.h"

#include <iostream>

#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/impl/imgui_impl_glfw.h>
#include <3rd_party/imgui/impl/imgui_impl_opengl3.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>


namespace easy3d {

    ImGuiContext* ViewerImGui::context_ = nullptr;

    ViewerImGui::ViewerImGui(
        const std::string& title /* = "Easy3D ImGui Viewer" */,
		int samples /* = 4 */,
		int gl_major /* = 3 */,
		int gl_minor /* = 2 */,
		bool full_screen /* = false */,
		bool resizable /* = true */,
		int depth_bits /* = 24 */,
		int stencil_bits /* = 8 */
	) 
		: Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits)
        , alpha_(0.8f), shadowing_enabled_(false)
	{
	}


    void ViewerImGui::init() {
        Viewer::init();

		if (!context_) {
			// Setup ImGui binding
			IMGUI_CHECKVERSION();

			context_ = ImGui::CreateContext();

			const char* glsl_version = "#version 150";
			ImGui_ImplGlfw_InitForOpenGL(window_, false);
			ImGui_ImplOpenGL3_Init(glsl_version);
			ImGuiIO& io = ImGui::GetIO();
            io.WantCaptureKeyboard = true;
            io.WantTextInput = true;
            io.IniFilename = nullptr;
			ImGui::StyleColorsLight();
			ImGuiStyle& style = ImGui::GetStyle();
			style.FrameRounding = 5.0f;

			// load font
			reload_font();
		}
	}


    double ViewerImGui::pixel_ratio() {
        // Computes pixel ratio for hidpi devices
        int fbo_size[2], win_size[2];
        glfwGetFramebufferSize(window_, &fbo_size[0], &fbo_size[1]);
        glfwGetWindowSize(window_, &win_size[0], &win_size[1]);
        return static_cast<double>(fbo_size[0]) / static_cast<double>(win_size[0]);
    }


    void ViewerImGui::reload_font(int font_size)
	{
		ImGuiIO& io = ImGui::GetIO();
		io.Fonts->Clear();
        io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data, droid_sans_compressed_size, static_cast<float>(font_size * dpi_scaling()));
        io.FontGlobalScale = static_cast<float>(1.0 / pixel_ratio());
        ImGui_ImplOpenGL3_DestroyDeviceObjects();
	}


    void ViewerImGui::post_resize(int w, int h) {
        Viewer::post_resize(w, h);
		if (context_) {
			ImGui::GetIO().DisplaySize.x = float(w);
			ImGui::GetIO().DisplaySize.y = float(h);
		}
	}


    bool ViewerImGui::callback_event_cursor_pos(double x, double y) {
		if (ImGui::GetIO().WantCaptureMouse)
			return true;
		else
			return Viewer::callback_event_cursor_pos(x, y);
	}


    bool ViewerImGui::callback_event_mouse_button(int button, int action, int modifiers) {
		if (ImGui::GetIO().WantCaptureMouse)
			return true;
		else
			return Viewer::callback_event_mouse_button(button, action, modifiers);
	}


    bool ViewerImGui::callback_event_keyboard(int key, int action, int modifiers) {
		if (ImGui::GetIO().WantCaptureKeyboard)
            return true;
		else
			return Viewer::callback_event_keyboard(key, action, modifiers);
	}


    bool ViewerImGui::callback_event_character(unsigned int codepoint) {
		if (ImGui::GetIO().WantCaptureKeyboard)
			return true;
		else
			return Viewer::callback_event_character(codepoint);
	}


    bool ViewerImGui::callback_event_scroll(double dx, double dy) {
		if (ImGui::GetIO().WantCaptureMouse)
			return true;
		else
			return Viewer::callback_event_scroll(dx, dy);
	}


    void ViewerImGui::cleanup() {
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();

		ImGui::DestroyContext(context_);

        Viewer::cleanup();
	}


    void ViewerImGui::pre_draw() {
        ImGui_ImplOpenGL3_NewFrame();  
        ImGui_ImplGlfw_NewFrame();    
        ImGui::NewFrame();   

        Viewer::pre_draw(); 
	}


    void ViewerImGui::post_draw() {
        static bool show_about = false;
		if (show_about) {
            ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Begin("About AdTree", &show_about, ImGuiWindowFlags_NoResize);
			ImGui::Text(
                "AdTree implements the reconstruction method of the following paper\n"
                "\tShenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.\n"
                "\tAdTree: Accurate, Detailed, and Automatic Modeling of Laser-Scanned Trees.\n"
                "\tRemote Sensing. 2019, 11(18), 2074.\n\n"
			);
			ImGui::Separator();
			ImGui::Text(
				"\n"
                "Shenglan Du (dushenglan940128@163.com)\n"
				"Liangliang Nan (liangliang.nan@gmail.com)\n"
                "3D Geoinformation Group, TU Delft\n"
                "https://3d.bk.tudelft.nl"
			);
			ImGui::End();
		}

		static bool show_manual = false;
		if (show_manual) {
			int w, h;
			glfwGetWindowSize(window_, &w, &h);
			ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
            ImGui::Begin("AdTree Manual", &show_manual, ImGuiWindowFlags_NoResize);
            ImGui::Text("%s", usage().c_str());
			ImGui::End();
		}

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
        if (ImGui::BeginMainMenuBar())
		{
            draw_menu_file();

            draw_menu_view();

            draw_menu_reconstruction();

			if (ImGui::BeginMenu("Help"))
			{
                ImGui::MenuItem("Manual", nullptr, &show_manual);
				ImGui::Separator();
                ImGui::MenuItem("About", nullptr, &show_about);
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}
        ImGui::PopStyleVar();

		ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); 

        Viewer::post_draw();
	}


    void ViewerImGui::draw_menu_file() {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open", "Ctrl+O"))
                open();
            if (ImGui::MenuItem("Save branches ...", "Ctrl+S"))
                save();
            if (ImGui::MenuItem("Save skeleton ..."))
                export_skeleton();
            if (ImGui::MenuItem("Save leaves ..."))
                export_leaves();

            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Alt+F4"))
                glfwSetWindowShouldClose(window_, GLFW_TRUE);

            ImGui::EndMenu();
        }
    }


    void ViewerImGui::draw_menu_view() {
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::BeginMenu("Options"))
            {
                ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.50f);

                static int style_idx = 2;
                if (ImGui::Combo("Style", &style_idx, "Classic\0Dark\0Light\0")) {
                    switch (style_idx) {
                    case 0: ImGui::StyleColorsClassic(); break;
                    case 1: ImGui::StyleColorsDark(); break;
                    case 2: ImGui::StyleColorsLight(); break;
                    }
                }
                ImGui::ColorEdit3("Background Color", background_color_.data(), ImGuiColorEditFlags_NoInputs);

                if (ImGui::Checkbox("Showdowing", &shadowing_enabled_))
                    update();

                ImGui::PopItemWidth();
                ImGui::EndMenu();
            }

            ImGui::EndMenu();
        }
    }

    void ViewerImGui::draw_menu_reconstruction() {
        if (ImGui::BeginMenu("Reconstruction"))
        {
            if (ImGui::MenuItem("Reconstruct Skeleton", nullptr))
                reconstruct_skeleton();
            if (ImGui::MenuItem("Add Leaves", nullptr))
                add_leaves();

            ImGui::EndMenu();
        }
    }

}
