#include "SelectionSetManager.h"

#include <igl/read_triangle_mesh.h>
#include <igl/list_to_matrix.h>
#include <igl/matlab_format.h>
#include <igl/AABB.h>
#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#define ASSET_FILEPATH(file) ("../assets/"#file)

bool is_shift_pressed()
{
	return glfwGetKey(__viewer->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
		|| glfwGetKey(__viewer->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool is_ctrl_pressed()
{
	return glfwGetKey(__viewer->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
		|| glfwGetKey(__viewer->window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

int main(int argc, char *argv[])
{
	// Load the mesh
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::string filepath = argc > 1 ? argv[1] : ASSET_FILEPATH(elephant.obj);
	while (!igl::read_triangle_mesh(filepath, V, F))
	{
		std::cerr << "Unable to load mesh at filepath: " << filepath << std::endl;
		std::cout << "Enter the filepath of the mesh you want to test on: " << std::endl;
		if (!(std::cin >> filepath))
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
	}

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
	viewer.plugins.push_back(&imgui_plugin);
	igl::opengl::glfw::imgui::ImGuiMenu menu_widget;
	imgui_plugin.widgets.push_back(&menu_widget);
	igl::opengl::glfw::imgui::SelectionWidget selection_widget;
	imgui_plugin.widgets.push_back(&selection_widget);

	SelectionSetManager selection_sets(V);

	menu_widget.callback_draw_custom_window = [&]()
		{
			ImGui::SetNextWindowPos(ImVec2(180.f * menu_widget.menu_scaling(), 10), ImGuiCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(400, 160), ImGuiCond_FirstUseEver);
			ImGui::Begin("Laplacian Deformation", nullptr, ImGuiWindowFlags_NoSavedSettings);

			ImGui::Text("Selection Set");
			if (ImGui::BeginTabBar("Selection"))
			{
				if (ImGui::BeginTabItem("NEUTRAL"))
				{
					selection_sets.state = SelectionSetManager::State::NEUTRAL;
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("ANCHOR"))
				{
					selection_sets.state = SelectionSetManager::State::ANCHOR;
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("CONTROL"))
				{
					selection_sets.state = SelectionSetManager::State::CONTROL;
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}

			if (ImGui::Checkbox("Select only visible", &selection_sets.only_visible))
				selection_sets.update(viewer.data());
			if (ImGui::Button("Deselect"))
			{
				selection_sets.deselect();
				selection_sets.update(viewer.data());
			}

			ImGui::End();
		};

	igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(V, F);
	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			switch (key)
			{
			case ' ': selection_sets.only_visible = !selection_sets.only_visible; selection_sets.update(viewer.data()); return true; // TODO use another key
			case 'D': case 'd': selection_sets.deselect(); selection_sets.update(viewer.data()); return true;
			}
			return false;
		};
	selection_widget.callback = [&]()
		{
			selection_sets.selection_callback([&](Eigen::VectorXd& set, Eigen::Array<double, Eigen::Dynamic, 1>& and_visible)
				{
					screen_space_selection(V, F, tree, viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible);
				});
			selection_sets.update(viewer.data());
			auto fv = selection_sets.filter_anchor_vertices(V);
			std::cout << fv.rows() << " " << fv.cols() << std::endl;
		};
	std::cout << R"(
Usage:
  [space]  Toggle whether to take visibility into account
  D,d      Clear selection
)";

	// Plot the mesh
	viewer.data().set_mesh(V, F);
	viewer.data().set_face_based(true);
	viewer.data().show_lines = F.rows() < 20000;
	selection_sets.update(viewer.data());

	viewer.launch();
}
