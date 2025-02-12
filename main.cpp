#include "Main.h"

#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>

#include "SelectionSetManager.h"
#include "MeshData.h"

#define PI 3.14159265358979323846
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

Eigen::RowVector3d vertex_colors::neutral = Eigen::RowVector3d(0.5, 0.5, 0.5);
Eigen::RowVector3d vertex_colors::anchor = Eigen::RowVector3d(0.9, 0.3, 0.3);
Eigen::RowVector3d vertex_colors::control = Eigen::RowVector3d(0.3, 0.9, 0.3);

struct LaplacianDeformationTool
{
	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
	igl::opengl::glfw::imgui::ImGuiMenu menu_widget;
	igl::opengl::glfw::imgui::ImGuizmoWidget gizmo_widget;
	igl::opengl::glfw::imgui::SelectionWidget selection_widget;

	MeshData mesh;
	SelectionSetManager selection_sets;

	struct
	{
		bool enabled = false;
		bool call_deform = false;
		double last_time_since_deform = 0.0;
		double deform_rate = 1.0 / 10.0;
		Eigen::Matrix4f gizmo_transform = Eigen::Matrix4f::Identity();
	} auto_deform;

	decltype(selection_widget.mode) previous_on_mode;

	void run(int argc, char* argv[]);
	void update_selection();
	void deform();
	void set_selection_state(SelectionSetManager::State state);
	void update_gizmo_transform();
};

int main(int argc, char* argv[])
{
	LaplacianDeformationTool tool;
	tool.run(argc, argv);
}

void LaplacianDeformationTool::run(int argc, char* argv[])
{
	// Load the mesh
	std::string filepath = argc > 1 ? argv[1] : ASSET_FILEPATH(elephant.obj);
	while (!mesh.init(filepath))
	{
		std::cerr << "Unable to load mesh at filepath: " << filepath << std::endl;
		std::cout << "Enter the filepath of the mesh you want to test on: " << std::endl;
		if (!(std::cin >> filepath))
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
	}
	std::cout << "Number of vertices: " << mesh.get_vertices().rows() << std::endl;
	std::cout << "Number of faces: " << mesh.get_faces().rows() << std::endl;

	viewer.plugins.push_back(&imgui_plugin);
	imgui_plugin.widgets.push_back(&menu_widget);
	imgui_plugin.widgets.push_back(&gizmo_widget);
	imgui_plugin.widgets.push_back(&selection_widget);

	selection_sets.init(mesh.get_vertices());
	previous_on_mode = selection_widget.mode;
	selection_widget.mode = decltype(selection_widget.mode)::OFF;

	selection_widget.callback_post_mode_change = [&](decltype(selection_widget.mode) mode) {
		if (mode != decltype(selection_widget.mode)::OFF)
			previous_on_mode = mode;
		};

	bool manually_set_gizmo_operation = false;
	bool manually_set_selection_state = false;
	menu_widget.callback_draw_viewer_window = [&]()
		{
			ImGui::SetNextWindowSize(ImVec2(400, 450), ImGuiCond_FirstUseEver);
			ImGui::Begin("Laplacian Deformation", nullptr, ImGuiWindowFlags_NoSavedSettings);

			ImGui::BeginChild("Selection", ImVec2(0, 125), true);
			ImGui::Text("Selection Set");
			if (ImGui::BeginTabBar("Selection"))
			{
				if (manually_set_selection_state)
				{
					manually_set_selection_state = false;
					if (ImGui::BeginTabItem("NEUTRAL", nullptr, selection_sets.state == SelectionSetManager::State::NEUTRAL ? ImGuiTabItemFlags_SetSelected : 0))
						ImGui::EndTabItem();
					if (ImGui::BeginTabItem("ANCHOR", nullptr, selection_sets.state == SelectionSetManager::State::ANCHOR ? ImGuiTabItemFlags_SetSelected : 0))
						ImGui::EndTabItem();
					if (ImGui::BeginTabItem("CONTROL", nullptr, selection_sets.state == SelectionSetManager::State::CONTROL ? ImGuiTabItemFlags_SetSelected : 0))
						ImGui::EndTabItem();
				}
				else
				{
					if (ImGui::BeginTabItem("NEUTRAL"))
					{
						set_selection_state(SelectionSetManager::State::NEUTRAL);
						ImGui::EndTabItem();
					}
					if (ImGui::BeginTabItem("ANCHOR"))
					{
						set_selection_state(SelectionSetManager::State::ANCHOR);
						ImGui::EndTabItem();
					}
					if (ImGui::BeginTabItem("CONTROL"))
					{
						set_selection_state(SelectionSetManager::State::CONTROL);
						ImGui::EndTabItem();
					}
				}
				ImGui::EndTabBar();
			}
			if (ImGui::Checkbox("Select only visible", &selection_sets.only_visible))
				update_selection();
			if (ImGui::Button("Deselect"))
			{
				selection_sets.deselect();
				update_selection();
			}
			ImGui::EndChild();

			ImGui::BeginChild("Transform", ImVec2(0, 100), true);
			if (ImGui::CollapsingHeader("Transform Control Set", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::Separator();
				ImGui::Text("Gizmo Type");

				if (ImGui::BeginTabBar("Gizmo Type"))
				{
					if (manually_set_gizmo_operation)
					{
						manually_set_gizmo_operation = false;
						if (ImGui::BeginTabItem("TRANSLATE", nullptr, gizmo_widget.operation == ImGuizmo::OPERATION::TRANSLATE ? ImGuiTabItemFlags_SetSelected : 0))
							ImGui::EndTabItem();
						if (ImGui::BeginTabItem("ROTATE", nullptr, gizmo_widget.operation == ImGuizmo::OPERATION::ROTATE ? ImGuiTabItemFlags_SetSelected : 0))
							ImGui::EndTabItem();
						if (ImGui::BeginTabItem("SCALE", nullptr, gizmo_widget.operation == ImGuizmo::OPERATION::SCALE ? ImGuiTabItemFlags_SetSelected : 0))
							ImGui::EndTabItem();
					}
					else
					{
						if (ImGui::BeginTabItem("TRANSLATE"))
						{
							gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
							ImGui::EndTabItem();
						}
						if (ImGui::BeginTabItem("ROTATE"))
						{
							gizmo_widget.operation = ImGuizmo::OPERATION::ROTATE;
							ImGui::EndTabItem();
						}
						if (ImGui::BeginTabItem("SCALE"))
						{
							gizmo_widget.operation = ImGuizmo::OPERATION::SCALE;
							ImGui::EndTabItem();
						}
					}
					ImGui::EndTabBar();
				}
			}
			ImGui::EndChild();

			ImGui::BeginChild("Deform", ImVec2(0, 70), true);
			if (ImGui::Checkbox("Auto-deform", &auto_deform.enabled))
			{
				if (auto_deform.enabled)
				{
					auto_deform.call_deform = false;
					auto_deform.last_time_since_deform = 0.0;
				}
			}
			if (auto_deform.enabled)
			{
				ImGui::InputDouble("Rate (sec/update)", &auto_deform.deform_rate);
			}
			else
			{
				if (ImGui::Button("Execute"))
				{
					update_gizmo_transform();
					deform();
				}
			}
			ImGui::EndChild();

			ImGui::End();
		};
	gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
	gizmo_widget.callback = [&](const Eigen::Matrix4f&) {
		if (auto_deform.enabled && !auto_deform.call_deform)
		{
			double time = glfwGetTime();
			if (time - auto_deform.last_time_since_deform > auto_deform.deform_rate)
			{
				auto_deform.last_time_since_deform = time;
				auto_deform.call_deform = true;
				update_gizmo_transform();
			}
		}
		};
	viewer.callback_pre_draw = [&](decltype(viewer)& viewer) {
		if (auto_deform.enabled && auto_deform.call_deform)
		{
			auto_deform.call_deform = false;
			auto_deform.last_time_since_deform = 0.0;
			deform(); // TODO fix auto-deform
		}
		return false;
		};
	selection_widget.callback = [&]()
		{
			selection_sets.selection_callback([&](Eigen::VectorXd& set, Eigen::Array<double, Eigen::Dynamic, 1>& and_visible)
				{
					screen_space_selection(mesh.get_vertices(), mesh.get_faces(), mesh.get_tree(), viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible);
				});
			update_selection();
		};

	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			switch (key)
			{
			case GLFW_KEY_1:
				if (mod & GLFW_MOD_ALT)
				{
					manually_set_gizmo_operation = true;
					gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
				}
				else
				{
					manually_set_selection_state = true;
					set_selection_state(SelectionSetManager::State::NEUTRAL);
				}
				return true;
			case GLFW_KEY_2:
				if (mod & GLFW_MOD_ALT)
				{
					manually_set_gizmo_operation = true;
					gizmo_widget.operation = ImGuizmo::OPERATION::ROTATE;
				}
				else
				{
					manually_set_selection_state = true;
					set_selection_state(SelectionSetManager::State::ANCHOR);
				}
				return true;
			case GLFW_KEY_3:
				if (mod & GLFW_MOD_ALT)
				{
					manually_set_gizmo_operation = true;
					gizmo_widget.operation = ImGuizmo::OPERATION::SCALE;
				}
				else
				{
					manually_set_selection_state = true;
					set_selection_state(SelectionSetManager::State::CONTROL);
				}
				return true;
			case GLFW_KEY_E:
				if (!auto_deform.enabled)
				{
					update_gizmo_transform();
					deform();
					return true;
				}
			}
			return false;
		};
	std::cout << R"(
Usage:
  1        Selection select NEUTRAL
  2        Selection select ANCHOR
  3        Selection select CONTROL
  ALT+1    Gizmo select TRANSLATE
  ALT+2    Gizmo select ROTATE
  ALT+3    Gizmo select SCALE
  E        Execute deform (if auto-deform is off)
)";

	// Plot the mesh
	viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
	viewer.data().point_size = 10.0f;
	viewer.data().set_face_based(true);
	viewer.data().show_lines = mesh.get_faces().rows() < 20000;
	update_selection();
	viewer.launch(false, "Laplacian Deformation Tool", 1920, 1080); // TODO change window size
}

void LaplacianDeformationTool::update_selection()
{
	const bool was_face_based = viewer.data().face_based;
	viewer.data().set_points(mesh.get_vertices(), selection_sets.get_colors());
	viewer.data().face_based = was_face_based;
}

void LaplacianDeformationTool::deform()
{
	if (auto_deform.gizmo_transform != Eigen::Matrix4f::Identity())
	{
		Eigen::MatrixXd anchors = selection_sets.filter_anchor_vertices(mesh.get_vertices());
		Eigen::MatrixXd controls = selection_sets.filter_control_vertices(mesh.get_vertices());
		Eigen::RowVectorXd mean_control_point = controls.colwise().mean();
		Eigen::Matrix4d transform = auto_deform.gizmo_transform.cast<double>();
		for (Eigen::Index i = 0; i < controls.rows(); ++i)
		{
			Eigen::VectorXd point3d = (controls.row(i) - mean_control_point).transpose();
			Eigen::Vector4d point4d(point3d(0), point3d(1), point3d(2), 1.0);
			point4d = transform * point4d;
			controls.row(i) = Eigen::Vector3d(point4d(0), point4d(1), point4d(2)).transpose() + mean_control_point;
		}

		Eigen::MatrixXd user_constraints(anchors.rows() + controls.rows(), anchors.cols());
		user_constraints << anchors, controls;
		Eigen::VectorXi user_constraint_indices(anchors.rows() + controls.rows());
		user_constraint_indices << selection_sets.anchor_indices(), selection_sets.control_indices();

		mesh.deform(user_constraints, user_constraint_indices);
		const bool was_face_based = viewer.data().face_based;
		viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
		viewer.data().set_points(mesh.get_vertices(), selection_sets.get_colors());
		viewer.data().face_based = was_face_based;
	}
}

void LaplacianDeformationTool::set_selection_state(SelectionSetManager::State state)
{
	if (selection_sets.state != state)
	{
		selection_sets.state = state;
		if (state == decltype(state)::NEUTRAL)
		{
			if (selection_widget.mode != decltype(selection_widget.mode)::OFF)
				previous_on_mode = selection_widget.mode;
			selection_widget.mode = decltype(selection_widget.mode)::OFF;
		}
		else
			selection_widget.mode = previous_on_mode;
		selection_widget.clear();
	}
}

void LaplacianDeformationTool::update_gizmo_transform()
{
	auto_deform.gizmo_transform = gizmo_widget.T * auto_deform.gizmo_transform.inverse();
}
