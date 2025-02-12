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
	std::unique_ptr<SelectionSetManager> selection_sets;

	void run(int argc, char* argv[]);
	void update_selection();
	void deform();
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

	viewer.plugins.push_back(&imgui_plugin);
	imgui_plugin.widgets.push_back(&menu_widget);
	imgui_plugin.widgets.push_back(&gizmo_widget);
	imgui_plugin.widgets.push_back(&selection_widget);

	selection_sets = std::make_unique<SelectionSetManager>(mesh.get_vertices());
	auto previous_on_mode = selection_widget.mode;
	selection_widget.mode = decltype(selection_widget.mode)::OFF;

	selection_widget.callback_post_mode_change = [&](decltype(selection_widget.mode) mode) {
		if (mode != decltype(selection_widget.mode)::OFF)
			previous_on_mode = mode;
		};

	menu_widget.callback_draw_viewer_window = [&]()
		{
			ImGui::SetNextWindowSize(ImVec2(400, 450), ImGuiCond_FirstUseEver);
			ImGui::Begin("Laplacian Deformation", nullptr, ImGuiWindowFlags_NoSavedSettings);

			ImGui::BeginChild("Selection", ImVec2(0, 125), true);
			ImGui::Text("Selection Set");
			if (ImGui::BeginTabBar("Selection"))
			{
				if (ImGui::BeginTabItem("NEUTRAL"))
				{
					if (selection_sets->state != SelectionSetManager::State::NEUTRAL)
					{
						selection_sets->state = SelectionSetManager::State::NEUTRAL;
						if (selection_widget.mode != decltype(selection_widget.mode)::OFF)
							previous_on_mode = selection_widget.mode;
						selection_widget.mode = decltype(selection_widget.mode)::OFF;
						selection_widget.clear();
					}
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("ANCHOR"))
				{
					if (selection_sets->state != SelectionSetManager::State::ANCHOR)
					{
						selection_sets->state = SelectionSetManager::State::ANCHOR;
						selection_widget.mode = previous_on_mode;
						selection_widget.clear();
					}
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("CONTROL"))
				{
					if (selection_sets->state != SelectionSetManager::State::CONTROL)
					{
						selection_sets->state = SelectionSetManager::State::CONTROL;
						selection_widget.mode = previous_on_mode;
						selection_widget.clear();
					}
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}
			if (ImGui::Checkbox("Select only visible", &selection_sets->only_visible))
				update_selection();
			if (ImGui::Button("Deselect"))
			{
				selection_sets->deselect();
				update_selection();
			}
			ImGui::EndChild();

			ImGui::BeginChild("Transform", ImVec2(0, 215), true);
			if (ImGui::CollapsingHeader("Transform Control Set", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::Separator();
				ImGui::Text("Gizmo Type");
				if (ImGui::BeginTabBar("Gizmo Type"))
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
					ImGui::EndTabBar();
				}
				if (ImGui::Button("Reset"))
				{
					gizmo_widget.T = Eigen::Matrix4f::Identity();
				}
			}
			ImGui::EndChild();

			ImGui::BeginChild("Deform", ImVec2(0, 45), true);
			if (ImGui::Button("Execute"))
			{
				deform();
			}
			ImGui::EndChild();

			ImGui::End();
		};
	gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
	gizmo_widget.callback = [&](const Eigen::Matrix4f& gizmo_transform) {
		// TODO eventually execute in real-time
		};
	selection_widget.callback = [&]()
		{
			selection_sets->selection_callback([&](Eigen::VectorXd& set, Eigen::Array<double, Eigen::Dynamic, 1>& and_visible)
				{
					screen_space_selection(mesh.get_vertices(), mesh.get_faces(), mesh.get_tree(), viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible);
				});
			update_selection();
		};

	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			switch (key)
			{
			case ' ': selection_sets->only_visible = !selection_sets->only_visible; update_selection(); return true; // TODO use another key
			case 'D': case 'd': selection_sets->deselect(); update_selection(); return true;
			}
			return false;
		};
	std::cout << R"(
Usage:
  [space]  Toggle whether to take visibility into account
  D,d      Clear selection
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
	viewer.data().set_points(mesh.get_vertices(), selection_sets->get_colors());
	viewer.data().face_based = was_face_based;
}

void LaplacianDeformationTool::deform()
{
	if (gizmo_widget.T != Eigen::Matrix4f::Identity())
	{
		Eigen::MatrixXd anchors = selection_sets->filter_anchor_vertices(mesh.get_vertices());
		Eigen::MatrixXd controls = selection_sets->filter_control_vertices(mesh.get_vertices());
		Eigen::RowVectorXd mean_control_point = controls.colwise().mean();
		Eigen::Matrix4d transform = gizmo_widget.T.cast<double>();
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
		user_constraint_indices << selection_sets->anchor_indices(), selection_sets->control_indices();

		mesh.deform(user_constraints, user_constraint_indices);
		const bool was_face_based = viewer.data().face_based;
		viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
		viewer.data().set_colors(selection_sets->get_colors());
		viewer.data().face_based = was_face_based;
	}
}
