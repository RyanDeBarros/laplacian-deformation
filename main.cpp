#include "SelectionSetManager.h"
#include "MeshData.h"

#include <igl/list_to_matrix.h>
#include <igl/matlab_format.h>
#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>

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

struct ControlTransform
{
	float position[3] = { 0, 0, 0 };
	float rotation[3] = { 0, 0, 0 };
	float scale[3] = { 1, 1, 1 };

	Eigen::Matrix4d matrix() const
	{
		Eigen::Matrix4d m_scale = Eigen::Matrix4d::Identity();
		m_scale(0, 0) = scale[0];
		m_scale(1, 1) = scale[1];
		m_scale(2, 2) = scale[2];

		double rx = rotation[0] * PI / 180;
		double ry = rotation[1] * PI / 180;
		double rz = rotation[2] * PI / 180;

		Eigen::Matrix4d m_rot_x = Eigen::Matrix4d::Identity();
		m_rot_x.block<3, 3>(0, 0) <<
			1,        0,        0,
			0,        cos(rx),  -sin(rx),
			0,        sin(rx),  cos(rx);

		Eigen::Matrix4d m_rot_y = Eigen::Matrix4d::Identity();
		m_rot_y.block<3, 3>(0, 0) <<
			cos(ry),  0,        sin(ry),
			0,        1,        0,
			-sin(ry), 0,        cos(ry);

		Eigen::Matrix4d m_rot_z = Eigen::Matrix4d::Identity();
		m_rot_z.block<3, 3>(0, 0) <<
			cos(rz),  -sin(rz), 0,
			sin(rz),  cos(rz),  0,
			0,        0,        1;

		Eigen::Matrix4d m_rotation = m_rot_z * m_rot_y * m_rot_x;

		Eigen::Matrix4d m_translation = Eigen::Matrix4d::Identity();
		m_translation(0, 3) = position[0];
		m_translation(1, 3) = position[1];
		m_translation(2, 3) = position[2];
		
		return m_translation * m_rotation * m_scale;
	}

	bool is_identity() const
	{
		return !position[0] && !position[1] && !position[2] && !rotation[0] && !rotation[1] && !rotation[2]
			&& scale[0] == 1 && scale[1] == 1 && scale[2] == 1;
	}

	void reset()
	{
		position[0] = 0;
		position[1] = 0;
		position[2] = 0;
		rotation[0] = 0;
		rotation[1] = 0;
		rotation[2] = 0;
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}
};

int main(int argc, char* argv[])
{
	// Load the mesh
	MeshData mesh;
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

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
	viewer.plugins.push_back(&imgui_plugin);
	igl::opengl::glfw::imgui::ImGuiMenu menu_widget;
	imgui_plugin.widgets.push_back(&menu_widget);
	igl::opengl::glfw::imgui::ImGuizmoWidget gizmo_widget;
	imgui_plugin.widgets.push_back(&gizmo_widget);
	igl::opengl::glfw::imgui::SelectionWidget selection_widget;
	imgui_plugin.widgets.push_back(&selection_widget);

	SelectionSetManager selection_sets(mesh.get_vertices());
	ControlTransform control_transform;

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
			ImGui::EndChild();

			ImGui::BeginChild("Transform", ImVec2(0, 215), true);
			if (ImGui::CollapsingHeader("Transform Control Set", ImGuiTreeNodeFlags_DefaultOpen))
			{
				bool modification = false;
				modification |= ImGui::InputFloat3("Position", control_transform.position);
				modification |= ImGui::InputFloat3("Rotation", control_transform.rotation);
				modification |= ImGui::InputFloat3("Scale", control_transform.scale);
				if (modification)
				{
					gizmo_widget.T = control_transform.matrix().cast<float>();
				}

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
					control_transform.reset();
					gizmo_widget.T = control_transform.matrix().cast<float>();
				}
			}
			ImGui::EndChild();

			ImGui::BeginChild("Deform", ImVec2(0, 45), true);
			if (ImGui::Button("Execute"))
			{
				if (!control_transform.is_identity())
				{
					Eigen::MatrixXd anchors = selection_sets.filter_anchor_vertices(mesh.get_vertices());
					Eigen::MatrixXd controls = selection_sets.filter_control_vertices(mesh.get_vertices());
					Eigen::RowVectorXd mean_control_point = controls.colwise().mean();
					for (Eigen::Index i = 0; i < controls.rows(); ++i)
					{
						Eigen::VectorXd point3d = (controls.row(i) - mean_control_point).transpose();
						Eigen::Vector4d point4d(point3d(0), point3d(1), point3d(2), 1.0);
						point4d = control_transform.matrix() * point4d;
						controls.row(i) = Eigen::Vector3d(point4d(0), point4d(1), point4d(2)).transpose() + mean_control_point;
					}
					
					Eigen::MatrixXd user_constraints(anchors.rows() + controls.rows(), anchors.cols());
					user_constraints << anchors, controls;
					Eigen::VectorXi user_constraint_indices(anchors.rows() + controls.rows());
					user_constraint_indices << selection_sets.anchor_indices(), selection_sets.control_indices();
					
					mesh.deform(user_constraints, user_constraint_indices);
					const bool was_face_based = viewer.data().face_based;
					viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
					viewer.data().face_based = was_face_based;
				}
			}
			ImGui::EndChild();

			ImGui::End();
		};
	gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
	gizmo_widget.callback = [&](const Eigen::Matrix4f& gizmo_transform) {
		// TODO
		};
	selection_widget.callback = [&]()
		{
			selection_sets.selection_callback([&](Eigen::VectorXd& set, Eigen::Array<double, Eigen::Dynamic, 1>& and_visible)
				{
					screen_space_selection(mesh.get_vertices(), mesh.get_faces(), mesh.get_tree(), viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible);
				});
			selection_sets.update(viewer.data());
		};

	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			switch (key)
			{
			case ' ': selection_sets.only_visible = !selection_sets.only_visible; selection_sets.update(viewer.data()); return true; // TODO use another key
			case 'D': case 'd': selection_sets.deselect(); selection_sets.update(viewer.data()); return true;
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
	viewer.data().set_face_based(true);
	viewer.data().show_lines = mesh.get_faces().rows() < 20000;
	selection_sets.update(viewer.data());

	viewer.launch(false, "Laplacian Deformation Tool", 1920, 1080); // TODO change window size
}
