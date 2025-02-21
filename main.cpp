#include "Main.h"

#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>

#include "SelectionSetManager.h"
#include "MeshData.h"

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

Eigen::RowVector3f vertex_colors::neutral = Eigen::RowVector3f(0.5f, 0.5f, 0.5f);
Eigen::RowVector3f vertex_colors::anchor = Eigen::RowVector3f(0.9f, 0.3f, 0.3f);
Eigen::RowVector3f vertex_colors::control = Eigen::RowVector3f(0.3f, 0.9f, 0.3f);

class LaplacianDeformationTool
{
	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
	igl::opengl::glfw::imgui::ImGuiMenu menu_widget;
	igl::opengl::glfw::imgui::ImGuizmoWidget gizmo_widget;
	igl::opengl::glfw::imgui::SelectionWidget selection_widget;

	decltype(selection_widget.mode) previous_on_mode = decltype(selection_widget.mode)::OFF;
	bool manually_set_gizmo_operation = false;
	bool manually_set_selection_state = false;
	bool prev_hard_constraints = false;
	bool set_as_reference_mesh = true;

	MeshData mesh;
	SelectionSetManager selection_sets;

	struct
	{
		bool enabled = true;
		bool call_deform = false;
		double last_time_since_deform = 0.0;
		double deform_per_second = 5.0;
		Eigen::Matrix4f delta_gizmo_transform = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f last_gizmo_transform = Eigen::Matrix4f::Identity();
	} auto_deform;

public:
	void run();
	void init_mesh(const std::string& filename);

private:
	void init_widgets();
	void launch();
	void render_gui();
	bool key_callback(unsigned int key, int mod);
	void open_mesh();
	void save_mesh();
	void selection_changed();
	void update_selection_colors();
	void deform();
	void manual_deform();
	void set_selection_state(SelectionSetManager::State state);
	void sync_auto_deform_enabled();
	void update_gizmo_transform();
	void recenter_gizmo();
	void reset_gizmo_orientation();
};

int main()
{
	LaplacianDeformationTool tool;
	tool.run();
}

static void path_drop_callback(GLFWwindow* window, int count, const char** paths)
{
	if (count > 0)
	{
		auto tool = (LaplacianDeformationTool*)glfwGetWindowUserPointer(window);
		tool->init_mesh(paths[0]);
		glfwFocusWindow(window);
	}
}

void LaplacianDeformationTool::run()
{
	init_widgets();

	selection_widget.callback_post_mode_change = [&](decltype(selection_widget.mode) mode) { if (mode != decltype(selection_widget.mode)::OFF) previous_on_mode = mode; };
	menu_widget.callback_draw_viewer_window = [&]() { render_gui(); };
	gizmo_widget.operation = ImGuizmo::OPERATION::TRANSLATE;
	gizmo_widget.callback = [&](const Eigen::Matrix4f&) {
		if (auto_deform.enabled && !auto_deform.call_deform)
		{
			double time = glfwGetTime();
			if ((time - auto_deform.last_time_since_deform) * auto_deform.deform_per_second > 1.0)
			{
				auto_deform.last_time_since_deform = time;
				auto_deform.call_deform = true;
				update_gizmo_transform();
			}
		}
		};
	viewer.callback_pre_draw = [&](decltype(viewer)&) {
		if (auto_deform.enabled && auto_deform.call_deform)
		{
			auto_deform.call_deform = false;
			auto_deform.last_time_since_deform = 0.0;
			deform();
		}
		return false;
		};
	viewer.callback_mouse_up = [&](decltype(viewer)&, int button, int) {
		if (auto_deform.enabled && button == GLFW_MOUSE_BUTTON_LEFT && ImGuizmo::IsUsing())
			manual_deform(); // last deform
		return false;
		};
	selection_widget.callback = [&]()
		{
			if (selection_sets.selection_callback([&](Eigen::VectorXd& set, Eigen::Array<double, Eigen::Dynamic, 1>&and_visible) {
				screen_space_selection(mesh.get_vertices(), mesh.get_faces(), mesh.get_tree(),
					viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible); }))
			{
				selection_changed();
				recenter_gizmo();
			}
		};
	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			return key_callback(key, mod);
		};
	viewer.callback_init = [this](decltype(viewer)& viewer) {
		glfwSetWindowUserPointer(viewer.window, this);
		glfwSetDropCallback(viewer.window, &path_drop_callback);
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
  A        Toggle auto-deform
  H        Toggle hard constraints for deformation
)";
	launch();
}

void LaplacianDeformationTool::init_mesh(const std::string& filepath)
{
	if (mesh.load(filepath))
	{
		selection_sets.init(mesh.get_vertices());
		std::cout << "Number of vertices: " << mesh.get_vertices().rows() << std::endl;
		std::cout << "Number of faces: " << mesh.get_faces().rows() << std::endl;
		viewer.data().clear();
		viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
		viewer.data().set_points(mesh.get_vertices(), selection_sets.get_colors());
		viewer.data().set_face_based(true);
		gizmo_widget.visible = false;
		set_as_reference_mesh = true;
	}
}

void LaplacianDeformationTool::init_widgets()
{
	viewer.plugins.push_back(&imgui_plugin);
	imgui_plugin.widgets.push_back(&menu_widget);
	imgui_plugin.widgets.push_back(&gizmo_widget);
	imgui_plugin.widgets.push_back(&selection_widget);
	previous_on_mode = selection_widget.mode;
	selection_widget.mode = decltype(selection_widget.mode)::OFF;
}

void LaplacianDeformationTool::launch()
{
	viewer.data().point_size = 8.0f;
	selection_changed();
	gizmo_widget.visible = false;
	prev_hard_constraints = mesh.hard_constraints;
	viewer.launch(false, "Laplacian Deformation Tool", 1920, 1080);
}

void LaplacianDeformationTool::render_gui()
{
	ImGui::SetNextWindowSize(ImVec2(450, 550), ImGuiCond_FirstUseEver);
	ImGui::Begin("Laplacian Deformation", nullptr, ImGuiWindowFlags_NoSavedSettings);

	if (ImGui::Button("Open Mesh"))
		open_mesh();
	ImGui::SameLine();
	if (ImGui::Button("Save Mesh"))
		save_mesh();

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
	ImGui::Checkbox("Select only visible", &selection_sets.only_visible);
	if (ImGui::Button("Deselect"))
	{
		selection_sets.deselect();
		selection_changed();
		if (selection_sets.state == decltype(selection_sets.state)::CONTROL)
			recenter_gizmo();
	}
	ImGui::EndChild();

	ImGui::BeginChild("Transform", ImVec2(0, 100), true);
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
	ImGui::BeginDisabled(gizmo_widget.T.block(0, 0, 3, 3).isIdentity());
	if (ImGui::Button("Reset Gizmo Orientation"))
		reset_gizmo_orientation();
	ImGui::EndDisabled();
	ImGui::EndChild();

	ImGui::BeginChild("Deform", ImVec2(0, 100), true);
	ImGui::Checkbox("Hard constraints", &mesh.hard_constraints);
	if (ImGui::Checkbox("Auto-deform", &auto_deform.enabled))
		sync_auto_deform_enabled();
	if (auto_deform.enabled)
	{
		ImGui::SetNextItemWidth(150);
		if (ImGui::InputDouble("auto-deform rate (update/sec)", &auto_deform.deform_per_second))
			auto_deform.deform_per_second = std::abs(auto_deform.deform_per_second);
	}
	else
	{
		if (ImGui::Button("Execute"))
			manual_deform();
	}
	ImGui::EndChild();

	if (ImGui::CollapsingHeader("Deform settings"))
	{
		ImGui::SetNextItemWidth(200);
		ImGui::SliderInt("max iterations", &mesh.arap.max_iterations, 0, 20);
		ImGui::SetNextItemWidth(200);
		if (ImGui::InputFloat("convergence threshold", &mesh.arap.convergence_threshold))
			mesh.arap.convergence_threshold = std::abs(mesh.arap.convergence_threshold);
		ImGui::BeginDisabled(set_as_reference_mesh);
		if (ImGui::Button("Set as reference mesh"))
		{
			mesh.set_as_reference_mesh();
			set_as_reference_mesh = true;
		}
		ImGui::EndDisabled();
	}

	if (ImGui::CollapsingHeader("Mesh settings"))
	{
		ImGui::SliderFloat("vertex point size", &viewer.data().point_size, 0.0f, 50.0f);
		if (ImGui::CollapsingHeader("Vertex colors"))
		{
			bool color_update = false;
			color_update |= ImGui::ColorPicker3("neutral color", vertex_colors::neutral.data());
			color_update |= ImGui::ColorPicker3("anchor color", vertex_colors::anchor.data());
			color_update |= ImGui::ColorPicker3("control color", vertex_colors::control.data());
			if (color_update)
				update_selection_colors();
		}
	}

	ImGui::End();
}

bool LaplacianDeformationTool::key_callback(unsigned int key, int mod)
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
		if (mod & GLFW_MOD_SHIFT && !auto_deform.enabled)
		{
			manual_deform();
			return true;
		}
		return false;
	case GLFW_KEY_A:
		if (mod & GLFW_MOD_SHIFT)
		{
			auto_deform.enabled = !auto_deform.enabled;
			sync_auto_deform_enabled();
			return true;
		}
		return false;
	case GLFW_KEY_H:
		if (mod & GLFW_MOD_SHIFT)
		{
			mesh.hard_constraints = !mesh.hard_constraints;
			return true;
		}
		return false;
	case GLFW_KEY_R:
		if (mod & GLFW_MOD_SHIFT && !gizmo_widget.T.block(0, 0, 3, 3).isIdentity())
		{
			reset_gizmo_orientation();
			return true;
		}
		return false;
	}
	return false;
}

void LaplacianDeformationTool::open_mesh()
{
	init_mesh(igl::file_dialog_open());
}

void LaplacianDeformationTool::save_mesh()
{
	mesh.save(igl::file_dialog_save());
}

void LaplacianDeformationTool::selection_changed()
{
	mesh.recompute_solver = true;
	update_selection_colors();
}

void LaplacianDeformationTool::update_selection_colors()
{
	viewer.data().set_points(mesh.get_vertices(), selection_sets.get_colors());
}

static Eigen::Vector3f extract_position(const Eigen::Matrix4f& matrix)
{
	return Eigen::Vector3f(matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void LaplacianDeformationTool::deform()
{
	if (auto_deform.delta_gizmo_transform != Eigen::Matrix4f::Identity())
	{
		Eigen::MatrixXd anchors = selection_sets.filter_anchor_vertices(mesh.get_vertices());
		Eigen::MatrixXd controls = selection_sets.filter_control_vertices(mesh.get_vertices());
		Eigen::Vector3d gizmo_position = extract_position(gizmo_widget.T).cast<double>();
		Eigen::Matrix4d transform = auto_deform.delta_gizmo_transform.cast<double>();
		// rotate and scale the control points relative to the gizmo's position
		for (Eigen::Index i = 0; i < controls.rows(); ++i)
		{
			Eigen::Vector3d point3d = controls.row(i).transpose() - gizmo_position;
			Eigen::Vector4d point4d(point3d(0), point3d(1), point3d(2), 1.0);
			point4d = transform * point4d;
			controls.row(i) = Eigen::Vector3d(point4d(0), point4d(1), point4d(2)) + gizmo_position;
		}

		Eigen::MatrixXd user_constraints(anchors.rows() + controls.rows(), anchors.cols());
		user_constraints << anchors, controls;
		Eigen::VectorXi user_constraint_indices(anchors.rows() + controls.rows());
		user_constraint_indices << selection_sets.anchor_indices(), selection_sets.control_indices();

		if (mesh.hard_constraints != prev_hard_constraints)
		{
			prev_hard_constraints = mesh.hard_constraints;
			mesh.recompute_solver = true;
		}
		mesh.deform(user_constraints, user_constraint_indices);
		set_as_reference_mesh = false;
		viewer.data().set_mesh(mesh.get_vertices(), mesh.get_faces());
		viewer.data().set_points(mesh.get_vertices(), selection_sets.get_colors());
	}
}

void LaplacianDeformationTool::manual_deform()
{
	update_gizmo_transform();
	deform();
	recenter_gizmo();
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

void LaplacianDeformationTool::sync_auto_deform_enabled()
{
	if (auto_deform.enabled)
	{
		manual_deform();
		auto_deform.call_deform = false;
		auto_deform.last_time_since_deform = 0.0;
	}
}

void LaplacianDeformationTool::update_gizmo_transform()
{
	Eigen::Vector3f translation = extract_position(gizmo_widget.T) - extract_position(auto_deform.last_gizmo_transform);
	Eigen::Matrix3f rotation_scale = gizmo_widget.T.block(0, 0, 3, 3) * auto_deform.last_gizmo_transform.block(0, 0, 3, 3).inverse();
	auto_deform.delta_gizmo_transform = Eigen::Matrix4f::Identity();
	auto_deform.delta_gizmo_transform.block(0, 3, 3, 1) = translation;
	auto_deform.delta_gizmo_transform.block(0, 0, 3, 3) = rotation_scale;
	auto_deform.last_gizmo_transform = gizmo_widget.T;
}

void LaplacianDeformationTool::recenter_gizmo()
{
	if (selection_sets.exists_controls())
	{
		Eigen::MatrixXd controls = selection_sets.filter_control_vertices(mesh.get_vertices());
		Eigen::Vector3f mean_control = controls.colwise().mean().cast<float>().transpose();
		Eigen::Vector3f position = extract_position(gizmo_widget.T);
		gizmo_widget.T.block(0, 3, 3, 1) = mean_control;
		auto_deform.last_gizmo_transform.block(0, 3, 3, 1) += (mean_control - position);
		gizmo_widget.visible = true;
	}
	else
		gizmo_widget.visible = false;
}

void LaplacianDeformationTool::reset_gizmo_orientation()
{
	gizmo_widget.T.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity();
	if (auto_deform.enabled)
		manual_deform();
}
