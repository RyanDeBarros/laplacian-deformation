#include <igl/read_triangle_mesh.h>
#include <igl/list_to_matrix.h>
#include <igl/matlab_format.h>
#include <igl/AABB.h>
#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#define ASSET_FILEPATH(file) ("../assets/"#file)

static bool is_shift_pressed()
{
	return glfwGetKey(__viewer->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
		|| glfwGetKey(__viewer->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

static bool is_ctrl_pressed()
{
	return glfwGetKey(__viewer->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
		|| glfwGetKey(__viewer->window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

class SelectionSetManager
{
	Eigen::VectorXd anchor_set;
	Eigen::VectorXd control_set;
	Eigen::MatrixXd color_map;

public:
	enum class State
	{
		NEUTRAL,
		ANCHOR,
		CONTROL
	} state = State::NEUTRAL;
	bool only_visible = false;

	SelectionSetManager(const Eigen::MatrixXd& V);

	void update(igl::opengl::ViewerData& data) const;
	const Eigen::VectorXd* current_set() const;
	Eigen::VectorXd* current_set();
	void deselect();
	void selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select);
};

SelectionSetManager::SelectionSetManager(const Eigen::MatrixXd& V)
{
	anchor_set = Eigen::VectorXd::Zero(V.rows());
	control_set = Eigen::VectorXd::Zero(V.rows());
	color_map = (Eigen::MatrixXd(2, 3) << 0.5, 0.3, 0.5, 55.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0).finished();
}

void SelectionSetManager::update(igl::opengl::ViewerData& data) const
{
	const bool was_face_based = data.face_based;
	auto set = current_set();
	if (set)
	{
		data.set_data(*set, 0, 1, igl::COLOR_MAP_TYPE_PLASMA, 2);
		data.face_based = was_face_based;
	}
	data.set_colormap(color_map);
}

const Eigen::VectorXd* SelectionSetManager::current_set() const
{
	if (state == State::ANCHOR)
		return &anchor_set;
	else if (state == State::CONTROL)
		return &control_set;
	else
		return nullptr;
}

Eigen::VectorXd* SelectionSetManager::current_set()
{
	if (state == State::ANCHOR)
		return &anchor_set;
	else if (state == State::CONTROL)
		return &control_set;
	else
		return nullptr;
}

void SelectionSetManager::deselect()
{
	if (state == State::ANCHOR)
		anchor_set.setZero();
	else if (state == State::CONTROL)
		control_set.setZero();
}

void SelectionSetManager::selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select)
{
	auto set = current_set();
	if (set)
	{
		Eigen::VectorXd old_set;
		if (is_shift_pressed() || is_ctrl_pressed())
			old_set = *set;
		Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(set->rows());
		screen_space_select(*set, and_visible);
		if (only_visible) { set->array() *= and_visible; }
		if (is_shift_pressed())
			set->array() = old_set.array().max(set->array());
		else if (is_ctrl_pressed())
			*set = set->binaryExpr(old_set, [](double c, double old_c) { return std::max(old_c - c, 0.0); });
	}
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
				{ screen_space_selection(V, F, tree, viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, set, and_visible); });
			selection_sets.update(viewer.data());
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
