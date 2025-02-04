#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/list_to_matrix.h>
#include <igl/matlab_format.h>
#include <igl/AABB.h>
#include <igl/screen_space_selection.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/SelectionWidget.h>

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

int main(int argc, char *argv[])
{
	// Load the mesh
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	const char* filepath = argc > 1 ? argv[1] : ASSET_FILEPATH(elephant.obj);
	if (!igl::read_triangle_mesh(filepath, V, F))
	{
		__debugbreak();
		return -1;
	}

	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
	viewer.plugins.push_back(&imgui_plugin);
	igl::opengl::glfw::imgui::SelectionWidget selection_widget;
	imgui_plugin.widgets.push_back(&selection_widget);

	Eigen::VectorXd W = Eigen::VectorXd::Zero(V.rows());
	Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(V.rows());
	const Eigen::MatrixXd CM = (Eigen::MatrixXd(2, 3) << 0.3, 0.3, 0.5, 55.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0).finished();
	bool only_visible = false;

	const auto update = [&]()
		{
			const bool was_face_based = viewer.data().face_based;
			viewer.data().set_data(W, 0, 1, igl::COLOR_MAP_TYPE_PLASMA, 2);
			viewer.data().face_based = was_face_based;
			viewer.data().set_colormap(CM);
		};

	igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(V, F);
	viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
		{
			switch (key)
			{
			case ' ': only_visible = !only_visible; update(); return true; // TODO use another key, and display its state in imgui.
			case 'D': case 'd': W.setZero(); update(); return true; // TODO eventually, only if the specific selection widget is toggled on. There is an Anchored selection widget, Transformed selection widget, and default neither.
			}
			return false;
		};
	selection_widget.callback = [&]()
		{
			Eigen::VectorXd old_W;
			if (is_shift_pressed() || is_ctrl_pressed())
				old_W = W;
			screen_space_selection(V, F, tree, viewer.core().view, viewer.core().proj, viewer.core().viewport, selection_widget.L, W, and_visible);
			if (only_visible) { W.array() *= and_visible; }
			if (is_shift_pressed())
			{
				W.array() = old_W.array().max(W.array());
				std::cout << "hi" << std::endl;
			}
			else if (is_ctrl_pressed())
				W = W.binaryExpr(old_W, [](double c, double old_c) { return std::max(old_c - c, 0.0); });
			update();
		};
	std::cout << R"(
Usage:
  [space]  Toggle whether to take visibility into account
  D,d      Clear selection
)";

	// Plot the mesh
	viewer.data().set_mesh(V, F);
	viewer.data().set_face_based(true);
	viewer.core().background_color.head(3) = CM.row(0).head(3).cast<float>();
	viewer.data().line_color.head(3) = (CM.row(0).head(3) * 0.5).cast<float>();
	viewer.data().show_lines = F.rows() < 20000;
	update();

	viewer.launch();
}
