#include "SelectionSetManager.h"
#include "Main.h"

static const double anchor_color_value = 0.5;
static const double control_color_value = 1.0;

SelectionSetManager::SelectionSetManager(const Eigen::MatrixXd& V)
{
	anchor_set = Eigen::VectorXd::Zero(V.rows());
	control_set = Eigen::VectorXd::Zero(V.rows());
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

Eigen::MatrixXd SelectionSetManager::filter_anchor_vertices(const Eigen::MatrixXd& vertices) const
{
	return filter_vertices(vertices, anchor_set);
}

Eigen::MatrixXd SelectionSetManager::filter_control_vertices(const Eigen::MatrixXd& vertices) const
{
	return filter_vertices(vertices, control_set);
}

Eigen::MatrixXd SelectionSetManager::filter_vertices(const Eigen::MatrixXd& vertices, const Eigen::VectorXd& filter) const
{
	Eigen::VectorXd filtered_indices = indices(filter);
	Eigen::MatrixXd filtered_vertices(filtered_indices.size(), vertices.cols());
	for (int i = 0; i < filtered_indices.size(); ++i)
		filtered_vertices.row(i) = vertices.row(filtered_indices(i));
	return filtered_vertices;
}

Eigen::VectorXi SelectionSetManager::anchor_indices() const
{
	return indices(anchor_set);
}

Eigen::VectorXi SelectionSetManager::control_indices() const
{
	return indices(control_set);
}

Eigen::VectorXi SelectionSetManager::indices(const Eigen::VectorXd& filter) const
{
	Eigen::VectorXi filtered_indices(filter.size());
	Eigen::Index count = 0;
	for (Eigen::Index i = 0; i < filter.size(); ++i)
		if (filter(i))
			filtered_indices(count++) = i;
	filtered_indices.conservativeResize(count);
	return filtered_indices;
}

void SelectionSetManager::update(igl::opengl::ViewerData& data) const
{
	const bool was_face_based = data.face_based;
	data.set_data(anchor_set.array() * anchor_color_value + control_set.array() * control_color_value, 0, 1);
	data.face_based = was_face_based;
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
	if (state == State::ANCHOR)
	{
		Eigen::VectorXd old_set;
		if (is_shift_pressed() || is_ctrl_pressed())
			old_set = anchor_set;
		Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(anchor_set.rows());
		screen_space_select(anchor_set, and_visible);
		if (only_visible)
			anchor_set.array() *= and_visible;
		if (is_shift_pressed())
			anchor_set.array() = old_set.array().max(anchor_set.array());
		else if (is_ctrl_pressed())
			anchor_set = anchor_set.binaryExpr(old_set, [](double c, double old_c) { return std::max(old_c - c, 0.0); });
		control_set = control_set.binaryExpr(anchor_set, [](double control, double anchor) { return std::max(control - anchor, 0.0); });
	}
	else if (state == State::CONTROL)
	{
		Eigen::VectorXd old_set;
		if (is_shift_pressed() || is_ctrl_pressed())
			old_set = control_set;
		Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(control_set.rows());
		screen_space_select(control_set, and_visible);
		if (only_visible)
			control_set.array() *= and_visible;
		if (is_shift_pressed())
			control_set.array() = old_set.array().max(control_set.array());
		else if (is_ctrl_pressed())
			control_set = control_set.binaryExpr(old_set, [](double c, double old_c) { return std::max(old_c - c, 0.0); });
		anchor_set = anchor_set.binaryExpr(control_set, [](double anchor, double control) { return std::max(anchor - control, 0.0); });
	}
}
