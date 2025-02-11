#include "SelectionSetManager.h"
#include "Main.h"

SelectionSetManager::SelectionSetManager(const Eigen::MatrixXd& V)
{
	selection = Eigen::VectorXd::Zero(V.rows());
}

Eigen::MatrixXd SelectionSetManager::filter_anchor_vertices(const Eigen::MatrixXd& vertices) const
{
	return filter_vertices(vertices, State::ANCHOR);
}

Eigen::MatrixXd SelectionSetManager::filter_control_vertices(const Eigen::MatrixXd& vertices) const
{
	return filter_vertices(vertices, State::CONTROL);
}

Eigen::MatrixXd SelectionSetManager::filter_vertices(const Eigen::MatrixXd& vertices, State state) const
{
	Eigen::VectorXi filtered_indices = indices(state);
	Eigen::MatrixXd filtered_vertices(filtered_indices.rows(), vertices.cols());
	for (int i = 0; i < filtered_indices.rows(); ++i)
		filtered_vertices.row(i) = vertices.row(filtered_indices(i));
	return filtered_vertices;
}

Eigen::VectorXi SelectionSetManager::anchor_indices() const
{
	return indices(State::ANCHOR);
}

Eigen::VectorXi SelectionSetManager::control_indices() const
{
	return indices(State::CONTROL);
}

Eigen::VectorXi SelectionSetManager::indices(State state) const
{
	Eigen::VectorXi filtered_indices(selection.rows());
	Eigen::Index count = 0;
	for (Eigen::Index i = 0; i < selection.rows(); ++i)
		if (round(selection(i)) == (int)state)
			filtered_indices(count++) = i;
	filtered_indices.conservativeResize(count);
	return filtered_indices;
}

void SelectionSetManager::update(igl::opengl::ViewerData& data, const Eigen::MatrixXd& vertices)
{
	const bool was_face_based = data.face_based;
	Eigen::RowVector3d v0(0.5, 0.8, 0.5);
	Eigen::RowVector3d v1(0.5, 0.5, 0.8);
	Eigen::RowVector3d v2(0.8, 0.5, 0.5);
	Eigen::MatrixXd colors(vertices.rows(), 3);
	for (Eigen::Index i = 0; i < colors.rows(); ++i)
	{
		if (round(selection(i)) == (int)State::ANCHOR)
			colors.row(i) = v1;
		else if (round(selection(i)) == (int)State::CONTROL)
			colors.row(i) = v2;
		else
			colors.row(i) = v0;
	}
	data.set_points(vertices, colors);
	data.face_based = was_face_based;
}

void SelectionSetManager::deselect()
{
	if (state != State::NEUTRAL)
	{
		for (auto iter = selection.begin(); iter != selection.end(); ++iter)
			if (round(*iter) == (int)state)
				*iter = 0;
	}
}

void SelectionSetManager::selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select)
{
	Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(selection.rows());
	Eigen::VectorXd new_selection;
	screen_space_select(new_selection, and_visible);
	if (only_visible)
		new_selection.array() *= and_visible;
	for (Eigen::Index i = 0; i < selection.rows(); ++i)
	{
		if (is_shift_pressed())
		{
			if (round(new_selection(i)))
				selection(i) = (int)state;
		}
		else if (is_ctrl_pressed())
		{
			if (round(new_selection(i)) && round(selection(i)) == (int)state)
				selection(i) = 0;
		}
		else
		{
			if (round(new_selection(i)))
				selection(i) = (int)state;
			else if (round(selection(i)) == (int)state)
				selection(i) = 0;
		}
	}
}
