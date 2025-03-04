#include "SelectionSetManager.h"

#include "Main.h"

void SelectionSetManager::init(const Eigen::MatrixXd& V)
{
	selection = Eigen::VectorXd::Zero(V.rows());
}

Eigen::MatrixXd SelectionSetManager::get_colors() const
{
	Eigen::MatrixXd colors(selection.rows(), 3);
	for (Eigen::Index i = 0; i < colors.rows(); ++i)
	{
		if (round(selection(i)) == (int)State::ANCHOR)
			colors.row(i) = vertex_colors::anchor.cast<double>();
		else if (round(selection(i)) == (int)State::CONTROL)
			colors.row(i) = vertex_colors::control.cast<double>();
		else
			colors.row(i) = vertex_colors::neutral.cast<double>();
	}
	return colors;
}

bool SelectionSetManager::exists_controls() const
{
	for (Eigen::Index i = 0; i < selection.rows(); ++i)
	{
		if (round(selection(i)) == (int)State::CONTROL)
			return true;
	}
	return false;
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

void SelectionSetManager::deselect()
{
	if (state != State::NEUTRAL)
	{
		for (auto iter = selection.begin(); iter != selection.end(); ++iter)
			if (round(*iter) == (int)state)
				*iter = 0;
	}
}

bool SelectionSetManager::selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select)
{
	Eigen::Array<double, Eigen::Dynamic, 1> and_visible = Eigen::Array<double, Eigen::Dynamic, 1>::Zero(selection.rows());
	Eigen::VectorXd new_selection;
	screen_space_select(new_selection, and_visible);
	if (only_visible)
		new_selection = (and_visible.array().cast<bool>() && (new_selection.array().abs() > 0.5)).cast<double>();
	bool selection_changed = false;
	for (Eigen::Index i = 0; i < selection.rows(); ++i)
	{
		if (is_shift_pressed())
		{
			if (round(new_selection(i)))
			{
				if (selection(i) != (int)state)
					selection_changed = true;
				selection(i) = (int)state;
			}
		}
		else if (is_ctrl_pressed())
		{
			if (round(new_selection(i)) && round(selection(i)) == (int)state)
			{
				if (selection(i) != 0)
					selection_changed = true;
				selection(i) = 0;
			}
		}
		else
		{
			if (round(new_selection(i)))
			{
				if (selection(i) != (int)state)
					selection_changed = true;
				selection(i) = (int)state;
			}
			else if (round(selection(i)) == (int)state)
			{
				if (selection(i) != 0)
					selection_changed = true;
				selection(i) = 0;
			}
		}
	}
	return selection_changed;
}
