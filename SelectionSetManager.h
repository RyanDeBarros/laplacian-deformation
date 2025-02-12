#pragma once

#include <igl/opengl/ViewerData.h>

class SelectionSetManager
{
	Eigen::VectorXd selection;

public:
	enum class State
	{
		NEUTRAL,
		ANCHOR,
		CONTROL
	} state = State::NEUTRAL;
	bool only_visible = false;

	SelectionSetManager(const Eigen::MatrixXd& V);

	Eigen::MatrixXd get_colors() const;
	Eigen::MatrixXd filter_anchor_vertices(const Eigen::MatrixXd& vertices) const;
	Eigen::MatrixXd filter_control_vertices(const Eigen::MatrixXd& vertices) const;
	Eigen::MatrixXd filter_vertices(const Eigen::MatrixXd& vertices, State state) const;
	Eigen::VectorXi anchor_indices() const;
	Eigen::VectorXi control_indices() const;
	Eigen::VectorXi indices(State state) const;
	void deselect();
	void selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select);
};
