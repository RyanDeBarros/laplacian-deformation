#pragma once

#include <igl/opengl/ViewerData.h>

class SelectionSetManager
{
	Eigen::VectorXd anchor_set;
	Eigen::VectorXd control_set;

public:
	enum class State
	{
		NEUTRAL,
		ANCHOR,
		CONTROL
	} state = State::NEUTRAL;
	bool only_visible = true;

	SelectionSetManager(const Eigen::MatrixXd& V);

	const Eigen::VectorXd* current_set() const;
	Eigen::MatrixXd filter_anchor_vertices(const Eigen::MatrixXd& vertices) const;
	Eigen::MatrixXd filter_control_vertices(const Eigen::MatrixXd& vertices) const;
	Eigen::MatrixXd filter_vertices(const Eigen::MatrixXd& vertices, const Eigen::VectorXd& filter) const;
	void update(igl::opengl::ViewerData& data) const;
	void deselect();
	void selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select);
};
