#pragma once

#include <igl/opengl/ViewerData.h>

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

	const Eigen::VectorXd* current_set() const;
	Eigen::MatrixXd parse_anchor_vertices(const Eigen::MatrixXd& vertices) const;
	Eigen::MatrixXd parse_control_vertices(const Eigen::MatrixXd& vertices) const;
	void update(igl::opengl::ViewerData& data) const;
	void deselect();
	void selection_callback(const std::function<void(Eigen::VectorXd&, Eigen::Array<double, Eigen::Dynamic, 1>&)>& screen_space_select);
};
