#pragma once

//#include <igl/screen_space_selection.h>
#include <Eigen/Core>

extern bool is_shift_pressed();
extern bool is_ctrl_pressed();

namespace vertex_colors
{
	extern Eigen::RowVector3f neutral;
	extern Eigen::RowVector3f anchor;
	extern Eigen::RowVector3f control;
}
