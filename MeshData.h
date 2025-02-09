#pragma once

#include <igl/opengl/ViewerCore.h>
#include <igl/AABB.h>

class MeshData
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi faces;
	igl::AABB<Eigen::MatrixXd, 3> tree;
	Eigen::SparseMatrix<double> laplacian;

public:
	const Eigen::MatrixXd& get_vertices() const { return vertices; }
	const Eigen::MatrixXi& get_faces() const { return faces; }
	const igl::AABB<Eigen::MatrixXd, 3>& get_tree() const { return tree; }

	bool init(const std::string& mesh_filepath);
	void compute_laplacian_matrix();
	void deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices);
};
