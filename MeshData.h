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
	void deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices);

private:
	std::vector<Eigen::Matrix3d> compute_rotations(const Eigen::MatrixXd& ldelta); // TODO use vector<matrix3d> data member instead of returning new one every time
};
