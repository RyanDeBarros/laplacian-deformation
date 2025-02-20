#pragma once

#include <igl/opengl/ViewerCore.h>
#include <igl/AABB.h>

class MeshData
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi faces;
	igl::AABB<Eigen::MatrixXd, 3> tree;
	Eigen::SparseMatrix<double> laplacian;
	std::vector<std::vector<Eigen::Index>> adjacency;
	std::vector<Eigen::Matrix3d> rotations;
	Eigen::MatrixXd ldelta;

public:
	struct
	{
		int max_iterations = 4;
		float convergence_threshold = 1.0f;
	} arap;
	bool hard_constraints = true;

	const Eigen::MatrixXd& get_vertices() const { return vertices; }
	const Eigen::MatrixXi& get_faces() const { return faces; }
	const igl::AABB<Eigen::MatrixXd, 3>& get_tree() const { return tree; }

	bool load(const std::string& filename);
	bool save(const std::string& filename);
	
	bool recompute_solver = true;
	void deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices);

private:
	struct
	{
		Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
		Eigen::SparseMatrix<double> A_transpose;
	} deform_cache;
	Eigen::MatrixXd solve_vertices(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices);
	Eigen::MatrixXd solve_vertices_hard_constraints(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices);
	void solve_rotations(const Eigen::MatrixXd& old_vertices);
};
