#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/adjacency_list.h>

bool MeshData::init(const std::string& mesh_filepath)
{
	if (igl::read_triangle_mesh(mesh_filepath, vertices, faces))
	{
		tree.init(vertices, faces);
		igl::cotmatrix(vertices, faces, laplacian);
		igl::adjacency_list(faces, adjacency);
		rotations.resize(vertices.rows());
		return true;
	}
	return false;
}

void MeshData::deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	if (user_constraints.rows() == 0)
		return;

	std::fill(rotations.begin(), rotations.end(), Eigen::Matrix3d::Identity());
	ldelta = laplacian * vertices;
	int arap_max_iterations = 2; // TODO GUI slider
	double arap_convergence_threshold = 1e-5; // TODO GUI slider
	Eigen::MatrixXd old_vertices = solve_vertices(user_constraints, user_constraint_indices);
	for (int i = 0; i < arap_max_iterations; ++i)
	{
		if ((vertices - old_vertices).norm() < std::abs(arap_convergence_threshold))
			break;
		solve_rotations(old_vertices);
		old_vertices = solve_vertices(user_constraints, user_constraint_indices);
	}
}

Eigen::MatrixXd MeshData::solve_vertices(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	Eigen::MatrixXd old_vertices = vertices;

	for (Eigen::Index i = 0; i < ldelta.rows(); ++i)
		ldelta.row(i) = (rotations[i] * ldelta.row(i).transpose()).transpose();
	Eigen::MatrixXd B(ldelta.rows() + user_constraints.rows(), ldelta.cols());
	B << ldelta, user_constraints;

	Eigen::SparseMatrix<double> A(laplacian.rows() + user_constraint_indices.rows(), laplacian.cols());
	A.reserve(laplacian.nonZeros());
	for (Eigen::Index i = 0; i < laplacian.outerSize(); ++i)
		for (decltype(laplacian)::InnerIterator iter(laplacian, i); iter; ++iter)
			A.insert(iter.row(), iter.col()) = iter.value();
	for (Eigen::Index i = 0; i < user_constraint_indices.rows(); ++i)
		A.insert(i + laplacian.rows(), user_constraint_indices(i)) = 1.0;

	// Minimize || A * new_vertices - B || ^ 2
	// --> Solve At * A * new_vertices = At * B

	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(A.transpose() * A);
	vertices = solver.solve(A.transpose() * B);
	return old_vertices;
}

void MeshData::solve_rotations(const Eigen::MatrixXd& old_vertices)
{
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::RowVector3d center = vertices.row(i);
		Eigen::RowVector3d old_center = old_vertices.row(i);
		Eigen::Matrix3d covariance;
		for (Eigen::Index j : adjacency[i])
			covariance += (old_vertices.row(j) - old_center).transpose() * (vertices.row(j) - center);

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
		rotations[i] = svd.matrixU() * svd.matrixV().transpose();
	}
}
