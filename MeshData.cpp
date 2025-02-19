#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/adjacency_list.h>

bool MeshData::load(const std::string& filename)
{
	if (igl::read_triangle_mesh(filename, vertices, faces))
	{
		tree.init(vertices, faces);
		igl::cotmatrix(vertices, faces, laplacian);
		igl::adjacency_list(faces, adjacency);
		rotations.resize(vertices.rows());
		arap.convergence_threshold = 0.001f * vertices.rows();
		return true;
	}
	return false;
}

bool MeshData::save(const std::string& filename)
{
	return igl::write_triangle_mesh(filename, vertices, faces);
}

void MeshData::deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	if (user_constraints.rows() == 0)
		return;

	ldelta = laplacian * vertices;
	Eigen::MatrixXd(MeshData::*solver)(const Eigen::MatrixXd&, const Eigen::VectorXi&);
	if (hard_constraints)
		solver = &MeshData::solve_vertices_hard_constraints;
	else
		solver = &MeshData::solve_vertices;

	std::fill(rotations.begin(), rotations.end(), Eigen::Matrix3d::Identity());
	Eigen::MatrixXd old_vertices = (this->*solver)(user_constraints, user_constraint_indices);
	for (int i = 0; i < arap.max_iterations; ++i)
	{
		solve_rotations(old_vertices);
		old_vertices = (this->*solver)(user_constraints, user_constraint_indices);
		if ((vertices - old_vertices).norm() < std::abs(arap.convergence_threshold))
			break;
	}
}

Eigen::MatrixXd MeshData::solve_vertices(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	Eigen::MatrixXd old_vertices = vertices;

	// construct B

	Eigen::MatrixXd B(ldelta.rows() + user_constraints.rows(), ldelta.cols());
	for (Eigen::Index i = 0; i < ldelta.rows(); ++i)
		B.row(i) = (rotations[i] * ldelta.row(i).transpose()).transpose();
	B.bottomRows(user_constraints.rows()) = user_constraints;

	// construct A
	
	std::vector<Eigen::Triplet<double>> triplets;
	triplets.reserve(laplacian.nonZeros() + user_constraint_indices.rows());
	Eigen::SparseMatrix<double> A(laplacian.rows() + user_constraint_indices.rows(), laplacian.cols());
	for (Eigen::Index i = 0; i < laplacian.outerSize(); ++i)
		for (decltype(laplacian)::InnerIterator iter(laplacian, i); iter; ++iter)
			triplets.emplace_back(iter.row(), iter.col(), iter.value());
	for (Eigen::Index i = 0; i < user_constraint_indices.rows(); ++i)
		triplets.emplace_back(i + laplacian.rows(), user_constraint_indices(i), 1.0);
	A.setFromTriplets(triplets.begin(), triplets.end());

	// Minimize || A * new_vertices - B || ^ 2
	// --> Solve At * A * new_vertices = At * B

	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(A.transpose() * A);
	vertices = solver.solve(A.transpose() * B);
	return old_vertices;
}

Eigen::MatrixXd MeshData::solve_vertices_hard_constraints(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	Eigen::MatrixXd old_vertices = vertices;

	std::vector<Eigen::Triplet<double>> triplets;
	triplets.reserve(laplacian.nonZeros());
	std::unordered_map<Eigen::Index, Eigen::Index> ui_row_map;

	// process hard constraints

	for (Eigen::Index i = 0; i < user_constraint_indices.rows(); ++i)
	{
		Eigen::Index v = user_constraint_indices(i);
		ui_row_map[v] = i;
		triplets.emplace_back(v, v, 1.0);
	}

	// construct B

	Eigen::MatrixXd B(ldelta.rows(), ldelta.cols());
	for (Eigen::Index i = 0; i < ldelta.rows(); ++i)
	{
		auto it = ui_row_map.find(i);
		if (it != ui_row_map.end())
			B.row(i) = user_constraints.row(it->second);
		else
			B.row(i) = (rotations[i] * ldelta.row(i).transpose()).transpose();
	}

	// construct A

	decltype(laplacian) A(laplacian.rows(), laplacian.cols());
	for (Eigen::Index i = 0; i < laplacian.outerSize(); ++i)
		for (decltype(laplacian)::InnerIterator iter(laplacian, i); iter; ++iter)
			if (!ui_row_map.count(iter.row()))
				triplets.emplace_back(iter.row(), iter.col(), iter.value());
	A.setFromTriplets(triplets.begin(), triplets.end());

	// Minimize || A * new_vertices - B || ^ 2
	// --> Solve At * A * new_vertices = At * B

	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(A.transpose() * A);
	vertices = solver.solve(A.transpose() * B);
	return old_vertices;
}

void MeshData::solve_rotations(const Eigen::MatrixXd& old_vertices)
{
	// ARAP extension

	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::RowVector3d center = vertices.row(i);
		Eigen::RowVector3d old_center = old_vertices.row(i);
		Eigen::Matrix3d covariance;
		covariance.setZero();
		for (Eigen::Index j : adjacency[i])
			covariance += laplacian.coeff(i, j) * (old_vertices.row(j) - old_center).transpose() * (vertices.row(j) - center);

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		rotations[i] = V * U.transpose();
		if (rotations[i].determinant() < 0)
		{
			U.col(2) *= -1;
			rotations[i] = V * U.transpose();
		}
	}
}
