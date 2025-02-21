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
		arap.convergence_threshold = 0.0005f * vertices.rows();
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
	void(MeshData::*solver)(const Eigen::MatrixXd&, const Eigen::VectorXi&);
	if (hard_constraints)
		solver = &MeshData::solve_vertices_hard_constraints;
	else
		solver = &MeshData::solve_vertices;

	if (arap.max_iterations > 0)
	{
		recompute_solver = false;
		Eigen::MatrixXd prev_vertices(vertices.rows(), vertices.cols());
		std::fill(rotations.begin(), rotations.end(), Eigen::Matrix3d::Identity());
		setup_arap_original_vectors();
		(this->*solver)(user_constraints, user_constraint_indices);
		for (int i = 0; i < arap.max_iterations; ++i)
		{
			solve_rotations();
			prev_vertices = vertices;
			(this->*solver)(user_constraints, user_constraint_indices);
			if ((vertices - prev_vertices).norm() < std::abs(arap.convergence_threshold))
				break;
		}
	}
	else
		(this->*solver)(user_constraints, user_constraint_indices);
}

void MeshData::solve_vertices(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	// construct B

	Eigen::MatrixXd B(ldelta.rows() + user_constraints.rows(), ldelta.cols());
	for (Eigen::Index i = 0; i < ldelta.rows(); ++i)
		B.row(i) = (rotations[i] * ldelta.row(i).transpose()).transpose();
	B.bottomRows(user_constraints.rows()) = user_constraints;

	if (recompute_solver)
	{
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
		deform_cache.A_transpose = A.transpose();

		// Minimize || A * new_vertices - B || ^ 2
		// --> Solve At * A * new_vertices = At * B

		deform_cache.solver.compute(deform_cache.A_transpose * A);
	}
	vertices = deform_cache.solver.solve(deform_cache.A_transpose * B);
}

void MeshData::solve_vertices_hard_constraints(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
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

	if (recompute_solver)
	{
		// construct A

		decltype(laplacian) A(laplacian.rows(), laplacian.cols());
		for (Eigen::Index i = 0; i < laplacian.outerSize(); ++i)
			for (decltype(laplacian)::InnerIterator iter(laplacian, i); iter; ++iter)
				if (!ui_row_map.count(iter.row()))
					triplets.emplace_back(iter.row(), iter.col(), iter.value());
		A.setFromTriplets(triplets.begin(), triplets.end());
		deform_cache.A_transpose = A.transpose();

		// Minimize || A * new_vertices - B || ^ 2
		// --> Solve At * A * new_vertices = At * B

		deform_cache.solver.compute(deform_cache.A_transpose * A);
	}
	vertices = deform_cache.solver.solve(deform_cache.A_transpose * B);
}

void MeshData::setup_arap_original_vectors()
{
	arap_original_vectors.resize(vertices.rows());
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		auto& original_vectors = arap_original_vectors[i];
		original_vectors.resize(3, adjacency[i].size());
		Eigen::Vector3d original_center = vertices.row(i);
		for (Eigen::Index j = 0; j < adjacency[i].size(); ++j)
			original_vectors.col(j) = vertices.row(adjacency[i][j]).transpose() - original_center;
	}
}

void MeshData::solve_rotations()
{
	// ARAP extension

	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::MatrixXd deformed_vectors(adjacency[i].size(), 3);
		Eigen::RowVector3d deformed_center = vertices.row(i);
		for (Eigen::Index j = 0; j < adjacency[i].size(); ++j)
			deformed_vectors.row(j) = vertices.row(adjacency[i][j]) - deformed_center;

		Eigen::Matrix3d covariance = arap_original_vectors[i] * deformed_vectors;

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
