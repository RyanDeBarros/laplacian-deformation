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
		igl::adjacency_list(faces, adjacency);
		rotations.resize(vertices.rows());
		arap.convergence_threshold = 0.0005f * vertices.rows();
		set_as_reference_mesh();
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

	void(MeshData::*solver)(const Eigen::MatrixXd&, const Eigen::VectorXi&);
	if (hard_constraints)
		solver = &MeshData::solve_vertices_hard_constraints;
	else
		solver = &MeshData::solve_vertices;

	std::fill(rotations.begin(), rotations.end(), Eigen::Matrix3d::Identity());
	(this->*solver)(user_constraints, user_constraint_indices);
	recompute_solver = false;
	if (arap.max_iterations > 0)
	{
		Eigen::MatrixXd prev_vertices(vertices.rows(), vertices.cols());
		for (int i = 0; i < arap.max_iterations; ++i)
		{
			solve_rotations();
			prev_vertices = vertices;
			(this->*solver)(user_constraints, user_constraint_indices);
			if ((vertices - prev_vertices).norm() < std::abs(arap.convergence_threshold))
				break;
		}
	}
}

void MeshData::set_as_reference_mesh()
{
	igl::cotmatrix(vertices, faces, deform_reference.laplacian);
	recompute_solver = true;
	deform_reference.ldelta = deform_reference.laplacian * vertices;
	deform_reference.arap_vectors.resize(vertices.rows());
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		auto& original_vectors = deform_reference.arap_vectors[i];
		original_vectors.resize(3, adjacency[i].size());
		Eigen::Vector3d original_center = vertices.row(i);
		for (Eigen::Index j = 0; j < adjacency[i].size(); ++j)
			original_vectors.col(j) = vertices.row(adjacency[i][j]).transpose() - original_center;
	}
}

void MeshData::solve_vertices(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	// construct B

	Eigen::MatrixXd B(deform_reference.ldelta.rows() + user_constraints.rows(), deform_reference.ldelta.cols());
	for (Eigen::Index i = 0; i < deform_reference.ldelta.rows(); ++i)
		B.row(i) = (rotations[i] * deform_reference.ldelta.row(i).transpose()).transpose();
	B.bottomRows(user_constraints.rows()) = user_constraints;

	if (recompute_solver)
	{
		// construct A
	
		std::vector<Eigen::Triplet<double>> triplets;
		triplets.reserve(deform_reference.laplacian.nonZeros() + user_constraint_indices.rows());
		Eigen::SparseMatrix<double> A(deform_reference.laplacian.rows() + user_constraint_indices.rows(), deform_reference.laplacian.cols());
		for (Eigen::Index i = 0; i < deform_reference.laplacian.outerSize(); ++i)
			for (decltype(deform_reference.laplacian)::InnerIterator iter(deform_reference.laplacian, i); iter; ++iter)
				triplets.emplace_back(iter.row(), iter.col(), iter.value());
		for (Eigen::Index i = 0; i < user_constraint_indices.rows(); ++i)
			triplets.emplace_back(i + deform_reference.laplacian.rows(), user_constraint_indices(i), 1.0);
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
	triplets.reserve(deform_reference.laplacian.nonZeros());
	std::unordered_map<Eigen::Index, Eigen::Index> ui_row_map;

	// process hard constraints

	for (Eigen::Index i = 0; i < user_constraint_indices.rows(); ++i)
	{
		Eigen::Index v = user_constraint_indices(i);
		ui_row_map[v] = i;
		triplets.emplace_back(v, v, 1.0);
	}

	// construct B

	Eigen::MatrixXd B(deform_reference.ldelta.rows(), deform_reference.ldelta.cols());
	for (Eigen::Index i = 0; i < deform_reference.ldelta.rows(); ++i)
	{
		auto it = ui_row_map.find(i);
		if (it != ui_row_map.end())
			B.row(i) = user_constraints.row(it->second);
		else
			B.row(i) = (rotations[i] * deform_reference.ldelta.row(i).transpose()).transpose();
	}

	if (recompute_solver)
	{
		// construct A

		decltype(deform_reference.laplacian) A(deform_reference.laplacian.rows(), deform_reference.laplacian.cols());
		for (Eigen::Index i = 0; i < deform_reference.laplacian.outerSize(); ++i)
			for (decltype(deform_reference.laplacian)::InnerIterator iter(deform_reference.laplacian, i); iter; ++iter)
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

void MeshData::solve_rotations()
{
	// ARAP extension

	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::MatrixXd deformed_vectors(adjacency[i].size(), 3);
		Eigen::RowVector3d deformed_center = vertices.row(i);
		for (Eigen::Index j = 0; j < adjacency[i].size(); ++j)
			deformed_vectors.row(j) = vertices.row(adjacency[i][j]) - deformed_center;

		Eigen::Matrix3d covariance = deform_reference.arap_vectors[i] * deformed_vectors;
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
