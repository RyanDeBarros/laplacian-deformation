#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>

bool MeshData::init(const std::string& mesh_filepath)
{
	if (igl::read_triangle_mesh(mesh_filepath, vertices, faces))
	{
		tree.init(vertices, faces);
		igl::cotmatrix(vertices, faces, laplacian);
		return true;
	}
	return false;
}

void MeshData::deform(const Eigen::MatrixXd& user_constraints, const Eigen::VectorXi& user_constraint_indices)
{
	if (user_constraints.rows() == 0)
		return;

	Eigen::MatrixXd previous_ldelta = laplacian * vertices;
	Eigen::MatrixXd B(previous_ldelta.rows() + user_constraints.rows(), previous_ldelta.cols());
	B << previous_ldelta, user_constraints;

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

	// ARAP iteration

	constexpr int max_iterations = 10; // TODO GUI slider
	constexpr double convergence_threshold = 1e-5; // TODO GUI slider

	for (int iteration = 0; iteration < max_iterations; ++iteration)
	{
		Eigen::MatrixXd prev_vertices = vertices;
		auto rotations = compute_rotations(previous_ldelta);
		
		Eigen::MatrixXd new_B = B;
		for (Eigen::Index i = 0; i < vertices.rows(); ++i)
		{
			Eigen::Vector3d delta_i = previous_ldelta.row(i);
			Eigen::Vector3d rotated_delta = rotations[i] * delta_i;
			new_B.row(i) = rotated_delta.transpose();
		}

		// re-solve
		vertices = solver.solve(A.transpose() * B);
		if ((vertices - prev_vertices).norm() < convergence_threshold) break;
	}
}

std::vector<Eigen::Matrix3d> MeshData::compute_rotations(const Eigen::MatrixXd& ldelta)
{
	std::vector<Eigen::Matrix3d> rotations(vertices.rows(), Eigen::Matrix3d::Identity());
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		// iterate over Laplacian neighbours
		std::vector<Eigen::Vector3d> P, Q;
		for (decltype(laplacian)::InnerIterator iter(laplacian, i); iter; ++iter)
		{
			P.push_back(ldelta.row(iter.row()));
			Q.push_back(vertices.row(iter.row()));
		}
		if (P.empty())
			continue;
		Eigen::MatrixXd Pi(3, P.size()), Qi(3, Q.size());
		for (size_t j = 0; j < P.size(); ++j)
		{
			Pi.col(j) = P[j];
			Qi.col(j) = Q[j];
		}
		// covariance matrix
		Eigen::Matrix3d S = Pi * Qi.transpose();
		// singular value decomposition S=UDV^T
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Matrix3d R = V * U.transpose();
		// handle reflection case
		if (R.determinant() < 0)
		{
			V.col(2) *= -1;
			R = V * U.transpose();
		}
		rotations[i] = R;
	}
	return rotations;
}
