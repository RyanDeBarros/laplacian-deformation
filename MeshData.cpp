#include "MeshData.h"

#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>

bool MeshData::init(const std::string& mesh_filepath)
{
	if (igl::read_triangle_mesh(mesh_filepath, vertices, faces))
	{
		tree.init(vertices, faces);
		compute_laplacian_matrix();
		return true;
	}
	return false;
}

void MeshData::compute_laplacian_matrix()
{
	igl::cotmatrix(vertices, faces, laplacian);
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

	Eigen::MatrixXd new_vertices(vertices.rows(), vertices.cols());
	Eigen::SparseMatrix<double> Acombo = A.transpose() * A;
	Eigen::MatrixXd Bcombo = A.transpose() * B;

	// Minimize || A * new_vertices - B || ^ 2
	// --> Solve Acombo * new_vertices = Bcombo
	
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(Acombo);
	vertices = solver.solve(Bcombo);

	compute_laplacian_matrix(); // update laplacian TODO remove or keep ? it's only approximately equal to the previous laplacian matrix.
}
