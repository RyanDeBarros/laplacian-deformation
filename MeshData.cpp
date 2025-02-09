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
	Eigen::MatrixXd previous_ldelta = laplacian * vertices;
	Eigen::MatrixXd B(previous_ldelta.rows() + user_constraints.size(), previous_ldelta.cols());
	B << previous_ldelta, user_constraints;

	Eigen::SparseMatrix<double> A(laplacian.rows() + user_constraint_indices.size(), laplacian.cols());
	A.topRows(laplacian.rows()) = laplacian;
	Eigen::MatrixXd addon(user_constraint_indices.size(), laplacian.cols());
	for (Eigen::Index i = 0; i < user_constraint_indices.size(); ++i)
		addon.row(i).setZero()(user_constraint_indices(i)) = 1.0;
	A.bottomRows(addon.rows()) = addon;

	Eigen::MatrixXd new_vertices(vertices.rows(), vertices.cols());
	Eigen::SparseMatrix<double> Acombo = A.transpose() * A;
	Eigen::VectorXd Bcombo = A.transpose() * B;

	// Minimize || A * new_vertices - B || ^ 2
	// --> Solve Acombo * new_vertices = Bcombo
	
	// TODO

	compute_laplacian_matrix(); // update laplacian TODO remove or keep ? it's only approximately equal to the previous laplacian matrix.
}
