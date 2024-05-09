#include "FastMassSpring.h"
#include <iostream>
#include <Eigen/Sparse>


namespace USTC_CG::node_mass_spring {
FastMassSpring::FastMassSpring(const Eigen::MatrixXd& X, const EdgeSet& E, const float stiffness, const float h): 
MassSpring(X, E){
    // construct L and J at initialization
    std::cout << "init fast mass spring" << std::endl;

    unsigned n_vertices = X.rows();
    this->stiffness = stiffness; 
    this->h = h; 

    // Eigen::SparseMatrix<double> A(n_vertices * 3, n_vertices * 3);
    // A.setZero();
    // Eigen::MatrixXd L_pre = Eigen::MatrixXd::Zero(n_vertices, n_vertices);
    // for (const auto& e : E) {
    //     auto i = e.first;
    //     auto j = e.second;
    //     L_pre(i, i) += 1;
    //     L_pre(j, j) += 1;
    //     L_pre(i, j) += -1;
    //     L_pre(j, i) += -1;
    // }
    // for (int i = 0; i < n_vertices; ++i) {
    //     for (int j = 0; j < n_vertices; ++j) {
    //         A.block<3, 3>(3 * i, 3 * j) = std::pow(h,2) * L_pre(i,j);
    //     }
    // }
    // Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    // solver.compute(A);
    // (HW Optional) precompute A and prefactorize
    // Note: one thing to take care of: A is related with stiffness, if stiffness changes, A need to be recomputed
}

void FastMassSpring::step()
{
    // (HW Optional) Necessary preparation
    // ...
    for (unsigned iter = 0; iter < max_iter; iter++) {
        // (HW Optional)
        // local_step and global_step alternating solving
        // Eigen::MatrixXd d(E.size(),X.cols());
        // unsigned n_vertices = X.rows();
        // Eigen::MatrixXd J_pre = Eigen::MatrixXd::Zero(n_vertices, n_vertices);
        // d.setZero();
        // unsigned i = 0;
        // for (const auto& e : E) {
        //     int j = e.first;
        //     int k = e.second;
        //     auto diff = X.row(j) - X.row(k);
        //     auto l = E_rest_length[i];
        //     d.row(i) = l * diff / diff.norm();
        //     J_pre(j, i) += 1;
        //     J_pre(k, i) -= 1;
        //     i++;
        // }
        // Eigen::SparseMatrix<double> J(n_vertices * 3, n_vertices * 3);
        // for (int i = 0; i < n_vertices; ++i) {
        //     for (int j = 0; j < n_vertices; ++j) {
        //         J.block<3, 3>(3 * i, 3 * j) = J_pre(i, j);
        //     }
        // }
        // Eigen::MatrixXd b;
        // Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;
        // Eigen::MatrixXd acceleration_ext_ = Eigen::MatrixXd::Zero(X.rows(), X.cols());
        // acceleration_ext_.rowwise() += acceleration_ext.transpose();
        // Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(3 * X.rows(), 1);
        // Y = X + h * vel + pow(h, 2) * flatten(acceleration_ext_);
        // b = std::pow(h, 2) * J * d + Y;
        // FastMassSpring::FastMassSpring(X, E, stiffness);
        
    }
}

}  // namespace USTC_CG::node_mass_spring
