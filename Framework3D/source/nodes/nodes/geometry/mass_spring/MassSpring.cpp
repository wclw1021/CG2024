#include "MassSpring.h"
#include <iostream>
#include <chrono>
#include <algorithm>

namespace USTC_CG::node_mass_spring {
MassSpring::MassSpring(const Eigen::MatrixXd& X, const EdgeSet& E)
{
    this->X = this->init_X = X;
    this->vel = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    this->E = E;

    std::cout << "number of edges: " << E.size() << std::endl;
    std::cout << "init mass spring" << std::endl;

    // Compute the rest pose edge length
    for (const auto& e : E) {
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        this->E_rest_length.push_back((x0 - x1).norm());
    }

    // Initialize the mask for Dirichlet boundary condition
    dirichlet_bc_mask.resize(X.rows(), false);

    // (HW_TODO) Fix two vertices, feel free to modify this 
    unsigned n_fix = sqrt(X.rows());  // Here we assume the cloth is square
    dirichlet_bc_mask[0] = true;
    dirichlet_bc_mask[n_fix - 1] = true;
}

void MassSpring::step()
{
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;

    unsigned n_vertices = X.rows();

    // The reason to not use 1.0 as mass per vertex: the cloth gets heavier as we increase the resolution
    double mass_per_vertex =
        mass / n_vertices; 

    //----------------------------------------------------
    // (HW Optional) Bonus part: Sphere collision
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius);
    //----------------------------------------------------

    if (time_integrator == IMPLICIT_EULER) {
        // Implicit Euler
        TIC(step)

        // (HW TODO) 
        auto H_elastic = computeHessianSparse(stiffness);
        for (int i = 0; i < X.rows(); i++) {
            if (dirichlet_bc_mask[i] == true)
            {
                for (int j = 0; j < 3*X.rows(); j++)
                {
                    H_elastic.coeffRef(3*i, j) = 0;
                    H_elastic.coeffRef(3*i+1, j) = 0;
                    H_elastic.coeffRef(3*i+2, j) = 0;
                }
                H_elastic.coeffRef(3*i, 3*i) = 1;
                H_elastic.coeffRef(3*i+1, 3*i+1) = 1;
                H_elastic.coeffRef(3*i+2, 3*i+2) = 1;
            }
        }
        auto nabla_e = computeGrad(stiffness);

        // compute Y 
        Eigen::MatrixXd acceleration_ext_ = Eigen::MatrixXd::Zero(X.rows(), X.cols());
        acceleration_ext_.rowwise() += acceleration_ext.transpose();
        // if (enable_sphere_collision) {
        //     acceleration_ext_ += acceleration_collision;
        // }
        Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(3 * X.rows(), 1);
        Y = flatten(X + h * vel + pow(h, 2) * acceleration_ext_);

        // Solve Newton's search direction with linear solver 
        Eigen::MatrixXd nabla_g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
        nabla_g = mass_per_vertex / std::pow(h, 2) * (X - unflatten(Y)) + nabla_e;
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(H_elastic);
        Eigen::MatrixXd Delta_X;
        for (int i = 0; i < X.rows(); i++) {
            if (dirichlet_bc_mask[i] == true)
            {
                nabla_g(i, 0) = nabla_g(i, 1) = nabla_g(i, 2) = 0;
            }
        }
        Delta_X = solver.solve(flatten(nabla_g));
        // update X and vel 
        X = unflatten(flatten(X) - Delta_X);
        vel = unflatten(-Delta_X / h);
        TOC(step)
    }
    else if (time_integrator == SEMI_IMPLICIT_EULER) {

        // Semi-implicit Euler
        Eigen::MatrixXd acceleration = -computeGrad(stiffness);
        acceleration.rowwise() += acceleration_ext.transpose();

        // -----------------------------------------------
        // (HW Optional)
        // if (enable_sphere_collision) {
        //     acceleration += acceleration_collision;
        // }
        // -----------------------------------------------

        // (HW TODO): Implement semi-implicit Euler time integration
        for (int i = 0; i < X.rows(); i++) {
            if (dirichlet_bc_mask[i] == true)
            {
                vel.row(i) = Eigen::MatrixXd::Zero(1, X.cols());
                acceleration.row(i) = Eigen::MatrixXd::Zero(1, X.cols());
            }
        }
        // Update X and vel 
        vel = unflatten(flatten(vel) + h * (flatten(acceleration))/mass_per_vertex);
        vel *= damping;
        X = X + h * vel;
    }
    else {
        std::cerr << "Unknown time integrator!" << std::endl;
        return;
    }
}

// There are different types of mass spring energy:
// For this homework we will adopt Prof. Huamin Wang's energy definition introduced in GAMES103
// course Lecture 2 E = 0.5 * stiffness * sum_{i=1}^{n} (||x_i - x_j|| - l)^2 There exist other
// types of energy definition, e.g., Prof. Minchen Li's energy definition
// https://www.cs.cmu.edu/~15769-f23/lec/3_Mass_Spring_Systems.pdf
double MassSpring::computeEnergy(double stiffness)
{
    double sum = 0.;
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        sum += 0.5 * stiffness * std::pow((diff.norm() - l), 2);
        i++;
    }
    return sum;
}
// compute the gradient
Eigen::MatrixXd MassSpring::computeGrad(double stiffness)
{
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        g.row(e.first) += stiffness * (diff.norm() - l) * diff / diff.norm();
        g.row(e.second) += stiffness * (diff.norm() - l) * -diff / diff.norm();
        i++;
    }
    return g;
}

Eigen::SparseMatrix<double> MassSpring::computeHessianSparse(double stiffness)
{
    unsigned n_vertices = X.rows();
    Eigen::SparseMatrix<double> H(n_vertices * 3, n_vertices * 3);

    unsigned i = 0;
    const auto I = Eigen::MatrixXd::Identity(3, 3);
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        Eigen::MatrixXd H_jk;
        int j = e.first;
        int k = e.second;
        auto diff_norm = diff.norm();
        H_jk = stiffness * diff.transpose()* diff / std::pow(diff_norm, 2) +
               stiffness * (1 - l / diff_norm) *
                   (I - diff.transpose() * diff/ std::pow(diff_norm, 2));
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                H.coeffRef(3 * j + row, 3 * j + col) += H_jk(row,col);
                H.coeffRef(3 * k + row, 3 * k + col) += H_jk(row, col);
                H.coeffRef(3 * j + row, 3 * k + col) -= H_jk(row, col);
                H.coeffRef(3 * k + row, 3 * j + col) -= H_jk(row, col);
            }
        }
        // --------------------------------------------------
        // (HW TODO): Implement the sparse version Hessian computation
        // Remember to consider fixed points 
        // You can also consider positive definiteness here
       
        // --------------------------------------------------

        i++;
    }
    double mass_per_vertex = mass / n_vertices; 
    for (int row = 0; row < n_vertices * 3; row++) {
        H.coeffRef(row, row) += mass_per_vertex/ h / h;
    }
    H.makeCompressed();
    return H;
}


bool MassSpring::checkSPD(const Eigen::SparseMatrix<double>& A)
{
    // Eigen::SimplicialLDLT<SparseMatrix_d> ldlt(A);
    // return ldlt.info() == Eigen::Success;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
    auto eigen_values = es.eigenvalues();
    return eigen_values.minCoeff() >= 1e-10;
}

void MassSpring::reset()
{
    std::cout << "reset" << std::endl;
    this->X = this->init_X;
    this->vel.setZero();
}

// ----------------------------------------------------------------------------------
// (HW Optional) Bonus part
Eigen::MatrixXd MassSpring::getSphereCollisionForce(Eigen::Vector3d center, double radius)
{
    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    // float s = 1.1;
    // for (int i = 0; i < X.rows(); i++) {
    //    // (HW Optional) Implement penalty-based force here 
    //     auto diff = X.row(i) - center.transpose(); 
    //     force.row(i) =
    //         collision_penalty_k * std::max(s * radius - diff.norm(), 0.0) * diff / diff.norm();
    // }
    return force;
}
// ----------------------------------------------------------------------------------
 
bool MassSpring::set_dirichlet_bc_mask(const std::vector<bool>& mask)
{
	if (mask.size() == X.rows())
	{
		dirichlet_bc_mask = mask;
		return true;
	}
	else
		return false;
}

bool MassSpring::update_dirichlet_bc_vertices(const MatrixXd &control_vertices)
{
   for (int i = 0; i < dirichlet_bc_control_pair.size(); i++)
   {
       int idx = dirichlet_bc_control_pair[i].first;
	   int control_idx = dirichlet_bc_control_pair[i].second;
	   X.row(idx) = control_vertices.row(control_idx);
   }

   return true; 
}

bool MassSpring::init_dirichlet_bc_vertices_control_pair(const MatrixXd &control_vertices,
    const std::vector<bool>& control_mask)
{
    
	if (control_mask.size() != control_vertices.rows())
			return false; 

   // TODO: optimize this part from O(n) to O(1)
   // First, get selected_control_vertices
   std::vector<VectorXd> selected_control_vertices; 
   std::vector<int> selected_control_idx; 
   for (int i = 0; i < control_mask.size(); i++)
   {
       if (control_mask[i])
       {
			selected_control_vertices.push_back(control_vertices.row(i));
            selected_control_idx.push_back(i);
		}
   }

   // Then update mass spring fixed vertices 
   for (int i = 0; i < dirichlet_bc_mask.size(); i++)
   {
       if (dirichlet_bc_mask[i])
       {
           // O(n^2) nearest point search, can be optimized
           // -----------------------------------------
           int nearest_idx = 0;
           double nearst_dist = 1e6; 
           VectorXd X_i = X.row(i);
           for (int j = 0; j < selected_control_vertices.size(); j++)
           {
               double dist = (X_i - selected_control_vertices[j]).norm();
               if (dist < nearst_dist)
               {
				   nearst_dist = dist;
				   nearest_idx = j;
			   }
           }
           //-----------------------------------------
           
		   X.row(i) = selected_control_vertices[nearest_idx];
           dirichlet_bc_control_pair.push_back(std::make_pair(i, selected_control_idx[nearest_idx]));
	   }
   }

   return true; 
}

}  // namespace USTC_CG::node_mass_spring

