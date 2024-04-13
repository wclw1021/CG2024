#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include <fstream>

/*
** @brief HW5_ARAP_Parameterization
**
** This file presents the basic framework of a "node", which processes inputs
** received from the left and outputs specific variables for downstream nodes to
** use.
**
** - In the first function, node_declare, you can set up the node's input and
** output variables.
**
** - The second function, node_exec is the execution part of the node, where we
** need to implement the node's functionality.
**
** - The third function generates the node's registration information, which
** eventually allows placing this node in the GUI interface.
**
** Your task is to fill in the required logic at the specified locations
** within this template, especially in node_exec.
*/

namespace USTC_CG::node_arap {

std::vector<Eigen::MatrixXd> Local(
    std::vector<std::vector<Eigen::MatrixXd>> X,
    std::vector<std::vector<float>> Theta,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh_ini);

void Update_(
    std::vector<Eigen::MatrixXd> L,
    std::vector<std::vector<Eigen::MatrixXd>> X,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh,
    Eigen::SparseMatrix<double> A,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh_ini);

std::pair<std::vector<std::vector<Eigen::MatrixXd>>, std::vector<std::vector<float>>> ToPlane_(
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh);

static void node_arap_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");
    // Input-2: Initialization
    b.add_input<decl::Geometry>("Initialization");
    // Input-3: iter
    b.add_input<decl::Int>("iter").min(0).max(200);
    // Output-1: u-v coordinates
    b.add_output<decl::Float2Buffer>("OutputUV");
    // Output-2: 2D figure
    b.add_output<decl::Geometry>("Output");
}

static void node_arap_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto ini = params.get_input<GOperandBase>("Initialization");
    auto iter = params.get_input<int>("iter");

    // Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Need Geometry Input.");
    }
    // throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto halfedge_mesh_ini = operand_to_openmesh(&ini);
    // Compute coordinates on the plane
    auto [X, Theta] = ToPlane_(halfedge_mesh);

    // Compute coefficient matrix
    int num = halfedge_mesh->n_vertices();
    Eigen::SparseMatrix<double> A(num, num);
    for (const auto& face_handle : halfedge_mesh_ini->faces()) 
    {
        int id = face_handle.idx();
        for (const auto& halfedge_handle : face_handle.halfedges_ccw()) 
        {
            int i = halfedge_handle.from().idx();
            int j = halfedge_handle.to().idx();
            const auto& v_i = halfedge_handle.from();
            const auto& v_j = halfedge_handle.to();
            const auto& v_ij = halfedge_handle.next().to();
            const auto& vec1 = halfedge_mesh_ini->point(v_i) - halfedge_mesh_ini->point(v_ij);
            const auto& vec2 = halfedge_mesh_ini->point(v_j) - halfedge_mesh_ini->point(v_ij);
            float cos_ = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
            float sin_ = (vec1.cross(vec2) / (vec1.norm() * vec2.norm())).norm();
            float cot_ = 1;
            A.coeffRef(i, i) = A.coeffRef(i, i) + cot_;
            A.coeffRef(j, j) = A.coeffRef(j, j) + cot_;
            A.coeffRef(i, j) = A.coeffRef(i, j) - cot_;
            A.coeffRef(j, i) = A.coeffRef(j, i) - cot_;
        }
    }

    // iterate
    for (int i = 0; i < iter; i++) 
    {
        auto L = Local(X, Theta, halfedge_mesh_ini);
        Update_(L, X, halfedge_mesh, A, halfedge_mesh_ini);
    }
    
    // u-v coordinates
    pxr::VtArray<pxr::GfVec2f> uv_result(halfedge_mesh_ini->n_vertices());

    // for (const auto& vertex_handle : halfedge_mesh_ini->vertices()) 
    // {
    //     int i = vertex_handle.idx();
    //     auto p = halfedge_mesh_ini->point(vertex_handle);
    //     uv_result[i] = {p[0],p[1]};
    // }

    // normalize the u-v coordinates
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& vertex_handle : halfedge_mesh_ini->vertices()) 
    {
        auto p = halfedge_mesh_ini->point(vertex_handle);
        min_x = std::min(min_x, static_cast<float>(p[0]));
        min_y = std::min(min_y, static_cast<float>(p[1]));
        max_x = std::max(max_x, static_cast<float>(p[0]));
        max_y = std::max(max_y, static_cast<float>(p[1]));
    }

    for (const auto& vertex_handle : halfedge_mesh_ini->vertices()) {
        int i = vertex_handle.idx();
        auto p = halfedge_mesh_ini->point(vertex_handle);
        float normalized_x = (p[0] - min_x) / (max_x - min_x);
        float normalized_y = (p[1] - min_y) / (max_y - min_y);
        uv_result[i] = {normalized_x, normalized_y};
    }


    // draw 2D geometry
    auto operand_base = openmesh_to_operand(halfedge_mesh_ini.get());
    auto& output = input;
    output.get_component<MeshComponent>()->vertices =
        operand_base->get_component<MeshComponent>()->vertices;

    // Set the output of the node
    params.set_output("OutputUV", uv_result);
    params.set_output("Output", std::move(output));
}

static void node_register()
{
    static NodeTypeInfo ntype;

    strcpy(ntype.ui_name, "ARAP Parameterization");
    strcpy_s(ntype.id_name, "geom_arap");

    geo_node_type_base(&ntype);
    ntype.node_execute = node_arap_exec;
    ntype.declare = node_arap_declare;
    nodeRegisterType(&ntype);
}

// Local process
std::vector<Eigen::MatrixXd> Local(
    std::vector<std::vector<Eigen::MatrixXd>> X,
    std::vector<std::vector<float>> Theta,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh_ini)
{
    std::vector<Eigen::MatrixXd> L;
    
    int num = static_cast<int>(X.size());
    std::vector<std::vector<Eigen::MatrixXd>> U(num, std::vector<Eigen::MatrixXd>(3));
    for (int k = 0; k < num; k++) 
    {
        auto face_handle = OpenMesh::SmartFaceHandle(k, halfedge_mesh_ini.get());
        std::vector<OpenMesh::SmartVertexHandle> points;
        for (const auto& halfedge_handle : face_handle.halfedges_ccw()) 
        {
            points.push_back(halfedge_handle.to());
        }
        std::vector<Eigen::MatrixXd> u(3, Eigen::MatrixXd::Zero(2, 1));
        for (int id = 0; id < 3; id++) { 
            u[id](0, 0) = halfedge_mesh_ini->point(points[id])[0];
            u[id](1, 0) = halfedge_mesh_ini->point(points[id])[1];
        }
        U[k] = u;
    }
    for (int k = 0 ; k < num ; k++)
    {
        Eigen::MatrixXd S;
        S = Theta[k][0] * (U[k][0] - U[k][1]) * (X[k][0] - X[k][1]).transpose() +
            Theta[k][1] * (U[k][1] - U[k][2]) * (X[k][1] - X[k][2]).transpose() +
            Theta[k][2] * (U[k][2] - U[k][0]) * (X[k][2] - X[k][0]).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd U_ = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::MatrixXd L_t = U_ * V.transpose();
        L.push_back(L_t);
    }  
    return L;
}

// Transformation to plane
std::pair<std::vector<std::vector<Eigen::MatrixXd>>, std::vector<std::vector<float>>> ToPlane_(
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh)
{
    int num = halfedge_mesh->n_faces();
    std::vector<std::vector<Eigen::MatrixXd>> X(num);
    std::vector<std::vector<float>> Theta(num);
    for (const auto& face_handle : halfedge_mesh->faces()) {
        // get each face's coodinates
        int id = face_handle.idx();
        std::vector<OpenMesh::SmartVertexHandle> points;
        for (const auto& halfedge_handle : face_handle.halfedges_ccw()) {
            points.push_back(halfedge_handle.to());
        }
        const auto& v0 = points[0];
        const auto& v1 = points[1];
        const auto& v2 = points[2];
        const auto& vec1 = halfedge_mesh->point(v1) - halfedge_mesh->point(v0);
        const auto& vec2 = halfedge_mesh->point(v2) - halfedge_mesh->point(v0);
        const auto& vec3 = halfedge_mesh->point(v1) - halfedge_mesh->point(v2);

        // The angles and coordinates
        std::vector<float> theta(3);
        float cos_ = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
        float sin_ = (vec1.cross(vec2) / (vec1.norm() * vec2.norm())).norm();
        theta[0] = 1.0 / tan(acosf(cos_));
        std::vector<Eigen::MatrixXd> x(3, Eigen::MatrixXd::Zero(2, 1));
        x[2](0) = vec2.norm() * cos_;
        x[2](1) = vec2.norm() * sin_;
        cos_ = vec1.dot(vec3) / (vec1.norm() * vec3.norm());
        theta[1] = 1.0 / tan(acosf(cos_));
        cos_ = vec2.dot(vec3) / (vec2.norm() * vec3.norm());
        theta[2] = 1.0 / tan(acosf(-cos_));
        Theta[id] = theta;
        x[1](0) = vec1.norm();
        X[id] = x;    
    }
    return { X, Theta };
}

// Global process
void Update_(
    std::vector<Eigen::MatrixXd> L,
    std::vector<std::vector<Eigen::MatrixXd>> X,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh,
    Eigen::SparseMatrix<double> A,
    std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh_ini)
{
    int num = halfedge_mesh->n_vertices();
    std::vector<Eigen::VectorXd> b(2, Eigen::VectorXd::Zero(num));
    for (const auto& face_handle : halfedge_mesh_ini->faces()) {
        int id = face_handle.idx(), t = 0;
        for (const auto& halfedge_handle : face_handle.halfedges_ccw()) {
            int i = halfedge_handle.from().idx();
            int j = halfedge_handle.to().idx();
            const auto& v_i = halfedge_handle.from();
            const auto& v_j = halfedge_handle.to();
            const auto& v_ij = halfedge_handle.next().to();
            const auto& vec1 = halfedge_mesh_ini->point(v_i) - halfedge_mesh_ini->point(v_ij);
            const auto& vec2 = halfedge_mesh_ini->point(v_j) - halfedge_mesh_ini->point(v_ij);
            float cos_ = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
            float sin_ = (vec1.cross(vec2) / (vec1.norm() * vec2.norm())).norm();
            float cot_ = 1;
            for (int k = 0; k < 2; k++) 
            {
                b[k][i] = b[k][i] + (cot_ * L[id] * (X[id][(t + 2) % 3] - X[id][t]))(k, 0);
                b[k][j] = b[k][j] - (cot_ * L[id] * (X[id][(t + 2) % 3] - X[id][t]))(k, 0);
            }
            t++;
        }
    }
    // decompose the matrix
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Failed");
    }
    std::vector<Eigen::VectorXd> Opt(2);
    for (int k = 0; k < 2; k++) {

        Opt[k] = solver.solve(b[k]);
    }
    // reconstruct
    for (const auto& vertex_handle : halfedge_mesh->vertices()) {
        int i = vertex_handle.idx();
        halfedge_mesh_ini->set_point(vertex_handle, { Opt[0](i), Opt[1](i), 0 });
    }
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_arap
