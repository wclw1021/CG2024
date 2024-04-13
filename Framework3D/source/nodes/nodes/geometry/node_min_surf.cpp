#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include "Eigen/Sparse"
#include "Eigen/Dense"

/*
** @brief HW4_TutteParameterization
**
** This file presents the basic framework of a "node", which processes inputs
** received from the left and outputs specific variables for downstream nodes to
** use.
** - In the first function, node_declare, you can set up the node's input and
** output variables.
** - The second function, node_exec is the execution part of the node, where we
** need to implement the node's functionality.
** - The third function generates the node's registration information, which
** eventually allows placing this node in the GUI interface.
**
** Your task is to fill in the required logic at the specified locations
** within this template, especially in node_exec.
*/

namespace USTC_CG::node_min_surf {
static void node_min_surf_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");
    b.add_input<decl::Int>("Cotangent weights").min(0).max(1).default_val(0);
    /*
    ** NOTE: You can add more inputs or outputs if necessary. For example, in some cases,
    ** additional information (e.g. other mesh geometry, other parameters) is required to perform
    ** the computation.
    **
    ** Be sure that the input/outputs do not share the same name. You can add one geometry as
    **
    **                b.add_input<decl::Geometry>("Input");
    **
    ** Or maybe you need a value buffer like:
    **
    **                b.add_input<decl::Float1Buffer>("Weights");
    */

    // Output-1: Minimal surface with fixed boundary
    b.add_output<decl::Geometry>("Output");
}

static void node_min_surf_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto is_cotangent = params.get_input<int>("Cotangent weights");
    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
    }
    // throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    
    // construct the sparse equation, we use all the vertex to construct
    int num = static_cast<int>(halfedge_mesh -> n_vertices());
    Eigen::SparseMatrix<double> A(num,num);
    std::vector<Eigen::MatrixXd> b(3,Eigen::MatrixXd::Zero(num, 1));
    if (! is_cotangent)
    {
        for (const auto& vertex_handle : halfedge_mesh->vertices())
        {
            int i = vertex_handle.idx();
            if (vertex_handle.is_boundary())
            {
                A.coeffRef(i, i) = 1;
                for (int k = 0; k < 3; k++)
                {
                    b[k](i) = halfedge_mesh->point(vertex_handle)[k];
                }
            }
            else
            {
                for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
                {
                    A.coeffRef(i, i) += 1;
                    A.coeffRef(i,halfedge_handle.to().idx()) = -1;
                }
            } 
        }
    }
    else
    {
        for (const auto& vertex_handle : halfedge_mesh->vertices())
        {
            int i = vertex_handle.idx();
            A.coeffRef(i, i) = 1;
            if (vertex_handle.is_boundary())
            {
                for (int k = 0; k < 3; k++)
                {
                    b[k](i) = halfedge_mesh->point(vertex_handle)[k];
                }
            }
            else
            {
                const auto& position = halfedge_mesh->point(vertex_handle);
                float omega_sum = 0.0f;
                for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
                {
                    const auto& v1 = halfedge_handle.to();
                    const auto& v2 = halfedge_handle.prev().opp().to();
                    const auto& v3 = halfedge_handle.prev().opp().prev().opp().to();
                    const auto& vec1 = halfedge_mesh->point(v1) - position;
                    const auto& vec2 = halfedge_mesh->point(v3) - position;
                    const auto& vec3 = halfedge_mesh->point(v1) - halfedge_mesh->point(v2);
                    const auto& vec4 = halfedge_mesh->point(v3) - halfedge_mesh->point(v2);
                    float cos_1 = vec1.dot(vec3) / (vec1.norm() * vec3.norm());
                    float cos_2 = vec2.dot(vec4) / (vec2.norm() * vec4.norm());
                    float Omega = 1/tan(acosf(cos_1)) + 1/tan(acosf(cos_2));
                    omega_sum = omega_sum + Omega;
                    A.coeffRef(i,halfedge_handle.prev().opp().to().idx()) = -Omega;
                }
                for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
                {
                    A.coeffRef(i, halfedge_handle.to().idx()) = A.coeffRef(i, halfedge_handle.to().idx()) / omega_sum;
                }
            } 
        }
    }
    
    // record the solution
    std::vector<Eigen::VectorXd> X(3);
    for (int k = 0; k < 3; k++)
    {
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(A);
        X[k] = solver.solve(b[k]);
    }
    // reconstruct
    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        int i = vertex_handle.idx();
        halfedge_mesh->set_point(vertex_handle, {X[0](i),X[1](i), X[2](i)});
    }
    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}

static void node_register()
{
    static NodeTypeInfo ntype;

    strcpy(ntype.ui_name, "Minimal Surface");
    strcpy_s(ntype.id_name, "geom_min_surf");

    geo_node_type_base(&ntype);
    ntype.node_execute = node_min_surf_exec;
    ntype.declare = node_min_surf_declare;
    nodeRegisterType(&ntype);
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_min_surf
