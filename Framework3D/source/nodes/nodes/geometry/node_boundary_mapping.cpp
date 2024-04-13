#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include <cmath>
#include "Eigen/Sparse"
#include "Eigen/Dense"

/*
** @brief HW4_TutteParameterization
**
** This file contains two nodes whose primary function is to map the boundary of a mesh to a plain
** convex closed curve (circle of square), setting the stage for subsequent Laplacian equation
** solution and mesh parameterization tasks.
**
** Key to this node's implementation is the adept manipulation of half-edge data structures
** to identify and modify the boundary of the mesh.
**
** Task Overview:
** - The two execution functions (node_map_boundary_to_square_exec,
** node_map_boundary_to_circle_exec) require an update to accurately map the mesh boundary to a and
** circles. This entails identifying the boundary edges, evenly distributing boundary vertices along
** the square's perimeter, and ensuring the internal vertices' positions remain unchanged.
** - A focus on half-edge data structures to efficiently traverse and modify mesh boundaries.
*/

namespace USTC_CG::node_boundary_mapping {
static void node_map_boundary_to_circle_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");
    b.add_input<decl::Int>("Cotangent weights").min(0).max(1).default_val(0);
    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_circle_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }
    // throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto is_cotangent = params.get_input<int>("Cotangent weights");
    // find a point in the boundary
    OpenMesh::SmartHalfedgeHandle halfedge_handle_start;
    for (const auto& halfedge_handle : halfedge_mesh ->halfedges())
    {
        if (halfedge_handle.is_boundary())
        {
           halfedge_handle_start = halfedge_handle;
           break;
        }
    }
    // find all the boundary points with sequence
    std::vector<OpenMesh::SmartVertexHandle> boundary_points;
    boundary_points.push_back(halfedge_handle_start.to());
    OpenMesh::SmartHalfedgeHandle handle = halfedge_handle_start.next();
    do
    {
        boundary_points.push_back(handle.to());
        handle = handle.next();
    } while (handle != halfedge_handle_start);

    // calculate the coordinates to make the correct contribution
    float sum = 0.f;
    int bp_num = static_cast<int>(boundary_points.size());
    std::vector<float> bp_distance(bp_num);
    for (int i = 1; i < bp_num; i++)
    {
        const auto& vec = halfedge_mesh->point(boundary_points[i]) - halfedge_mesh->point(boundary_points[i-1]);
        sum = sum + vec.norm();
        bp_distance[i-1] = sum;
    }
    sum = sum + (halfedge_mesh->point(boundary_points[0]) - halfedge_mesh->point(boundary_points[bp_num-1])).norm();
    bp_distance[bp_num-1] = sum;

    // construct the equation
    int num = static_cast<int>(halfedge_mesh -> n_vertices());
    Eigen::SparseMatrix<double> A(num,num);
    std::vector<Eigen::MatrixXd> b(3,Eigen::MatrixXd::Zero(num, 1));  
    b[0](boundary_points[0].idx()) = 1;
    b[1](boundary_points[0].idx()) = 0.5;
    for (int i = 1; i < bp_num; i++)
    {
        int idx = boundary_points[i].idx();
        b[0](idx) = 0.5 + 0.5 * cos(bp_distance[i-1] / sum  * 2 * M_PI);
        b[1](idx) = 0.5 + 0.5 * sin(bp_distance[i-1] / sum  * 2 * M_PI);
    }
    // change the weights
    if (! is_cotangent)
    {
        for (const auto& vertex_handle : halfedge_mesh->vertices())
        {
            int i = vertex_handle.idx();

            if (vertex_handle.is_boundary())
            {
                A.coeffRef(i, i) = 1;
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
                continue;
            }
            const auto& position = halfedge_mesh->point(vertex_handle);
            std::vector<OpenMesh::SmartVertexHandle> near_points;
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
                float Omega = std::abs(1/tan(acosf(cos_1))) + std::abs(1/tan(acosf(cos_2)));
                omega_sum = omega_sum + Omega;
                A.coeffRef(i,halfedge_handle.prev().opp().to().idx()) = -Omega;
            }
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
            {
                A.coeffRef(i, halfedge_handle.to().idx()) = A.coeffRef(i, halfedge_handle.to().idx()) / omega_sum;
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

    auto& output = input;
    output.get_component<MeshComponent>()->vertices =
        operand_base->get_component<MeshComponent>()->vertices;

    // Set the output of the nodes
    params.set_output("Output", std::move(output));
}

/*
** HW4_TODO: Node to map the mesh boundary to a square.
*/

static void node_map_boundary_to_square_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");
    b.add_input<decl::Int>("Cotangent weights").min(0).max(1).default_val(0);
    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_square_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto is_cotangent = params.get_input<int>("Cotangent weights");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Input does not contain a mesh");
    }
    // throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    OpenMesh::SmartHalfedgeHandle halfedge_handle_start;
    for (const auto& halfedge_handle : halfedge_mesh ->halfedges())
    {
        if (halfedge_handle.is_boundary())
        {
           halfedge_handle_start = halfedge_handle;
           break;
        }
    }
    // find all the boundary points with sequence
    std::vector<OpenMesh::SmartVertexHandle> boundary_points;
    boundary_points.push_back(halfedge_handle_start.to());
    OpenMesh::SmartHalfedgeHandle handle = halfedge_handle_start.next();
    do
    {
        boundary_points.push_back(handle.to());
        handle = handle.next();
    } while (handle != halfedge_handle_start);

    // calculate the coordinates to make the correct contribution
    float sum = 0.f;
    int bp_num = static_cast<int>(boundary_points.size());
    std::vector<float> bp_distance(bp_num);
    for (int i = 1; i < bp_num; i++)
    {
        const auto& vec = halfedge_mesh->point(boundary_points[i]) - halfedge_mesh->point(boundary_points[i-1]);
        sum = sum + vec.norm();
        bp_distance[i-1] = sum;
    }
    sum = sum + (halfedge_mesh->point(boundary_points[0]) - halfedge_mesh->point(boundary_points[bp_num-1])).norm();
    bp_distance[bp_num-1] = sum;

    // construct the equation
    int num = static_cast<int>(halfedge_mesh -> n_vertices());
    Eigen::SparseMatrix<double> A(num,num);
    std::vector<Eigen::MatrixXd> b(3,Eigen::MatrixXd::Zero(num, 1));  
   
        for (int i = 1; i < bp_num; i++)
        {
            int idx = boundary_points[i].idx();
            if (0 < bp_distance[i-1] / sum && bp_distance[i-1] / sum < 0.25)
            {
                b[0](idx) = bp_distance[i-1] / sum * 4;
                b[1](idx) = 0;
            }
            else if (0.25 <= bp_distance[i-1] / sum && bp_distance[i-1] / sum < 0.5)
            {
                b[0](idx) = 1;
                b[1](idx) = bp_distance[i-1] / sum * 4 - 1;
            }
            else if (0.5 <= bp_distance[i-1] / sum && bp_distance[i-1] / sum < 0.75)
            {
                b[0](idx) = 3 - bp_distance[i-1] / sum * 4;
                b[1](idx) = 1;
            }
            else if (0.75 <= bp_distance[i-1] / sum)
            {
                b[0](idx) = 0;
                b[1](idx) = 4 - bp_distance[i-1] / sum * 4;
            }
        }
    // change the weights
    if (! is_cotangent)
    {
        for (const auto& vertex_handle : halfedge_mesh->vertices())
        {
            int i = vertex_handle.idx();

            if (vertex_handle.is_boundary())
            {
                A.coeffRef(i, i) = 1;
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
                continue;
            }
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
    static NodeTypeInfo ntype_square, ntype_circle;

    strcpy(ntype_square.ui_name, "Map Boundary to Square");
    strcpy_s(ntype_square.id_name, "geom_map_boundary_to_square");

    geo_node_type_base(&ntype_square);
    ntype_square.node_execute = node_map_boundary_to_square_exec;
    ntype_square.declare = node_map_boundary_to_square_declare;
    nodeRegisterType(&ntype_square);

    strcpy(ntype_circle.ui_name, "Map Boundary to Circle");
    strcpy_s(ntype_circle.id_name, "geom_map_boundary_to_circle");

    geo_node_type_base(&ntype_circle);
    ntype_circle.node_execute = node_map_boundary_to_circle_exec;
    ntype_circle.declare = node_map_boundary_to_circle_declare;
    nodeRegisterType(&ntype_circle);
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_boundary_mapping
