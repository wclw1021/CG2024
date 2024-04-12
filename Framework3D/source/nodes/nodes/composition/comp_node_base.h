#pragma once
#include "Nodes/socket_types/stage_socket_types.hpp"
#include "Nodes/node.hpp"

USTC_CG_NAMESPACE_OPEN_SCOPE
inline void comp_node_type_base(NodeTypeInfo* ntype)
{
    ntype->color[0] = 114 / 255.f;
    ntype->color[1] = 94 / 255.f;
    ntype->color[2] = 29 / 255.f;
    ntype->color[3] = 1.0f;

    ntype->node_type_of_grpah = NodeTypeOfGrpah::Composition;
}

USTC_CG_NAMESPACE_CLOSE_SCOPE
