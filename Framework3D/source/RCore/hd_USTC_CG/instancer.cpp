//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#include "instancer.h"

#include "sampler.h"
#include "pxr/imaging/hd/sceneDelegate.h"
#include "pxr/imaging/hd/tokens.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/quaternion.h"
#include "pxr/base/tf/staticTokens.h"

USTC_CG_NAMESPACE_OPEN_SCOPE
using namespace pxr;
Hd_USTC_CG_Instancer::Hd_USTC_CG_Instancer(
    HdSceneDelegate* delegate,
    const SdfPath& id)
    : HdInstancer(delegate, id)
{
}

Hd_USTC_CG_Instancer::~Hd_USTC_CG_Instancer()
{
    TF_FOR_ALL(it, _primvarMap)
    {
        delete it->second;
    }
    _primvarMap.clear();
}

void
Hd_USTC_CG_Instancer::Sync(
    HdSceneDelegate* delegate,
    HdRenderParam* renderParam,
    HdDirtyBits* dirtyBits)
{
    _UpdateInstancer(delegate, dirtyBits);

    if (HdChangeTracker::IsAnyPrimvarDirty(*dirtyBits, GetId()))
    {
        _SyncPrimvars(delegate, *dirtyBits);
    }
}

void
Hd_USTC_CG_Instancer::_SyncPrimvars(
    HdSceneDelegate* delegate,
    HdDirtyBits dirtyBits)
{
    HD_TRACE_FUNCTION();
    HF_MALLOC_TAG_FUNCTION();

    const SdfPath& id = GetId();

    HdPrimvarDescriptorVector primvars =
        delegate->GetPrimvarDescriptors(id, HdInterpolationInstance);

    for (const HdPrimvarDescriptor& pv : primvars)
    {
        if (HdChangeTracker::IsPrimvarDirty(dirtyBits, id, pv.name))
        {
            VtValue value = delegate->Get(id, pv.name);
            if (!value.IsEmpty())
            {
                if (_primvarMap.count(pv.name) > 0)
                {
                    delete _primvarMap[pv.name];
                }
                _primvarMap[pv.name] =
                    new HdVtBufferSource(pv.name, value);
            }
        }
    }
}

VtMatrix4dArray
Hd_USTC_CG_Instancer::ComputeInstanceTransforms(const SdfPath& prototypeId)
{
    HD_TRACE_FUNCTION();
    HF_MALLOC_TAG_FUNCTION();

    // The transforms for this level of instancer are computed by:
    // foreach(index : indices) {
    //     instancerTransform
    //     * hydra:instanceTranslations(index)
    //     * hydra:instanceRotations(index)
    //     * hydra:instanceScales(index)
    //     * hydra:instanceTransforms(index)
    // }
    // If any transform isn't provided, it's assumed to be the identity.

    GfMatrix4d instancerTransform =
        GetDelegate()->GetInstancerTransform(GetId());
    VtIntArray instanceIndices =
        GetDelegate()->GetInstanceIndices(GetId(), prototypeId);

    VtMatrix4dArray transforms(instanceIndices.size());
    for (size_t i = 0; i < instanceIndices.size(); ++i)
    {
        transforms[i] = instancerTransform;
    }

    if (GetParentId().IsEmpty())
    {
        return transforms;
    }

    HdInstancer* parentInstancer =
        GetDelegate()->GetRenderIndex().GetInstancer(GetParentId());
    if (!TF_VERIFY(parentInstancer))
    {
        return transforms;
    }

    // The transforms taking nesting into account are computed by:
    // parentTransforms = parentInstancer->ComputeInstanceTransforms(GetId())
    // foreach (parentXf : parentTransforms, xf : transforms) {
    //     parentXf * xf
    // }
    VtMatrix4dArray parentTransforms =
        static_cast<Hd_USTC_CG_Instancer*>(parentInstancer)->
        ComputeInstanceTransforms(GetId());

    VtMatrix4dArray final(parentTransforms.size() * transforms.size());
    for (size_t i = 0; i < parentTransforms.size(); ++i)
    {
        for (size_t j = 0; j < transforms.size(); ++j)
        {
            final[i * transforms.size() + j] = transforms[j] *
                                               parentTransforms[i];
        }
    }
    return final;
}

USTC_CG_NAMESPACE_CLOSE_SCOPE
