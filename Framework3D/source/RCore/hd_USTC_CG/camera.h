#pragma once

#include "renderBuffer.h"
#include "pxr/pxr.h"
#include "pxr/base/gf/ray.h"
#include "pxr/base/gf/rect2i.h"
#include "pxr/imaging/hd/camera.h"
#include "pxr/imaging/hdx/renderSetupTask.h"
PXR_NAMESPACE_OPEN_SCOPE
class Hd_USTC_CG_Camera : public HdCamera
{
public:
    explicit Hd_USTC_CG_Camera(const SdfPath& id)
        : HdCamera(id)
    {
    }

    void Sync(
        HdSceneDelegate* sceneDelegate,
        HdRenderParam* renderParam,
        HdDirtyBits* dirtyBits) override;
    virtual GfRay generateRay(
        GfVec2f pixel_center,
        const std::function<float()>& function) const;

    void update(const HdRenderPassStateSharedPtr& renderPassState) const;

    void attachFilm(Hd_USTC_CG_RenderBuffer* new_film) const;

    mutable Hd_USTC_CG_RenderBuffer* film;
    mutable GfRect2i _dataWindow;
private:
    mutable GfMatrix4d _inverseProjMatrix;
    mutable GfMatrix4d _projMatrix;
    mutable GfMatrix4d _inverseViewMatrix;
    mutable GfMatrix4d _viewMatrix;
};

PXR_NAMESPACE_CLOSE_SCOPE
