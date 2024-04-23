#include "path.h"

#include <random>

#include "surfaceInteraction.h"
#include <utils/sampling.hpp>
#include <cmath>

USTC_CG_NAMESPACE_OPEN_SCOPE
using namespace pxr;

float RussianRouletteProbability();

VtValue PathIntegrator::Li(const GfRay& ray, std::default_random_engine& random)
{
    std::uniform_real_distribution<float> uniform_dist(
        0.0f, 1.0f - std::numeric_limits<float>::epsilon());
    std::function<float()> uniform_float = std::bind(uniform_dist, random);

    auto color = EstimateOutGoingRadiance(ray, uniform_float, 0);

    return VtValue(GfVec3f(color[0], color[1], color[2]));
}

GfVec3f PathIntegrator::EstimateOutGoingRadiance(
    const GfRay& ray,
    const std::function<float()>& uniform_float,
    int recursion_depth)
{
    if (recursion_depth >= 50) {
        return {};
    }

    SurfaceInteraction si;
    if (!Intersect(ray, si)) {
        if (recursion_depth == 0) {
            return IntersectDomeLight(ray);
        }

        return GfVec3f{ 0, 0, 0 };
    }

    // This can be customized : Do we want to see the lights? (Other than dome lights?)
    if (recursion_depth == 0) {
    }

    // Flip the normal if opposite
    if (GfDot(si.shadingNormal, ray.GetDirection()) > 0) {
        si.flipNormal();
        si.PrepareTransforms();
    }

    GfVec3f color{ 0 };
    GfVec3f directLight = EstimateDirectLight(si, uniform_float);

    GfVec3f globalLight;
    float pdf;
    float p_rr = 0.9;

    // judge the prr
    if (uniform_float() < p_rr) {
        
        // sample on the point
        auto wi = UniformSampleHemiSphere(GfVec2f{ uniform_float(), uniform_float() }, pdf).GetNormalized();

        // get the normal
        auto normal = si.geometricNormal;

        auto basis = constructONB(normal);
        wi = basis * wi;

        // get a new ray
        GfRay new_ray(si.position, wi);
        
        GfVec3f indirectRadiance =
                EstimateOutGoingRadiance(new_ray, uniform_float, recursion_depth + 1);

        // calculate the light
        auto brdf = si.Eval(wi);
        float cos_theta = GfDot(normal, wi.GetNormalized());
        globalLight = GfCompMult(indirectRadiance, brdf) * cos_theta / pdf / p_rr;
    }

    color = directLight + globalLight;

    return color;
}


USTC_CG_NAMESPACE_CLOSE_SCOPE
