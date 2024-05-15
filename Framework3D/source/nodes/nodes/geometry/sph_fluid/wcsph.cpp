#include "wcsph.h"
#include <iostream>
#include <cmath>
using namespace Eigen;

namespace USTC_CG::node_sph_fluid {

WCSPH::WCSPH(const MatrixXd& X, const Vector3d& box_min, const Vector3d& box_max)
    : SPHBase(X, box_min, box_max)
{
}

//void WCSPH::compute_density()
//{
	// -------------------------------------------------------------
	// (HW TODO) Implement the density computation
    // You can also compute pressure in this function 
	// -------------------------------------------------------------
//}

// compute pressure
double WCSPH::compute_pressure_acceleration(const std::shared_ptr<Particle>& p)
{
    auto p_density = p->density();
    auto p_i = stiffness_ * (std::pow(p_density/ps_.density0(),exponent_)-1);
    p_i = std::max(p_i,0.0);
    return p_i;
}

void WCSPH::step()
{
    TIC(step)
    // -------------------------------------------------------------
    // (HW TODO) Follow the instruction in documents and PPT, 
    // implement the pipeline of fluid simulation 
    // -------------------------------------------------------------

	// Search neighbors, compute density, advect, solve pressure acceleration, etc. 
    ps_.assign_particles_to_cells();
    ps_.search_neighbors(); 
    compute_density();

    compute_non_pressure_acceleration();
    for (auto& p : ps_.particles()) {
        p->vel_ += dt_ * p->acceleration();
    }

    compute_pressure_gradient_acceleration();
    for (auto& p : ps_.particles()) {
        p->vel_ += dt_ * p->acceleration();
        p->X_ += dt_ * p->vel_;
    }

    advect();




    TOC(step)
}
}  // namespace USTC_CG::node_sph_fluid