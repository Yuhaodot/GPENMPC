#pragma once
// Converts a validated snapshot to the canonical state vector.
// Snapshot positions already include the task-NED origin translation.
#include "../px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp"
namespace gpenmpc_snapshot_mapping {
inline bool state13(const gpenmpc_odometry::Snapshot &snapshot,gpenmpc_portable::Array<double,13> &out)noexcept{
    out.fill(gpenmpc_portable::Ieee754<double>::quiet_NaN());
    if(!snapshot.valid())return false;
    const auto&s=snapshot.estimator();
    double norm2=0.0;for(double value:s.q)norm2+=value*value;const double norm=std::sqrt(norm2);
    // px4EstimateState.m uses C*position and C*velocity, C=diag(1,1,-1).
    // Keep matrix-product zero additions, including resulting signed zero.
    const auto diagonal_product=[](const gpenmpc_portable::Array<double,3>&v,unsigned row)noexcept{
        double value=0.0;
        for(unsigned col=0;col<3;++col){
            const double coefficient=row==col?(row==2?-1.0:1.0):0.0;
            value+=coefficient*v[col];
        }
        return value;
    };
    out={diagonal_product(s.p,0),diagonal_product(s.p,1),diagonal_product(s.p,2),
         diagonal_product(s.v,0),diagonal_product(s.v,1),diagonal_product(s.v,2),
         s.q[0]/norm,-s.q[1]/norm,-s.q[2]/norm,s.q[3]/norm,
         -s.body_rates[0],-s.body_rates[1],s.body_rates[2]};
    return gpenmpc_consumption::finite(out);
}
} // namespace gpenmpc_snapshot_mapping
