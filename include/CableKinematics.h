#ifndef CABLE_KINEMATICS_H
#define CABLE_KINEMATICS_H

#include "../Dependencies/eigen-3.3.7/Eigen/Dense"
#include <array>
#include <cstddef>
#include <vector>

namespace cu_brick {

constexpr std::size_t kMaximumCableCount = 8;
constexpr std::size_t kMaximumRailCount = 4;

struct CableKinematicsConfig {
    std::size_t cableCount{0};
    double pulleyRadius{0.0};
    std::array<Eigen::Vector3d, kMaximumCableCount> frameAttachments{};
    std::array<Eigen::Vector3d, kMaximumCableCount>
        endEffectorAttachments{};
    std::array<Eigen::Vector3d, kMaximumCableCount> pulleyAxes{};
    std::array<double, kMaximumRailCount> startingRailOffsets{};
};

std::vector<double> ComputeCableLengths(
    const CableKinematicsConfig& config,
    const std::array<double, 6>& endEffectorPose,
    const std::array<double, kMaximumRailCount>& railOffsets
);

} // namespace cu_brick

#endif
