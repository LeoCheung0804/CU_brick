#include "../include/CableKinematics.h"

#include <cmath>
#include <stdexcept>

namespace cu_brick {

std::vector<double> ComputeCableLengths(
    const CableKinematicsConfig& config,
    const std::array<double, 6>& endEffectorPose,
    const std::array<double, kMaximumRailCount>& railOffsets
) {
    if(config.cableCount > kMaximumCableCount)
        throw std::out_of_range("Cable count exceeds kinematics capacity.");

    Eigen::Matrix3d rotationX;
    Eigen::Matrix3d rotationY;
    Eigen::Matrix3d rotationZ;
    const double roll = endEffectorPose[3];
    const double pitch = endEffectorPose[4];
    const double yaw = endEffectorPose[5];

    rotationX << 1, 0, 0,
        0, std::cos(roll), -std::sin(roll),
        0, std::sin(roll), std::cos(roll);
    rotationY << std::cos(pitch), 0, std::sin(pitch),
        0, 1, 0,
        -std::sin(pitch), 0, std::cos(pitch);
    rotationZ << std::cos(yaw), -std::sin(yaw), 0,
        std::sin(yaw), std::cos(yaw), 0,
        0, 0, 1;

    const Eigen::Matrix3d frameRotation =
        rotationX * rotationY * rotationZ;
    const Eigen::Vector3d endEffectorTranslation(
        endEffectorPose[0],
        endEffectorPose[1],
        endEffectorPose[2]
    );

    std::vector<double> lengths;
    lengths.reserve(config.cableCount);
    for(std::size_t index = 0; index < config.cableCount; index++){
        const Eigen::Vector3d endEffectorOutlet =
            frameRotation * config.endEffectorAttachments[index]
            + endEffectorTranslation;
        const Eigen::Vector3d pulleyPlaneNormal =
            config.pulleyAxes[index].cross(
                endEffectorOutlet - config.frameAttachments[index]
            );
        const Eigen::Vector3d centerDirection =
            pulleyPlaneNormal.cross(config.pulleyAxes[index]);

        Eigen::Vector3d railOffset(0, 0, 0);
        if(index < kMaximumRailCount)
            railOffset << 0, 0, railOffsets[index];

        const Eigen::Vector3d pulleyCenter =
            centerDirection / centerDirection.norm() * config.pulleyRadius
            + config.frameAttachments[index]
            + railOffset;
        const double triangleRatio =
            config.pulleyRadius / (endEffectorOutlet - pulleyCenter).norm();
        const Eigen::Vector3d pulleyOutlet =
            pulleyCenter
            + triangleRatio * triangleRatio
                * (endEffectorOutlet - pulleyCenter)
            - triangleRatio * std::sqrt(1 - triangleRatio * triangleRatio)
                * (
                    (pulleyPlaneNormal / pulleyPlaneNormal.norm())
                    .cross(endEffectorOutlet - pulleyCenter)
                );

        const Eigen::Vector3d fixedDirection =
            -centerDirection / centerDirection.norm();
        const Eigen::Vector3d outletDirection =
            (pulleyOutlet - pulleyCenter)
            / (pulleyOutlet - pulleyCenter).norm();
        const double arcLength =
            config.pulleyRadius
            * std::acos(fixedDirection.dot(outletDirection));
        const double straightLength =
            (pulleyOutlet - endEffectorOutlet).norm();

        if(index < kMaximumRailCount){
            lengths.push_back(
                arcLength
                + straightLength
                - (railOffsets[index] - config.startingRailOffsets[index])
            );
        }else{
            lengths.push_back(arcLength + straightLength);
        }
    }
    return lengths;
}

} // namespace cu_brick
