#ifndef CDPR_SLICER_MECHANICS_H
#define CDPR_SLICER_MECHANICS_H

#include "cdpr_slicer/Planner.h"

namespace cdpr_slicer {

struct CableFreeSpan {
    std::array<double,3> pulley_m{};
    std::array<double,3> attachment_m{};
};

// Uses the existing adapter's guarded pulley intermediates and shared lengths.
Result<std::array<CableFreeSpan,8>> ComputeCheckedCableSpans(
    const RobotModelSnapshot& model, const std::array<double,6>& legacy_pose);

// A separate, sealed sampled report. Never changes the planning artifact or
// asserts hardware readiness. JSON input requires explicit physical parameters.
Result<std::string> AssessMechanics(const PrintJob& job,
    const RobotModelSnapshot& model, const std::string& physical_model_json);
Result<bool> ValidateMechanicsAssessment(const PrintJob& job,
    const RobotModelSnapshot& model, const std::string& assessment_json);

Result<std::string> CreatePreviewHtml(const PrintJob& job,
    const RobotModelSnapshot& model, const std::string& assessment_json);

} // namespace cdpr_slicer
#endif
