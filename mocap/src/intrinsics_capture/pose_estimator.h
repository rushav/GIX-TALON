#pragma once

#include "board_detector.h"
#include "config.h"

#include <optional>
#include <vector>

namespace intrinsics {

struct BoardPose
{
    double tiltDegrees = 0.0;    // angle of the board normal away from the camera axis
    double distanceMm = 0.0;
};

// Estimates board pose per capture so the set can be checked for tilt variety.
//
// A set of purely fronto-parallel views leaves focal length and distance
// mutually ambiguous - the board can be small and near or large and far with
// the same image. The solve still converges; the tell is fx drifting away from
// fy. Tilted views break the ambiguity.
class PoseEstimator
{
public:
    PoseEstimator( const PoseConfig& cfg, const BoardDetector& detector );

    // Needs at least 4 corners. Uses the rough intrinsics guess from config -
    // this ranks views by tilt, it does not calibrate.
    std::optional<BoardPose> estimate( const Detection& d ) const;

    // Fraction of the given tilts at or above the configured threshold.
    double tiltedFraction( const std::vector<double>& tiltsDegrees ) const;
    std::vector<int> histogram( const std::vector<double>& tiltsDegrees ) const;

    const PoseConfig& config() const { return mCfg; }

private:
    PoseConfig mCfg;
    const BoardDetector& mDetector;
};

}  // namespace intrinsics
