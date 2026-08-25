#include "pose_estimator.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace intrinsics {

PoseEstimator::PoseEstimator( const PoseConfig& cfg, const BoardDetector& detector )
    : mCfg( cfg )
    , mDetector( detector )
{
}

std::optional<BoardPose> PoseEstimator::estimate( const Detection& d ) const
{
    if( d.cornerCount() < 4 ) return std::nullopt;

    const std::vector<cv::Point3f> object = mDetector.objectPoints( d.charucoIds );
    if( object.size() != d.charucoCorners.size() ) return std::nullopt;

    const cv::Matx33d K( mCfg.guessFx, 0, mCfg.guessCx,
                         0, mCfg.guessFy, mCfg.guessCy,
                         0, 0, 1 );
    cv::Vec3d rvec, tvec;
    if( !cv::solvePnP( object, d.charucoCorners, K, cv::Mat(), rvec, tvec, false,
            cv::SOLVEPNP_ITERATIVE ) )
        return std::nullopt;

    cv::Matx33d R;
    cv::Rodrigues( rvec, R );

    // The board normal is +Z in board coordinates, so R(2,2) is its cosine
    // against the camera axis. |.| because a board seen from behind is the same
    // tilt, and clamped because rounding can push it just past 1.
    const double c = std::min( 1.0, std::abs( R( 2, 2 ) ) );

    BoardPose pose;
    pose.tiltDegrees = std::acos( c ) * 180.0 / CV_PI;
    pose.distanceMm = cv::norm( tvec );
    return pose;
}

double PoseEstimator::tiltedFraction( const std::vector<double>& tilts ) const
{
    if( tilts.empty() ) return 0.0;
    const auto n = std::count_if( tilts.begin(), tilts.end(),
        [this]( double t ) { return t >= mCfg.minTiltDegrees; } );
    return static_cast<double>( n ) / tilts.size();
}

std::vector<int> PoseEstimator::histogram( const std::vector<double>& tilts ) const
{
    std::vector<int> bins( static_cast<size_t>( mCfg.histogramBins ), 0 );
    for( double t : tilts )
    {
        int b = static_cast<int>( t / mCfg.histogramBinDegrees );
        b = std::clamp( b, 0, mCfg.histogramBins - 1 );  // last bin absorbs the tail
        ++bins[static_cast<size_t>( b )];
    }
    return bins;
}

}  // namespace intrinsics
