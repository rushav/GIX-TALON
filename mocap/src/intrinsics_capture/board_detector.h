#pragma once

#include "config.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

#include <vector>

namespace intrinsics {

struct Detection
{
    // ArUco markers - what the live overlay draws and counts.
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // Interpolated ChArUco chessboard corners - what coverage, pose and the
    // solve actually consume. Markers locate the board; corners measure it.
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;

    int markerCount() const { return static_cast<int>( ids.size() ); }
    int cornerCount() const { return static_cast<int>( charucoCorners.size() ); }
};

class BoardDetector
{
public:
    // Throws std::runtime_error if board.dictionary is not a known cv::aruco name.
    explicit BoardDetector( const BoardConfig& board );

    // gray must be CV_8UC1. Runs marker detection and, when enough markers are
    // present, ChArUco corner interpolation.
    Detection detect( const cv::Mat& gray ) const;

    // 3D board coordinates of the given ChArUco corner ids, in millimetres.
    std::vector<cv::Point3f> objectPoints( const std::vector<int>& charucoIds ) const;

    static void draw( cv::Mat& bgr, const Detection& d );

private:
    BoardConfig mBoard;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;
    cv::Ptr<cv::aruco::CharucoBoard> mCharucoBoard;
    cv::Ptr<cv::aruco::DetectorParameters> mParams;
};

}  // namespace intrinsics
