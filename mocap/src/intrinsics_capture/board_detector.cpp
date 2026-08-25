#include "board_detector.h"

#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <unordered_map>

namespace intrinsics {
namespace {

// Keeps the dictionary name in the config file rather than in code.
int dictionaryIdFromName( const std::string& name )
{
    static const std::unordered_map<std::string, int> kNames = {
        { "DICT_4X4_50", cv::aruco::DICT_4X4_50 },
        { "DICT_4X4_100", cv::aruco::DICT_4X4_100 },
        { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
        { "DICT_4X4_1000", cv::aruco::DICT_4X4_1000 },
        { "DICT_5X5_50", cv::aruco::DICT_5X5_50 },
        { "DICT_5X5_100", cv::aruco::DICT_5X5_100 },
        { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
        { "DICT_5X5_1000", cv::aruco::DICT_5X5_1000 },
        { "DICT_6X6_50", cv::aruco::DICT_6X6_50 },
        { "DICT_6X6_100", cv::aruco::DICT_6X6_100 },
        { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
        { "DICT_6X6_1000", cv::aruco::DICT_6X6_1000 },
        { "DICT_7X7_50", cv::aruco::DICT_7X7_50 },
        { "DICT_7X7_100", cv::aruco::DICT_7X7_100 },
        { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
        { "DICT_7X7_1000", cv::aruco::DICT_7X7_1000 },
        { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL },
        { "DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 },
        { "DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 },
        { "DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 },
        { "DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 },
    };
    auto it = kNames.find( name );
    if( it == kNames.end() )
        throw std::runtime_error( "unknown aruco dictionary '" + name + "'" );
    return it->second;
}

}  // namespace

BoardDetector::BoardDetector( const BoardConfig& board )
    : mBoard( board )
{
    mDictionary = cv::aruco::getPredefinedDictionary( dictionaryIdFromName( board.dictionary ) );

    // OpenCV 4.6's CharucoBoard already uses what later versions call the
    // "legacy" pattern, so there is no flag to set here. Only the OpenCV 5.x
    // solver needs setLegacyPattern(True) to reproduce this same layout.
    mCharucoBoard = cv::aruco::CharucoBoard::create( board.squaresX, board.squaresY,
        static_cast<float>( board.squareLengthMm ), static_cast<float>( board.markerLengthMm ),
        mDictionary );

    mParams = cv::aruco::DetectorParameters::create();

    // Corner refinement costs a few ms but is what makes the overlay honest
    // about whether a marker is sharp enough to be worth capturing.
    mParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

Detection BoardDetector::detect( const cv::Mat& gray ) const
{
    Detection d;
    if( gray.empty() ) return d;
    cv::aruco::detectMarkers( gray, mDictionary, d.corners, d.ids, mParams );
    if( d.ids.empty() ) return d;

    // Chessboard corners are what get calibrated; they interpolate to sub-pixel
    // accuracy between the markers that bound them.
    cv::Mat cornerMat, idMat;
    cv::aruco::interpolateCornersCharuco( d.corners, d.ids, gray, mCharucoBoard, cornerMat, idMat );
    if( !cornerMat.empty() )
    {
        cornerMat.reshape( 2, cornerMat.total() ).copyTo( d.charucoCorners );
        idMat.reshape( 1, idMat.total() ).copyTo( d.charucoIds );
    }
    return d;
}

std::vector<cv::Point3f> BoardDetector::objectPoints( const std::vector<int>& charucoIds ) const
{
    std::vector<cv::Point3f> out;
    out.reserve( charucoIds.size() );
    const auto& all = mCharucoBoard->chessboardCorners;
    for( int id : charucoIds )
        if( id >= 0 && id < static_cast<int>( all.size() ) ) out.push_back( all[id] );
    return out;
}

void BoardDetector::draw( cv::Mat& bgr, const Detection& d )
{
    if( !d.ids.empty() )
        cv::aruco::drawDetectedMarkers( bgr, d.corners, d.ids, cv::Scalar( 0, 255, 0 ) );

    // Chessboard corners in a second colour: these are the measurements that
    // reach the solve, so it should be obvious when markers are found but
    // corners are not.
    for( const auto& p : d.charucoCorners )
    {
        cv::drawMarker( bgr, p, cv::Scalar( 0, 190, 255 ), cv::MARKER_CROSS, 11, 2 );
    }
}

}  // namespace intrinsics
