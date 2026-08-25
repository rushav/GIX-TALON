#pragma once

#include "board_detector.h"
#include "camera_source.h"
#include "config.h"
#include "coverage_grid.h"
#include "pose_estimator.h"

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace intrinsics {

struct CaptureRecord
{
    std::string file;
    std::string capturedUtc;
    int frameId = 0;
    double cameraTimestamp = 0.0;
    CameraSettings settings;
    int markerCount = 0;
    int cornerCount = 0;
    std::vector<int> markerIds;
    std::vector<cv::Point2f> charucoCorners;  // kept so a delete can undo its coverage
    double tiltDegrees = -1.0;                // negative when pose estimation failed
    cv::Mat thumbnail;                        // small BGR copy, overlay included
};

// Owns the captures for one session: the files on disk, the manifest, and the
// coverage and pose statistics derived from them. Deleting a capture has to
// unwind all three, which is why they live together.
class CaptureSession
{
public:
    CaptureSession( const AppConfig& cfg, const CameraCaps& caps, std::string sdkVersion );

    // Created on the first capture, so quitting without capturing leaves
    // nothing behind.
    const std::filesystem::path& directory() const { return mDir; }
    bool created() const { return mCreated; }

    const std::vector<CaptureRecord>& records() const { return mRecords; }
    int count() const { return static_cast<int>( mRecords.size() ); }
    const CoverageGrid& coverage() const { return mCoverage; }
    std::vector<double> tilts() const;

    // Writes view.raw as a PNG, records it, folds it into coverage and pose
    // stats, and rewrites session.json. Returns the PNG filename.
    // Throws std::runtime_error on any filesystem or encode failure.
    std::string add( const ViewFrame& view, double tiltDegrees );

    // Deletes the PNG and unwinds the record from coverage and pose stats.
    void remove( int index );

    std::filesystem::path intrinsicsPath() const { return mDir / "intrinsics.json"; }

private:
    void writeManifest() const;

    const AppConfig& mCfg;
    CameraCaps mCaps;
    std::string mSdkVersion;
    std::string mCreatedUtc;
    std::filesystem::path mDir;
    bool mCreated = false;
    int mNextIndex = 1;   // never reused, so filenames stay stable across deletes
    std::vector<CaptureRecord> mRecords;
    CoverageGrid mCoverage;
};

}  // namespace intrinsics
