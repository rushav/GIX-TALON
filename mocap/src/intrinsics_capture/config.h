#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace intrinsics {

struct CameraEntry
{
    unsigned serial = 0;
    std::string status;   // active | available | excluded
    std::string reason;
};

struct BoardConfig
{
    std::string dictionary;      // e.g. "DICT_5X5_1000"; resolved by BoardDetector
    int squaresX = 0;
    int squaresY = 0;
    double squareLengthMm = 0.0;
    double markerLengthMm = 0.0;
    int markerCount = 0;
    // Only the OpenCV 5.x solver needs this; 4.6 already uses the legacy layout.
    bool legacyPattern = true;

    int charucoCornerCount() const { return ( squaresX - 1 ) * ( squaresY - 1 ); }
};

struct DiscoveryConfig
{
    int pollIntervalMs = 250;
    int settleMs = 1500;
    int timeoutMs = 12000;
};

struct Range
{
    int lo = 0;
    int hi = 0;
    bool sane() const { return hi > lo; }
};

// Applied only when the SDK reports a degenerate range for a control.
struct FallbackRanges
{
    Range exposure{ 1, 6000 };
    Range imagerGain{ 0, 7 };
    Range irIntensity{ 0, 15 };
    Range frameRate{ 30, 120 };
};

struct CameraDefaults
{
    int exposure = 8133;
    int imagerGain = 7;
    int irIntensity = 15;
    int frameRateHz = 30;
    bool irFilter = true;
};

struct CoverageConfig
{
    int cols = 5;
    int rows = 6;
    int targetPerCell = 20;
    int cellCount() const { return cols * rows; }
};

struct PoseConfig
{
    double guessFx = 780.0;
    double guessFy = 780.0;
    double guessCx = 640.0;
    double guessCy = 512.0;
    double minTiltDegrees = 20.0;
    double minFractionTilted = 0.30;
    double histogramBinDegrees = 5.0;
    int histogramBins = 12;
};

struct SolverConfig
{
    std::filesystem::path python;   // absolute, resolved against the repo root
    std::filesystem::path script;
    int minCorners = 6;
    int timeoutSeconds = 300;
};

// Context for reading a solve, never pass/fail.
struct ReferenceBounds
{
    double rmsLo = 0.32, rmsHi = 0.39;
    double focalLo = 778, focalHi = 786;
    double focalAgreementPercent = 1.0;
    double cxLo = 627, cxHi = 637;
    double cyLo = 516, cyHi = 521;
    double nominalCx = 640, nominalCy = 512;
};

struct AppConfig
{
    std::filesystem::path repoRoot;
    std::filesystem::path camerasFile;
    std::filesystem::path toolFile;
    std::filesystem::path boardFile;
    std::filesystem::path sessionRoot;

    std::string cameraModel;
    int nativeWidth = 0;
    int nativeHeight = 0;
    int nominalFrameRateHz = 0;
    std::vector<CameraEntry> cameras;

    BoardConfig board;
    DiscoveryConfig discovery;
    CameraDefaults defaults;
    FallbackRanges fallback;
    CoverageConfig coverage;
    PoseConfig pose;
    SolverConfig solver;
    ReferenceBounds reference;

    int displayFps = 30;
    bool refreshExposureRangeOnFrameRateChange = true;

    // Picks `requested` if it is listed, otherwise the first entry with
    // status: active. Throws std::runtime_error explaining the inventory.
    unsigned resolveSerial( std::optional<unsigned> requested ) const;
};

// Walks up from `hint` (then from the executable's directory) looking for
// mocap/config/cameras.yaml. Throws if the repo layout cannot be located.
std::filesystem::path findRepoRoot( const std::filesystem::path& hint );

// Throws std::runtime_error naming the offending file on bad config.
AppConfig loadConfig( const std::filesystem::path& repoRoot );

}  // namespace intrinsics
