#pragma once

#include "board_detector.h"
#include "config.h"

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// SDK
#include "cameramanager.h"

namespace CameraLibrary { class Camera; }

namespace intrinsics {

struct DiscoveredDevice
{
    unsigned serial = 0;
    std::string name;
    int revision = 0;
    std::string state;
};

// What the camera reports back, next to what we asked for. Exposure in
// particular is clamped by the frame period without any error, so the two
// columns are not decoration.
struct CameraSettings
{
    int exposureRequested = -1;
    int exposureActual = -1;
    int gainRequested = -1;
    int gainActual = -1;
    int intensityRequested = -1;
    int intensityActual = -1;
    int frameRateRequested = -1;
    int frameRateActual = -1;    // the configured rate the SDK reports back
    int deviceFrameRate = -1;    // what the imager is actually running at
    int irFilterRequested = -1;   // -1 unknown, 0 off, 1 on
    int irFilterActual = -1;
    int videoType = -1;
};

struct CameraCaps
{
    unsigned serial = 0;
    std::string name;
    int revision = 0;
    int width = 0;
    int height = 0;
    Range exposure;
    Range imagerGain;
    Range irIntensity;
    Range frameRate;
    bool gainAvailable = false;
    bool irAvailable = false;
    bool filterSwitchAvailable = false;
};

// One published live-view frame. `raw` is exactly the image `display` was
// drawn from, so a capture and its recorded marker count always agree.
struct ViewFrame
{
    cv::Mat raw;      // CV_8UC1, native resolution, no overlay
    cv::Mat display;  // CV_8UC3 BGR with marker overlay
    Detection detection;
    CameraSettings settings;
    int frameId = 0;
    double timestamp = 0.0;
    double measuredFps = 0.0;
    std::uint64_t seq = 0;
};

class CameraSource : public CameraLibrary::cCameraManagerListener
{
public:
    CameraSource( const AppConfig& cfg, const BoardConfig& board );
    ~CameraSource() override;

    CameraSource( const CameraSource& ) = delete;
    CameraSource& operator=( const CameraSource& ) = delete;

    // Blocks until the device count has been stable for discovery.settle_ms
    // or discovery.timeout_ms elapses.
    std::vector<DiscoveredDevice> discover();

    bool open( unsigned serial, std::string& error );
    void start();
    void stop();

    CameraCaps caps() const;
    std::shared_ptr<const ViewFrame> latest() const;

    void setExposure( int value );
    void setImagerGain( int level );
    void setIntensity( int value );
    void setFrameRate( int hz );
    void setIRFilter( bool enabled );

    // The SDK re-reports its exposure range after a frame rate change; the
    // maximum tracks the frame period. Re-reads it and returns the new range.
    Range refreshExposureRange();
    void setDetectionEnabled( bool on );

    std::vector<std::string> drainLog();

    static std::string sdkVersion();

    // cCameraManagerListener. These fire on the SDK's own discovery thread,
    // so they only touch our own state under mDiscoveryMutex and never call
    // back into the SDK - re-entering it from its own callback deadlocks.
    void CameraInitialized() override;
    void CameraConnected() override;
    void CameraRemoved() override;

private:
    void captureLoop();
    std::shared_ptr<CameraLibrary::Camera> camera() const;
    void readCaps( const std::shared_ptr<CameraLibrary::Camera>& cam );
    CameraSettings readSettings( const std::shared_ptr<CameraLibrary::Camera>& cam ) const;
    void log( const std::string& msg );

    const AppConfig& mCfg;
    BoardDetector mDetector;

    mutable std::mutex mStateMutex;  // guards mCamera, mCaps, mRequested, mLatest
    std::shared_ptr<CameraLibrary::Camera> mCamera;
    CameraCaps mCaps;
    CameraSettings mRequested;
    std::shared_ptr<const ViewFrame> mLatest;
    std::uint64_t mSeq = 0;

    mutable std::mutex mDiscoveryMutex;
    std::condition_variable mDiscoveryCv;
    unsigned mInitEvents = 0;

    mutable std::mutex mLogMutex;
    std::vector<std::string> mLog;

    std::thread mThread;
    std::atomic_bool mRunning{ false };
    std::atomic_bool mDetectEnabled{ true };
    std::atomic_int mFrameRateForCap{ 0 };
};

}  // namespace intrinsics
