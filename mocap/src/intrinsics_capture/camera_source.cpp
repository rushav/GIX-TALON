#include "camera_source.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <set>

#include "cameralibrary.h"
#include "version.h"

using namespace CameraLibrary;
using clock_type = std::chrono::steady_clock;

namespace intrinsics {
namespace {

const char* videoModeName( int mode )
{
    switch( mode )
    {
        case Core::SegmentMode: return "Segment";
        case Core::GrayscaleMode: return "Grayscale";
        case Core::ObjectMode: return "Object";
        case Core::InterleavedGrayscaleMode: return "InterleavedGrayscale";
        case Core::PrecisionMode: return "Precision";
        case Core::BitPackedPrecisionMode: return "BitPackedPrecision";
        case Core::MJPEGMode: return "MJPEG";
        case Core::VideoMode: return "Video";
        case Core::DuplexMode: return "Duplex";
        default: return "Unknown";
    }
}

Range pickRange( int lo, int hi, const Range& fallback, const char* what,
    std::vector<std::string>& notes )
{
    if( hi > lo ) return { lo, hi };
    notes.push_back( std::string( "SDK reported no usable " ) + what + " range (" +
        std::to_string( lo ) + ".." + std::to_string( hi ) + "); using config fallback " +
        std::to_string( fallback.lo ) + ".." + std::to_string( fallback.hi ) );
    return fallback;
}

}  // namespace

CameraSource::CameraSource( const AppConfig& cfg, const BoardConfig& board )
    : mCfg( cfg )
    , mDetector( board )
{
    CameraLibraryStartup();
    CameraManager::X().RegisterListener( this );
    CameraManager::X().ScanForCameras();
}

CameraSource::~CameraSource()
{
    stop();

    std::shared_ptr<Camera> cam;
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        cam = std::move( mCamera );
        mCamera.reset();
    }
    if( cam && cam->IsCameraRunning() ) cam->Stop();
    cam.reset();

    CameraManager::X().UnregisterListener();
    CameraLibraryShutdown();
}

std::string CameraSource::sdkVersion()
{
    const char* v = CameraLibrary::VersionString();
    std::string s = v ? v : CAMERALIBRARY_VERSIONSTR;
    int build = CameraLibrary::BuildNumber();
    if( build > 0 ) s += " (build " + std::to_string( build ) + ")";
    return s;
}

void CameraSource::log( const std::string& msg )
{
    std::fprintf( stderr, "[intrinsics_capture] %s\n", msg.c_str() );
    std::lock_guard<std::mutex> lock( mLogMutex );
    if( mLog.size() < 512 ) mLog.push_back( msg );
}

std::vector<std::string> CameraSource::drainLog()
{
    std::lock_guard<std::mutex> lock( mLogMutex );
    return std::move( mLog );
}

// --- listener: SDK discovery thread ---------------------------------------
// Nothing here calls back into the SDK. GetCameraList()/GetCameraBySerial()
// from inside a listener callback re-enters the library on its own thread and
// can deadlock, so these only tick a counter the poller waits on.

void CameraSource::CameraInitialized()
{
    std::lock_guard<std::mutex> lock( mDiscoveryMutex );
    ++mInitEvents;
    mDiscoveryCv.notify_all();
}

void CameraSource::CameraConnected() { CameraInitialized(); }

void CameraSource::CameraRemoved()
{
    std::lock_guard<std::mutex> lock( mDiscoveryMutex );
    ++mInitEvents;
    mDiscoveryCv.notify_all();
}

// --- discovery -------------------------------------------------------------

std::vector<DiscoveredDevice> CameraSource::discover()
{
    const auto& d = mCfg.discovery;
    const auto deadline = clock_type::now() + std::chrono::milliseconds( d.timeoutMs );

    // The SDK reports the first camera long before the rest finish
    // initializing - a plain wait-for-one-device returns in well under a
    // second while a cold four-camera network takes 4-8s to fully enumerate.
    // So: poll the list, and only accept it once the count has held steady.
    std::size_t stableCount = 0;
    int reportedDuplicates = 0;
    clock_type::time_point stableSince = clock_type::now();
    std::vector<DiscoveredDevice> best;

    for( ;; )
    {
        CameraList list;
        CameraManager::X().GetCameraList( list );

        // A camera can appear in CameraList more than once - observed on the
        // legacy Prime path, where the same serial is listed twice. Collapse by
        // serial, preferring an entry that has finished initializing, so the
        // settle test below counts distinct cameras.
        std::vector<DiscoveredDevice> now;
        int duplicates = 0;
        for( int i = 0; i < list.Count(); ++i )
        {
            DiscoveredDevice dev;
            dev.serial = list[i].Serial();
            dev.name = list[i].Name() ? list[i].Name() : "";
            dev.revision = list[i].Revision();
            dev.state = CameraStateText( list[i].State() );

            auto existing = std::find_if( now.begin(), now.end(),
                [&]( const DiscoveredDevice& d ) { return d.serial == dev.serial; } );
            if( existing == now.end() )
            {
                now.push_back( dev );
            }
            else
            {
                ++duplicates;
                if( list[i].State() == Initialized ) *existing = dev;
            }
        }
        if( duplicates && duplicates != reportedDuplicates )
        {
            reportedDuplicates = duplicates;
            log( "discovery: collapsed " + std::to_string( duplicates ) +
                " duplicate list entr" + ( duplicates == 1 ? "y" : "ies" ) );
        }

        if( now.size() != stableCount )
        {
            stableCount = now.size();
            stableSince = clock_type::now();
            log( "discovery: " + std::to_string( stableCount ) + " device(s) so far" );
        }
        best = std::move( now );

        const auto held = clock_type::now() - stableSince;
        if( !best.empty() && held >= std::chrono::milliseconds( d.settleMs ) ) break;
        if( clock_type::now() >= deadline )
        {
            log( "discovery: timed out after " + std::to_string( d.timeoutMs ) + " ms with " +
                std::to_string( best.size() ) + " device(s)" );
            break;
        }

        // Sleep on the listener's condvar so a late arrival cuts the wait
        // short instead of costing a whole poll interval.
        std::unique_lock<std::mutex> lock( mDiscoveryMutex );
        mDiscoveryCv.wait_for( lock, std::chrono::milliseconds( d.pollIntervalMs ) );
    }

    log( "discovery: settled on " + std::to_string( best.size() ) + " device(s)" );
    return best;
}

// --- open / capabilities ---------------------------------------------------

bool CameraSource::open( unsigned serial, std::string& error )
{
    auto cam = CameraManager::X().GetCameraBySerial( serial );
    if( !cam )
    {
        error = "camera " + std::to_string( serial ) + " did not enumerate";
        return false;
    }

    if( !cam->IsVideoTypeSupported( Core::GrayscaleMode ) )
        log( "warning: camera reports grayscale mode unsupported; requesting it anyway" );

    // Ask for grayscale before Start() so fewer object-mode frames land in the
    // queue, then again after Start() - the mode change is asynchronous and
    // Start() re-primes the pipeline behind it.
    cam->SetVideoType( Core::GrayscaleMode );
    cam->Start();
    cam->SetVideoType( Core::GrayscaleMode );
    log( std::string( "video mode after Start(): " ) + videoModeName( cam->VideoType() ) +
        " (frames still in flight may report the previous mode)" );

    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mCamera = cam;
    }

    readCaps( cam );

    const CameraCaps c = caps();

    // Frame rate first: the exposure ceiling is derived from the frame period,
    // so the range read during readCaps() belongs to whatever rate the camera
    // powered up at. Re-read it before clamping the exposure default to it.
    setFrameRate( mCfg.defaults.frameRateHz );
    const Range exposureRange = refreshExposureRange();
    if( exposureRange.lo != c.exposure.lo || exposureRange.hi != c.exposure.hi )
        log( "exposure range at " + std::to_string( mCfg.defaults.frameRateHz ) + " Hz: " +
            std::to_string( exposureRange.lo ) + ".." + std::to_string( exposureRange.hi ) );

    setExposure( std::clamp( mCfg.defaults.exposure, exposureRange.lo, exposureRange.hi ) );
    if( c.gainAvailable ) setImagerGain( mCfg.defaults.imagerGain );
    if( c.irAvailable ) setIntensity( mCfg.defaults.irIntensity );
    setIRFilter( mCfg.defaults.irFilter );

    return true;
}

void CameraSource::readCaps( const std::shared_ptr<Camera>& cam )
{
    std::vector<std::string> notes;
    CameraCaps c;

    c.serial = cam->Serial();
    c.name = cam->Name() ? cam->Name() : "";
    c.revision = cam->Revision();
    c.width = cam->PhysicalPixelWidth();
    c.height = cam->PhysicalPixelHeight();
    if( c.width <= 0 || c.height <= 0 )
    {
        c.width = mCfg.nativeWidth;
        c.height = mCfg.nativeHeight;
        notes.push_back( "SDK reported no imager size; using cameras.yaml resolution" );
    }

    c.exposure = pickRange( cam->MinimumExposureValue(), cam->MaximumExposureValue(),
        mCfg.fallback.exposure, "exposure", notes );
    c.frameRate = pickRange( cam->MinimumFrameRateValue(), cam->MaximumFrameRateValue(),
        mCfg.fallback.frameRate, "frame rate", notes );
    c.irIntensity = pickRange( cam->MinimumIntensity(), cam->MaximumIntensity(),
        mCfg.fallback.irIntensity, "IR intensity", notes );

    c.gainAvailable = cam->IsImagerGainAvailable();
    const int gainLevels = cam->ImagerGainLevels();
    c.imagerGain = pickRange( 0, gainLevels > 0 ? gainLevels - 1 : 0,
        mCfg.fallback.imagerGain, "imager gain", notes );

    c.irAvailable = cam->IsIRIlluminationAvailable();
    c.filterSwitchAvailable = cam->IsFilterSwitchAvailable();

    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mCaps = c;
    }

    log( "camera " + std::to_string( c.serial ) + " rev " + std::to_string( c.revision ) + " '" +
        c.name + "' " + std::to_string( c.width ) + "x" + std::to_string( c.height ) );
    log( "ranges: exposure " + std::to_string( c.exposure.lo ) + ".." +
        std::to_string( c.exposure.hi ) + ", fps " + std::to_string( c.frameRate.lo ) + ".." +
        std::to_string( c.frameRate.hi ) + ", gain " + std::to_string( c.imagerGain.lo ) + ".." +
        std::to_string( c.imagerGain.hi ) + ( c.gainAvailable ? "" : " (reported unavailable)" ) +
        ", IR " + std::to_string( c.irIntensity.lo ) + ".." + std::to_string( c.irIntensity.hi ) +
        ( c.irAvailable ? "" : " (reported unavailable)" ) +
        ", IR filter switch " + ( c.filterSwitchAvailable ? "available" : "unavailable" ) );
    for( const auto& n : notes ) log( n );
}

CameraCaps CameraSource::caps() const
{
    std::lock_guard<std::mutex> lock( mStateMutex );
    return mCaps;
}

std::shared_ptr<Camera> CameraSource::camera() const
{
    std::lock_guard<std::mutex> lock( mStateMutex );
    return mCamera;
}

std::shared_ptr<const ViewFrame> CameraSource::latest() const
{
    std::lock_guard<std::mutex> lock( mStateMutex );
    return mLatest;
}

// --- controls --------------------------------------------------------------
// Each setter records the request under the lock, then drops the lock before
// touching the SDK. Holding our mutex across an SDK call risks deadlocking
// against the library's own threads.

void CameraSource::setExposure( int value )
{
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mRequested.exposureRequested = value;
    }
    if( auto cam = camera() ) cam->SetExposure( value );
}

void CameraSource::setImagerGain( int level )
{
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mRequested.gainRequested = level;
    }
    if( auto cam = camera() ) cam->SetImagerGain( static_cast<eImagerGain>( level ) );
}

void CameraSource::setIntensity( int value )
{
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mRequested.intensityRequested = value;
    }
    if( auto cam = camera() ) cam->SetIntensity( value );
}

void CameraSource::setFrameRate( int hz )
{
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mRequested.frameRateRequested = hz;
    }
    mFrameRateForCap.store( hz );
    if( auto cam = camera() ) cam->SetFrameRate( hz );
}

// Measured on 33661: the exposure maximum is floor(1e6 / fps) - 200, and the
// SDK updates it as soon as the frame rate command lands. 240 Hz -> 3966,
// 120 Hz -> 8133, 60 Hz -> 16466, 30 Hz -> 33133. Leaving the slider on a stale
// bound silently caps how bright the image can get.
Range CameraSource::refreshExposureRange()
{
    auto cam = camera();
    if( !cam ) return caps().exposure;

    const int lo = cam->MinimumExposureValue();
    const int hi = cam->MaximumExposureValue();

    std::lock_guard<std::mutex> lock( mStateMutex );
    if( hi > lo ) mCaps.exposure = { lo, hi };
    return mCaps.exposure;
}

// The 850 nm bandpass has to come out of the light path for a printed board:
// with it in, the camera sees only IR and the checkerboard barely registers.
void CameraSource::setIRFilter( bool enabled )
{
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        mRequested.irFilterRequested = enabled ? 1 : 0;
    }
    if( auto cam = camera() ) cam->SetIRFilter( enabled );
}

void CameraSource::setDetectionEnabled( bool on ) { mDetectEnabled.store( on ); }

CameraSettings CameraSource::readSettings( const std::shared_ptr<Camera>& cam ) const
{
    CameraSettings s;
    {
        std::lock_guard<std::mutex> lock( mStateMutex );
        s = mRequested;
    }
    // Readback outside the lock. Exposure especially: a request above what the
    // frame period allows is applied at the cap with no error returned.
    s.exposureActual = cam->Exposure();
    s.gainActual = static_cast<int>( cam->ImagerGain() );
    s.intensityActual = cam->Intensity();
    s.frameRateActual = cam->FrameRate();
    s.deviceFrameRate = cam->ActualFrameRate();
    s.irFilterActual = cam->IRFilter() ? 1 : 0;
    s.videoType = static_cast<int>( cam->VideoType() );
    return s;
}

// --- capture ---------------------------------------------------------------

void CameraSource::start()
{
    if( mRunning.exchange( true ) ) return;
    mThread = std::thread( [this] { captureLoop(); } );
}

void CameraSource::stop()
{
    if( !mRunning.exchange( false ) ) return;
    if( mThread.joinable() ) mThread.join();
}

void CameraSource::captureLoop()
{
    const auto publishPeriod = std::chrono::microseconds( 1000000 / std::max( 1, mCfg.displayFps ) );
    auto nextPublish = clock_type::now();

    int wrongModeStreak = 0;
    auto lastReassert = clock_type::now();
    bool loggedSizeMismatch = false;

    int framesSinceTick = 0;
    auto lastTick = clock_type::now();
    double measuredFps = 0.0;

    while( mRunning.load() )
    {
        auto cam = camera();
        if( !cam )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
            continue;
        }

        // NextFrame() returns the OLDEST queued frame, so taking one per
        // iteration falls further behind every pass at 120 Hz. Drain to the
        // end and keep only the newest.
        std::shared_ptr<const Frame> newest;
        while( auto f = cam->NextFrame() )
        {
            newest = f;
            ++framesSinceTick;
        }

        const auto now = clock_type::now();
        if( now - lastTick >= std::chrono::milliseconds( 500 ) )
        {
            const double secs = std::chrono::duration<double>( now - lastTick ).count();
            measuredFps = framesSinceTick / secs;
            framesSinceTick = 0;
            lastTick = now;
        }

        if( !newest )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
            continue;
        }

        // SetVideoType is asynchronous: VideoType() flips at once, but frames
        // already queued still carry the old mode and GrayscaleData() on an
        // object-mode frame returns null. Skip them; if they keep coming the
        // mode command was lost, so reassert.
        if( newest->FrameType() != Core::GrayscaleMode )
        {
            if( ++wrongModeStreak == 1 )
                log( std::string( "draining frames still in " ) +
                    videoModeName( newest->FrameType() ) + " mode" );

            if( wrongModeStreak > 120 && now - lastReassert > std::chrono::seconds( 2 ) )
            {
                cam->SetVideoType( Core::GrayscaleMode );
                lastReassert = now;
                log( "still not grayscale after " + std::to_string( wrongModeStreak ) +
                    " frames; reasserting grayscale mode" );
            }
            continue;
        }
        if( wrongModeStreak )
        {
            log( "grayscale frames flowing after " + std::to_string( wrongModeStreak ) +
                " stale frame(s)" );
            wrongModeStreak = 0;
        }

        if( now < nextPublish ) continue;  // drained above; nothing else owed
        nextPublish = now + publishPeriod;

        const int w = newest->Width();
        const int h = newest->Height();
        if( w <= 0 || h <= 0 ) continue;

        const unsigned char* bits = newest->GrayscaleData( *cam );
        if( !bits ) continue;
        if( newest->GrayscaleDataSize() < w * h )
        {
            if( !loggedSizeMismatch )
            {
                loggedSizeMismatch = true;
                log( "grayscale payload is " + std::to_string( newest->GrayscaleDataSize() ) +
                    " bytes for a " + std::to_string( w ) + "x" + std::to_string( h ) +
                    " frame; skipping" );
            }
            continue;
        }

        auto view = std::make_shared<ViewFrame>();
        // Wraps SDK-owned memory; clone before the frame goes out of scope.
        view->raw = cv::Mat( h, w, CV_8UC1, const_cast<unsigned char*>( bits ) ).clone();
        view->frameId = newest->FrameID();
        view->timestamp = newest->TimeStamp();
        view->measuredFps = measuredFps;
        view->settings = readSettings( cam );

        if( mDetectEnabled.load() ) view->detection = mDetector.detect( view->raw );

        cv::cvtColor( view->raw, view->display, cv::COLOR_GRAY2BGR );
        BoardDetector::draw( view->display, view->detection );

        {
            std::lock_guard<std::mutex> lock( mStateMutex );
            view->seq = ++mSeq;
            mLatest = view;
        }
    }
}

}  // namespace intrinsics
