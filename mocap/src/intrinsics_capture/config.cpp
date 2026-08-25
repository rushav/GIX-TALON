#include "config.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;

namespace intrinsics {
namespace {

const char* kCamerasRel = "mocap/config/cameras.yaml";

fs::path executableDir()
{
    std::error_code ec;
    fs::path exe = fs::read_symlink( "/proc/self/exe", ec );
    return ec ? fs::path{} : exe.parent_path();
}

std::optional<fs::path> searchUp( fs::path dir )
{
    std::error_code ec;
    dir = fs::absolute( dir, ec );
    for( ; !dir.empty(); dir = dir.parent_path() )
    {
        if( fs::exists( dir / kCamerasRel ) ) return dir;
        if( dir == dir.parent_path() ) break;
    }
    return std::nullopt;
}

// yaml-cpp throws on a type mismatch; these keep "absent means default" from
// being confused with "present but wrong", which should surface as an error.
template <typename T>
T get( const YAML::Node& node, const char* key, T fallback )
{
    if( !node || !node[key] || node[key].IsNull() ) return fallback;
    return node[key].as<T>();
}

Range getRange( const YAML::Node& node, const char* key, Range fallback )
{
    if( !node || !node[key] || node[key].size() != 2 ) return fallback;
    return { node[key][0].as<int>(), node[key][1].as<int>() };
}

void getBounds( const YAML::Node& node, const char* key, double& lo, double& hi )
{
    if( !node || !node[key] || node[key].size() != 2 ) return;
    lo = node[key][0].as<double>();
    hi = node[key][1].as<double>();
}

YAML::Node load( const fs::path& path )
{
    try
    {
        return YAML::LoadFile( path.string() );
    }
    catch( const YAML::Exception& e )
    {
        throw std::runtime_error( path.string() + ": " + e.what() );
    }
}

}  // namespace

fs::path findRepoRoot( const fs::path& hint )
{
    if( !hint.empty() )
    {
        if( fs::exists( hint / kCamerasRel ) ) return hint;
        throw std::runtime_error( "no " + std::string( kCamerasRel ) + " under " + hint.string() );
    }
    if( auto r = searchUp( fs::current_path() ) ) return *r;
    if( auto d = executableDir(); !d.empty() )
        if( auto r = searchUp( d ) ) return *r;

    throw std::runtime_error( "could not locate " + std::string( kCamerasRel ) +
        " by searching up from the working directory or the executable; pass --repo-root" );
}

unsigned AppConfig::resolveSerial( std::optional<unsigned> requested ) const
{
    if( requested )
    {
        for( const auto& c : cameras )
        {
            if( c.serial != *requested ) continue;
            if( c.status == "excluded" )
            {
                throw std::runtime_error( "camera " + std::to_string( c.serial ) +
                    " is marked excluded in " + camerasFile.string() +
                    ( c.reason.empty() ? "" : " (" + c.reason + ")" ) );
            }
            return c.serial;
        }
        throw std::runtime_error( "serial " + std::to_string( *requested ) + " is not listed in " +
            camerasFile.string() );
    }

    for( const auto& c : cameras )
        if( c.status == "active" ) return c.serial;

    throw std::runtime_error( "no camera with status: active in " + camerasFile.string() +
        "; pass --serial to choose one explicitly" );
}

AppConfig loadConfig( const fs::path& repoRoot )
{
    AppConfig cfg;
    cfg.repoRoot = repoRoot;
    cfg.camerasFile = repoRoot / kCamerasRel;
    cfg.toolFile = repoRoot / "mocap/config/intrinsics_capture.yaml";

    // --- camera inventory -------------------------------------------------
    YAML::Node cams = load( cfg.camerasFile );
    cfg.cameraModel = get<std::string>( cams, "model", "unknown" );
    cfg.nominalFrameRateHz = get( cams, "frame_rate_hz", 0 );

    if( cams["resolution"] && cams["resolution"].size() == 2 )
    {
        cfg.nativeWidth = cams["resolution"][0].as<int>();
        cfg.nativeHeight = cams["resolution"][1].as<int>();
    }
    if( cfg.nativeWidth <= 0 || cfg.nativeHeight <= 0 )
        throw std::runtime_error( cfg.camerasFile.string() + ": missing 'resolution: [w, h]'" );

    if( !cams["cameras"] || cams["cameras"].size() == 0 )
        throw std::runtime_error( cfg.camerasFile.string() + ": no 'cameras:' entries" );
    for( const auto& item : cams["cameras"] )
    {
        CameraEntry e;
        e.serial = get<unsigned>( item, "serial", 0 );
        e.status = get<std::string>( item, "status", "available" );
        e.reason = get<std::string>( item, "reason", "" );
        if( e.serial != 0 ) cfg.cameras.push_back( e );
    }

    // --- tool settings ----------------------------------------------------
    YAML::Node tool = load( cfg.toolFile );
    cfg.sessionRoot = repoRoot / get<std::string>( tool, "session_root", "mocap/sessions" );
    cfg.boardFile =
        cfg.toolFile.parent_path() / get<std::string>( tool, "board_config", "charuco_board.yaml" );

    cfg.displayFps = std::max( 1, get( tool["video"], "display_fps", cfg.displayFps ) );

    if( const auto d = tool["defaults"] )
    {
        cfg.defaults.exposure = get( d, "exposure", cfg.defaults.exposure );
        cfg.defaults.imagerGain = get( d, "imager_gain", cfg.defaults.imagerGain );
        cfg.defaults.irIntensity = get( d, "ir_intensity", cfg.defaults.irIntensity );
        cfg.defaults.frameRateHz = get( d, "frame_rate_hz",
            cfg.nominalFrameRateHz > 0 ? cfg.nominalFrameRateHz : cfg.defaults.frameRateHz );
        cfg.defaults.irFilter = get( d, "ir_filter", cfg.defaults.irFilter );
    }

    if( const auto d = tool["discovery"] )
    {
        cfg.discovery.pollIntervalMs = get( d, "poll_interval_ms", cfg.discovery.pollIntervalMs );
        cfg.discovery.settleMs = get( d, "settle_ms", cfg.discovery.settleMs );
        cfg.discovery.timeoutMs = get( d, "timeout_ms", cfg.discovery.timeoutMs );
    }

    if( const auto fb = tool["control_ranges_fallback"] )
    {
        cfg.fallback.exposure = getRange( fb, "exposure", cfg.fallback.exposure );
        cfg.fallback.imagerGain = getRange( fb, "imager_gain", cfg.fallback.imagerGain );
        cfg.fallback.irIntensity = getRange( fb, "ir_intensity", cfg.fallback.irIntensity );
        cfg.fallback.frameRate = getRange( fb, "frame_rate_hz", cfg.fallback.frameRate );
    }

    cfg.refreshExposureRangeOnFrameRateChange = get( tool["exposure"],
        "refresh_range_on_frame_rate_change", cfg.refreshExposureRangeOnFrameRateChange );

    if( const auto c = tool["coverage"] )
    {
        cfg.coverage.cols = std::max( 1, get( c, "grid_cols", cfg.coverage.cols ) );
        cfg.coverage.rows = std::max( 1, get( c, "grid_rows", cfg.coverage.rows ) );
        cfg.coverage.targetPerCell = std::max( 1, get( c, "target_per_cell", cfg.coverage.targetPerCell ) );
    }

    if( const auto p = tool["pose"] )
    {
        cfg.pose.guessFx = get( p, "guess_fx", cfg.pose.guessFx );
        cfg.pose.guessFy = get( p, "guess_fy", cfg.pose.guessFy );
        cfg.pose.guessCx = get( p, "guess_cx", cfg.pose.guessCx );
        cfg.pose.guessCy = get( p, "guess_cy", cfg.pose.guessCy );
        cfg.pose.minTiltDegrees = get( p, "min_tilt_degrees", cfg.pose.minTiltDegrees );
        cfg.pose.minFractionTilted = get( p, "min_fraction_tilted", cfg.pose.minFractionTilted );
        cfg.pose.histogramBinDegrees = get( p, "histogram_bin_degrees", cfg.pose.histogramBinDegrees );
        cfg.pose.histogramBins = std::max( 1, get( p, "histogram_bins", cfg.pose.histogramBins ) );
    }

    {
        const auto s = tool["solver"];
        // Relative entries resolve against the repo root so the tool runs from
        // anywhere; an absolute path in the config is taken as given.
        auto resolve = [&]( const std::string& raw ) {
            fs::path p( raw );
            return p.is_absolute() ? p : repoRoot / p;
        };
        cfg.solver.python = resolve( get<std::string>( s, "python", ".venv/bin/python" ) );
        cfg.solver.script =
            resolve( get<std::string>( s, "script", "mocap/python/solve_intrinsics.py" ) );
        cfg.solver.minCorners = get( s, "min_corners", cfg.solver.minCorners );
        cfg.solver.timeoutSeconds = get( s, "timeout_seconds", cfg.solver.timeoutSeconds );
    }

    if( const auto r = tool["reference"] )
    {
        getBounds( r, "rms_px", cfg.reference.rmsLo, cfg.reference.rmsHi );
        getBounds( r, "focal_px", cfg.reference.focalLo, cfg.reference.focalHi );
        getBounds( r, "cx_px", cfg.reference.cxLo, cfg.reference.cxHi );
        getBounds( r, "cy_px", cfg.reference.cyLo, cfg.reference.cyHi );
        cfg.reference.focalAgreementPercent =
            get( r, "focal_agreement_percent", cfg.reference.focalAgreementPercent );
        cfg.reference.nominalCx = get( r, "nominal_cx", cfg.reference.nominalCx );
        cfg.reference.nominalCy = get( r, "nominal_cy", cfg.reference.nominalCy );
    }

    // --- board geometry ---------------------------------------------------
    YAML::Node board = load( cfg.boardFile );
    cfg.board.dictionary = get<std::string>( board, "dictionary", "" );
    cfg.board.squaresX = get( board, "squares_x", 0 );
    cfg.board.squaresY = get( board, "squares_y", 0 );
    cfg.board.squareLengthMm = get( board, "square_length_mm", 0.0 );
    cfg.board.markerLengthMm = get( board, "marker_length_mm", 0.0 );
    cfg.board.markerCount = get( board, "marker_count", 0 );
    cfg.board.legacyPattern = get( board, "legacy_pattern", true );

    if( cfg.board.dictionary.empty() || cfg.board.squaresX <= 1 || cfg.board.squaresY <= 1 )
        throw std::runtime_error( cfg.boardFile.string() + ": dictionary/squares_x/squares_y required" );
    if( cfg.board.markerCount <= 0 )
        cfg.board.markerCount = ( cfg.board.squaresX * cfg.board.squaresY ) / 2;

    return cfg;
}

}  // namespace intrinsics
