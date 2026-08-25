#include "capture_session.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <QtGlobal>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace intrinsics {
namespace {

const int kThumbnailWidth = 160;

std::string stamp( const char* fmt )
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t( now );
    std::tm tm{};
    gmtime_r( &t, &tm );
    char buf[64];
    std::strftime( buf, sizeof( buf ), fmt, &tm );
    return buf;
}

std::string jsonEscape( const std::string& s )
{
    std::string out;
    out.reserve( s.size() + 8 );
    for( char c : s )
    {
        switch( c )
        {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if( static_cast<unsigned char>( c ) < 0x20 )
                {
                    char esc[8];
                    std::snprintf( esc, sizeof( esc ), "\\u%04x", c );
                    out += esc;
                }
                else
                {
                    out += c;
                }
        }
    }
    return out;
}

std::string q( const std::string& s ) { return "\"" + jsonEscape( s ) + "\""; }

std::string numberList( const std::vector<int>& v )
{
    std::ostringstream os;
    os << '[';
    for( size_t i = 0; i < v.size(); ++i ) os << ( i ? ", " : "" ) << v[i];
    os << ']';
    return os.str();
}

}  // namespace

CaptureSession::CaptureSession( const AppConfig& cfg, const CameraCaps& caps,
    std::string sdkVersion )
    : mCfg( cfg )
    , mCaps( caps )
    , mSdkVersion( std::move( sdkVersion ) )
    , mCreatedUtc( stamp( "%Y-%m-%dT%H:%M:%SZ" ) )
    , mCoverage( cfg.coverage, caps.width > 0 ? caps.width : cfg.nativeWidth,
          caps.height > 0 ? caps.height : cfg.nativeHeight )
{
    mDir = mCfg.sessionRoot /
        ( "intrinsics_" + std::to_string( caps.serial ) + "_" + stamp( "%Y%m%d_%H%M%S" ) );
}

std::vector<double> CaptureSession::tilts() const
{
    std::vector<double> out;
    out.reserve( mRecords.size() );
    for( const auto& r : mRecords )
        if( r.tiltDegrees >= 0 ) out.push_back( r.tiltDegrees );
    return out;
}

std::string CaptureSession::add( const ViewFrame& view, double tiltDegrees )
{
    if( view.raw.empty() ) throw std::runtime_error( "no frame to capture yet" );

    if( !mCreated )
    {
        std::error_code ec;
        fs::create_directories( mDir, ec );
        if( ec ) throw std::runtime_error( "cannot create " + mDir.string() + ": " + ec.message() );
        mCreated = true;
    }

    std::ostringstream name;
    name << "frame_" << std::setw( 4 ) << std::setfill( '0' ) << mNextIndex << ".png";

    const fs::path path = mDir / name.str();
    // PNG level 1: these are 1.3 MP 8-bit frames and capture cadence matters
    // more than a few hundred KB per file.
    if( !cv::imwrite( path.string(), view.raw, { cv::IMWRITE_PNG_COMPRESSION, 1 } ) )
        throw std::runtime_error( "cv::imwrite failed for " + path.string() );
    ++mNextIndex;

    CaptureRecord rec;
    rec.file = name.str();
    rec.capturedUtc = stamp( "%Y-%m-%dT%H:%M:%SZ" );
    rec.frameId = view.frameId;
    rec.cameraTimestamp = view.timestamp;
    rec.settings = view.settings;
    rec.markerCount = view.detection.markerCount();
    rec.cornerCount = view.detection.cornerCount();
    rec.markerIds = view.detection.ids;
    rec.charucoCorners = view.detection.charucoCorners;
    rec.tiltDegrees = tiltDegrees;

    const double scale = static_cast<double>( kThumbnailWidth ) / std::max( 1, view.display.cols );
    cv::resize( view.display, rec.thumbnail, cv::Size(), scale, scale, cv::INTER_AREA );

    mCoverage.add( rec.charucoCorners );
    mRecords.push_back( std::move( rec ) );

    // Rewritten after every capture so an interrupted session still has a
    // manifest that matches the PNGs on disk.
    writeManifest();
    return mRecords.back().file;
}

void CaptureSession::remove( int index )
{
    if( index < 0 || index >= count() ) return;

    const CaptureRecord& rec = mRecords[static_cast<size_t>( index )];
    std::error_code ec;
    fs::remove( mDir / rec.file, ec );  // a missing file is not worth failing over

    mCoverage.remove( rec.charucoCorners );
    mRecords.erase( mRecords.begin() + index );
    writeManifest();
}

void CaptureSession::writeManifest() const
{
    const fs::path path = mDir / "session.json";
    std::ofstream out( path );
    if( !out ) throw std::runtime_error( "cannot write " + path.string() );

    out << std::fixed << std::setprecision( 6 );
    out << "{\n";
    out << "  \"tool\": \"intrinsics_capture\",\n";
    out << "  \"stage\": \"acquisition\",\n";
    out << "  \"created_utc\": " << q( mCreatedUtc ) << ",\n";

    out << "  \"camera\": {\n";
    out << "    \"serial\": " << mCaps.serial << ",\n";
    out << "    \"model\": " << q( mCfg.cameraModel ) << ",\n";
    out << "    \"name\": " << q( mCaps.name ) << ",\n";
    out << "    \"revision\": " << mCaps.revision << ",\n";
    out << "    \"width\": " << mCaps.width << ",\n";
    out << "    \"height\": " << mCaps.height << "\n";
    out << "  },\n";

    out << "  \"software\": {\n";
    out << "    \"camera_sdk_version\": " << q( mSdkVersion ) << ",\n";
    out << "    \"opencv_version\": " << q( CV_VERSION ) << ",\n";
    out << "    \"qt_version\": " << q( QT_VERSION_STR ) << "\n";
    out << "  },\n";

    const BoardConfig& b = mCfg.board;
    out << "  \"board\": {\n";
    out << "    \"type\": \"charuco\",\n";
    out << "    \"dictionary\": " << q( b.dictionary ) << ",\n";
    out << "    \"squares_x\": " << b.squaresX << ",\n";
    out << "    \"squares_y\": " << b.squaresY << ",\n";
    out << "    \"square_length_mm\": " << b.squareLengthMm << ",\n";
    out << "    \"marker_length_mm\": " << b.markerLengthMm << ",\n";
    out << "    \"marker_count\": " << b.markerCount << ",\n";
    out << "    \"charuco_corner_count\": " << b.charucoCornerCount() << ",\n";
    out << "    \"legacy_pattern\": " << ( b.legacyPattern ? "true" : "false" ) << "\n";
    out << "  },\n";

    // Coverage travels with the session so the solve result can be read next to
    // the sampling that produced it.
    out << "  \"coverage\": {\n";
    out << "    \"grid_cols\": " << mCoverage.cols() << ",\n";
    out << "    \"grid_rows\": " << mCoverage.rows() << ",\n";
    out << "    \"target_per_cell\": " << mCoverage.targetPerCell() << ",\n";
    out << "    \"empty_cells\": " << mCoverage.emptyCells() << ",\n";
    out << "    \"total_corners\": " << mCoverage.totalCorners() << ",\n";
    out << "    \"cells\": [";
    for( int r = 0; r < mCoverage.rows(); ++r )
    {
        out << ( r ? ",\n              " : "\n              " ) << "[";
        for( int c = 0; c < mCoverage.cols(); ++c )
            out << ( c ? ", " : "" ) << mCoverage.at( c, r );
        out << "]";
    }
    out << "\n    ]\n";
    out << "  },\n";

    out << "  \"frames\": [";
    for( size_t i = 0; i < mRecords.size(); ++i )
    {
        const CaptureRecord& f = mRecords[i];
        out << ( i ? ",\n" : "\n" );
        out << "    {\n";
        out << "      \"file\": " << q( f.file ) << ",\n";
        out << "      \"captured_utc\": " << q( f.capturedUtc ) << ",\n";
        out << "      \"camera_frame_id\": " << f.frameId << ",\n";
        out << "      \"camera_timestamp\": " << f.cameraTimestamp << ",\n";
        out << "      \"exposure_requested\": " << f.settings.exposureRequested << ",\n";
        out << "      \"exposure_actual\": " << f.settings.exposureActual << ",\n";
        out << "      \"imager_gain_requested\": " << f.settings.gainRequested << ",\n";
        out << "      \"imager_gain_actual\": " << f.settings.gainActual << ",\n";
        out << "      \"ir_intensity_requested\": " << f.settings.intensityRequested << ",\n";
        out << "      \"ir_intensity_actual\": " << f.settings.intensityActual << ",\n";
        out << "      \"frame_rate_requested\": " << f.settings.frameRateRequested << ",\n";
        out << "      \"frame_rate_actual\": " << f.settings.frameRateActual << ",\n";
        out << "      \"imager_frame_rate\": " << f.settings.deviceFrameRate << ",\n";
        out << "      \"ir_filter_requested\": "
            << ( f.settings.irFilterRequested > 0 ? "true" : "false" ) << ",\n";
        out << "      \"ir_filter_actual\": "
            << ( f.settings.irFilterActual > 0 ? "true" : "false" ) << ",\n";
        out << "      \"markers_detected\": " << f.markerCount << ",\n";
        out << "      \"charuco_corners\": " << f.cornerCount << ",\n";
        out << "      \"tilt_degrees\": " << f.tiltDegrees << ",\n";
        out << "      \"marker_ids\": " << numberList( f.markerIds ) << "\n";
        out << "    }";
    }
    out << ( mRecords.empty() ? "" : "\n  " ) << "]\n";
    out << "}\n";

    if( !out ) throw std::runtime_error( "write failed for " + path.string() );
}

}  // namespace intrinsics
