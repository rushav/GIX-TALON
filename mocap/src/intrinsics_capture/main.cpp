#include "camera_source.h"
#include "config.h"
#include "main_window.h"

#include <QApplication>
#include <QMessageBox>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>

using namespace intrinsics;

namespace {

struct Options
{
    std::optional<unsigned> serial;
    std::filesystem::path repoRoot;
    bool listOnly = false;
};

void usage( const char* argv0 )
{
    std::printf(
        "usage: %s [--serial N] [--repo-root PATH] [--list]\n"
        "\n"
        "  --serial N      capture from this camera; must be listed in\n"
        "                  mocap/config/cameras.yaml and not marked excluded.\n"
        "                  Default: the first entry with status: active.\n"
        "  --repo-root P   repo root holding mocap/config/. Default: search\n"
        "                  upward from the working directory, then the binary.\n"
        "  --list          enumerate the cameras the SDK sees, then exit.\n",
        argv0 );
}

bool parseArgs( int argc, char** argv, Options& opt )
{
    for( int i = 1; i < argc; ++i )
    {
        const std::string a = argv[i];
        auto next = [&]( const char* what ) -> const char* {
            if( i + 1 >= argc )
            {
                std::fprintf( stderr, "%s requires a value\n", what );
                return nullptr;
            }
            return argv[++i];
        };

        if( a == "--serial" )
        {
            const char* v = next( "--serial" );
            if( !v ) return false;
            opt.serial = static_cast<unsigned>( std::strtoul( v, nullptr, 10 ) );
        }
        else if( a == "--repo-root" )
        {
            const char* v = next( "--repo-root" );
            if( !v ) return false;
            opt.repoRoot = v;
        }
        else if( a == "--list" )
        {
            opt.listOnly = true;
        }
        else if( a == "-h" || a == "--help" )
        {
            usage( argv[0] );
            std::exit( 0 );
        }
        else
        {
            std::fprintf( stderr, "unknown argument: %s\n", a.c_str() );
            usage( argv[0] );
            return false;
        }
    }
    return true;
}

}  // namespace

int main( int argc, char** argv )
{
    Options opt;
    if( !parseArgs( argc, argv, opt ) ) return 2;

    QApplication app( argc, argv );

    AppConfig cfg;
    try
    {
        cfg = loadConfig( findRepoRoot( opt.repoRoot ) );
    }
    catch( const std::exception& e )
    {
        std::fprintf( stderr, "config error: %s\n", e.what() );
        QMessageBox::critical( nullptr, QStringLiteral( "Config error" ), QString::fromUtf8( e.what() ) );
        return 1;
    }

    std::fprintf( stderr, "[intrinsics_capture] repo root: %s\n", cfg.repoRoot.c_str() );

    try
    {
        CameraSource source( cfg, cfg.board );

        // Discovery is staggered; this returns once the device count settles.
        const auto devices = source.discover();

        if( opt.listOnly )
        {
            std::printf( "%-10s %-6s %-24s %s\n", "serial", "rev", "name", "state" );
            for( const auto& d : devices )
                std::printf( "%-10u %-6d %-24s %s\n", d.serial, d.revision, d.name.c_str(),
                    d.state.c_str() );
            return 0;
        }

        const unsigned serial = cfg.resolveSerial( opt.serial );

        bool present = false;
        for( const auto& d : devices ) present = present || d.serial == serial;
        if( !present )
        {
            std::string seen;
            for( const auto& d : devices ) seen += ( seen.empty() ? "" : ", " ) + std::to_string( d.serial );
            throw std::runtime_error( "camera " + std::to_string( serial ) +
                " was not discovered. Seen: " + ( seen.empty() ? "none" : seen ) +
                ".\nCheck the camera network (enp6s0, UDP 13013) and that the binary has "
                "cap_net_raw,cap_net_admin - setcap is stripped on every relink." );
        }

        std::string error;
        if( !source.open( serial, error ) ) throw std::runtime_error( error );

        source.start();

        MainWindow window( cfg, source );
        window.show();

        const int rc = app.exec();
        source.stop();
        return rc;
    }
    catch( const std::exception& e )
    {
        std::fprintf( stderr, "error: %s\n", e.what() );
        QMessageBox::critical( nullptr, QStringLiteral( "Camera error" ), QString::fromUtf8( e.what() ) );
        return 1;
    }
}
