#include "coverage_grid.h"

#include <algorithm>

namespace intrinsics {

CoverageGrid::CoverageGrid( const CoverageConfig& cfg, int frameWidth, int frameHeight )
    : mCfg( cfg )
    , mWidth( std::max( 1, frameWidth ) )
    , mHeight( std::max( 1, frameHeight ) )
    , mCells( static_cast<size_t>( cfg.cellCount() ), 0 )
{
}

bool CoverageGrid::cellOf( const cv::Point2f& p, int& col, int& row ) const
{
    if( p.x < 0 || p.y < 0 || p.x >= mWidth || p.y >= mHeight ) return false;
    col = std::min( mCfg.cols - 1, static_cast<int>( p.x * mCfg.cols / mWidth ) );
    row = std::min( mCfg.rows - 1, static_cast<int>( p.y * mCfg.rows / mHeight ) );
    return true;
}

void CoverageGrid::add( const std::vector<cv::Point2f>& corners )
{
    int c = 0, r = 0;
    for( const auto& p : corners )
        if( cellOf( p, c, r ) ) ++mCells[index( c, r )];
}

void CoverageGrid::remove( const std::vector<cv::Point2f>& corners )
{
    int c = 0, r = 0;
    for( const auto& p : corners )
        if( cellOf( p, c, r ) ) mCells[index( c, r )] = std::max( 0, mCells[index( c, r )] - 1 );
}

void CoverageGrid::clear() { std::fill( mCells.begin(), mCells.end(), 0 ); }

int CoverageGrid::emptyCells() const
{
    return static_cast<int>( std::count( mCells.begin(), mCells.end(), 0 ) );
}

int CoverageGrid::totalCorners() const
{
    int sum = 0;
    for( int v : mCells ) sum += v;
    return sum;
}

double CoverageGrid::fill( int col, int row ) const
{
    return std::min( 1.0, static_cast<double>( at( col, row ) ) / mCfg.targetPerCell );
}

}  // namespace intrinsics
