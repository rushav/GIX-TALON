#pragma once

#include "config.h"

#include <opencv2/core.hpp>

#include <vector>

namespace intrinsics {

// Counts accepted ChArUco corners per frame region.
//
// This exists because RMS is blind to it. Reprojection error is only computed
// where observations were made, so a set with whole regions empty can report a
// healthy RMS while the distortion model extrapolates across the gaps. Measured
// on a synthetic set with known truth: 18 of 30 cells empty gave RMS 0.262 px
// and focal recovered to 0.2%, yet undistortion at the frame edge was off by
// 56 px. Filling the same grid dropped that to 27 px with RMS essentially
// unchanged at 0.223 px. Coverage has to be watched during capture; discovering
// it afterwards means recapturing.
class CoverageGrid
{
public:
    CoverageGrid( const CoverageConfig& cfg, int frameWidth, int frameHeight );

    void add( const std::vector<cv::Point2f>& corners );
    void remove( const std::vector<cv::Point2f>& corners );
    void clear();

    int cols() const { return mCfg.cols; }
    int rows() const { return mCfg.rows; }
    int at( int col, int row ) const { return mCells[index( col, row )]; }
    int targetPerCell() const { return mCfg.targetPerCell; }

    int emptyCells() const;
    int totalCorners() const;
    // 0..1 per cell, saturating at targetPerCell.
    double fill( int col, int row ) const;

private:
    int index( int col, int row ) const { return row * mCfg.cols + col; }
    // Returns false for a point outside the frame.
    bool cellOf( const cv::Point2f& p, int& col, int& row ) const;

    CoverageConfig mCfg;
    int mWidth;
    int mHeight;
    std::vector<int> mCells;
};

}  // namespace intrinsics
