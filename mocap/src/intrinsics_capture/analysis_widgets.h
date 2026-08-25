#pragma once

#include <QString>
#include <QWidget>
#include <vector>

namespace intrinsics {

// The 5x6 frame-coverage map. Shading is corner count against the per-cell
// target; empty cells are called out because they are the ones RMS cannot see.
class CoverageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CoverageWidget( QWidget* parent = nullptr );

    void setData( int cols, int rows, std::vector<int> cells, int targetPerCell,
        double frameAspect );

protected:
    void paintEvent( QPaintEvent* ) override;

private:
    int mCols = 0;
    int mRows = 0;
    int mTarget = 1;
    double mAspect = 1.25;   // frame width / height, so a cell maps to a real region
    std::vector<int> mCells;
};

// Distribution of board tilt across captures, with the fronto-parallel warning.
class TiltHistogramWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TiltHistogramWidget( QWidget* parent = nullptr );

    void setData( std::vector<int> bins, double binDegrees, double minTiltDegrees,
        double tiltedFraction, double requiredFraction );

protected:
    void paintEvent( QPaintEvent* ) override;

private:
    std::vector<int> mBins;
    double mBinDegrees = 5.0;
    double mMinTilt = 20.0;
    double mFraction = 0.0;
    double mRequired = 0.30;
};

}  // namespace intrinsics
