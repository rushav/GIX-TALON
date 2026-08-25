#include "analysis_widgets.h"

#include <QPainter>

#include <algorithm>

namespace intrinsics {
namespace {
const QColor kInk( 210, 210, 218 );
const QColor kMuted( 138, 138, 146 );
const QColor kWarn( 224, 160, 48 );
const QColor kBad( 200, 80, 80 );
const QColor kGood( 76, 175, 80 );
}  // namespace

// --- coverage ---------------------------------------------------------------

CoverageWidget::CoverageWidget( QWidget* parent )
    : QWidget( parent )
{
    setMinimumHeight( 260 );
}

void CoverageWidget::setData( int cols, int rows, std::vector<int> cells, int targetPerCell,
    double frameAspect )
{
    mCols = cols;
    mRows = rows;
    mCells = std::move( cells );
    mTarget = std::max( 1, targetPerCell );
    if( frameAspect > 0 ) mAspect = frameAspect;
    update();
}

void CoverageWidget::paintEvent( QPaintEvent* )
{
    QPainter p( this );
    p.fillRect( rect(), QColor( 24, 24, 27 ) );

    if( mCols <= 0 || mRows <= 0 || mCells.empty() )
    {
        p.setPen( kMuted );
        p.drawText( rect(), Qt::AlignCenter, QStringLiteral( "no captures yet" ) );
        return;
    }

    // The grid carries the frame's own aspect ratio, so a cell on screen is the
    // same shape as the frame region it counts.
    const int margin = 6;
    int gw = width() - 2 * margin;
    int gh = static_cast<int>( gw / mAspect );
    if( gh > height() - 2 * margin )
    {
        gh = height() - 2 * margin;
        gw = static_cast<int>( gh * mAspect );
    }
    const int x0 = ( width() - gw ) / 2;
    const int y0 = ( height() - gh ) / 2;

    QFont f = p.font();
    f.setPointSizeF( std::max( 7.0, std::min( 11.0, gw / static_cast<double>( mCols ) / 3.0 ) ) );
    p.setFont( f );

    for( int r = 0; r < mRows; ++r )
    {
        for( int c = 0; c < mCols; ++c )
        {
            const int n = mCells[static_cast<size_t>( r * mCols + c )];
            const double fill = std::min( 1.0, static_cast<double>( n ) / mTarget );

            const QRect cell( x0 + gw * c / mCols, y0 + gh * r / mRows,
                gw * ( c + 1 ) / mCols - gw * c / mCols,
                gh * ( r + 1 ) / mRows - gh * r / mRows );

            const QColor bg = n == 0 ? QColor( 60, 28, 28 )
                                     : QColor::fromHsvF( 0.33, 0.55, 0.18 + 0.50 * fill );
            p.fillRect( cell.adjusted( 1, 1, -1, -1 ), bg );

            p.setPen( n == 0 ? kBad : QColor( 70, 70, 78 ) );
            p.drawRect( cell.adjusted( 1, 1, -1, -1 ) );

            // Contrast against the shading rather than a fixed ink, or the
            // counts vanish exactly on the well-covered cells.
            p.setPen( n == 0 ? kBad
                             : ( bg.valueF() > 0.45 ? QColor( 12, 24, 12 ) : QColor( 235, 245, 235 ) ) );
            p.drawText( cell, Qt::AlignCenter, QString::number( n ) );
        }
    }
}

// --- tilt histogram ---------------------------------------------------------

TiltHistogramWidget::TiltHistogramWidget( QWidget* parent )
    : QWidget( parent )
{
    setMinimumHeight( 150 );
}

void TiltHistogramWidget::setData( std::vector<int> bins, double binDegrees,
    double minTiltDegrees, double tiltedFraction, double requiredFraction )
{
    mBins = std::move( bins );
    mBinDegrees = binDegrees;
    mMinTilt = minTiltDegrees;
    mFraction = tiltedFraction;
    mRequired = requiredFraction;
    update();
}

void TiltHistogramWidget::paintEvent( QPaintEvent* )
{
    QPainter p( this );
    p.fillRect( rect(), QColor( 24, 24, 27 ) );

    const int peak = mBins.empty() ? 0 : *std::max_element( mBins.begin(), mBins.end() );
    if( peak == 0 )
    {
        p.setPen( kMuted );
        p.drawText( rect(), Qt::AlignCenter, QStringLiteral( "no poses yet" ) );
        return;
    }

    // Headroom so the count above the tallest bar is not clipped.
    const int left = 6, right = 6, top = 18, bottom = 34;
    const int plotW = width() - left - right;
    const int plotH = height() - top - bottom;
    const int n = static_cast<int>( mBins.size() );

    for( int i = 0; i < n; ++i )
    {
        const int x = left + plotW * i / n;
        const int w = plotW * ( i + 1 ) / n - plotW * i / n;
        const int h = static_cast<int>( plotH * ( static_cast<double>( mBins[i] ) / peak ) );

        const double binLo = i * mBinDegrees;
        // Bars at or past the threshold are the ones breaking the focal/distance
        // ambiguity, so they read as the useful ones.
        p.fillRect( QRect( x + 1, top + plotH - h, w - 2, h ),
            binLo >= mMinTilt ? kGood : QColor( 90, 90, 100 ) );

        if( mBins[i] > 0 )
        {
            p.setPen( kInk );
            p.drawText( QRect( x, top + plotH - h - 14, w, 13 ), Qt::AlignCenter,
                QString::number( mBins[i] ) );
        }
        if( i % 2 == 0 )
        {
            p.setPen( kMuted );
            p.drawText( QRect( x, top + plotH + 2, w, 12 ), Qt::AlignCenter,
                QString::number( static_cast<int>( binLo ) ) );
        }
    }

    // Threshold line.
    const int tx = left + static_cast<int>( plotW * ( mMinTilt / ( n * mBinDegrees ) ) );
    p.setPen( QPen( kWarn, 1, Qt::DashLine ) );
    p.drawLine( tx, top, tx, top + plotH );

    const bool ok = mFraction >= mRequired;
    p.setPen( ok ? kGood : kWarn );
    p.drawText( QRect( left, height() - 18, plotW, 16 ), Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral( "%1% of captures past %2 deg tilt (want %3%)" )
            .arg( mFraction * 100.0, 0, 'f', 0 )
            .arg( mMinTilt, 0, 'f', 0 )
            .arg( mRequired * 100.0, 0, 'f', 0 ) );
    p.setPen( kMuted );
    p.drawText( QRect( left, height() - 18, plotW, 16 ), Qt::AlignRight | Qt::AlignVCenter,
        QStringLiteral( "tilt, degrees" ) );
}

}  // namespace intrinsics
