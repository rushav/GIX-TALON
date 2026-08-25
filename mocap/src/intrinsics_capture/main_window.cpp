#include "main_window.h"

#include "analysis_widgets.h"
#include "capture_list.h"
#include "video_view.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

namespace intrinsics {
namespace {

const char* kMuted = "color:#8a8a92; font-size:11px;";
const char* kWarnCss = "color:#e0a030; font-weight:600;";
const char* kGoodCss = "color:#4caf50; font-weight:600;";

QImage toQImage( const cv::Mat& bgr )
{
    // QImage does not own the pixels, so copy before the cv::Mat can be freed.
    return QImage( bgr.data, bgr.cols, bgr.rows, static_cast<int>( bgr.step ),
        QImage::Format_BGR888 ).copy();
}

// Reference bounds are context, never pass/fail - so "outside" is styled as a
// note to look at, not a failure.
QString inRange( double v, double lo, double hi )
{
    return ( v >= lo && v <= hi ) ? QStringLiteral( "color:#4caf50" )
                                  : QStringLiteral( "color:#e0a030" );
}

QString fmt( double v, int dp = 2 ) { return QString::number( v, 'f', dp ); }

}  // namespace

void ControlRow::setRange( const Range& r )
{
    QSignalBlocker b1( slider );
    QSignalBlocker b2( spin );
    slider->setRange( r.lo, r.hi );
    spin->setRange( r.lo, r.hi );
}

void ControlRow::setRequested( int value )
{
    QSignalBlocker b1( slider );
    QSignalBlocker b2( spin );
    slider->setValue( value );
    spin->setValue( value );
}

void ControlRow::showActual( int value, const QString& suffix )
{
    const bool clamped = value != spin->value();
    actual->setText( QStringLiteral( "camera: %1%2" ).arg( value ).arg( suffix ) );
    actual->setStyleSheet( clamped ? QStringLiteral( "color:#e0a030; font-weight:600;" )
                                   : QStringLiteral( "color:#8a8a92;" ) );
    actual->setToolTip( clamped ? QStringLiteral( "Camera applied a different value." )
                                : QString() );
}

MainWindow::MainWindow( const AppConfig& cfg, CameraSource& source, QWidget* parent )
    : QMainWindow( parent )
    , mCfg( cfg )
    , mSource( source )
    , mCaps( source.caps() )
    , mDetector( cfg.board )
    , mPose( cfg.pose, mDetector )
    , mSession( cfg, source.caps(), CameraSource::sdkVersion() )
{
    setWindowTitle( QStringLiteral( "Intrinsics Capture - %1 %2" )
                        .arg( QString::fromStdString( cfg.cameraModel ) )
                        .arg( mCaps.serial ) );

    mSolver = new SolverRunner( cfg.solver, this );
    connect( mSolver, &SolverRunner::finished, this, &MainWindow::onSolveFinished );
    connect( mSolver, &SolverRunner::progress, this, &MainWindow::appendLog );

    // --- left: live view + headline counts --------------------------------
    mVideo = new VideoView( this );
    mVideo->setPlaceholder(
        QStringLiteral( "waiting for grayscale frames from %1..." ).arg( mCaps.serial ) );

    auto* leftBox = new QWidget( this );
    auto* left = new QVBoxLayout( leftBox );
    left->setContentsMargins( 0, 0, 0, 0 );
    left->addWidget( mVideo, 1 );

    auto* counts = new QWidget( leftBox );
    auto* countRow = new QHBoxLayout( counts );
    countRow->setContentsMargins( 4, 0, 4, 0 );
    mMarkerLabel = new QLabel( QStringLiteral( "0 / %1 markers" ).arg( cfg.board.markerCount ), counts );
    mMarkerLabel->setStyleSheet( QStringLiteral( "font-size:22px; font-weight:700;" ) );
    mCornerLabel = new QLabel(
        QStringLiteral( "0 / %1 corners" ).arg( cfg.board.charucoCornerCount() ), counts );
    mCornerLabel->setStyleSheet( QStringLiteral( "font-size:22px; font-weight:700;" ) );
    countRow->addWidget( mMarkerLabel );
    countRow->addSpacing( 18 );
    countRow->addWidget( mCornerLabel );
    countRow->addStretch( 1 );
    mStatsLabel = new QLabel( QStringLiteral( "-" ), counts );
    mStatsLabel->setStyleSheet( QString::fromLatin1( kMuted ) );
    countRow->addWidget( mStatsLabel );
    left->addWidget( counts );

    // --- right: capture bar + tabs ----------------------------------------
    auto* rightBox = new QWidget( this );
    rightBox->setFixedWidth( 460 );
    auto* right = new QVBoxLayout( rightBox );
    right->setContentsMargins( 0, 0, 0, 0 );

    mCaptureButton = new QPushButton( QStringLiteral( "Capture frame  (Space)" ), rightBox );
    mCaptureButton->setMinimumHeight( 42 );
    connect( mCaptureButton, &QPushButton::clicked, this, &MainWindow::onCapture );
    right->addWidget( mCaptureButton );

    mSessionLabel = new QLabel( QStringLiteral( "no frames captured yet" ), rightBox );
    mSessionLabel->setWordWrap( true );
    mSessionLabel->setTextInteractionFlags( Qt::TextSelectableByMouse );
    mSessionLabel->setStyleSheet( QString::fromLatin1( kMuted ) );
    right->addWidget( mSessionLabel );

    mTabs = new QTabWidget( rightBox );
    mTabs->addTab( buildCameraTab(), QStringLiteral( "Camera" ) );
    mTabs->addTab( buildCoverageTab(), QStringLiteral( "Coverage" ) );
    mTabs->addTab( buildCapturesTab(), QStringLiteral( "Captures" ) );
    mTabs->addTab( buildResultsTab(), QStringLiteral( "Solve" ) );
    right->addWidget( mTabs, 1 );

    mLogView = new QPlainTextEdit( rightBox );
    mLogView->setReadOnly( true );
    mLogView->setMaximumBlockCount( 500 );
    mLogView->setMaximumHeight( 130 );
    mLogView->setStyleSheet( QStringLiteral( "font-family:monospace; font-size:11px;" ) );
    right->addWidget( mLogView );

    auto* central = new QWidget( this );
    auto* layout = new QHBoxLayout( central );
    layout->setContentsMargins( 8, 8, 8, 8 );
    layout->addWidget( leftBox, 1 );
    layout->addWidget( rightBox, 0 );
    setCentralWidget( central );

    statusBar()->showMessage(
        QStringLiteral( "%1 (rev %2)  |  %3x%4  |  SDK %5  |  OpenCV %6" )
            .arg( QString::fromStdString( cfg.cameraModel ) )
            .arg( mCaps.revision )
            .arg( mCaps.width )
            .arg( mCaps.height )
            .arg( QString::fromStdString( CameraSource::sdkVersion() ) )
            .arg( QStringLiteral( CV_VERSION ) ) );

    auto* timer = new QTimer( this );
    connect( timer, &QTimer::timeout, this, &MainWindow::onTick );
    timer->start( 1000 / std::max( 1, cfg.displayFps ) );

    auto* shortcut = new QShortcut( QKeySequence( Qt::Key_Space ), this );
    connect( shortcut, &QShortcut::activated, this, &MainWindow::onCapture );

    refreshAnalysis();
    resize( 1520, 960 );
}

// --- tabs -------------------------------------------------------------------

ControlRow MainWindow::addControlRow( QFormLayout* form, const QString& label, const QString& unit )
{
    ControlRow row;
    row.slider = new QSlider( Qt::Horizontal, this );
    row.spin = new QSpinBox( this );
    row.spin->setSuffix( unit );
    row.spin->setMinimumWidth( 92 );
    row.actual = new QLabel( QStringLiteral( "camera: -" ), this );
    row.actual->setStyleSheet( QStringLiteral( "color:#8a8a92;" ) );

    auto* holder = new QWidget( this );
    auto* v = new QVBoxLayout( holder );
    v->setContentsMargins( 0, 0, 0, 0 );
    v->setSpacing( 2 );
    auto* h = new QHBoxLayout;
    h->addWidget( row.slider, 1 );
    h->addWidget( row.spin, 0 );
    v->addLayout( h );
    v->addWidget( row.actual );
    form->addRow( label, holder );

    connect( row.slider, &QSlider::valueChanged, row.spin, &QSpinBox::setValue );
    connect( row.spin, qOverload<int>( &QSpinBox::valueChanged ), row.slider, &QSlider::setValue );
    return row;
}

QWidget* MainWindow::buildCameraTab()
{
    auto* tab = new QWidget( this );
    auto* outer = new QVBoxLayout( tab );
    auto* form = new QFormLayout;
    outer->addLayout( form );

    mExposure = addControlRow( form, QStringLiteral( "Exposure" ), QStringLiteral( " us" ) );
    mExposureHint = new QLabel( tab );
    mExposureHint->setWordWrap( true );
    mExposureHint->setStyleSheet( QString::fromLatin1( kMuted ) );
    form->addRow( QString(), mExposureHint );

    mGain = addControlRow( form, QStringLiteral( "Imager gain" ), QString() );
    mIntensity = addControlRow( form, QStringLiteral( "IR intensity" ), QString() );
    mFrameRate = addControlRow( form, QStringLiteral( "Frame rate" ), QStringLiteral( " Hz" ) );

    mFilterToggle = new QCheckBox( QStringLiteral( "850 nm IR bandpass filter" ), tab );
    mFilterToggle->setChecked( mCfg.defaults.irFilter );
    mFilterToggle->setEnabled( mCaps.filterSwitchAvailable );
    mFilterToggle->setToolTip( QStringLiteral(
        "Leave this IN for ChArUco capture at IR intensity 15 / gain 7 / exposure 8133 - "
        "calibrating in the same optical configuration you track in means no filter-removal "
        "focal shift to account for." ) );
    connect( mFilterToggle, &QCheckBox::toggled, this,
        [this]( bool on ) { mSource.setIRFilter( on ); } );
    form->addRow( QString(), mFilterToggle );

    mExposure.setRange( mCaps.exposure );
    mGain.setRange( mCaps.imagerGain );
    mIntensity.setRange( mCaps.irIntensity );
    mFrameRate.setRange( mCaps.frameRate );

    mExposure.setRequested( mCfg.defaults.exposure );
    mGain.setRequested( mCfg.defaults.imagerGain );
    mIntensity.setRequested( mCfg.defaults.irIntensity );
    mFrameRate.setRequested( mCfg.defaults.frameRateHz );

    connect( mExposure.spin, qOverload<int>( &QSpinBox::valueChanged ), this,
        [this]( int v ) { mSource.setExposure( v ); } );
    connect( mGain.spin, qOverload<int>( &QSpinBox::valueChanged ), this,
        [this]( int v ) { mSource.setImagerGain( v ); } );
    connect( mIntensity.spin, qOverload<int>( &QSpinBox::valueChanged ), this,
        [this]( int v ) { mSource.setIntensity( v ); } );
    connect( mFrameRate.spin, qOverload<int>( &QSpinBox::valueChanged ), this, [this]( int v ) {
        mSource.setFrameRate( v );
        // The exposure ceiling moves with the frame period, so the slider has to
        // follow or it silently caps brightness at the old rate's maximum.
        if( mCfg.refreshExposureRangeOnFrameRateChange )
            QTimer::singleShot( 250, this, &MainWindow::refreshExposureBounds );
    } );

    mDetectToggle = new QCheckBox( QStringLiteral( "Detect board on the live view" ), tab );
    mDetectToggle->setChecked( true );
    connect( mDetectToggle, &QCheckBox::toggled, this,
        [this]( bool on ) { mSource.setDetectionEnabled( on ); } );
    outer->addWidget( mDetectToggle );

    auto* geom = new QLabel( QStringLiteral( "%1  |  %2x%3 squares  |  %4 mm square / %5 mm marker" )
                                 .arg( QString::fromStdString( mCfg.board.dictionary ) )
                                 .arg( mCfg.board.squaresX )
                                 .arg( mCfg.board.squaresY )
                                 .arg( mCfg.board.squareLengthMm )
                                 .arg( mCfg.board.markerLengthMm ),
        tab );
    geom->setWordWrap( true );
    geom->setStyleSheet( QString::fromLatin1( kMuted ) );
    outer->addWidget( geom );

    outer->addStretch( 1 );
    refreshExposureBounds();
    return tab;
}

QWidget* MainWindow::buildCoverageTab()
{
    auto* tab = new QWidget( this );
    auto* v = new QVBoxLayout( tab );

    auto* head = new QLabel( QStringLiteral( "Corners per frame region" ), tab );
    head->setStyleSheet( QStringLiteral( "font-weight:600;" ) );
    v->addWidget( head );

    mCoverage = new CoverageWidget( tab );
    v->addWidget( mCoverage, 1 );

    mCoverageSummary = new QLabel( tab );
    mCoverageSummary->setWordWrap( true );
    mCoverageSummary->setStyleSheet( QString::fromLatin1( kMuted ) );
    v->addWidget( mCoverageSummary );

    auto* why = new QLabel(
        QStringLiteral( "RMS only measures error where corners were observed. An empty cell means "
                        "the distortion model is extrapolating there, and nothing in the solve "
                        "output will say so." ),
        tab );
    why->setWordWrap( true );
    why->setStyleSheet( QString::fromLatin1( kMuted ) );
    v->addWidget( why );

    auto* head2 = new QLabel( QStringLiteral( "Board tilt across captures" ), tab );
    head2->setStyleSheet( QStringLiteral( "font-weight:600;" ) );
    v->addWidget( head2 );

    mTilt = new TiltHistogramWidget( tab );
    v->addWidget( mTilt );

    auto* why2 = new QLabel(
        QStringLiteral( "All-fronto-parallel views leave focal length and distance mutually "
                        "ambiguous; the symptom is fx drifting from fy." ),
        tab );
    why2->setWordWrap( true );
    why2->setStyleSheet( QString::fromLatin1( kMuted ) );
    v->addWidget( why2 );

    return tab;
}

QWidget* MainWindow::buildCapturesTab()
{
    mCaptures = new CaptureListWidget( this );
    connect( mCaptures, &CaptureListWidget::deleteRequested, this, &MainWindow::onDeleteCapture );
    return mCaptures;
}

QWidget* MainWindow::buildResultsTab()
{
    auto* tab = new QWidget( this );
    auto* v = new QVBoxLayout( tab );

    mSolveButton = new QPushButton( QStringLiteral( "Process - solve intrinsics" ), tab );
    mSolveButton->setMinimumHeight( 40 );
    connect( mSolveButton, &QPushButton::clicked, this, &MainWindow::onSolve );
    v->addWidget( mSolveButton );

    mResults = new QTextBrowser( tab );
    mResults->setOpenExternalLinks( false );
    mResults->setHtml( QStringLiteral(
        "<p style='color:#8a8a92'>Capture a spread of views, then press Process.<br><br>"
        "The solve runs out of process in the repo venv because it needs OpenCV 5.x, which "
        "cannot share a process with the OpenCV 4.6 this tool links against.</p>" ) );
    v->addWidget( mResults, 1 );

    return tab;
}

// --- live loop --------------------------------------------------------------

void MainWindow::refreshExposureBounds()
{
    const Range r = mSource.refreshExposureRange();
    const int wanted = mExposure.spin->value();
    mExposure.setRange( r );
    // Re-apply so the camera and the widget agree after a bound change.
    mExposure.setRequested( std::clamp( wanted, r.lo, r.hi ) );
    if( mExposure.spin->value() != wanted ) mSource.setExposure( mExposure.spin->value() );

    mCaps.exposure = r;
    mExposureHint->setText(
        QStringLiteral( "SDK reports %1..%2 us at %3 Hz. The ceiling tracks the frame period, so "
                        "it is re-read whenever the rate changes." )
            .arg( r.lo )
            .arg( r.hi )
            .arg( mFrameRate.spin->value() ) );
}

void MainWindow::appendLog( const QString& line ) { mLogView->appendPlainText( line ); }

void MainWindow::onTick()
{
    for( const auto& line : mSource.drainLog() )
        appendLog( QString::fromStdString( line ) );

    auto frame = mSource.latest();
    if( !frame ) return;

    mCurrent = frame;
    if( frame->seq == mShownSeq ) return;
    mShownSeq = frame->seq;

    mVideo->setImage( toQImage( frame->display ) );

    const int markers = frame->detection.markerCount();
    const int corners = frame->detection.cornerCount();
    const int wantMarkers = mCfg.board.markerCount;
    const int wantCorners = mCfg.board.charucoCornerCount();

    auto badge = [&]( int got, int want ) {
        return QStringLiteral( "font-size:22px; font-weight:700; color:%1;" )
            .arg( got == 0 ? QStringLiteral( "#c05050" )
                           : ( got >= want ? QStringLiteral( "#4caf50" )
                                           : QStringLiteral( "#e0a030" ) ) );
    };
    mMarkerLabel->setText( QStringLiteral( "%1 / %2 markers" ).arg( markers ).arg( wantMarkers ) );
    mMarkerLabel->setStyleSheet( badge( markers, wantMarkers ) );
    mCornerLabel->setText( QStringLiteral( "%1 / %2 corners" ).arg( corners ).arg( wantCorners ) );
    mCornerLabel->setStyleSheet( badge( corners, wantCorners ) );

    const CameraSettings& s = frame->settings;
    mExposure.showActual( s.exposureActual );
    mGain.showActual( s.gainActual );
    mIntensity.showActual( s.intensityActual );
    mFrameRate.showActual( s.frameRateActual, QStringLiteral( " Hz" ) );
    if( s.irFilterActual >= 0 )
    {
        const bool mismatch = s.irFilterActual != ( mFilterToggle->isChecked() ? 1 : 0 );
        mFilterToggle->setText( mismatch
                ? QStringLiteral( "850 nm IR bandpass filter  (camera: %1)" )
                      .arg( s.irFilterActual ? QStringLiteral( "on" ) : QStringLiteral( "off" ) )
                : QStringLiteral( "850 nm IR bandpass filter" ) );
    }

    // Imager rate and delivered rate are different numbers in grayscale mode.
    mStatsLabel->setText( QStringLiteral( "imager %1 Hz  |  %2 frames/s delivered  |  %3" )
                              .arg( s.deviceFrameRate )
                              .arg( frame->measuredFps, 0, 'f', 1 )
                              .arg( s.videoType == 1 ? QStringLiteral( "grayscale" )
                                                     : QStringLiteral( "mode %1" ).arg( s.videoType ) ) );
}

// --- capture / delete -------------------------------------------------------

void MainWindow::onCapture()
{
    if( !mCurrent || mCurrent->raw.empty() )
    {
        statusBar()->showMessage( QStringLiteral( "No frame to capture yet." ), 3000 );
        return;
    }
    if( mCurrent->detection.cornerCount() == 0 )
    {
        statusBar()->showMessage(
            QStringLiteral( "No ChArUco corners in this frame - nothing to calibrate from." ), 3000 );
        return;
    }

    try
    {
        const auto pose = mPose.estimate( mCurrent->detection );
        const std::string file = mSession.add( *mCurrent, pose ? pose->tiltDegrees : -1.0 );

        statusBar()->showMessage(
            QStringLiteral( "Saved %1 - %2 corners%3" )
                .arg( QString::fromStdString( file ) )
                .arg( mCurrent->detection.cornerCount() )
                .arg( pose ? QStringLiteral( ", %1 deg tilt" ).arg( pose->tiltDegrees, 0, 'f', 0 )
                           : QString() ),
            2500 );
        refreshAnalysis();
    }
    catch( const std::exception& e )
    {
        QMessageBox::warning( this, QStringLiteral( "Capture failed" ), QString::fromUtf8( e.what() ) );
    }
}

void MainWindow::onDeleteCapture( int index )
{
    if( index < 0 || index >= mSession.count() ) return;
    const QString name = QString::fromStdString( mSession.records()[index].file );

    mSession.remove( index );
    appendLog( QStringLiteral( "deleted %1 - coverage and pose stats updated" ).arg( name ) );
    statusBar()->showMessage( QStringLiteral( "Deleted %1" ).arg( name ), 2500 );
    refreshAnalysis();
}

void MainWindow::refreshAnalysis()
{
    const CoverageGrid& g = mSession.coverage();
    std::vector<int> cells;
    cells.reserve( static_cast<size_t>( g.cols() * g.rows() ) );
    for( int r = 0; r < g.rows(); ++r )
        for( int c = 0; c < g.cols(); ++c ) cells.push_back( g.at( c, r ) );
    mCoverage->setData( g.cols(), g.rows(), std::move( cells ), g.targetPerCell(),
        mCaps.height > 0 ? static_cast<double>( mCaps.width ) / mCaps.height : 1.25 );

    const int empty = g.emptyCells();
    const int total = g.cols() * g.rows();
    mCoverageSummary->setText( QStringLiteral( "%1 corners over %2 captures - %3" )
                                   .arg( g.totalCorners() )
                                   .arg( mSession.count() )
                                   .arg( empty == 0
                                           ? QStringLiteral( "every cell has data" )
                                           : QStringLiteral( "%1 of %2 cells still empty" )
                                                 .arg( empty )
                                                 .arg( total ) ) );
    mCoverageSummary->setStyleSheet( empty == 0 ? QString::fromLatin1( kGoodCss )
                                                : QString::fromLatin1( kWarnCss ) );

    const auto tilts = mSession.tilts();
    mTilt->setData( mPose.histogram( tilts ), mCfg.pose.histogramBinDegrees,
        mCfg.pose.minTiltDegrees, mPose.tiltedFraction( tilts ), mCfg.pose.minFractionTilted );

    mCaptures->setRecords( mSession.records(), mCfg.board.charucoCornerCount() );
    mSolveButton->setEnabled( mSession.count() >= 3 && !mSolver->running() );

    // Kept here rather than only in onCapture so the label cannot drift out of
    // step with the session after a delete.
    mSessionLabel->setText( mSession.count() == 0
            ? QStringLiteral( "no frames captured yet" )
            : QStringLiteral( "%1\n%2 capture(s)" )
                  .arg( QString::fromStdString( mSession.directory().string() ) )
                  .arg( mSession.count() ) );
}

// --- solve ------------------------------------------------------------------

void MainWindow::onSolve()
{
    if( mSession.count() < 3 )
    {
        QMessageBox::information( this, QStringLiteral( "Not enough captures" ),
            QStringLiteral( "The solve needs at least 3 captures." ) );
        return;
    }

    QString why;
    if( !mSolver->start( QString::fromStdString( mSession.directory().string() ), why ) )
    {
        QMessageBox::warning( this, QStringLiteral( "Cannot run the solver" ), why );
        return;
    }

    mSolveButton->setEnabled( false );
    mSolveButton->setText( QStringLiteral( "Solving..." ) );
    mResults->setHtml( QStringLiteral( "<p style='color:#8a8a92'>Running the solve...</p>" ) );
    mTabs->setCurrentIndex( 3 );
}

void MainWindow::onSolveFinished( const SolveResult& r )
{
    mSolveButton->setText( QStringLiteral( "Process - solve intrinsics" ) );
    mSolveButton->setEnabled( mSession.count() >= 3 );

    if( !r.ok )
    {
        mResults->setHtml( QStringLiteral( "<p style='color:#c05050'><b>Solve failed.</b><br>%1</p>" )
                               .arg( r.error.toHtmlEscaped() ) );
        return;
    }

    const ReferenceBounds& b = mCfg.reference;
    const double focalDiff = r.fxFyPercentDiff;

    QString html;
    html += QStringLiteral( "<style>td{padding:2px 10px 2px 0}</style>" );
    html += QStringLiteral( "<h3 style='margin:2px 0'>Intrinsics</h3><table>" );
    html += QStringLiteral( "<tr><td>fx</td><td style='%1'><b>%2</b></td>"
                            "<td>fy</td><td style='%3'><b>%4</b></td></tr>" )
                .arg( inRange( r.fx, b.focalLo, b.focalHi ), fmt( r.fx ),
                    inRange( r.fy, b.focalLo, b.focalHi ), fmt( r.fy ) );
    html += QStringLiteral( "<tr><td>fx/fy</td><td colspan=3 style='%1'>%2% apart</td></tr>" )
                .arg( focalDiff <= b.focalAgreementPercent ? QStringLiteral( "color:#4caf50" )
                                                           : QStringLiteral( "color:#e0a030" ),
                    fmt( focalDiff, 3 ) );
    html += QStringLiteral( "<tr><td>cx</td><td style='%1'><b>%2</b></td>"
                            "<td colspan=2>%3 px from nominal %4</td></tr>" )
                .arg( inRange( r.cx, b.cxLo, b.cxHi ), fmt( r.cx ),
                    fmt( r.cx - b.nominalCx, 1 ), fmt( b.nominalCx, 0 ) );
    html += QStringLiteral( "<tr><td>cy</td><td style='%1'><b>%2</b></td>"
                            "<td colspan=2>%3 px from nominal %4</td></tr>" )
                .arg( inRange( r.cy, b.cyLo, b.cyHi ), fmt( r.cy ),
                    fmt( r.cy - b.nominalCy, 1 ), fmt( b.nominalCy, 0 ) );
    html += QStringLiteral( "<tr><td>RMS</td><td colspan=3 style='%1'><b>%2 px</b></td></tr>" )
                .arg( inRange( r.rms, b.rmsLo, b.rmsHi ), fmt( r.rms, 4 ) );
    html += QStringLiteral( "</table>" );

    html += QStringLiteral( "<p style='color:#8a8a92;font-size:11px'>Colour is against the "
                            "previous rig's ranges (RMS %1-%2, focal %3-%4, cx %5-%6, cy %7-%8) - "
                            "context for reading the number, not pass/fail.</p>" )
                .arg( fmt( b.rmsLo, 2 ), fmt( b.rmsHi, 2 ), fmt( b.focalLo, 0 ), fmt( b.focalHi, 0 ),
                    fmt( b.cxLo, 0 ), fmt( b.cxHi, 0 ), fmt( b.cyLo, 0 ), fmt( b.cyHi, 0 ) );

    // Coverage sits next to the result deliberately: a healthy RMS over a
    // half-empty grid is the exact failure this tool exists to prevent.
    const int empty = mSession.coverage().emptyCells();
    if( empty > 0 )
        html += QStringLiteral( "<p style='color:#e0a030'><b>%1 coverage cells were empty.</b> "
                                "RMS cannot see this: the distortion model is extrapolating in "
                                "those regions rather than being measured there.</p>" )
                    .arg( empty );

    const auto tilts = mSession.tilts();
    const double frac = mPose.tiltedFraction( tilts );
    if( frac < mCfg.pose.minFractionTilted )
        html += QStringLiteral( "<p style='color:#e0a030'><b>Only %1% of captures exceed %2 deg "
                                "tilt.</b> Focal length and distance are weakly separated; watch "
                                "the fx/fy gap above.</p>" )
                    .arg( frac * 100.0, 0, 'f', 0 )
                    .arg( mCfg.pose.minTiltDegrees, 0, 'f', 0 );

    html += QStringLiteral( "<h3 style='margin:8px 0 2px'>Distortion (rational, 8 coefficients)</h3>" );
    html += QStringLiteral( "<p style='font-family:monospace;font-size:11px'>" );
    static const char* names[] = { "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6" };
    for( size_t i = 0; i < r.distortion.size() && i < 8; ++i )
        html += QStringLiteral( "%1 %2<br>" ).arg( QString::fromLatin1( names[i] ) )
                    .arg( r.distortion[i], 0, 'g', 6 );
    html += QStringLiteral( "</p><p style='color:#8a8a92;font-size:11px'>Rational coefficients are "
                            "a ratio of two polynomials and are not individually meaningful - "
                            "large k values with k1~k4 and k2~k5 are normal. Judge the model by "
                            "undistortion behaviour, not by these numbers.</p>" );

    // Worst views first, so a bad capture can be found and deleted.
    auto views = r.views;
    std::sort( views.begin(), views.end(),
        []( const ViewError& a, const ViewError& c ) { return a.errorPx > c.errorPx; } );

    html += QStringLiteral( "<h3 style='margin:8px 0 2px'>Per-view error (worst first)</h3><table>" );
    QStringList worst;
    for( int i = 0; i < static_cast<int>( views.size() ); ++i )
    {
        const bool flag = i < 3 && views.size() > 3;
        if( flag ) worst << views[static_cast<size_t>( i )].file;
        html += QStringLiteral( "<tr><td style='%1'>%2</td><td style='%1'>%3 px</td>"
                                "<td style='color:#8a8a92'>%4 corners</td></tr>" )
                    .arg( flag ? QStringLiteral( "color:#e0a030" ) : QStringLiteral( "color:#d2d2da" ),
                        views[static_cast<size_t>( i )].file,
                        fmt( views[static_cast<size_t>( i )].errorPx, 4 ) )
                    .arg( views[static_cast<size_t>( i )].corners );
    }
    html += QStringLiteral( "</table>" );

    if( !r.skipped.isEmpty() )
        html += QStringLiteral( "<p style='color:#e0a030'>Skipped: %1</p>" )
                    .arg( r.skipped.join( QStringLiteral( "; " ) ).toHtmlEscaped() );

    html += QStringLiteral( "<p style='color:#8a8a92;font-size:11px'>%1 views, OpenCV %2, solved "
                            "%3.<br>Written to %4</p>" )
                .arg( r.viewsUsed )
                .arg( r.opencvVersion, r.solvedUtc,
                    QString::fromStdString( mSession.intrinsicsPath().string() ) );

    mResults->setHtml( html );

    // Highlight the worst views in the capture list so they can be deleted.
    mCaptures->setWorstViews( worst );
    mCaptures->setRecords( mSession.records(), mCfg.board.charucoCornerCount() );
}

}  // namespace intrinsics
