#pragma once

#include "camera_source.h"
#include "capture_session.h"
#include "config.h"
#include "pose_estimator.h"
#include "solver_runner.h"

#include <QMainWindow>
#include <cstdint>
#include <memory>

class QCheckBox;
class QFormLayout;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTabWidget;
class QTextBrowser;

namespace intrinsics {

class CaptureListWidget;
class CoverageWidget;
class TiltHistogramWidget;
class VideoView;

// A slider + spin box that agree with each other, plus the value the camera
// reports back. They differ whenever the SDK clamps a request.
struct ControlRow
{
    QSlider* slider = nullptr;
    QSpinBox* spin = nullptr;
    QLabel* actual = nullptr;

    void setRange( const Range& r );
    void setRequested( int value );
    void showActual( int value, const QString& suffix = {} );
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow( const AppConfig& cfg, CameraSource& source, QWidget* parent = nullptr );

private slots:
    void onTick();
    void onCapture();
    void onDeleteCapture( int index );
    void onSolve();
    void onSolveFinished( const intrinsics::SolveResult& result );

private:
    QWidget* buildCameraTab();
    QWidget* buildCoverageTab();
    QWidget* buildCapturesTab();
    QWidget* buildResultsTab();
    ControlRow addControlRow( QFormLayout* form, const QString& label, const QString& unit );

    void refreshExposureBounds();
    void refreshAnalysis();
    void appendLog( const QString& line );

    const AppConfig& mCfg;
    CameraSource& mSource;
    CameraCaps mCaps;
    BoardDetector mDetector;
    PoseEstimator mPose;
    CaptureSession mSession;
    SolverRunner* mSolver = nullptr;

    VideoView* mVideo = nullptr;
    QTabWidget* mTabs = nullptr;

    ControlRow mExposure;
    ControlRow mGain;
    ControlRow mIntensity;
    ControlRow mFrameRate;
    QCheckBox* mFilterToggle = nullptr;
    QCheckBox* mDetectToggle = nullptr;
    QLabel* mExposureHint = nullptr;

    QLabel* mMarkerLabel = nullptr;
    QLabel* mCornerLabel = nullptr;
    QLabel* mStatsLabel = nullptr;
    QLabel* mSessionLabel = nullptr;
    QPushButton* mCaptureButton = nullptr;

    CoverageWidget* mCoverage = nullptr;
    QLabel* mCoverageSummary = nullptr;
    TiltHistogramWidget* mTilt = nullptr;
    CaptureListWidget* mCaptures = nullptr;

    QPushButton* mSolveButton = nullptr;
    QTextBrowser* mResults = nullptr;
    QPlainTextEdit* mLogView = nullptr;

    std::shared_ptr<const ViewFrame> mCurrent;
    std::uint64_t mShownSeq = 0;
};

}  // namespace intrinsics
