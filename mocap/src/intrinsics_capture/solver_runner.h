#pragma once

#include "config.h"

#include <QObject>
#include <QString>
#include <QStringList>

#include <vector>

class QProcess;

namespace intrinsics {

struct ViewError
{
    QString file;
    int corners = 0;
    double errorPx = 0.0;
};

struct SolveResult
{
    bool ok = false;
    QString error;          // set when ok is false

    double fx = 0, fy = 0, cx = 0, cy = 0;
    double fxFyPercentDiff = 0;
    double rms = 0;
    std::vector<double> distortion;
    std::vector<ViewError> views;
    QStringList skipped;
    int viewsUsed = 0;
    QString opencvVersion;
    QString solvedUtc;
    QString outputPath;
};

// Runs mocap/python/solve_intrinsics.py in the repo venv and parses its JSON.
//
// The solve needs opencv-contrib-python 5.x - OpenCV 5 dropped
// aruco.calibrateCameraCharuco and aruco.interpolateCornersCharuco - and 5.x
// cannot be loaded into a process already linked against the system OpenCV 4.6.
// So it runs out of process and talks JSON.
class SolverRunner : public QObject
{
    Q_OBJECT
public:
    explicit SolverRunner( const SolverConfig& cfg, QObject* parent = nullptr );

    bool running() const;
    // Returns false and emits nothing if the venv or script is missing.
    bool start( const QString& sessionDir, QString& why );
    void cancel();

signals:
    void progress( const QString& line );
    void finished( const intrinsics::SolveResult& result );

private:
    void handleFinished( int exitCode );

    SolverConfig mCfg;
    QProcess* mProcess = nullptr;
    QByteArray mStdout;
};

}  // namespace intrinsics

Q_DECLARE_METATYPE( intrinsics::SolveResult )
