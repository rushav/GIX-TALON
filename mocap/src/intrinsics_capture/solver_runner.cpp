#include "solver_runner.h"

#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>

namespace intrinsics {

SolverRunner::SolverRunner( const SolverConfig& cfg, QObject* parent )
    : QObject( parent )
    , mCfg( cfg )
{
    qRegisterMetaType<intrinsics::SolveResult>( "intrinsics::SolveResult" );
}

bool SolverRunner::running() const
{
    return mProcess && mProcess->state() != QProcess::NotRunning;
}

bool SolverRunner::start( const QString& sessionDir, QString& why )
{
    if( running() )
    {
        why = QStringLiteral( "a solve is already running" );
        return false;
    }

    const QString python = QString::fromStdString( mCfg.python.string() );
    const QString script = QString::fromStdString( mCfg.script.string() );

    if( !QFileInfo::exists( python ) )
    {
        why = QStringLiteral( "no interpreter at %1.\nCreate the venv - see mocap/docs/RUNBOOK.md." )
                  .arg( python );
        return false;
    }
    if( !QFileInfo::exists( script ) )
    {
        why = QStringLiteral( "no solver script at %1" ).arg( script );
        return false;
    }

    mStdout.clear();
    mProcess = new QProcess( this );
    mProcess->setProgram( python );
    mProcess->setArguments( { script, sessionDir,
        QStringLiteral( "--min-corners" ), QString::number( mCfg.minCorners ) } );

    // stdout carries the result JSON; stderr is progress. Keep them apart.
    connect( mProcess, &QProcess::readyReadStandardOutput, this,
        [this] { mStdout += mProcess->readAllStandardOutput(); } );
    connect( mProcess, &QProcess::readyReadStandardError, this, [this] {
        const QString text = QString::fromUtf8( mProcess->readAllStandardError() );
        for( const QString& line : text.split( '\n', Qt::SkipEmptyParts ) ) emit progress( line );
    } );
    connect( mProcess, &QProcess::finished, this,
        [this]( int code, QProcess::ExitStatus ) { handleFinished( code ); } );
    connect( mProcess, &QProcess::errorOccurred, this, [this]( QProcess::ProcessError ) {
        SolveResult r;
        r.error = QStringLiteral( "could not run the solver: %1" ).arg( mProcess->errorString() );
        emit finished( r );
    } );

    mProcess->start();
    return true;
}

void SolverRunner::cancel()
{
    if( running() ) mProcess->kill();
}

void SolverRunner::handleFinished( int exitCode )
{
    SolveResult r;

    if( exitCode != 0 )
    {
        r.error = QStringLiteral( "solver exited with code %1 - see the log below" ).arg( exitCode );
        emit finished( r );
        return;
    }

    QJsonParseError parseError{};
    const QJsonDocument doc = QJsonDocument::fromJson( mStdout, &parseError );
    if( doc.isNull() || !doc.isObject() )
    {
        r.error = QStringLiteral( "could not parse solver output: %1" ).arg( parseError.errorString() );
        emit finished( r );
        return;
    }

    const QJsonObject root = doc.object();
    const QJsonObject in = root["intrinsics"].toObject();
    r.fx = in["fx"].toDouble();
    r.fy = in["fy"].toDouble();
    r.cx = in["cx"].toDouble();
    r.cy = in["cy"].toDouble();
    r.fxFyPercentDiff = in["fx_fy_percent_diff"].toDouble();
    r.rms = root["rms"].toDouble();
    r.opencvVersion = root["software"].toObject()["opencv_version"].toString();
    r.solvedUtc = root["solved_utc"].toString();
    r.viewsUsed = root["solver"].toObject()["views_used"].toInt();

    for( const auto v : root["distortion"].toObject()["coefficients"].toArray() )
        r.distortion.push_back( v.toDouble() );

    for( const auto v : root["views"].toArray() )
    {
        const QJsonObject o = v.toObject();
        r.views.push_back( { o["file"].toString(), o["corners"].toInt(),
            o["reprojection_error_px"].toDouble() } );
    }

    for( const auto v : root["skipped"].toArray() )
    {
        const QJsonObject o = v.toObject();
        r.skipped << QStringLiteral( "%1: %2" ).arg( o["file"].toString(),
            o["reason"].toString() );
    }

    r.ok = true;
    emit finished( r );
}

}  // namespace intrinsics
