#pragma once

#include "capture_session.h"

#include <QWidget>

class QListWidget;
class QPushButton;
class QLabel;

namespace intrinsics {

// Thumbnails of every capture with its corner count and tilt. Deleting here is
// the way a bad view gets out of the solve, so it has to unwind coverage and
// pose stats too - which CaptureSession::remove does.
class CaptureListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CaptureListWidget( QWidget* parent = nullptr );

    void setRecords( const std::vector<CaptureRecord>& records, int expectedCorners );
    // Highlights the worst-reprojecting views after a solve, by filename.
    void setWorstViews( const QStringList& files );

signals:
    void deleteRequested( int index );

private:
    void emitDelete();

    QListWidget* mList = nullptr;
    QPushButton* mDelete = nullptr;
    QLabel* mSummary = nullptr;
    QStringList mWorst;
};

}  // namespace intrinsics
