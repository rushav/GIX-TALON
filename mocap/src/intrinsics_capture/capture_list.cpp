#include "capture_list.h"

#include <QIcon>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QPixmap>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

namespace intrinsics {
namespace {

QIcon iconFor( const cv::Mat& bgr )
{
    if( bgr.empty() ) return {};
    const QImage img( bgr.data, bgr.cols, bgr.rows, static_cast<int>( bgr.step ),
        QImage::Format_BGR888 );
    return QIcon( QPixmap::fromImage( img.copy() ) );
}

}  // namespace

CaptureListWidget::CaptureListWidget( QWidget* parent )
    : QWidget( parent )
{
    auto* v = new QVBoxLayout( this );
    v->setContentsMargins( 0, 0, 0, 0 );

    mSummary = new QLabel( QStringLiteral( "no captures yet" ), this );
    mSummary->setStyleSheet( QStringLiteral( "color:#8a8a92; font-size:11px;" ) );
    v->addWidget( mSummary );

    mList = new QListWidget( this );
    mList->setViewMode( QListView::IconMode );
    // Tiles carry three lines of text under the thumbnail; the grid has to be
    // tall enough or the tilt line is clipped.
    mList->setIconSize( QSize( 120, 96 ) );
    mList->setGridSize( QSize( 136, 166 ) );
    mList->setResizeMode( QListView::Adjust );
    mList->setMovement( QListView::Static );
    mList->setWordWrap( true );
    mList->setSelectionMode( QAbstractItemView::ExtendedSelection );
    v->addWidget( mList, 1 );

    mDelete = new QPushButton( QStringLiteral( "Delete selected capture" ), this );
    mDelete->setEnabled( false );
    v->addWidget( mDelete );

    connect( mList, &QListWidget::itemSelectionChanged, this,
        [this] { mDelete->setEnabled( !mList->selectedItems().isEmpty() ); } );
    connect( mDelete, &QPushButton::clicked, this, &CaptureListWidget::emitDelete );

    auto* del = new QShortcut( QKeySequence( Qt::Key_Delete ), mList );
    del->setContext( Qt::WidgetWithChildrenShortcut );
    connect( del, &QShortcut::activated, this, &CaptureListWidget::emitDelete );
}

void CaptureListWidget::emitDelete()
{
    const auto selected = mList->selectedItems();
    if( selected.isEmpty() ) return;
    // Emit one index; the owner re-renders the list, so deleting several is a
    // matter of pressing Delete again rather than tracking shifting indices.
    emit deleteRequested( mList->row( selected.first() ) );
}

void CaptureListWidget::setRecords( const std::vector<CaptureRecord>& records, int expectedCorners )
{
    mList->clear();

    for( const auto& r : records )
    {
        auto* item = new QListWidgetItem( iconFor( r.thumbnail ), QString() );
        item->setText( QStringLiteral( "%1\n%2/%3 corners%4" )
                           .arg( QString::fromStdString( r.file ).remove( QStringLiteral( ".png" ) ) )
                           .arg( r.cornerCount )
                           .arg( expectedCorners )
                           .arg( r.tiltDegrees >= 0
                                   ? QStringLiteral( "\n%1 deg tilt" ).arg( r.tiltDegrees, 0, 'f', 0 )
                                   : QString() ) );
        item->setTextAlignment( Qt::AlignHCenter | Qt::AlignTop );

        if( mWorst.contains( QString::fromStdString( r.file ) ) )
        {
            item->setForeground( QColor( 224, 160, 48 ) );
            item->setToolTip( QStringLiteral( "One of the worst-reprojecting views in the last solve." ) );
        }
        else if( r.cornerCount < expectedCorners )
        {
            item->setForeground( QColor( 170, 170, 178 ) );
        }
        mList->addItem( item );
    }

    mSummary->setText( records.empty()
            ? QStringLiteral( "no captures yet" )
            : QStringLiteral( "%1 capture(s)" ).arg( records.size() ) );
    mDelete->setEnabled( !mList->selectedItems().isEmpty() );
}

void CaptureListWidget::setWorstViews( const QStringList& files )
{
    mWorst = files;
}

}  // namespace intrinsics
