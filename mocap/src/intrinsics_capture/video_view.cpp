#include "video_view.h"

#include <QPainter>

namespace intrinsics {

VideoView::VideoView( QWidget* parent )
    : QWidget( parent )
    , mPlaceholder( QStringLiteral( "waiting for frames..." ) )
{
    setMinimumSize( 480, 384 );
    setAutoFillBackground( false );
}

void VideoView::setImage( const QImage& image )
{
    mImage = image;
    update();
}

void VideoView::setPlaceholder( const QString& text )
{
    mPlaceholder = text;
    if( mImage.isNull() ) update();
}

void VideoView::paintEvent( QPaintEvent* )
{
    QPainter p( this );
    p.fillRect( rect(), QColor( 18, 18, 20 ) );

    if( mImage.isNull() )
    {
        p.setPen( QColor( 160, 160, 170 ) );
        p.drawText( rect(), Qt::AlignCenter, mPlaceholder );
        return;
    }

    const QSize scaled = mImage.size().scaled( size(), Qt::KeepAspectRatio );
    const QRect target( ( width() - scaled.width() ) / 2, ( height() - scaled.height() ) / 2,
        scaled.width(), scaled.height() );

    p.setRenderHint( QPainter::SmoothPixmapTransform, true );
    p.drawImage( target, mImage );
}

}  // namespace intrinsics
