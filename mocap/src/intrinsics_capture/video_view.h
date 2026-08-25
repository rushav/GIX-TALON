#pragma once

#include <QImage>
#include <QWidget>

namespace intrinsics {

// Paints the live frame letterboxed into whatever space the layout gives it.
// The image itself is always native resolution; only the blit is scaled.
class VideoView : public QWidget
{
    Q_OBJECT
public:
    explicit VideoView( QWidget* parent = nullptr );

    void setImage( const QImage& image );
    void setPlaceholder( const QString& text );

protected:
    void paintEvent( QPaintEvent* event ) override;

private:
    QImage mImage;
    QString mPlaceholder;
};

}  // namespace intrinsics
