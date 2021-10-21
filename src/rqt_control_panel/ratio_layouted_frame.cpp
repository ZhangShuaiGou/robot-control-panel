#include <rqt_control_panel/ratio_layouted_frame.h>

#include <assert.h>
#include <QMouseEvent>

namespace rqt_control_panel {

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags)
  : QFrame()
  , outer_layout_(NULL)
  , aspect_ratio_(16, 9)
  , smoothImage_(false)
{
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
}

const QImage& RatioLayoutedFrame::getImage() const
{
  return qimage_;
}

QImage RatioLayoutedFrame::getImageCopy() const
{
  QImage img;
  qimage_mutex_.lock();
  img = qimage_.copy();
  qimage_mutex_.unlock();
  return img;
}

void RatioLayoutedFrame::setImage(const QImage& image)//, QMutex* image_mutex)
{
  qimage_mutex_.lock();
  qimage_ = image.copy();
  setAspectRatio(qimage_.width(), qimage_.height());
  qimage_mutex_.unlock();
  emit delayed_update();
}

void RatioLayoutedFrame::resizeToFitAspectRatio()
{
  QRect rect = contentsRect();

  // reduce longer edge to aspect ration
  double width;
  double height;

  if (outer_layout_)
  {
    width = outer_layout_->contentsRect().width();
    height = outer_layout_->contentsRect().height();
  }
  else
  {
    // if outer layout isn't available, this will use the old
    // width and height, but this can shrink the display image if the
    // aspect ratio changes.
    width = rect.width();
    height = rect.height();
  }

  double layout_ar = width / height;
  const double image_ar = double(aspect_ratio_.width()) / double(aspect_ratio_.height());
  if (layout_ar > image_ar)
  {
    // too large width
    width = height * image_ar;
  }
  else
  {
    // too large height
    height = width / image_ar;
  }
  rect.setWidth(int(width + 0.5));
  rect.setHeight(int(height + 0.5));

  // resize taking the border line into account
  int border = lineWidth();
  resize(rect.width() + 2 * border, rect.height() + 2 * border);
}

void RatioLayoutedFrame::setOuterLayout(QHBoxLayout* outer_layout)
{
  outer_layout_ = outer_layout;
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
{
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setAspectRatio(unsigned short width, unsigned short height)
{
  int divisor = greatestCommonDivisor(width, height);
  if (divisor != 0) {
    aspect_ratio_.setWidth(width / divisor);
    aspect_ratio_.setHeight(height / divisor);
  }
}

void RatioLayoutedFrame::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  qimage_mutex_.lock();
  if (!qimage_.isNull())
  {
    resizeToFitAspectRatio();
    // TODO: check if full draw is really necessary
    //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
    //painter.drawImage(paint_event->rect(), qimage_);
    if (!smoothImage_) {
      painter.drawImage(contentsRect(), qimage_);
    } else {
      if (contentsRect().width() == qimage_.width()) {
        painter.drawImage(contentsRect(), qimage_);
      } else {
        QImage image = qimage_.scaled(contentsRect().width(), contentsRect().height(),
                                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
        painter.drawImage(contentsRect(), image);
      }
    }
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  qimage_mutex_.unlock();
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
  if (b==0)
  {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

void RatioLayoutedFrame::mousePressEvent(QMouseEvent * mouseEvent)
{
  if(mouseEvent->button() == Qt::LeftButton)
  {
    emit mouseLeft(mouseEvent->x(), mouseEvent->y());
  }
  QFrame::mousePressEvent(mouseEvent);
}

void RatioLayoutedFrame::onSmoothImageChanged(bool checked)
{
  smoothImage_ = checked;
}

}
