//#ifndef MYVIZ_H
//#define MYVIZ_H
#ifndef rqt_control_panel__MyRviz_H
#define rqt_control_panel__MyRviz_H

//#include "rqt_control_panel/control_panel.h"

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"

#include <QWidget>
#include <QFrame>

namespace rviz
{
  class Display;
  class RenderPanel;
  class VisualizationManager;
}



class MyRviz
//  : public QWidget
  : public QFrame
{
  Q_OBJECT

public:
  MyRviz( QWidget* parent);
  virtual void setFixedFrame(QString fixed_frame_);
  virtual ~MyRviz();

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
  rviz::Display* robot_modol_;

};


#endif
