#include <rqt_control_panel/my_rviz.h>
#include <assert.h>

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QColor>
#include <QVBoxLayout>


MyRviz::MyRviz( QWidget* parent)
  //: QWidget( parent )
  : QFrame()
{

  render_panel_ = new rviz::RenderPanel();

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( render_panel_ );
  setLayout( main_layout );
//  QString fixed_frame_ = "base_link";

  manager_ = new rviz::VisualizationManager( render_panel_ );
//  manager_->setFixedFrame(fixed_frame_);
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();


  grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
//  grid_->subProp( "Color" )->setValue(QColor(27,27,27,127));
  ROS_ASSERT( grid_ != NULL );


  robot_modol_ = manager_->createDisplay("rviz/RobotModel", "your robot", true);
//  robot_modol_->setFixedFrame(fixed_frame_);
//  robot_modol_->subProp("Robot Description")->('/fetch.urdf');
//  robot_modol_->subProp("Update Rate")->setValue(0.1);

  ROS_ASSERT( robot_modol_ != NULL );

}

void MyRviz::setFixedFrame(QString fixed_frame_)
{
  manager_->setFixedFrame(fixed_frame_);
}


MyRviz::~MyRviz()
{
  delete manager_;
}
