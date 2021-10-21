#include "rqt_control_panel/control_panel.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QStringList>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

namespace rqt_control_panel {

ControlPanel::ControlPanel()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
//  , num_gridlines_(0)   ready to remove
  , rotate_state_(ROTATE_0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("ControlPanel");
}


void ControlPanel::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }

  context.addWidget(widget_);

//create new thread as a worker. In order to free the main thread while doing some time consuming action
  Worker *worker = new Worker;
  worker->moveToThread(&workerThread);
  workerThread.start();

//ui_.functionTab->removeTab(3);

//connects and relative settings used in setting tab
ui_.lineEdit_base_topic_1->setEnabled(false);
ui_.checkBox_base_topic_1->setEnabled(false);
ui_.comboBox_base_topic_1->setEnabled(false);
updateTopicList(ui_.lineEdit_base_topic_1,ui_.comboBox_base_topic_1);
connect(ui_.lineEdit_base_topic_1, SIGNAL(textChanged(const QString &)),this, SLOT(updateOnBase1()));
connect(ui_.checkBox_base_topic_1, SIGNAL(toggled(bool)),this, SLOT(updateOnBase1()));

ui_.lineEdit_base_topic_2->setEnabled(false);
ui_.checkBox_base_topic_2->setEnabled(false);
ui_.comboBox_base_topic_2->setEnabled(false);
updateTopicList(ui_.lineEdit_base_topic_2,ui_.comboBox_base_topic_2);
connect(ui_.lineEdit_base_topic_2, SIGNAL(textChanged(const QString &)),this, SLOT(updateOnBase2()));
connect(ui_.checkBox_base_topic_2, SIGNAL(toggled(bool)),this, SLOT(updateOnBase2()));


ui_.lineEdit_arm_topic->setEnabled(false);
ui_.comboBox_arm_topic->setEnabled(false);
updateTopicList(ui_.lineEdit_arm_topic,ui_.comboBox_arm_topic);
connect(ui_.lineEdit_arm_topic, SIGNAL(textChanged(const QString &)),this, SLOT(updateOnArm()));
connect(ui_.checkBox_arm_topic, SIGNAL(toggled(bool)),this, SLOT(updateOnArm()));


ui_.lineEdit_scan_topic->setEnabled(false);
ui_.comboBox_scan_topic->setEnabled(false);
updateTopicList(ui_.lineEdit_scan_topic,ui_.comboBox_scan_topic);
connect(ui_.lineEdit_scan_topic, SIGNAL(textChanged(const QString &)),this, SLOT(updateOnScan()));
connect(ui_.checkBox_scan_topic, SIGNAL(toggled(bool)),this, SLOT(updateOnScan()));

connect(ui_.checkBox_base_topic,SIGNAL(clicked(bool)), this, SLOT(onBaseTopic(bool)));
connect(ui_.checkBox_base_topic_1,SIGNAL(clicked(bool)), this, SLOT(onBaseTopic1(bool)));
connect(ui_.checkBox_base_topic_2,SIGNAL(clicked(bool)), this, SLOT(onBaseTopic2(bool)));
connect(ui_.checkBox_arm_topic,SIGNAL(clicked(bool)), this, SLOT(onArmTopic(bool)));
connect(ui_.checkBox_scan_topic,SIGNAL(clicked(bool)), this, SLOT(onScanTopic(bool)));

ui_.topics_resetButton->setEnabled(false);
connect(ui_.topics_resetButton, SIGNAL(clicked()), this, SLOT(topics_resetButtonClicked()));
ui_.topics_confirmButton->setEnabled(false);
connect(ui_.topics_confirmButton, SIGNAL(clicked()), this, SLOT(topics_confirmButtonClicked()));


//connects and relative settings used in rviz tab
//init Rviz in rviz tab
  ui_.myRvizframe->setParent(ui_.tab_rviz);
  ui_.myRvizframe->show();
  updateFrameList(ui_.rvizFrameBox);
  connect(ui_.rvizFrameBox, SIGNAL(currentTextChanged(QString)), this, SLOT(onRvizFrameChanged(QString)));
  connect(ui_.rvizFrameRefreshButton, SIGNAL(clicked()), this, SLOT(refreshFrameList()));

//make create custom pose button windows
  ui_.cusWidget->setParent(0);

  ui_.armHomeButton->setEnabled(false);
  ui_.addCusPoseButton->setEnabled(false);
  ui_.openGripperButton->setEnabled(false);
  ui_.armStraightButton->setEnabled(false);
  ui_.closeGripperButton->setEnabled(false);
  ui_.armTargetPoseX->setEnabled(false);
  ui_.armTargetPoseY->setEnabled(false);
  ui_.armTargetPoseZ->setEnabled(false);
  ui_.armTargetMoveButton->setEnabled(false);
  ui_.getPoseButton->setEnabled(false);
  ui_.armStopButton->setEnabled(false);
  ui_.rvizFrameBox->setEnabled(false);
  ui_.rvizFrameRefreshButton->setEnabled(false);


//  connect(ui_.armDiscoButton, SIGNAL(clicked()), worker, SLOT(armDisco()));  //considering to remove
  connect(ui_.printJointsButton, SIGNAL(clicked()), worker, SLOT(printJoints()));   //considering to remove
  connect(ui_.armHomeButton, SIGNAL(clicked()), worker, SLOT(armHome()));
  connect(ui_.armStraightButton, SIGNAL(clicked()), worker, SLOT(armStraight()));
  connect(ui_.armStopButton, SIGNAL(clicked()), worker, SLOT(armStop()));     //stop all the actions in Rviz tab
  connect(ui_.openGripperButton, SIGNAL(clicked()), worker, SLOT(openGripper()));
  connect(ui_.closeGripperButton, SIGNAL(clicked()), worker, SLOT(closeGripper()));
  connect(ui_.armTargetMoveButton, SIGNAL(clicked()), this, SLOT(onArmTargetMoveButtonClicked()));
  connect(ui_.getPoseButton, SIGNAL(clicked()), worker, SLOT(getPose()));
  connect(worker, SIGNAL(do_update_target_poses_(geometry_msgs::PoseStamped)), this, SLOT(updateTargetPoses(geometry_msgs::PoseStamped)));

//  connect(this, SIGNAL(do_arm_(QString)), worker, SLOT(createArmClient(QString))); //create a actionlib client
  connect(this, SIGNAL(do_arm_(QString)), worker, SLOT(createArmController(QString))); //create an arm controller
  connect(worker, SIGNAL(do_armMove_(std::vector<double>)), worker, SLOT(armMove(std::vector<double>)));
  connect(this, SIGNAL(do_armMove_(std::vector<double>)), worker, SLOT(armMove(std::vector<double>)));

  qRegisterMetaType<geometry_msgs::PoseStamped>("geometry_msgs::PoseStamped");
  qRegisterMetaType<std::vector<double>>("std::vector<double>");
//  connect(this, SIGNAL(do_armMove_(geometry_msgs::PoseStamped)), worker, SLOT(armMove(geometry_msgs::PoseStamped)));
  connect(this, SIGNAL(do_armMove_(double,double,double)), worker, SLOT(armMove(double,double,double)));
  connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);



  //connects and relative settings used in base tab
  //disable the camera elements in advance
  ui_.cameraButton->setEnabled(false);
  ui_.pointHeadcomboBox->setEnabled(false);
  ui_.scanFrameRefreshButton->setEnabled(false);
  ui_.zoomButton->setEnabled(false);
  ui_.leftRotateButton->setEnabled(false);
  ui_.rightRotateButton->setEnabled(false);
  ui_.dynamicRangecheckBox->setEnabled(false);
  ui_.maxRangespinBox->setEnabled(false);
  ui_.saveImageButton->setEnabled(false);
  ui_.headHomeButton->setEnabled(false);
  ui_.upHeadButton->setEnabled(false);
  ui_.leftHeadButton->setEnabled(false);
  ui_.downHeadButton->setEnabled(false);
  ui_.rightHeadButton->setEnabled(false);

  ui_.cameraButton->setCheckable(true);
  connect(ui_.cameraButton, SIGNAL(clicked(bool)), this, SLOT(oncameraButtonCLicked(bool)));
  ui_.zoomButton->setCheckable(true);
  connect(ui_.zoomButton, SIGNAL(toggled(bool)), this, SLOT(onZoom(bool)));
  connect(ui_.dynamicRangecheckBox, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange(bool)));
  connect(ui_.saveImageButton, SIGNAL(pressed()), this, SLOT(saveImage()));
//  connect(ui_.gridlinesNumspinBox, SIGNAL(valueChanged(int)), this, SLOT(updateNumGridlines()));    ready to remove
//  connect(ui_.smoothcheckBox, SIGNAL(toggled(bool)), ui_.imageFrame, SLOT(onSmoothImageChanged(bool)));   ready to remove
  connect(ui_.leftRotateButton, SIGNAL(clicked(bool)), this, SLOT(onRotateLeft()));
  connect(ui_.rightRotateButton, SIGNAL(clicked(bool)), this, SLOT(onRotateRight()));

  connect(ui_.scanFrameRefreshButton, SIGNAL(clicked()), this, SLOT(refreshFrameList()));
  ui_.upHeadButton->setAutoRepeat(true);
  ui_.upHeadButton->setAutoRepeatInterval(90);
  connect(ui_.upHeadButton, SIGNAL(clicked()), this, SLOT(onUpHeadButtonClicked()));

  ui_.downHeadButton->setAutoRepeat(true);
  ui_.downHeadButton->setAutoRepeatInterval(90);
  connect(ui_.downHeadButton, SIGNAL(clicked()), this, SLOT(onDownHeadButtonClicked()));

  ui_.leftHeadButton->setAutoRepeat(true);
  ui_.leftHeadButton->setAutoRepeatInterval(90);
  connect(ui_.leftHeadButton, SIGNAL(clicked()), this, SLOT(onLeftHeadButtonClicked()));

  ui_.rightHeadButton->setAutoRepeat(true);
  ui_.rightHeadButton->setAutoRepeatInterval(90);
  connect(ui_.rightHeadButton, SIGNAL(clicked()), this, SLOT(onRightHeadButtonClicked()));

  connect(ui_.headHomeButton, SIGNAL(clicked()), this, SLOT(headHome()));

//  connect(this, SIGNAL(do_move_head_()), worker, SLOT(moveHead()));     ready to remove
  connect(this, SIGNAL(do_point_head_()), worker, SLOT(pointHead()));

//update the frame list
  updateFrameList(ui_.pointHeadcomboBox);

//disable the move base elements in advance
  ui_.p_x->setEnabled(false);
  ui_.p_y->setEnabled(false);
  ui_.o_y->setEnabled(false);


  ui_.goButton->setEnabled(false);
  connect(ui_.goButton,SIGNAL(clicked()),this,SLOT(ongoButtonClicked()));
  ui_.stopButton->setEnabled(false);
  connect(ui_.stopButton, SIGNAL(clicked()), this, SLOT(onstopButtonClicked()));

  ui_.lineEdit_base_forward_dist->setEnabled(false);
  ui_.lineEdit_base_turn_rad->setEnabled(false);
  ui_.lineEdit_base_sec->setEnabled(false);
  ui_.forwardButton->setEnabled(false);
  connect(ui_.forwardButton, SIGNAL (clicked()), this, SLOT(onforwardButtonClicked()));
  ui_.stopButton_2->setEnabled(false);
  connect(ui_.stopButton_2, SIGNAL(clicked()), this, SLOT(onForwardstopButtonClicked()));
  //Run in another thread
  connect(this, SIGNAL(do_forward_(float,float,float)), worker, SLOT(forward_Base(float,float,float)));


  connect(ui_.addCusPoseButton, SIGNAL(clicked()), this, SLOT(addButtonClicked()));
  connect(ui_.createCustomButton, SIGNAL(clicked()), this, SLOT(createNewPoseButton()));
  connect(ui_.cusCancelButton, SIGNAL(clicked()), this, SLOT(cusCancelButtonClicked()));

  ui_.cusWidget->setVisible(false);

//  ui_.detectButton->setCheckable(true);
  connect(ui_.detectButton, SIGNAL(clicked()), worker, SLOT(boxSegment()));
  connect(ui_.removeButton, SIGNAL(clicked()), worker, SLOT(removeBox()));
  connect(ui_.pickButton, SIGNAL(clicked()), worker, SLOT(pick()));
//  ui_.stopButton->setEnabled(false);    ready to remove
//  connect(ui_.stopButton, SIGNAL(clicked()), this, SLOT(onstopButtonClicked()));      ready to remove

  //connect(ui_.pointHeadcheckBox, SIGNAL(toggled(bool)), this, SLOT(onPointHeadCheckBoxClicked(bool)));    ready to remove
  //onPointHeadCheckBoxClicked(0);    ready to remove
}

//below is for setting function
void ControlPanel::topics_resetButtonClicked()
{
  ui_.lineEdit_base_topic_1->clear();
  updateTopicList(ui_.lineEdit_base_topic_1,ui_.comboBox_base_topic_1);

  ui_.lineEdit_base_topic_2->clear();
  updateTopicList(ui_.lineEdit_base_topic_2,ui_.comboBox_base_topic_2);

  ui_.lineEdit_arm_topic->clear();
  updateTopicList(ui_.lineEdit_arm_topic,ui_.comboBox_arm_topic);

  ui_.lineEdit_scan_topic->clear();
  updateTopicList(ui_.lineEdit_scan_topic,ui_.comboBox_scan_topic);

/*  ui_.comboBox_base_topic_1->clear();
  ui_.comboBox_base_topic_2->clear();
  ui_.comboBox_arm_topic->clear();
  ui_.comboBox_scan_topic->clear();*/
}

void ControlPanel::onBaseTopic(bool checked)
{
  if (ui_.checkBox_base_topic_1->checkState())
  {
    ui_.checkBox_base_topic_1->setChecked(0);
    onBaseTopic1(0);
  }
  if (ui_.checkBox_base_topic_2->checkState())
  {
    ui_.checkBox_base_topic_2->setChecked(0);
    onBaseTopic2(0);
  }

  ui_.checkBox_base_topic_1->setEnabled(checked);
  ui_.checkBox_base_topic_2->setEnabled(checked);
}

void ControlPanel::onBaseTopic1(bool checked)
{
  ui_.lineEdit_base_topic_1->setEnabled(checked);
  ui_.comboBox_base_topic_1->setEnabled(checked);

//  ui_.p_z->setEnabled(checked);
  if (ui_.goButton->isEnabled())
    ui_.goButton->setEnabled(checked);
  if (ui_.stopButton->isEnabled())
    ui_.stopButton->setEnabled(checked);
  if (ui_.p_x->isEnabled())
    ui_.p_x->setEnabled(checked);
  if (ui_.p_y->isEnabled())
    ui_.p_y->setEnabled(checked);
  if (ui_.o_y->isEnabled())
    ui_.o_y->setEnabled(checked);


  checked?con_res_button_flag++:con_res_button_flag--;
  ui_.topics_resetButton->setEnabled(con_res_button_flag);
  ui_.topics_confirmButton->setEnabled(con_res_button_flag);
}

void ControlPanel::onBaseTopic2(bool checked)
{
  ui_.lineEdit_base_topic_2->setEnabled(checked);
  ui_.comboBox_base_topic_2->setEnabled(checked);

  if (ui_.forwardButton->isEnabled())
    ui_.forwardButton->setEnabled(checked);
  if (ui_.stopButton_2->isEnabled())
    ui_.stopButton_2->setEnabled(checked);
  if (ui_.lineEdit_base_forward_dist->isEnabled())
    ui_.lineEdit_base_forward_dist->setEnabled(checked);
  if (ui_.lineEdit_base_turn_rad->isEnabled())
    ui_.lineEdit_base_turn_rad->setEnabled(checked);
  if (ui_.lineEdit_base_sec->isEnabled())
    ui_.lineEdit_base_sec->setEnabled(checked);


  checked?con_res_button_flag++:con_res_button_flag--;
  ui_.topics_resetButton->setEnabled(con_res_button_flag);
  ui_.topics_confirmButton->setEnabled(con_res_button_flag);
}

void ControlPanel::onArmTopic(bool checked)
{
  ui_.lineEdit_arm_topic->setEnabled(checked);
  ui_.comboBox_arm_topic->setEnabled(checked);

  if (ui_.armHomeButton->isEnabled())
    ui_.armHomeButton->setEnabled(checked);
  if (ui_.addCusPoseButton->isEnabled())
    ui_.addCusPoseButton->setEnabled(checked);
  if (ui_.openGripperButton->isEnabled())
    ui_.openGripperButton->setEnabled(checked);
  if (ui_.armStraightButton->isEnabled())
    ui_.armStraightButton->setEnabled(checked);
  if (ui_.closeGripperButton->isEnabled())
    ui_.closeGripperButton->setEnabled(checked);
  if (ui_.armTargetPoseX->isEnabled())
    ui_.armTargetPoseX->setEnabled(checked);
  if (ui_.armTargetPoseY->isEnabled())
    ui_.armTargetPoseY->setEnabled(checked);
  if (ui_.armTargetPoseZ->isEnabled())
    ui_.armTargetPoseZ->setEnabled(checked);
  if (ui_.armTargetMoveButton->isEnabled())
    ui_.armTargetMoveButton->setEnabled(checked);
  if (ui_.getPoseButton->isEnabled())
    ui_.getPoseButton->setEnabled(checked);
  if (ui_.armStopButton->isEnabled())
    ui_.armStopButton->setEnabled(checked);
  if (ui_.rvizFrameBox->isEnabled())
    ui_.rvizFrameBox->setEnabled(checked);
  if (ui_.rvizFrameRefreshButton->isEnabled())
    ui_.rvizFrameRefreshButton->setEnabled(checked);


  checked?con_res_button_flag++:con_res_button_flag--;
  ui_.topics_resetButton->setEnabled(con_res_button_flag);
  ui_.topics_confirmButton->setEnabled(con_res_button_flag);
}

void ControlPanel::onScanTopic(bool checked)
{
  ui_.lineEdit_scan_topic->setEnabled(checked);
  ui_.comboBox_scan_topic->setEnabled(checked);

  if (ui_.cameraButton->isEnabled())
    ui_.cameraButton->setEnabled(checked);
  if (ui_.pointHeadcomboBox->isEnabled())
    ui_.pointHeadcomboBox->setEnabled(checked);
  if (ui_.scanFrameRefreshButton->isEnabled())
    ui_.scanFrameRefreshButton->setEnabled(checked);
  if (ui_.zoomButton->isEnabled())
    ui_.zoomButton->setEnabled(checked);
  if (ui_.leftRotateButton->isEnabled())
    ui_.leftRotateButton->setEnabled(checked);
  if (ui_.rightRotateButton->isEnabled())
    ui_.rightRotateButton->setEnabled(checked);
  if (ui_.dynamicRangecheckBox->isEnabled())
    ui_.dynamicRangecheckBox->setEnabled(checked);
  if (ui_.maxRangespinBox->isEnabled())
    ui_.maxRangespinBox->setEnabled(checked);
  if (ui_.saveImageButton->isEnabled())
    ui_.saveImageButton->setEnabled(checked);
  if (ui_.headHomeButton->isEnabled())
    ui_.headHomeButton->setEnabled(checked);
  if (ui_.upHeadButton->isEnabled())
    ui_.upHeadButton->setEnabled(checked);
  if (ui_.leftHeadButton->isEnabled())
    ui_.leftHeadButton->setEnabled(checked);
  if (ui_.downHeadButton->isEnabled())
    ui_.downHeadButton->setEnabled(checked);
  if (ui_.rightHeadButton->isEnabled())
    ui_.rightHeadButton->setEnabled(checked);


  checked?con_res_button_flag++:con_res_button_flag--;
  ui_.topics_resetButton->setEnabled(con_res_button_flag);
  ui_.topics_confirmButton->setEnabled(con_res_button_flag);
}

void ControlPanel::updateOnBase1()
{
//in order to append topic in the comboBox
  pose_goal_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
  updateTopicList(ui_.lineEdit_base_topic_1,ui_.comboBox_base_topic_1);
}

void ControlPanel::updateOnBase2()
{
  updateTopicList(ui_.lineEdit_base_topic_2,ui_.comboBox_base_topic_2);
}

void ControlPanel::updateOnArm()
{
  updateTopicList(ui_.lineEdit_arm_topic,ui_.comboBox_arm_topic);
}

void ControlPanel::updateOnScan()
{
  updateTopicList(ui_.lineEdit_scan_topic,ui_.comboBox_scan_topic);
}

void ControlPanel::updateTopicList(QLineEdit* lineEdit, QComboBox* comboBox)
{
//  QSet<QString> message_types;
    QString message_types;
//  message_types.insert(lineEdit->text());
    message_types = lineEdit->text();
//  message_types.insert("sensor_msgs/Image");
//  message_types.insert("geometry_msgs/PoseStamped");

  QString selected = comboBox->currentText();
//  QString selected = ui_.comboBox_base_topic_1->currentText();

  QList<QString> topics;
  topics.append(getTopics(message_types).values());
  //if you want a blank on the top, uncommend the next line
  topics.append("");
  qSort(topics);
  comboBox->clear();
  for (QList<QString>::const_iterator it = topics.begin() ; it != topics.end(); it++)
  {
    QString label(*it);
    comboBox->addItem(label, QVariant(*it));
  }
}

void ControlPanel::updateFrameList(QComboBox* comboBox)
{
    QVector<std::string> frames_;
    std::vector<std::string> std_frames_;
  //  rviz::FrameManager frameManager;
  //  frameManager.getTFClient()->getFrameStrings(std_frames_);
    tf::TransformListener listener;
//    listener.waitForTransform ("base_link");
  //  ROS_INFO("time is : %d", listener.getCacheLength ());
    sleep(0.5);
    listener.getFrameStrings(std_frames_);

  if (std_frames_.empty())
  {
     ROS_INFO("List empty! Try again please!");
  }
//    frames_ = QVector<std::string>::fromStdVector(std_frames_);
  else
  {
    std_frames_.push_back("");
    qSort(std_frames_);

    comboBox->clear();
    for (int i=1; i<std_frames_.size(); i++)
  //for (QVector<std::string>::const_iterator it = frames_.begin() ; it != frames_.end(); it++)
    {
  //    QString label = it->c_str();
      QString label = std_frames_.at(i).c_str();

      comboBox->addItem(label.toUtf8().constData(), i);
    //  comboBox->addItem();
      ROS_INFO("Frames are: %s", label.toUtf8().constData());
    }
  }
}

//QSet<QString> ControlPanel::getTopics(const QSet<QString>& message_types)
QSet<QString> ControlPanel::getTopics(QString message_types)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
//    if (message_types.contains(it->datatype.c_str()))
    QString datatype_ = it->datatype.c_str();
    if (datatype_.contains(message_types))
    {
      QString topic = it->name.c_str();
      topics.insert(topic);
    }

  }

  return topics;
}

void ControlPanel::topics_confirmButtonClicked()
{
  if (ui_.checkBox_base_topic_1->isChecked())
  {
    go_topic = ui_.comboBox_base_topic_1->currentText();
    pose_goal_ = nh.advertise<geometry_msgs::PoseStamped>(go_topic.toUtf8().constData(),1);
    cancel_goal_ = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

    ROS_INFO("position move: selected topic: %s", go_topic.toUtf8().constData());

    ui_.goButton->setEnabled(true);
    ui_.stopButton->setEnabled(true);
    ui_.p_x->setEnabled(true);
    ui_.p_y->setEnabled(true);
    ui_.o_y->setEnabled(true);

  }

  if (ui_.checkBox_base_topic_2->isChecked())
  {
    forward_topic = ui_.comboBox_base_topic_2->currentText();
    forward_goal_ = nh.advertise<geometry_msgs::Twist>(forward_topic.toUtf8().constData(),1);

    ROS_INFO("forward: selected topic: %s", forward_topic.toUtf8().constData());

    ui_.forwardButton->setEnabled(true);
    ui_.stopButton_2->setEnabled(true);
    ui_.lineEdit_base_forward_dist->setEnabled(true);
    ui_.lineEdit_base_turn_rad->setEnabled(true);
    ui_.lineEdit_base_sec->setEnabled(true);
  }

  if (ui_.checkBox_arm_topic->isChecked())
  {
    arm_topic = ui_.comboBox_arm_topic->currentText();
    ROS_INFO("arm: selected topic: %s", arm_topic.toUtf8().constData());
    emit do_arm_(arm_topic);
    ui_.armHomeButton->setEnabled(true);
    ui_.addCusPoseButton->setEnabled(true);
    ui_.openGripperButton->setEnabled(true);
    ui_.armStraightButton->setEnabled(true);
    ui_.closeGripperButton->setEnabled(true);
    ui_.armTargetPoseX->setEnabled(true);
    ui_.armTargetPoseY->setEnabled(true);
    ui_.armTargetPoseZ->setEnabled(true);
    ui_.armTargetMoveButton->setEnabled(true);
    ui_.getPoseButton->setEnabled(true);
    ui_.armStopButton->setEnabled(true);
    ui_.rvizFrameBox->setEnabled(true);
    ui_.rvizFrameRefreshButton->setEnabled(true);
  }

  if (ui_.checkBox_scan_topic->isChecked())
  {
    scan_topic = ui_.comboBox_scan_topic->currentText();

    ui_.cameraButton->setEnabled(true);
    ui_.pointHeadcomboBox->setEnabled(true);
    ui_.scanFrameRefreshButton->setEnabled(true);
    ui_.zoomButton->setEnabled(true);
    ui_.leftRotateButton->setEnabled(true);
    ui_.rightRotateButton->setEnabled(true);
    ui_.dynamicRangecheckBox->setEnabled(true);
    ui_.maxRangespinBox->setEnabled(true);
    ui_.saveImageButton->setEnabled(true);
    ui_.headHomeButton->setEnabled(true);
    ui_.upHeadButton->setEnabled(true);
    ui_.leftHeadButton->setEnabled(true);
    ui_.downHeadButton->setEnabled(true);
    ui_.rightHeadButton->setEnabled(true);

  }

  else
  {
    ROS_INFO("Please select at least one topic/frame!");
    return;
  }
}

//below is for rviz
void ControlPanel::refreshFrameList()
{
  if(sender() == ui_.rvizFrameRefreshButton)
    {
      ROS_INFO("Rviz frames updating!");
      updateFrameList(ui_.rvizFrameBox);
    }
  if (sender() == ui_.scanFrameRefreshButton)
    {
      ROS_INFO("Scan frames updating!");
      updateFrameList(ui_.pointHeadcomboBox);
    }
}

void ControlPanel::onRvizFrameChanged(QString fixed_frame_)
{
//  ROS_INFO("ready to change!");   ready to remove
  ui_.myRvizframe->setFixedFrame(fixed_frame_);
//  ROS_INFO("changed!");   ready to remove
}

//below is for move function
void ControlPanel::ongoButtonClicked()
{
  //advertise need some time, so it causes sometimes you need click buton twice to post new pose_msg_ or stop command,
  //so I move below line to topics_confirmButtonClicked(), once selected, it will advertise.
//  pose_goal_ = nh.advertise<geometry_msgs::PoseStamped>(go_topic.toUtf8().constData(),1);
  geometry_msgs::PoseStamped pose_msg_;
  geometry_msgs::Quaternion pose_quaternion;

    p_x_ = ui_.p_x->text().toFloat();
    p_y_ = ui_.p_y->text().toFloat();
    o_y_ = ui_.o_y->text().toFloat();

    o_y_ *= (PI/180);//convert degree to radian

    pose_quaternion = tf::createQuaternionMsgFromYaw(o_y_);

    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.header.frame_id = "map";
    pose_msg_.pose.position.x = p_x_;
    pose_msg_.pose.position.y = p_y_;

    pose_msg_.pose.orientation = pose_quaternion;
    pose_goal_.publish(pose_msg_);
    ROS_INFO("Sending goal, yaw: %f", o_y_);
    ROS_INFO_STREAM(pose_quaternion);
}

void ControlPanel::onforwardButtonClicked()
{
  forwar_dist_ = ui_.lineEdit_base_forward_dist->text().toFloat();
  turn_rad_ = ui_.lineEdit_base_turn_rad->text().toFloat();
  if (ui_.lineEdit_base_sec->text().toFloat())
    forward_sec_ = ui_.lineEdit_base_sec->text().toFloat();

    turn_rad_ *= (PI/180);

  linear_x_ = forwar_dist_ / forward_sec_;
  angular_z_ = turn_rad_ / forward_sec_;

  forward_flag = true;
  emit do_forward_(linear_x_,angular_z_,forward_sec_);
  forward_sec_ = 1.0;
}

void Worker::forward_Base(float x, float z, float s)
{
  geometry_msgs::Twist forward_msg_;
  ROS_INFO("command: %s", forward_topic.toUtf8().constData());

  forward_msg_.linear.x = x;
  forward_msg_.angular.z = z;

  ros::Time start_ = ros::Time::now();
  ros::Rate rate_(5);

  while (ros::Time::now() - start_ < ros::Duration(s))
  {
    if(!forward_flag)
      break;
    forward_goal_.publish(forward_msg_);
    ROS_INFO("Sending goal, linear.x = %f, angular.z = %f, flag = %d",x, z, forward_flag);
    rate_.sleep();
  }
  ROS_INFO("Stop sending goal");
}

void ControlPanel::onstopButtonClicked()
{
//  cancel_goal_ = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  actionlib_msgs::GoalID cancel_msg_;
  cancel_goal_.publish(cancel_msg_);
  ROS_INFO("Stoped manually");
}

void ControlPanel::onForwardstopButtonClicked()
{
  forward_flag = false;
  ROS_INFO("Stoped manually");
}

//below is for arm function         ready to remove
/*
void Worker::createArmClient(QString arm_topic)
{
  ROS_INFO("Creating action client to arm controller ...");

  //in order to change topic into server
  int index = arm_topic.lastIndexOf('/');
  arm_topic = arm_topic.mid(0,index);

//  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );
  move_head_client.reset( new move_head_client(arm_topic.toUtf8().constData()) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !move_head_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }
  ROS_INFO("client server: %s", arm_topic.toUtf8().constData());

}

*/

//Disco by actionlib     ready to remove
/*
void Worker::arm_disco()
{
  control_msgs::FollowJointTrajectoryGoal arm_disco_goal;
  arm_disco_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

  double disco_poses [7][8] = {{0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0},
                               {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                               {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                               {0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0},
                               {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                               {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                               {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0}};

  arm_disco_goal.trajectory.joint_names.push_back("torso_lift_joint");
  arm_disco_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  arm_disco_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  arm_disco_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  arm_disco_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  arm_disco_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  arm_disco_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  arm_disco_goal.trajectory.joint_names.push_back("wrist_roll_joint");

  arm_disco_goal.trajectory.points.resize(7);

  for (int i=0; i<7; i++)
  {
    arm_disco_goal.trajectory.points[i].positions.resize(8);
    arm_disco_goal.trajectory.points[i].velocities.resize(8);

    for (int j=0; j<8; j++)
    {
      arm_disco_goal.trajectory.points[i].positions[j] = disco_poses [i][j];
      arm_disco_goal.trajectory.points[i].velocities[j] = 1.0;
    }
    arm_disco_goal.trajectory.points[i].time_from_start = ros::Duration(2.0);
    ROS_INFO("I am dancing! %d",i);
  }

  int max = 5;
  while(max)
  {
    ArmClient->sendGoal(arm_disco_goal);
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
      ros::Duration(4).sleep(); // sleep for four seconds
    }
    max--;
  }

}
*/

void Worker::createArmController(QString arm_topic)
{
  ROS_INFO("Creating an arm controller ...");

  //in order to change topic into server
  int index = arm_topic.lastIndexOf('/');
  arm_topic = arm_topic.mid(0,index);
  //from /arm_controller/follow_joint_trajectory to /arm_controller
  index = arm_topic.lastIndexOf('/');
  arm_topic = arm_topic.mid(0,index);
  //from /arm_controller to arm
  index = arm_topic.lastIndexOf('_');
  arm_topic = arm_topic.mid(0,index);
  arm_topic = arm_topic.mid(1,arm_topic.length()-1);  //in order to remove '/' before topic string

  ROS_INFO("%s",arm_topic.toUtf8().constData());
  PLANNING_GROUP = arm_topic.toUtf8().constData();
  arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
  ROS_INFO("Created arm controller");

}

void Worker::printJoints()
{
  for (int i=0; i<arm_group->getJointNames().size(); i++)
  {
    QString a = arm_group->getJointNames().at(i).c_str();
    ROS_INFO("Joints: %s", a.toUtf8().constData());
  }
}

//Disco by Moveit!
/*
void Worker::armDisco()   ready to remove
{
//  PLANNING_GROUP = "arm_with_torso";
//  moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP);
//  arm_group.reset( new arm_group_controller(PLANNING_GROUP) );

if (PLANNING_GROUP != "arm_with_torso")
{
  PLANNING_GROUP = "arm_with_torso";
  arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
}

  moveit::planning_interface::MoveGroupInterface::Plan move_plan;

//const robot_state::JointModelGroup* joint_model_group =
//      arm_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
const robot_state::JointModelGroup* joint_model_group =
     arm_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//     bool success = (arm_group.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
     bool success = (arm_group->plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     moveit::core::RobotStatePtr current_state = arm_group.getCurrentState();
     moveit::core::RobotStatePtr current_state = arm_group->getCurrentState();
  std::vector<double> joint_group_positions;

//print all the joints name;
//for (int i=0; i<arm_group.getJointNames().size(); i++)
for (int i=0; i<arm_group->getJointNames().size(); i++)
  {
//    QString a = arm_group.getJointNames().at(i).c_str();
    QString a = arm_group->getJointNames().at(i).c_str();
    ROS_INFO("Joints: %s", a.toUtf8().constData());
  }


   double disco_poses [7][8] = {{0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0},
                                {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                                {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                                {0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0},
                                {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                                {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                                {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0}};

  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (int i=0; i<7; i++)
  {
    for (int j=0; j<8; j++)
    {
      joint_group_positions[j] = disco_poses[i][j];
    }
//    arm_group.setStartStateToCurrentState();
    arm_group->setStartStateToCurrentState();
//    arm_group.setJointValueTarget(joint_group_positions);
    arm_group->setJointValueTarget(joint_group_positions);
    ROS_INFO("Ready to plan %d (joint space goal)", i);
//    success = (arm_group.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    success = (arm_group->plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO("Joint tolerance is %d", arm_group.getGoalJointTolerance());
    ROS_INFO("Joint tolerance is %d", arm_group->getGoalJointTolerance());
//    ROS_INFO("Position tolerance is %d", arm_group.getGoalPositionTolerance());
    ROS_INFO("Position tolerance is %d", arm_group->getGoalPositionTolerance());
	  ROS_INFO("Finish plan %d (joint space goal) with %s", i, success?"SUCESS":"FAILED");

    ROS_INFO("Ready to execute %d (joint space goal)", i);
  //  success = (arm_group.execute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    success = (arm_group->execute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO("Execute plan %d (joint space goal) %s", i, success?"SUCESS":"FAILED");
  }
  ROS_INFO("Disco done");
}
*/
void Worker::armMove(std::vector<double> joint_values)
{
  bool success;
  arm_group->stop();
  arm_group->setGoalJointTolerance(0.003);
  arm_group->setStartStateToCurrentState();
  sleep(0.5);
  arm_group->setJointValueTarget(joint_values);

  ROS_INFO("Ready to plan arm motion ");
  success = (arm_group->plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Finish plan arm motion with %s", success?"SUCESS":"FAILED");

  ROS_INFO("Ready to execute arm motion");
  success = (arm_group->asyncExecute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute plan arm motion with %s",  success?"SUCESS":"FAILED");

}

/* move gripper as geometry_msgs::PoseStamped
void Worker::armMove(geometry_msgs::PoseStamped arm_target_pose_msg)
{
  bool success;
  arm_group->stop();

  arm_group->clearPoseTargets ();
  arm_group->setStartStateToCurrentState();
  sleep(0.2);
  arm_group->setPoseTarget(arm_target_pose_msg,"gripper_link");

  ROS_INFO("Ready to plan arm motion ");
  success = (arm_group->plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Finish plan arm motion with %s", success?"SUCESS":"FAILED");

  ROS_INFO("Ready to execute arm motion");
  success = (arm_group->asyncExecute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute plan arm motion with %s",  success?"SUCESS":"FAILED");

}
*/

void Worker::armMove(double x, double y, double z)
{
  bool success;
  arm_group->stop();
  arm_group->setGoalJointTolerance(0.003);

  arm_group->clearPoseTargets ();
  arm_group->setStartStateToCurrentState();
  sleep(0.5);
  arm_group->setPositionTarget(x,y,z,"gripper_link");

  ROS_INFO("Ready to plan arm motion ");
  success = (arm_group->plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Finish plan arm motion with %s", success?"SUCESS":"FAILED");

  ROS_INFO("Ready to execute arm motion");
  success = (arm_group->asyncExecute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute plan arm motion with %s",  success?"SUCESS":"FAILED");
}

void ControlPanel::onArmTargetMoveButtonClicked()
{
  emit do_armMove_(ui_.armTargetPoseX->text().toFloat(), ui_.armTargetPoseY->text().toFloat(), ui_.armTargetPoseZ->text().toFloat());
}

/*move gripper to target by PoseStamped     ready to remove
void ControlPanel::onArmTargetMoveButtonClicked()
{
  geometry_msgs::PoseStamped arm_target_pose_msg;
  geometry_msgs::Quaternion arm_target_quan;
  arm_target_pose_msg.header.stamp = ros::Time::now();

  arm_target_pose_msg.pose.position.x = ui_.armTargetPoseX->text().toFloat();
  arm_target_pose_msg.pose.position.y = ui_.armTargetPoseY->text().toFloat();
  arm_target_pose_msg.pose.position.z = ui_.armTargetPoseZ->text().toFloat();

  double target_oY = 0.0;
  target_oY = ui_.armTargetOritY->text().toFloat();
  target_oY *= (PI/180);
  arm_target_quan = tf::createQuaternionMsgFromYaw(target_oY);
  arm_target_pose_msg.pose.orientation = arm_target_quan;

  emit do_armMove_(arm_target_pose_msg);
}
*/

void Worker::armHome()
{
  if (arm_group == nullptr)
  {
    arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
  }
/*  if (PLANNING_GROUP != "arm_with_torso")
  {
    PLANNING_GROUP = "arm_with_torso";
    arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
  }
*/
  joint_values = {0.0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0};
  emit do_armMove_(joint_values);
}

void Worker::armStraight()
{
  if (arm_group == nullptr)
  {
    arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
  }
/*
  if (PLANNING_GROUP != "arm_with_torso")
  {
    PLANNING_GROUP = "arm_with_torso";
    arm_group.reset( new arm_group_controller(PLANNING_GROUP) );
  }
*/
  joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  emit do_armMove_(joint_values);
}

void Worker::openGripper()
{
  if (gripper_group == nullptr)
  {
    gripper_group.reset( new arm_group_controller("gripper") );
  }

  gripper_group->setStartStateToCurrentState();
  joint_values = {0.05,0.05};
  gripper_group->setJointValueTarget(joint_values);
  gripper_group->plan(move_plan);
  gripper_group->asyncExecute(move_plan);
//  emit do_armMove_(joint_values);
}

void Worker::closeGripper()
{
  if (gripper_group == nullptr)
  {
    gripper_group.reset( new arm_group_controller("gripper") );
  }

  gripper_group->setStartStateToCurrentState();
  joint_values = {0.0,0.0};
  gripper_group->setJointValueTarget(joint_values);
  gripper_group->plan(move_plan);
  gripper_group->asyncExecute(move_plan);
//  emit do_armMove_(joint_values);
}

void Worker::closeGripper(std::vector<double> pick_joint_values)
{
  if (gripper_group == nullptr)
  {
    gripper_group.reset( new arm_group_controller(PLANNING_GROUP) );
  }

  gripper_group->setStartStateToCurrentState();
  gripper_group->setJointValueTarget(pick_joint_values);
  gripper_group->plan(move_plan);
  gripper_group->asyncExecute(move_plan);
}

void Worker::getPose()
{
  geometry_msgs::PoseStamped gripper_pose_msg_;

  gripper_pose_msg_ = arm_group->getCurrentPose("gripper_link");
  emit do_update_target_poses_(gripper_pose_msg_);
}

void ControlPanel::updateTargetPoses(geometry_msgs::PoseStamped gripper_pose_msg_)
{
  ROS_INFO_STREAM( gripper_pose_msg_);

  ROS_INFO("ready to update");
  ui_.armTargetPoseX->setText(QString::number(gripper_pose_msg_.pose.position.x));
  ui_.armTargetPoseY->setText(QString::number(gripper_pose_msg_.pose.position.y));
  ui_.armTargetPoseZ->setText(QString::number(gripper_pose_msg_.pose.position.z));
  ROS_INFO("Updated!");
}

/*this block is armHome useless now   ready to remove
void Worker::armHome()
{
  std::string PLANNING_GROUP = "arm_with_torso";
  moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;
  const robot_state::JointModelGroup* joint_model_group =
      arm_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::core::RobotStatePtr current_state = arm_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//  double arm_home_poses [] = {0.0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0};
  std::vector<double> arm_home_poses = {0.0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0};

  for (int i=0; i<arm_home_poses.size(); i++)
  {
    joint_group_positions[i] = arm_home_poses[i];
  }

  arm_group.setJointValueTarget(joint_group_positions);

  ROS_INFO("Ready to plan armHome ");
  bool success = (arm_group.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Finish plan armHome with %s", success?"SUCESS":"FAILED");

  ROS_INFO("Ready to execute armHome");
  success = (arm_group.asyncExecute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute plan armHome with %s",  success?"SUCESS":"FAILED");

}*/

void ControlPanel::addButtonClicked()
{
  ui_.cusWidget->show();
}

void ControlPanel::cusCancelButtonClicked()
{
  ui_.cusWidget->setVisible(false);
}

void ControlPanel::createNewPoseButton()
{
//  custom_pose_button[num_cutom_button] = new QFrame;    ready to remove
  ui_.cusWidget->setVisible(false);

  QFrame *custom_frame = new QFrame;
  QString name = ui_.cusButtonNameLine->text();

  if (name.isEmpty())
  {
    custom_frame = createNewPoseButton(QString("Custom %1").arg(++num_cutom_button));
  }
  else
  {
    custom_frame = createNewPoseButton(name);
  }
  custom_frame->setParent(ui_.customPoseButtons);
  custom_frame->show();

//  num_cutom_button++;   ready to remove
  custom_button_list.append(custom_frame);
  arrangeCusButtons();
}

void ControlPanel::arrangeCusButtons()
{
  int num = custom_button_list.length();
  for (int i=0; i<num; i++)
  {
    if (i % 2)
    {
      custom_button_list[i]->setGeometry(110, 10+(i/2*40), 90, 25);
    }
    else
    {
      custom_button_list[i]->setGeometry(10, 10+(i/2*40), 90, 25);
    }
  }
}

QFrame* ControlPanel::createNewPoseButton(QString name)
{
  QFrame *frame = new QFrame;
  QPushButton *button = new QPushButton(name, frame);
  QPushButton *del_button = new QPushButton("X", frame);

  button->setFixedSize(80,25);
  button->show();

  QFont del_font;
  del_font.setPointSize(6);
  del_button->setFont(del_font);
  del_button->setFixedSize(10,10);
  connect(button, SIGNAL(clicked()), this, SLOT(cusArmMove()));
  connect(del_button, SIGNAL(clicked()), this, SLOT(onCusXButtonClicked()));
  connect(del_button, SIGNAL(destroyed()), button, SLOT(deleteLater()));
  connect(button, SIGNAL(destroyed()), this, SLOT(arrangeCusButtons()));
  del_button->show();
  return frame;
}

void ControlPanel::cusArmMove()
{
  QStringList values_temp = ui_.cusButtonContentLine->text().split(",");
  std::vector<double> joint_values;

  for (int i=0; i<values_temp.length(); i++)
  {
    joint_values.push_back(values_temp.at(i).toDouble());
  }

//  joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  emit do_armMove_(joint_values);
}

void ControlPanel::onCusXButtonClicked()
{
  QFrame* temp = qobject_cast<QFrame *>(sender()->parent());
  custom_button_list.removeAt(custom_button_list.indexOf(temp));
  sender()->deleteLater();
}

void Worker::armStop()
{
  if (arm_group != nullptr)
  {
    arm_group->stop();
    ROS_INFO("Stop moving");
  }

  if (gripper_group != nullptr)
  {
    gripper_group->stop();
    ROS_INFO("Stop moving");
  }
}

//below is for scan function
/*  ready to remove
void ControlPanel::onPointHeadCheckBoxClicked(bool checked)
{
  ui_.pointHeadcomboBox->setEnabled(checked);
  if (checked)
  {
    is_point_head = 1;
    pH_x = 0;
    pH_y = 0;
    pH_z = 0;
//    setPointHead(pH_x, pH_y, pH_z);
    setPointHead(mH_h, mH_v);
    ROS_INFO("Piont head mode selected!");
  }
  else
  {
    set_head_mutex_.lock();
    is_point_head = 0;
    mH_h = 0;
    mH_v = 0;
    setMoveHead(mH_h, mH_v);
    ROS_INFO("Point head mode unselected!");
  }
}
*/

void ControlPanel::preSetPointHead()
{
  p_point_head_client.reset(new point_head_client("head_controller/point_head"));
  int iterations = 0, max_iterations = 3;
  while( !p_point_head_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_client server to come up");
    ++iterations;
  }

//  below two lines are needed by other robots, because Fetch robot cannot modify target frame. MOre info, see: http://docs.fetchrobotics.com/api_overview.html
//  point_head_selected_frame = ui_.pointHeadcomboBox->currentText();
//  point_head_goal.target.header.frame_id = ui_.pointHeadcomboBox->currentText().toUtf8().constData();
  point_head_goal.target.header.stamp = ros::Time::now();
  point_head_goal.min_duration = ros::Duration(0.2);
  point_head_goal.max_velocity = 1.35;
}

//void ControlPanel::setPointHead(double pH_x, double pH_y, double pH_z)    ready to remove
void ControlPanel::setPointHead(double pH_h, double pH_v)
{
  if (!p_point_head_client)
  {
    preSetPointHead();
  }

  point_head_goal.target.header.frame_id = ui_.pointHeadcomboBox->currentText().toUtf8().constData();


  pH_x = 2 * qSin(pH_v) * qCos(pH_h);
  pH_y = 2 * qSin(pH_v) * qSin(pH_h);
  pH_z = 2 * qCos(pH_v);

  point_head_goal.target.point.x = pH_x;
  point_head_goal.target.point.y = pH_y;
  point_head_goal.target.point.z = pH_z;

  emit do_point_head_();
}

/*  ready to remove
void ControlPanel::preSetMoveHead()
{
  p_move_head_client.reset(new move_head_client("head_controller/follow_joint_trajectory"));

  int iterations = 0, max_iterations = 3;
  while( !p_move_head_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the move_head_client server to come up");
    ++iterations;
  }

  move_head_goal.goal_time_tolerance = ros::Duration(0.6);

//  move_head_goal.trajectory.header.stamp = ros::Time::now();
  move_head_goal.trajectory.joint_names.push_back("head_pan_joint");
  move_head_goal.trajectory.joint_names.push_back("head_tilt_joint");

  move_head_goal.trajectory.points.resize(1);
  move_head_goal.trajectory.points[0].positions.resize(2);
  move_head_goal.trajectory.points[0].velocities.push_back(0.3);
}

void ControlPanel::setMoveHead(double mH_h, double mH_v)
{

  if (!p_move_head_client)
  {
    preSetMoveHead();
  }

  move_head_goal.goal_time_tolerance = ros::Duration(0.5);
  move_head_goal.trajectory.header.stamp = ros::Time::now();

  move_head_goal.trajectory.points[0].positions[0] = mH_h;
  move_head_goal.trajectory.points[0].positions[1] = mH_v;

  emit do_move_head_();
  set_head_mutex_.unlock();
  ROS_INFO("Move head set!");
}

void Worker::moveHead()
{
  p_move_head_client->sendGoal(move_head_goal);
  //p_move_head_client->sendGoalAndWait(move_head_goal);
}
*/

void Worker::pointHead()
{
  p_point_head_client->sendGoal(point_head_goal);
}

void ControlPanel::onUpHeadButtonClicked()
{
//  if (is_point_head)
//  {
    pH_v -= 0.05;
    setPointHead(pH_h, pH_v);
//  }
/*  ready to remove
    else
    {
      set_head_mutex_.lock();
      if (mH_v > -0.76)
      {
        mH_v -= 0.006;
        setMoveHead(mH_h, mH_v);
      }
      else
      {
        ROS_INFO("Out of limit!");
      }
    }
*/
/*
  if (is_point_head)
  {
    pH_x += 0.03;
    pH_z += 0.03;
    setPointHead(pH_x, pH_y, pH_z);
  }
  else
  {
    set_head_mutex_.lock();
    if (mH_v > -0.76)
    {
      mH_v -= 0.006;
      setMoveHead(mH_h, mH_v);
    }
    else
    {
      ROS_INFO("Out of limit!");
    }
  }
*/
}

void ControlPanel::onDownHeadButtonClicked()
{
//  if (is_point_head)
//  {
    pH_v += 0.05;
    setPointHead(pH_h, pH_v);

//  }
/*  ready to remove
    else
    {
      set_head_mutex_.lock();
      if (mH_v > -0.76)
      {
        mH_v += 0.006;
        setMoveHead(mH_h, mH_v);
      }
      else
      {
        ROS_INFO("Out of limit!");
      }
    }
*/

/*
  if (is_point_head)
  {
    pH_x -= 0.03;
    pH_z -= 0.03;
    setPointHead(pH_x, pH_y, pH_z);
  }
  else
  {
    set_head_mutex_.lock();

    if (mH_v < 1.46)
    {
      mH_v += 0.006;
      setMoveHead(mH_h, mH_v);
    }
    else
    {
      ROS_INFO("Out of limit!");
    }
  }
*/
}

void ControlPanel::onLeftHeadButtonClicked()
{
//  if (is_point_head)
//  {
    pH_h += 0.05;
    setPointHead(pH_h, pH_v);
//  }
/*    ready to remove
    else
    {
      set_head_mutex_.lock();
      if (mH_h < 1.56)
      {
        mH_h += 0.012;
        setMoveHead(mH_h, mH_v);
      }
      else
      {
        ROS_INFO("Out of limit!");
      }
    }
*/

/*
  if (is_point_head)
  {
    pH_y += 0.03;
    setPointHead(pH_x, pH_y, pH_z);
  }
  else
  {
    set_head_mutex_.lock();

    if (mH_h < 1.56)
    {
      mH_h += 0.012;
      setMoveHead(mH_h, mH_v);
    }
    else
    {
      ROS_INFO("Out of limit!");
    }
  }*/

}

void ControlPanel::onRightHeadButtonClicked()
{
//  if (is_point_head)
//  {
    pH_h -= 0.05;
    setPointHead(pH_h, pH_v);
//  }
/*    ready to remove
    else
    {
      set_head_mutex_.lock();
      if (mH_h > -1.56)
      {
        mH_h -= 0.012;
        setMoveHead(mH_h, mH_v);
      }
      else
      {
        ROS_INFO("Out of limit!");
      }
    }
*/
/*
  if (is_point_head)
  {
    pH_y -= 0.03;
    setPointHead(pH_x, pH_y, pH_z);
  }
  else
  {
    set_head_mutex_.lock();

    if (mH_h > -1.56)
    {
      mH_h -= 0.012;
      setMoveHead(mH_h, mH_v);
    }
    else
    {
      ROS_INFO("Out of limit!");
    }
  }
*/
}

void ControlPanel::headHome()
{
/*      ready to remove
  if (!p_move_head_client && !p_point_head_client)
  {
    ROS_INFO("Not start moving, please click any button to active");
    return;
  }
  else
  {
    if (is_point_head)
    {   */
      pH_x = 0;
      pH_y = 0;
      pH_z = 0;
    //  setPointHead(pH_x, pH_y, pH_z);
    ROS_INFO("Heading home!");
    setPointHead(0, 1.57);
    ROS_INFO("Homed");
/*    }
    else
    {
      set_head_mutex_.lock();

      mH_h = 0;
      mH_v = 0;
      setMoveHead(mH_h, mH_v);
    }
  }
  */
}

void ControlPanel::oncameraButtonCLicked(bool checked)
{
  if(!checked)
  {
    subscriber_.shutdown();
    ui_.cameraButton->setText("Camera");
  }
  else
  {
    ui_.imageFrame->setImage(QImage());
//    QString topic = "/head_camera/rgb/image_raw";
    image_transport::ImageTransport it(getNodeHandle());

    try{
  //    subscriber_ = it.subscribe("/head_camera/rgb/image_raw", 1, &ControlPanel::callbackImage, this);
      subscriber_ = it.subscribe(scan_topic.toUtf8().constData(), 1, &ControlPanel::callbackImage, this);
    }catch(image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
    ui_.cameraButton->setText("Uncamera");
  }
}

void ControlPanel::onZoom(bool checked)
{
  QSize image_size = ui_.imageFrame->getImage().size();
  if (!checked)
  {
    ui_.zoomButton->setIcon(QIcon::fromTheme("zoom-in"));
    if (ui_.imageFrame->getImage().isNull())
    {
      return;
    }
    //ui_.imageFrame->setInnerFrameFixedSize(ui_.imageFrame->getImage().size());
    ui_.imageFrame->setInnerFrameFixedSize(image_size/2);
  }
  else
  {
    ui_.imageFrame->setInnerFrameMinimumSize(QSize(80,45));
  //  ui_.imageFrame->setMaximumSize(QSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX));
    ui_.imageFrame->setMaximumSize(image_size);
    widget_->setMinimumSize(QSize(80,45));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX));
    ui_.zoomButton->setIcon(QIcon::fromTheme("zoom-out"));
  }
}

void ControlPanel::onDynamicRange(bool checked)
{
  ui_.maxRangespinBox->setEnabled(!checked);
}

void ControlPanel::saveImage()
{
  QImage img = ui_.imageFrame->getImageCopy();

  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty())
  {
    return;
  }
  img.save(file_name);
}

/*  ready to remove
void ControlPanel::updateNumGridlines()
{
  num_gridlines_ = ui_.gridlinesNumspinBox->value();
}
*/

void ControlPanel::onRotateLeft()
{
  int m = rotate_state_ - 1;
  if (m<0)
  {
    m = ROTATE_STATE_COUNT - 1;
  }

  rotate_state_ = static_cast<RotateState>(m);
  syncRotateLabel();
}

void ControlPanel::onRotateRight()
{
  rotate_state_ = static_cast<RotateState>((rotate_state_ + 1) % ROTATE_STATE_COUNT);
  syncRotateLabel();
}

void ControlPanel::syncRotateLabel()
{
  switch(rotate_state_)
  {
    default:
    case ROTATE_0:   ui_.rotateLabel->setText("0°"); break;
    case ROTATE_90:  ui_.rotateLabel->setText("90°"); break;
    case ROTATE_180: ui_.rotateLabel->setText("180°"); break;
    case ROTATE_270: ui_.rotateLabel->setText("270°"); break;
  }
}

void ControlPanel::invertPixels(int x, int y)
{
  // Could do 255-conversion_mat_.at<cv::Vec3b>(cv::Point(x,y))[i], but that doesn't work well on gray
  cv::Vec3b & pixel = conversion_mat_.at<cv::Vec3b>(cv::Point(x, y));
  if (pixel[0] + pixel[1] + pixel[2] > 3 * 127)
    pixel = cv::Vec3b(0,0,0);
  else
    pixel = cv::Vec3b(255,255,255);
}

/*ready to remove
QList<int> ControlPanel::getGridIndices(int size) const
{
  QList<int> indices;

  //the spacing between adjacent grid lines
  float grid_width = 1.0f * size / (num_gridlines_ + 1);

  //selet grid line(s) closest to the center
  float index;
  if (num_gridlines_ % 2) //odd
  {
    indices.append(size / 2);
    //make the center line 2px wide in case of an even resolution
    if (size % 2 == 0) //even
      indices.append(size / 2 - 1);
    index = 1.0f * (size - 1) / 2;
  }
  else //even
  {
    index = grid_width * (num_gridlines_ / 2);
    //one grid line before the center
    indices.append(round(index));
    //one grid line after the center
    indices.append(size - 1 - round(index));
  }

  //add additional grid lines from the center to the border of the image
  int lines = (num_gridlines_ - 1) / 2;
  while (lines > 0)
  {
    index -= grid_width;
    indices.append(round(index));
    indices.append(size - 1 - round(index));
    lines--;
  }

  return indices;
}

void ControlPanel::overlayGrid()
{
  QList<int> columns = getGridIndices(conversion_mat_.cols);
  for (QList<int>::const_iterator x = columns.begin(); x != columns.end(); ++x)
  {
    for (int y = 0; y < conversion_mat_.rows; ++y)
      invertPixels(*x, y);
  }

  QList<int> rows = getGridIndices(conversion_mat_.rows);
  for (QList<int>::const_iterator y = rows.begin(); y != rows.end(); ++y)
  {
    for (int x = 0; x < conversion_mat_.cols; ++x)
      invertPixels(x, *y);
  }
}
*/

void ControlPanel::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    //convert ROS image to opencv
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;

//    if (num_gridlines_ > 0)   ready to remove
//      overlayGrid();    ready to remove
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      //if reached here it means taht no conversion make sense, so let us try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        //assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      }else if (msg->encoding == "8UC1") {
        //convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      }else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
          // scale / quantify
        double min = 0;
        double max = ui_.maxRangespinBox->value();
        if (msg->encoding == "16UC1")
          max *=1000;
        if (ui_.dynamicRangecheckBox->isChecked())
        {
          //dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max)
          {
            //completely homogenngous images are displayed in gray
            min = 0;
            max = 2;
          }
        }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      }
      else
      {
        qWarning("ControlPanel.callbackImage() could not convert image from '%s' to 'RGB8' (%s)", msg->encoding.c_str(), e.what());
        ui_.imageFrame->setImage(QImage());
        return;
      }
    }

      catch(cv_bridge::Exception& e)
      {
        qWarning("ControlPanel.callbackImage() could not convert image from '%s' to 'rgb8'", msg->encoding.c_str(), e.what());
        ui_.imageFrame->setImage(QImage());
        return;
      }
    }

    //Handle rotation
    switch (rotate_state_)
    {
      case ROTATE_90:
      {
        cv::Mat tmp;
        cv::transpose(conversion_mat_, tmp);
        cv::flip(tmp, conversion_mat_, 1);
        break;
      }
      case ROTATE_180:
      {
        cv::Mat tmp;
        cv::flip(conversion_mat_, tmp, -1);
        conversion_mat_ = tmp;
        break;
      }
      case ROTATE_270:
      {
        cv::Mat tmp;
        cv::transpose(conversion_mat_, tmp);
        cv::flip(tmp, conversion_mat_, 0);
        break;
      }
      default:
        break;
    }

    //image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    ui_.imageFrame->setImage(image);

/*
    if (!ui_.zoomButton->isEnabled())
    {
      ui_.zoomButton->setEnabled(true);
    }
*/
    onZoom(ui_.zoomButton->isChecked());

}

/*    this part is for saving settings, it was created by rqt as default
void ControlPanel::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void ControlPanel::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

//below is for pick and place 14/02/2018
//template is from https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/perception_pipeline/src/cylinder_segment.cpp#L59
/* ready to remove
void ControlPanel::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void ControlPanel::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);

}
*/

//void Worker::pick(moveit::planning_interface::MoveGroupInterface& move_group)
void Worker::pick()
{
  if (gripper_group == nullptr)
  {
    gripper_group.reset( new arm_group_controller("gripper") );
  }

  for (int i=0; i<gripper_group->getJointNames().size(); i++)
  {
    QString a = gripper_group->getJointNames().at(i).c_str();
    ROS_INFO("Joints: %s", a.toUtf8().constData());
  }

  for (int i=0; i<gripper_group->getLinkNames().size(); i++)
  {
    QString a = gripper_group->getLinkNames().at(i).c_str();
    ROS_INFO("Joints: %s", a.toUtf8().constData());
  }

  gripper_group->setEndEffectorLink("wrist_roll_link");
  gripper_group->setEndEffector ("gripper");

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "base_link";
  grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  grasps[0].grasp_pose.pose.position.x = box_params->obj_pose.pose.position.x - 0.05;
  grasps[0].grasp_pose.pose.position.y = box_params->obj_pose.pose.position.y;
  grasps[0].grasp_pose.pose.position.z = box_params->obj_pose.pose.position.z;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  grasps[0].pre_grasp_posture.header.frame_id = "gripper_link";
  grasps[0].pre_grasp_posture.joint_names.resize(2);
  grasps[0].pre_grasp_posture.joint_names[0] = "l_gripper_finger_joint";
  grasps[0].pre_grasp_posture.joint_names[1] = "r_gripper_finger_joint";

  grasps[0].pre_grasp_posture.points.resize(1);
  grasps[0].pre_grasp_posture.points[0].positions.resize(2);
  grasps[0].pre_grasp_posture.points[0].positions[0] = 0.04;
  grasps[0].pre_grasp_posture.points[0].positions[1] = 0.04;
  grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  grasps[0].grasp_posture.header.frame_id = "gripper_link";
  grasps[0].grasp_posture.joint_names.resize(2);
  grasps[0].grasp_posture.joint_names[0] = "l_gripper_finger_joint";
  grasps[0].grasp_posture.joint_names[1] = "r_gripper_finger_joint";

  grasps[0].grasp_posture.points.resize(1);
  grasps[0].grasp_posture.points[0].positions.resize(2);
  grasps[0].grasp_posture.points[0].positions[0] = 0.00;
  grasps[0].grasp_posture.points[0].positions[1] = 0.00;
  grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

//  openGripper(grasps[0].pre_grasp_posture);
//  closedGripper(grasps[0].grasp_posture);

  gripper_group->setSupportSurfaceName("table");
  gripper_group->pick("cube", grasps);
}
/*
void Worker::place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";

  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("table");
  group.place("cube", place_location);

}
*/
void Worker::boxSegment()
{
  ROS_INFO("Strat detecting");

  detect_sub_ = nh.subscribe("/head_camera/depth_registered/points", 50, &Worker::cloudCB, this);

}

void Worker::removeBox()
{
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  collision_object.operation = collision_object.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object);
  points_not_found = true;
  ROS_INFO("Objct removed!");
}
/*
void ControlPanel::addBox(QString obj_id, int index)
{
  // Define a collision object ROS message.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);
//  moveit_msgs::CollisionObject collision_object;
//  tf::TransformListener listener;
//  geometry_msgs::PoseStamped box_pose;
  tf::Stamped<tf::Pose> pre_pose(pre_pose, ros::Time(0), "head_camera_rgb_optical_frame");

//  tf::Stamped<tf::Pose>::Stamped(pre_pose, ros::Time::now(), "head_camera_rgb_optical_frame");
  tf::poseStampedMsgToTF(box_params->pre_pose, pre_pose);
  tf::Stamped<tf::Pose> post_pose(post_pose, ros::Time(0), "base_link");
//  tf::Stamped<tf::Pose>::Stamped(post_pose, ros::Time::now(), "base_link");
//  box_pose.header.frame_id = "base_link";
//  box_pose.header.stamp = ros::Time::now();
//  std::string target_frame = "/base_link";
  ros::Time t = box_params->pre_pose.header.stamp;
//  ros::Time t = ros::Time::now();
//  ros::Time t = ros::Time(0);

    try{
      listener.waitForTransform("/base_link", "/head_camera_rgb_optical_frame", t, ros::Duration(3.0));
      listener.transformPose("/base_link", pre_pose, post_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    sleep(0.1);


//  collision_object.header.frame_id = "head_camera_rgb_optical_frame";

  //collision_object.header.frame_id = "base_link";
  //collision_object.id = "box";
  collision_objects[index].header.frame_id = "base_link";
  collision_objects[index].id = obj_id.toUtf8().constData();

  // Define a box which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  //setting for dimensions, box_params is pointer to the struct AddBoxParams which has not been built. when built, remove this comment
  primitive.dimensions[0] = box_params->box_x;
  primitive.dimensions[1] = box_params->box_y;
  primitive.dimensions[2] = box_params->box_z;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::PoseStamped box_pose;
  tf::poseStampedTFToMsg(post_pose, box_pose);
  //set the position of box
//  box_pose.position.x = box_params->center_pt[0];
//  box_pose.position.y = box_params->center_pt[1];
  //box_pose.position.z = box_params->center_pt[2];
  //set the orientation of box
//  box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (box_params->direction_vec[0], box_params->direction_vec[1], box_params->direction_vec[2]);

  // Add box as collision object
//  collision_object.primitives.push_back(primitive);
  collision_objects[index].primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose.pose);
//  collision_object.primitive_poses.push_back(box_pose.pose);
//  collision_object.operation = collision_object.ADD;
//  planning_scene_interface.applyCollisionObject(collision_object);

  collision_objects[index].primitive_poses.push_back(box_pose.pose);
  collision_objects[index].operation = collision_objects[index].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);

}
*/
void Worker::addBox(QString obj_id, int index)
{
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.resize(2); moved to cloudCB

  collision_objects[index].header.frame_id = "base_link";
  collision_objects[index].id = obj_id.toUtf8().constData();

  // Define a box which will be added to the world.
  collision_objects[index].primitives.resize(1);
  collision_objects[index].primitives[0].type = collision_objects[index].primitives[0].BOX;
  collision_objects[index].primitives[0].dimensions.resize(3);
  collision_objects[index].primitives[0].dimensions[0] = box_params->box_x;
  collision_objects[index].primitives[0].dimensions[1] = box_params->box_y;
  collision_objects[index].primitives[0].dimensions[2] = box_params->box_z;

  collision_objects[index].primitive_poses.resize(1);

  if (index == 0)
  {
    collision_objects[index].primitive_poses[0] = box_params->obj_pose.pose;
    collision_objects[index].primitive_poses[0].position.x = box_params->obj_pose.pose.position.x + (box_params->height / 2);
    collision_objects[index].primitive_poses[0].position.z = box_params->height / 2;
    collision_objects[index].primitives[0].dimensions[0] = box_params->height;
    collision_objects[index].primitives[0].dimensions[1] = box_params->height;
    collision_objects[index].primitives[0].dimensions[2] = box_params->height;
  }
  else
  {
    collision_objects[index].primitive_poses[0] = box_params->obj_pose.pose;
  }

  collision_objects[index].operation = collision_objects[index].ADD;
//  planning_scene_interface.applyCollisionObjects(collision_objects);
}

//in this case, since we use a cube, height is enough
//extractLocationHeight is ready to remove
/* ready to remove
void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  double max_angle_y = 0.0;
  double min_angle_y = std::numeric_limits<double>::infinity();

  double lowest_point[3];
  double highest_point[3];
  for (auto const point : cloud->points)
  {
    if (atan2(point.z, point.y) < min_angle_y)
    {
      min_angle_y = atan2(point.z, point.y);
      lowest_point[0] = point.x;
      lowest_point[1] = point.y;
      lowest_point[2] = point.z;
    }
    else if (atan2(point.z, point.y) > max_angle_y)
    {
      max_angle_y = atan2(point.z, point.y);
      highest_point[0] = point.x;
      highest_point[1] = point.y;
      highest_point[2] = point.z;
    }
  }
  ROS_INFO("%f",highest_point[0]);
  ROS_INFO("%f",highest_point[1]);
  ROS_INFO("%f",highest_point[2]);
  ROS_INFO("%f",lowest_point[0]);
  ROS_INFO("%f",lowest_point[1]);
  ROS_INFO("%f",lowest_point[2]);
  box_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2.0;
  box_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2.0;
  box_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2.0;

  box_params->box_z =
      sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
           pow((lowest_point[2] - highest_point[2]), 2));

//since it si a cube
  box_params->box_x = box_params->box_z;
  box_params->box_y = box_params->box_z;
   ROS_INFO("height is: %f", box_params->box_z);
}
*/
void Worker::extractBoxLengWidth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pre_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pre_cloud->header.frame_id = "/head_camera_rgb_optical_frame";
  try
  {
    pcl_ros::transformPointCloud("/base_link", *pre_cloud, *cloud, pcl_trans_listener);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  double x_min = 10000000.0;
  double x_max = -10000000.0;
  double width = 0;

  double y_min = 10000000.0;
  double y_max = -10000000.0;

  double z_min = 10000000.0;
  double z_max = -10000000.0;

  for (auto const point : cloud->points)
  {
    if (point.x < x_min)
    {
      x_min = point.x;
    }
    if (point.x > x_max)
    {
      x_max = point.x;
    }

    if (point.y < y_min)
    {
      y_min = point.y;
    }
    if (point.y > y_max)
    {
      y_max = point.y;
    }

    if (point.z < z_min)
    {
      z_min = point.z;
    }
    if (point.z > z_max)
    {
      z_max = point.z;
    }
  }

  width = x_max - x_min;

  box_params->obj_pose.header.frame_id = "base_link";
  box_params->obj_pose.header.stamp = ros::Time::now();
  box_params->obj_pose.pose.position.x = (x_min + x_max) / 2.0;
  box_params->obj_pose.pose.position.y = (y_min + y_max) / 2.0;
  box_params->obj_pose.pose.position.z = (z_min + z_max) / 2.0;

  box_params->center_pt[0] = (x_min + x_max) / 2.0;
  box_params->center_pt[1] = (y_min + y_max) / 2.0;
  box_params->center_pt[2] = (z_min + z_max) / 2.0;

  box_params->box_x = width;
  box_params->box_y = y_max - y_min;
  box_params->box_z = z_max - z_min;

  box_params->height = z_max;

//  ROS_INFO("Width = %f", width);
}

//passThroughFilter ready to remove
/*
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  // min and max values in z axis to keep
  pass.setFilterLimits(-0.15, 0.15);
  pass.filter(*cloud);
}

//cloud PointCloud
//cloud_normals The point normals once computer will be stored in this.
//computeNormals ready to remove

void ControlPanel::computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

//cloud_normals Point normals
//inliers_plane - Indices whose normals need to be extracted
//extractNormals  ready to remove

void ControlPanel::extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

//inliers_plane - Indices representing the plane。
//removePlaneSurface is ready to remove

void ControlPanel::removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
{
  // create a SAC segmenter without using normals
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  segmentor.setMaxIterations(1000);
  segmentor.setDistanceThreshold(0.01);
  segmentor.setInputCloud(cloud);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}

//coefficients_box - Box parameters used to define an infinite box will be stored here.
//extractBox ready to remove
/*
void ControlPanel::extractBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_box,
                     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  // Create the segmentation object for box segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_box(new pcl::PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(10000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(0.05);
  // min max values of radius in meters to consider  not in use
  //segmentor.setRadiusLimits(0, 1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Obtain the box inliers and coefficients
  segmentor.segment(*inliers_box, *coefficients_box);

  // Extract the box inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_box);
  extract.setNegative(false);
  extract.filter(*cloud);
}
*/

//need to tunning the size of cube 22/02
void Worker::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("Called");
  collision_objects.resize(2);
  // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for most of the processing.
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  ROS_INFO("Created cloud");
  pcl::fromROSMsg(*input, *cloud);
  box_params = new AddBoxParams;

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.003f, 0.003f, 0.003f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (500);
  seg.setDistanceThreshold (0.01);


  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.15 * nr_points)//was 0.07
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (125000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    extractBoxLengWidth(cloud_cluster);
  //  ros::WallDuration(1.0).sleep();
  //  ROS_INFO("Width: %f", box_params->box_y);

    if (box_params->box_y>0.5)//box_params->box_y is because transform from camera link to base link
    {
      ROS_INFO("This is table, width: %f", box_params->box_y);
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "table.pcd";
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
      addBox("table", 0);
      ROS_INFO("\n");
      continue;
    }

    if ((box_params->box_y<0.07) && (box_params->box_y>0.055))//was 0.055
    {
      ROS_INFO("This is cube, width: %f", box_params->box_y);
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cube.pcd";
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
      addBox("cube", 1);
      ROS_INFO("\n");
      continue;
    }

    ROS_INFO("Disscarded");
//    addBox();
//    ROS_INFO("Added");
/*
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
    */
  }
  planning_scene_interface.applyCollisionObjects(collision_objects);

/*
  box_params = new AddBoxParams;
  // Using passthough filter to get region of interest. A passthrough filter just eliminates the point cloud values
  // which do not lie in the user specified range.
  //not implement now
  passThroughFilter(cloud);

  // Declare normals and call function to compute point normals.
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ROS_INFO("Created cloud_normals");
  computeNormals(cloud, cloud_normals);
  ROS_INFO("Computed");

  // inliers_plane will hold the indices of the point cloud that correspond to a plane.
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
  removePlaneSurface(cloud, inliers_plane);
  ROS_INFO("Removed");

  // We had calculated the point normals in a previous call to computeNormals,
  // now we will be extracting the normals that correspond to the plane on which cylinder lies.
  // It will be used to extract the cylinder.
  extractNormals(cloud_normals, inliers_plane);
  ROS_INFO("Normaled");

   // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length.
   pcl::ModelCoefficients::Ptr coefficients_box(new pcl::ModelCoefficients);
   //Extract the cylinder using SACSegmentation.
   extractBox(cloud, coefficients_box, cloud_normals);
   ROS_INFO("Extracted");

   if (cloud->points.empty())
   {
     ROS_INFO("Can't find the box component.");
     return;
   }

   if (points_not_found)
   {
     ROS_INFO("Adding para");
     for (int i=0;i<coefficients_box->values.size();++i)
     {
       ROS_INFO("%f", coefficients_box->values[i]);
     }

     ROS_INFO("Extracting height");
     extractLocationHeight(cloud);
     ROS_INFO("Height extracted");
     addBox();
     ROS_INFO("Added");

     points_not_found = false;
   }
   */
   detect_sub_.shutdown();
   ROS_INFO("Detect finish");
}

void ControlPanel::shutdownPlugin()
{
  workerThread.quit();
  workerThread.wait();
  pose_goal_.shutdown();
  forward_goal_.shutdown();
  cancel_goal_.shutdown();
  subscriber_.shutdown();
}

Worker::~Worker()
{
  //just for in case of error occuring in run rqt
    detect_sub_.shutdown();
    delete box_params;
}

} // namespace


PLUGINLIB_EXPORT_CLASS(rqt_control_panel::ControlPanel, rqt_gui_cpp::Plugin)
