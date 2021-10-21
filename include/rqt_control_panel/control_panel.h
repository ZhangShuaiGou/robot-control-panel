#ifndef rqt_control_panel__ControlPanel_H
#define rqt_control_panel__ControlPanel_H

#include <rqt_gui_cpp/plugin.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/LinearMath/Transform.h"
#include "tf/tf.h"
#include <ui_control_panel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <simple_grasping/object_support_segmentation.h>
#include <simple_grasping/shape_grasp_planner.h>
#include <simple_grasping/shape_extraction.h>
#include <grasping_msgs/FindGraspableObjectsAction.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core/core.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/shared_ptr.hpp>

#include <rqt_control_panel/my_rviz.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>


#include <QImage>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QThread>
#include <vector>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QtMath>
#include <QFont>

#define PI 3.141592653589793238462643383279502884L  //for convert degree to radian

namespace rqt_control_panel {

//those parameters are used in Worker, I dont find a better way to deal with them than put them in global static
static bool forward_flag = false;
static QString forward_topic;
static QString go_topic;
static QString arm_topic;
static QString scan_topic;
static ros::NodeHandle nh;
static ros::Publisher forward_goal_;

static control_msgs::PointHeadGoal point_head_goal;
//static control_msgs::FollowJointTrajectoryGoal move_head_goal;


//typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move_head_client;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_client;
//typedef boost::shared_ptr< move_head_client>  move_head_client_Ptr;
typedef boost::shared_ptr< point_head_client>  point_head_client_Ptr;


//static move_head_client_Ptr p_move_head_client;
static point_head_client_Ptr p_point_head_client;


static std::string PLANNING_GROUP;  //for moveit used in Worker
typedef moveit::planning_interface::MoveGroupInterface arm_group_controller;
typedef boost::shared_ptr< arm_group_controller>  arm_group_controller_Ptr;


class ControlPanel
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
  QThread workerThread;

public:
  ControlPanel();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  ros::Publisher pose_goal_;
  ros::Publisher cancel_goal_;
//  virtual void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
//  virtual void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
//  virtual void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane);
//  virtual void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane);
//  virtual void extractBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_box, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);


signals:
    void do_forward_(float, float, float);
    void do_arm_(QString);
    void do_point_head_();
    void do_move_head_();
  //  void do_armMove_(geometry_msgs::PoseStamped);
    void do_armMove_(double,double,double);
    void do_armMove_(std::vector<double>);

protected slots:

  virtual void refreshFrameList();
  virtual void onRvizFrameChanged(QString fixed_frame_);
  virtual void ongoButtonClicked();
  virtual void onforwardButtonClicked();
  virtual void onstopButtonClicked();
  virtual void onForwardstopButtonClicked();
  virtual void oncameraButtonCLicked(bool checked);
  virtual void onZoom(bool checked);
  virtual void onDynamicRange(bool checked);
  virtual void saveImage();
//  virtual void updateNumGridlines();    ready to remove
  virtual void updateOnBase1();
  virtual void updateOnBase2();
  virtual void updateOnArm();
  virtual void updateOnScan();
  virtual void onBaseTopic(bool checked);
  virtual void onBaseTopic1(bool checked);
  virtual void onBaseTopic2(bool checked);
  virtual void onArmTopic(bool checked);
  virtual void onScanTopic(bool checked);
  virtual void topics_resetButtonClicked();
  virtual void topics_confirmButtonClicked();

  virtual void preSetPointHead();
//  virtual void preSetMoveHead();    ready to remove

  virtual void onRotateLeft();
  virtual void onRotateRight();

  virtual void onArmTargetMoveButtonClicked();
  virtual void updateTargetPoses(geometry_msgs::PoseStamped);

//  virtual void onPointHeadCheckBoxClicked(bool checked);      ready to remove
  virtual void onUpHeadButtonClicked();
  virtual void onDownHeadButtonClicked();
  virtual void onLeftHeadButtonClicked();
  virtual void onRightHeadButtonClicked();
  virtual void headHome();


  virtual void createNewPoseButton();
  virtual QFrame* createNewPoseButton(QString);
  virtual void onCusXButtonClicked();
  virtual void arrangeCusButtons();
  virtual void addButtonClicked();
  virtual void cusCancelButtonClicked();
  virtual void cusArmMove();

protected:
  virtual void invertPixels(int x, int y);
//  QList<int> getGridIndices(int size) const;    ready to remove
//  virtual void overlayGrid();   ready to remove
  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
//  virtual QSet<QString> getTopics(const QSet<QString>& message_types);
  virtual QSet<QString> getTopics(QString message_types);
  virtual void updateTopicList(QLineEdit* lineEdit, QComboBox* checkBox);
  virtual void updateFrameList(QComboBox* checkBox);


//  virtual void setPointHead(double pH_x, double pH_y, double pH_z);
  virtual void setPointHead(double pH_h, double pH_v);
//  virtual void setMoveHead(double mH_v, double mH_h);     ready to remove





private:
  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();
  void stopforward();
  Ui::ControlPanelWidget ui_;
  QWidget* widget_;
  image_transport::Subscriber subscriber_;
  cv::Mat conversion_mat_;
  int con_res_button_flag = 0;
//  int num_gridlines_;     ready to remove
  RotateState rotate_state_;

//those are for goPose
  float p_x_ = 0.0;
  float p_y_ = 0.0;
  float o_y_ = 0.0;

//those parameters are for forward
  float linear_x_ = 0.0;
  float angular_z_ = 0.0;
  float forwar_dist_ = 0.0;
  float turn_rad_ = 0.0;
  float forward_sec_ = 1.0;

//those parameters for pointHead
  bool is_point_head = 0;
  mutable QMutex set_head_mutex_;

  QString point_head_selected_frame;
  double pH_x = 0.0, pH_y = 0.0, pH_z = 0.0;
  double pH_h = 0.0, pH_v = 1.5;
//  double mH_h = 0.0, mH_v = 0.0;    ready ro remove

  QList<QFrame*> custom_button_list;
  int num_cutom_button = 0;
  };

class Worker
  : public QObject
{
  Q_OBJECT

public:

  virtual ~Worker();

  arm_group_controller_Ptr arm_group;
//  arm_group.reset(new arm_group_controller("arm_with_torso"));
  arm_group_controller_Ptr gripper_group;
//  gripper_group.reset(new arm_group_controller("gripper"));
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;


signals:
  void do_armMove_(std::vector<double>);
  void do_update_target_poses_(geometry_msgs::PoseStamped);

//  void do_armStop_(MoveGroupInterface &);

public slots:
  void forward_Base(float x, float z, float s);
  void createArmClient(QString arm_topic);
  void createArmController(QString arm_topic);
//  void armDisco();
  void armHome();
  void armStraight();
  void armStop();
  void armMove(std::vector<double>);
  void armMove(double, double, double);
//  void armMove(geometry_msgs::PoseStamped);
  void printJoints();
  void openGripper();
  void closeGripper();
  void closeGripper(std::vector<double> pick_joint_values);
  void getPose();

  void pointHead();
//  void moveHead();    ready to remove

  virtual void boxSegment();
  virtual void removeBox();
//  virtual void pick(moveit::planning_interface::MoveGroupInterface& move_group);
  virtual void pick();


protected:
  virtual void addBox(QString obj_id, int index);
  virtual void extractBoxLengWidth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  virtual void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
//  virtual void place(moveit::planning_interface::MoveGroupInterface& move_group);

private:
  std::vector<double> joint_values;
  QString type_of_target = 0; //0: joint target, 1: position target
  mutable QMutex move_head_mutex_;

  //define a box struct
    struct AddBoxParams
    {
      double box_x;
      double box_y;
      double box_z;
      double height;//for table
      geometry_msgs::PoseStamped obj_pose;
      double direction_vec[3];
      double center_pt[3];
    };
    // Declare a variable of type AddCylinderParams and store relevant values from ModelCoefficients.
  AddBoxParams* box_params;
  bool points_not_found = true;
  tf::TransformListener pcl_trans_listener;
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Subscriber detect_sub_;


};

} // namespace
Q_DECLARE_METATYPE(geometry_msgs::PoseStamped)
Q_DECLARE_METATYPE(std::vector<double>)

#endif // my_namespace__my_plugin_H
