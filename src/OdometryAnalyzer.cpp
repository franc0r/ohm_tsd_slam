/*
 * OdometryAnalyzer.cpp
 *
 *  Created on: Dec 6, 2018
 *      Refactured by: jasmin
 */

#include "OdometryAnalyzer.h"
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ohm_tsd_slam
{

OdometryAnalyzer::OdometryAnalyzer(obvious::TsdGrid& grid, const std::shared_ptr<rclcpp::Node>& node):
					_grid(grid),
					_stampLaser(ros::Time::now())

{
  node->declare_parameter<double>("wait_for_odom_tf", 1.0);
  node->declare_parameter<double>("reg_trs_max", TRNS_THRESH);
  node->declare_parameter<double>("reg_sin_rot_max", ROT_THRESH);
  node->declare_parameter<double>("max_velocity_rot", ROT_VEL_MAX);
  node->declare_parameter<double>("max_velocity_lin", TRNS_VEL_MAX);

  node->declare_parameter<std::string>("tf_odom_frame", "odom");
  node->declare_parameter<std::string>("tf_footprint_frame", "base_footprint");
  node->declare_parameter<std::string>("tf_child_frame", "laser");
  node->declare_parameter<std::string>("tf_base_frame", "/map");

  //odom rescue
  _waitForOdomTf = rclcpp::Duration(node->get_parameter("wait_for_odom_tf").as_double());
  _odomTfIsValid = false;

  //Maximum allowed offset between two aligned scans
  _trnsMax = node->get_parameter("reg_trs_max").as_double();
  _rotMax = node->get_parameter("reg_sin_rot_max").as_double();

  //Maximum robot speed at footprint frame
  _rotVelocityMax = node->get_parameter("max_velocity_rot").as_double();
  _trnsVelocityMax = node->get_parameter("max_velocity_lin").as_double();

  _tfOdomFrameId = node->get_parameter("tf_odom_frame").as_string();
  _tfFootprintFrameId = node->get_parameter("tf_footprint_frame").as_string();
  _tfChildFrameId = node->get_parameter("tf_child_frame").as_string();
  _tfBaseFrameId = node->get_parameter("tf_base_frame").as_string();

  // Set up tf with default buffer length.
  _tfBuffer = std::make_unique<tf2_ros::Buffer>();
  _tfTransformListener = std::make_unique<tf2_ros::TransformListener>(*_tfBuffer);
}

OdometryAnalyzer::~OdometryAnalyzer()
{

}

void OdometryAnalyzer::odomRescueInit()
{
	std::cout << __PRETTY_FUNCTION__ << " enter RESCUE INIT " << std::endl;
  //get tf -> laser transform at init, assuming its a static transform
	std::cout << __PRETTY_FUNCTION__ << " try from " << _tfFootprintFrameId << " to " << _tfChildFrameId << std::endl;
  try
  {
    _tfListener.waitForTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), ros::Duration(10.0));
	_tfListener.lookupTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException& ex)
  {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
	exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received static base_footprint to laser tf for odom rescue");
  _tfLaser = _tfReader;
	obvious::Matrix tfLaserMatrix = tfToObviouslyMatrix3x3(_tfLaser);
	std::cout << __PRETTY_FUNCTION__ << "tfLaserMatrix = \n" << std::endl;
	tfLaserMatrix.print();

  //get first mal -> odom transform for initialization
	std::cout << __PRETTY_FUNCTION__ << " try from " << _tfBaseFrameId << " to " << _tfOdomFrameId << std::endl;
  try
  {
    _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), ros::Duration(10.0));
    _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received first odom tf for initialization of odom rescue");
  //transform odom to laser frame
  _tfOdomOld = _tfReader;
	obvious::Matrix tfOdomOldMatrix = tfToObviouslyMatrix3x3(_tfOdomOld);
	std::cout << __PRETTY_FUNCTION__ << "tfOdomOldMatrix = \n" << std::endl;
	tfOdomOldMatrix.print();
}

void OdometryAnalyzer::odomRescueUpdate()
{
  //get new odom tf
  try
  {
	  _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _waitForOdomTf);
	  _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _tfReader);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    _odomTfIsValid = false;
  }

  _tfOdom = _tfReader;
	obvious::Matrix tfOdomMatrix = tfToObviouslyMatrix3x3(_tfOdom);
	std::cout << __PRETTY_FUNCTION__ << "tfOdomMatrix = \n" << std::endl;
	tfOdomMatrix.print();

  //calculate diff odom -> odom(t-1) - odom(t)
  _tfRelativeOdom = _tfOdomOld.inverse() * _tfOdom;

  std::cout << __PRETTY_FUNCTION__ << "_odomTfIsValid = " << _odomTfIsValid << std::endl;

 obvious::Matrix tfRelativeMatrix = tfToObviouslyMatrix3x3(_tfRelativeOdom);

  std::cout << __PRETTY_FUNCTION__ << "odom(t-1) - odom(t) = tfRelativeOdom = \n " << std::endl;
  tfRelativeMatrix.print();

  //push state ahead
  _tfOdomOld = _tfOdom;

	obvious::Matrix tfOdomOldMatrix = tfToObviouslyMatrix3x3(_tfOdomOld);
	std::cout << __PRETTY_FUNCTION__ << "tfOdomOldMatrix = \n" << std::endl;
	tfOdomOldMatrix.print();

  _odomTfIsValid = true;
}

void OdometryAnalyzer::odomRescueCheck(obvious::Matrix& T_slam)
{
	std::cout << __PRETTY_FUNCTION__ << "entered odomRescueCheck method \n" << std::endl;

	obvious::Matrix tfLaserMatrix = tfToObviouslyMatrix3x3(_tfLaser);

	std::cout << __PRETTY_FUNCTION__ << "\n\t x y z " <<_tfLaser.getOrigin().x() << " " << _tfLaser.getOrigin().y() << " " << _tfLaser.getOrigin().z() << std::endl;

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;

	//tf::Quaternion q;
	_tfLaser.getBasis().getRPY(roll, pitch, yaw);
	std::cout << __PRETTY_FUNCTION__ << "\n\t r p y  " << roll << " " << pitch << " " << yaw << std::endl;

	//_tfLaser vor tfToObviouslyMatrix3x3 Funktion ausgeben lassen

	std::cout << __PRETTY_FUNCTION__ << "tfLaserMatrix = \n" << std::endl;
	tfLaserMatrix.print();

  //transform transformation from slam to odom e.g. from laser to base footprint system
  obvious::Matrix T_laserOnBaseFootprint = tfToObviouslyMatrix3x3(_tfLaser) * T_slam * tfToObviouslyMatrix3x3(_tfLaser).getInverse();

  std::cout << __PRETTY_FUNCTION__ << "T_slam transformed into odom frame (from laser to basefootprint): T_laserOnBaseFootprint = \n" << T_laserOnBaseFootprint << std::endl;

  //get dt
  ros::Duration dtRos 	= _stampLaser - _stampLaserOld;
  double dt 			= dtRos.sec + dtRos.nsec * 1e-9;
  std::cout << "dt = " << dt << std::endl;
  //get velocities
  double dx 		= T_laserOnBaseFootprint(0, 2);
  std::cout << "dx = " << dx << std::endl;
  double dy 		= T_laserOnBaseFootprint(1, 2);
  std::cout << "dy = " << dy << std::endl;
  double dtrans 	= sqrt(pow(dx,2) + pow(dy,2));
  std::cout << "dtrans = " << dtrans << std::endl;

  double arcsin = asin(T_laserOnBaseFootprint(0,1));
  std::cout << "arcsin = " << arcsin << std::endl;

 // double drot 		= abs(asin(T_laserOnBaseFootprint(90,1)));	//don't use acos here, missing sign -- WAS IST DIE 90 bitte für ein riesenquatsch??
 // double drot 		= abs(asin(T_laserOnBaseFootprint(0,1)));

//  obvious::Matrix* T_laserOnBaseFootprintPtr = &T_laserOnBaseFootprint;
//  double drot 		= calcAngle(T_laserOnBaseFootprintPtr);

  double drot 		= calcAngle(&T_laserOnBaseFootprint);
  std::cout << "drot = " << drot << std::endl;

  double vtrans 	= dtrans / dt;
  std::cout << "vtrans = " << vtrans << std::endl;

  //use odom instead of slam if slam translation is impossible for robot
	std::cout << __PRETTY_FUNCTION__ << "grid cell size = \n" << _grid.getCellSize() << std::endl;
  if(1)//dtrans > _grid.getCellSize() * 0.5)
  //if(dtrans > _grid.getCellSize() * 2.0)
	{
	  if(1)//drot > _rotVelocityMax || vtrans > _trnsVelocityMax)
	  {
		  ROS_INFO("------ODOM-RECOVER------");

		  T_slam = 	  tfToObviouslyMatrix3x3(_tfLaser).getInverse() *
				  	  tfToObviouslyMatrix3x3(_tfRelativeOdom) *
				  	  tfToObviouslyMatrix3x3(_tfLaser);

		  std::cout << __PRETTY_FUNCTION__ << "if slam translation is impossible for robot, use odom. Calculation: T_slam = _tfLaser(inverse) * _tfRelativeOdom * _tfLaser =  "
				  << T_laserOnBaseFootprint << std::endl;
	  }
	}
}

obvious::Matrix OdometryAnalyzer::tfToObviouslyMatrix3x3(const tf::Transform& tf)
{
	std::cout << __PRETTY_FUNCTION__ << " enter " << std::endl;
  obvious::Matrix ob(3,3);
  ob.setIdentity();
  tf.getRotation();
  double theta = tf::getYaw(tf.getRotation());
  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  // problem with sin() returns -0.0 (avoid with +0.0)
  ob(0, 0) = cos(theta) + 0.0;
  ob(0, 1) = -sin(theta) + 0.0;
  ob(0, 2) = x + 0.0;
  ob(1, 0) = sin(theta) + 0.0;
  ob(1, 1) = cos(theta) + 0.0;
  ob(1, 2) = y + 0.0;
  	  std::cout << __PRETTY_FUNCTION__ << " exit " << std::endl;
  return ob;
}

double OdometryAnalyzer::calcAngle(obvious::Matrix* T)
{
	std::cout << __PRETTY_FUNCTION__ << " enter " << std::endl;
  double angle			= 0.0;
  const double ARCSIN	= asin((*T)(1, 0));
  const double ARCSINEG	= asin((*T)(0, 1));
  const double ARCOS	= acos((*T)(0, 0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
	  angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
	  angle = 2.0 * M_PI - ARCOS;
	std::cout << __PRETTY_FUNCTION__ << " exit " << std::endl;
  return(angle);
}

} /* namespace ohm_tsd_slam_ref */








