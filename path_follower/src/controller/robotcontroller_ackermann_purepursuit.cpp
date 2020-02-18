/*
 * robotcontrollerackermanngeometrical.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: Lukas Hollaender
 */

#include <path_follower/controller/robotcontroller_ackermann_purepursuit.h>


#include <ros/ros.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <cslibs_navigation_utilities/MathHelper.h>

#include <deque>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(Robotcontroller_Ackermann_PurePursuit, ackermann_purepursuit, ackermann);

Robotcontroller_Ackermann_PurePursuit::Robotcontroller_Ackermann_PurePursuit () :
    RobotController(),
    waypoint_(0)
{

	path_interpol_pub_ = node_handle_.advertise<nav_msgs::Path>("interp_path", 10);


	ROS_INFO("Parameters: factor_lookahead_distance_forward=%f, factor_lookahead_distance_backward=%f"
				"\nvehicle_length=%f\nfactor_steering_angle=%f\ngoal_tolerance=%f",
				params_.factor_lookahead_distance_forward(), params_.factor_lookahead_distance_backward(),
				params_.vehicle_length(), params_.factor_steering_angle(), params_.goal_tolerance());


  //wheel_vel_sub_ = nh_.subscribe("/cmd_vel_raw", 10, &Robotcontroller_Ackermann_PurePursuit::WheelVelocities, this);

}

Robotcontroller_Ackermann_PurePursuit::~Robotcontroller_Ackermann_PurePursuit() {
}

void Robotcontroller_Ackermann_PurePursuit::reset() {
	waypoint_ = 0;
    RobotController::reset();
}

void Robotcontroller_Ackermann_PurePursuit::setPath(Path::Ptr path) {
    RobotController::setPath(path);
}


void Robotcontroller_Ackermann_PurePursuit::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void Robotcontroller_Ackermann_PurePursuit::start() {

}

RobotController::MoveCommandStatus Robotcontroller_Ackermann_PurePursuit::computeMoveCommand(
		MoveCommand* cmd) {

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

    Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

//	double lookahead_distance = velocity_;
//	if(getDirSign() >= 0.)
//		lookahead_distance *= params_.factor_lookahead_distance_forward();
//	else
//		lookahead_distance *= params_.factor_lookahead_distance_backward();

    double lookahead_distance = params_.look_ahead_dist();

    // angle between vehicle theta and the connection between the rear axis and the look ahead point
    const double alpha = computeAlpha(lookahead_distance, pose);

    double delta = atan2(2. * params_.vehicle_length() * sin(alpha), lookahead_distance);

//    if(getDirSign() < 0.){
//      delta = MathHelper::NormalizeAngle(M_PI + delta);
//    }

    double exp_factor = RobotController::exponentialSpeedControl();
    move_cmd_.setDirection(params_.factor_steering_angle() * (float)delta);
    double v = getDirSign() * (float) velocity_ * exp_factor;
    move_cmd_.setVelocity(v);


    *cmd = move_cmd_;

    return RobotController::MoveCommandStatus::OKAY;
}

void Robotcontroller_Ackermann_PurePursuit::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

double Robotcontroller_Ackermann_PurePursuit::computeAlpha(double& lookahead_distance,
																			  const Eigen::Vector3d& pose) {

	// TODO: correct angle, when the goal is near

    double distance=0, dx=0, dy=0;
	for (unsigned int i = waypoint_; i < path_interpol.n(); ++i) {
		dx = path_interpol.p(i) - pose[0];
		dy = path_interpol.q(i) - pose[1];

		distance = hypot(dx, dy);
		waypoint_ = i;
		if (distance >= lookahead_distance)
			break;
	}

	// angle between the connection line and the vehicle orientation
	double alpha = MathHelper::AngleDelta(pose[2], atan2(dy, dx));

	// TODO this is not consistent with dir_sign!!!
//  if (alpha > M_PI_2){
//    alpha = M_PI - alpha;
//  }
//  else if (alpha < -M_PI_2){
//    alpha = -M_PI - alpha;
//  }

  if (alpha > M_PI_2){
      alpha = -M_PI + alpha;
      setDirSign(-1);
    }
    else if (alpha < -M_PI_2){
      alpha = M_PI + alpha;
      setDirSign(-1);
    }
  else{
    setDirSign(1);
  }

	// set lookahead_distance to the actual distance
	lookahead_distance = distance;


	// line to lookahead point
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
	to.x = path_interpol.p(waypoint_); to.y = path_interpol.q(waypoint_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "geo", 1, 0, 0, 1, 0.01);

#ifdef DEBUG
	ROS_INFO("LookAheadPoint: index=%i, x=%f, y=%f", waypoint_, path_interpol.p(waypoint_)
				, path_interpol.q(waypoint_));
	ROS_INFO("Pose: x=%f, y=%f, theta=%f", pose[0], pose[1], pose[2]);
	ROS_INFO("Alpha=%f, dir_sign=%f", alpha, dir_sign_);
#endif

	return alpha;
}

void Robotcontroller_Ackermann_PurePursuit::WheelVelocities(const geometry_msgs::Twist& cmd_vel)
{

  double v_lin = cmd_vel.linear.x;
  double v_ang = cmd_vel.angular.z;

  double l;
  nh_.getParam("/vehicle_controller/wheel_separation", l);

  Vl_ = v_lin - l/2 * v_ang;
  Vr_ = v_lin + l/2 * v_ang;

  ROS_INFO("L: %f, vl: %f, vr: %f", l, Vl_, Vr_);

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time_).toSec();
  last_time_ = current_time;
  //if(Vl_ > 1e-3 || Vr_ > 1e-3){
    //ekf_.predict(array, dt);
    ekf_.predict(Vl_, Vr_, dt);
    pose_ekf_ << ekf_.x_(0), ekf_.x_(1), ekf_.x_(2);
    ICR_ekf_  << ekf_.x_(3), ekf_.x_(4), ekf_.x_(5);
  //}
  Eigen::Vector3d pose = pose_tracker_->getRobotPose();

  ROS_INFO("Predicted:");
  ROS_INFO_STREAM("pose_ekf " << pose_ekf_ << std::endl);
  ROS_INFO("Pose: x=%f, y=%f, theta=%f", pose[0], pose[1], pose[2]);
  ROS_INFO_STREAM("icr_ekf " << ICR_ekf_ << std::endl);

  Eigen::Vector3d delta;
  delta(0) = pose[0];
  delta(1) = pose[1];
  delta(2) = pose[2];
  ekf_.correct(delta);

  ROS_INFO("Corrected:");
  pose_ekf_ << ekf_.x_(0), ekf_.x_(1), ekf_.x_(2);
  ROS_INFO_STREAM("pose_ekf " << pose_ekf_ << std::endl);
  ROS_INFO("Pose: x=%f, y=%f, theta=%f", pose[0], pose[1], pose[2]);

  last_cmd_vel = cmd_vel;

}
