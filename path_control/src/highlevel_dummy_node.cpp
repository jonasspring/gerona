#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/FollowPathAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_listener.h>

#include <move_base_lite_msgs/FollowPathAction.h>
#include <move_base_lite_msgs/FollowPathActionGoal.h>

using namespace path_msgs;

/**
 * @brief Simple high level control dummy for testing.
 *
 *
 */

class HighDummy
{
public:
  HighDummy(ros::NodeHandle &nh):
    nh_(nh),
    send_path_client_("follow_path", true)
  {

    cmd_vel_Subscriber = nh_.subscribe("/cmd_vel", 10, &HighDummy::cmd_vel_Callback, this);

    cmd_vel_raw_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_raw" , 10, true);
    send_path_client_goal_Publisher = nh_.advertise<path_msgs::FollowPathActionGoal>("follow_path/goal", 10, true);
    send_path_client_preempt_Publisher = nh_.advertise<actionlib_msgs::GoalID>("follow_path/cancel", 1, true);


    follow_path_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>(nh_, "/controller/follow_path", 0, false));

    follow_path_server_->registerGoalCallback(boost::bind(&HighDummy::followPathGoalCallback, this));
    follow_path_server_->registerPreemptCallback(boost::bind(&HighDummy::followPathPreemptCallback, this));

    follow_path_server_->start();

    //ROS_INFO("Start: %f", follow_path_server_->isActive());

  }




private:
  ros::NodeHandle nh_;

  ros::Subscriber cmd_vel_Subscriber;

  ros::Publisher cmd_vel_raw_publisher;
  ros::Publisher send_path_client_goal_Publisher;
  ros::Publisher send_path_client_preempt_Publisher;

  actionlib::SimpleActionClient<path_msgs::FollowPathAction> send_path_client_;

  boost::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction> > follow_path_server_;
  actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>::GoalConstPtr follow_path_goal_;


  void followPathGoalCallback()
  {
    ROS_INFO("Hi");

    follow_path_goal_ = follow_path_server_->acceptNewGoal();

    path_msgs::FollowPathGoal gerona_path_action_goal;

    path_msgs::DirectionalPath directional_path;

    nav_msgs::Path path = follow_path_goal_->target_path;

    ROS_INFO("Hi2");

    for(int i = 0; i < path.poses.size()/10; i++){

      path_msgs::DirectionalPath empty_path;
      directional_path = empty_path;

      for(int k = 0; k < 10; k++){
        if((i*10+k)< path.poses.size()){
          directional_path.poses.push_back(path.poses[i*10+k]);
        }
      }
      gerona_path_action_goal.path.paths.push_back(directional_path);
    }
//    directional_path.poses = path.poses;

//    ROS_INFO("Hi3");
//    //ROS_INFO("Dummy received path with size: %f" , path.poses.size());

//    gerona_path_action_goal.path.paths.push_back(directional_path);

//    ROS_INFO("Hi4");

    gerona_path_action_goal.path.header.frame_id = follow_path_goal_->target_path.header.frame_id;
    gerona_path_action_goal.follower_options.velocity = 0.6;
    gerona_path_action_goal.follower_options.init_mode = FollowerOptions::INIT_MODE_CONTINUE;

    ROS_INFO("Hi5");

    send_path_client_.cancelAllGoals();
    send_path_client_.sendGoal(gerona_path_action_goal);

    //ROS_INFO("Dummy sent path with size1 %f: , size2: %f" , gerona_path_action_goal.path.paths.size(), directional_path.poses.size());
  }

  void followPathPreemptCallback(){
    send_path_client_.cancelAllGoals();
    follow_path_server_->setPreempted();
  }

  void cmd_vel_Callback(const geometry_msgs::Twist& cmd_vel){
    cmd_vel_raw_publisher.publish(cmd_vel);
  }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "highlevel_dummy", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    HighDummy dummy(nh);

    ros::spin();
    return 0;
}

