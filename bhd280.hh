#include <ros/ros.h>
#include <CANbus.hh>
#include <pr_msgs/BHState.h>
#include <pr_msgs/MoveHand.h>
#include <pr_msgs/ResetHand.h>
#include <pr_msgs/GetDOF.h>
#include <pr_msgs/RelaxHand.h>

class BHD280 {
public:
  ros::Publisher
    pub_handstate;
  ros::ServiceServer 
    ss_gethanddof,
    ss_movehand,
    ss_resethand,
    ss_relaxhand;
  CANbus *bus;

  ros::NodeHandle node;
  
  pr_msgs::BHState bhstate;

  BHD280(CANbus *cb);
  ~BHD280();
  void AdvertiseAndSubscribe(ros::NodeHandle &n);
  void Unadvertise();
  bool Publish();
  bool GetDOF(pr_msgs::GetDOF::Request &req,
	      pr_msgs::GetDOF::Response &res);
  bool RelaxHand(pr_msgs::RelaxHand::Request &req,
		 pr_msgs::RelaxHand::Response &res);
  bool ResetHand(pr_msgs::ResetHand::Request &req,
		 pr_msgs::ResetHand::Response &res);
  bool MoveHand(pr_msgs::MoveHand::Request &req,
		pr_msgs::MoveHand::Response &res);
  

};
  


