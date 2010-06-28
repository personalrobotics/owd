#include <ros/ros.h>
#include <CANbus.hh>
#include <tf/transform_broadcaster.h>
#include <pr_msgs/BHState.h>
#include <pr_msgs/MoveHand.h>
#include <pr_msgs/ResetHand.h>
#include <pr_msgs/GetDOF.h>
#include <pr_msgs/RelaxHand.h>
#include <pr_msgs/SetHandProperty.h>
#include <pr_msgs/GetHandProperty.h>

class BHD_280 {
public:
  ros::Publisher
    pub_handstate;
  ros::ServiceServer 
    ss_gethanddof,
    ss_movehand,
    ss_resethand,
    ss_sethandprop,
    ss_gethandprop,
    ss_relaxhand;
  CANbus *bus;

  ros::NodeHandle node;
  tf::TransformBroadcaster *tf_broadcaster;
  btTransform baseShift;  

  pr_msgs::BHState bhstate;
  double max_velocity;

  BHD_280(CANbus *cb);
  ~BHD_280();
  void Pump(const ros::TimerEvent& e);
  bool Publish();
  bool GetDOF(pr_msgs::GetDOF::Request &req,
	      pr_msgs::GetDOF::Response &res);
  bool RelaxHand(pr_msgs::RelaxHand::Request &req,
		 pr_msgs::RelaxHand::Response &res);
  bool ResetHand(pr_msgs::ResetHand::Request &req,
		 pr_msgs::ResetHand::Response &res);
  bool MoveHand(pr_msgs::MoveHand::Request &req,
		pr_msgs::MoveHand::Response &res);
  bool SetHandProperty(pr_msgs::SetHandProperty::Request &req,
		pr_msgs::SetHandProperty::Response &res);
  bool GetHandProperty(pr_msgs::GetHandProperty::Request &req,
		pr_msgs::GetHandProperty::Response &res);
  
private:
  void AdvertiseAndSubscribe(ros::NodeHandle &n);
  void GetParameters(ros::NodeHandle &n);
  void Unadvertise();
  void createT(double a, double alpha, double d, double theta, double result[4][4]);

};
  


