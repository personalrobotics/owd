#include <openmath/SO3.hh>
#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <openwam/Butterworth.h>
#include <ros/ros.h>
#include <pr_msgs/Servo.h>
#include <pr_msgs/SetGains.h>


class Servo2Traj : public OWD::Trajectory {
 public:
  Servo2Traj(const std::vector<double> &arm_position);
  ~Servo2Traj();

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool Register();
  static bool Shutdown();
  static void wamservo_callback(const boost::shared_ptr<const pr_msgs::Servo> &servo);
  static bool SetServoGains(pr_msgs::SetGains::Request &req,
                            pr_msgs::SetGains::Response &res);

  static Servo2Traj *current_traj;
  static std::vector<double> Kp, Kd;
  static ros::Subscriber wamservo_sub;
  static ros::ServiceServer ss_SetServoGains;
  static double lower_jlimit[], upper_jlimit[], jlimit_buffer;

  std::vector<double> target_velocity;
  std::vector<double> last_jpos;
  std::vector<double> last_vel_error;
  std::vector<double> stoptime;
  std::vector<bool> active;
  std::vector<double> static_q;
  std::vector<Butterworth<double> *> vel_filter;
  double last_time;
};
