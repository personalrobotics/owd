#include <openmath/SO3.hh>
#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <openwam/Butterworth.h>
#include <ros/ros.h>
#include <owd_msgs/Servo.h>
#include <owd_msgs/SetGains.h>


class Servo2Traj : public OWD::Trajectory {
 public:
  Servo2Traj(const std::vector<double> &arm_position);
  ~Servo2Traj();

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool Register();
  static bool Shutdown();
  static void wamservo_callback(const boost::shared_ptr<const owd_msgs::Servo> &servo);
  static bool SetServoGains(owd_msgs::SetGains::Request &req,
                            owd_msgs::SetGains::Response &res);

  static Servo2Traj *current_traj;
  static std::vector<double> Kp, Kd, Ki;
  static ros::Subscriber wamservo_sub;
  static ros::ServiceServer ss_SetServoGains;

  std::vector<double> target_velocity;
  std::vector<double> last_vel_error;
  std::vector<double> total_vel_error;
  std::vector<double> stoptime;
  std::vector<bool> active;
  std::vector<double> static_q;
  double start_time;
  size_t vel_address;
};
