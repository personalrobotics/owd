#include "ApplyForceTraj.h"
#include <owd_msgs/InsertKey.h>

class InsertKeyTraj : public OWD::Trajectory {
  public:
  InsertKeyTraj();
  ~InsertKeyTraj();

  static bool InsertKey(owd_msgs::InsertKey::Request &req,
			owd_msgs::InsertKey::Response &res);

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_InsertKey;

  typedef enum {
    STEP1_FIND_SURFACE=0,
    STEP2_FIND_CYLINDER,
    STEP3_FIND_12OCLOCK,
    STEP4_FIND_6OCLOCK,
    STEP5_MOVE_TO_KEYHOLE,
    STEP6_VERIFY_KEYHOLE_TOP,
    STEP7_VERIFY_KEYHOLE_BOTTOM,
    STEP8_INSERT
  } INSERTION_STEP;

  class InsertKeyStep : public OWD::Trajectory {
  public:
    InsertKeyStep(INSERTION_STEP);
    ~InsertKeyStep();
    
    INSERTION_STEP stepname;
    virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) =0;

    double last_time;
  };
  
  class InsertKeyStep8 : public InsertKeyStep {
  public:
    InsertKeyStep8();
    ~InsertKeyStep8();
    
    virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

    ApplyForceTraj *AFTraj;
    double total_shift;
    OWD::JointPos start_jointpos;
    R3 original_position;
    SO3 original_rotation;
    Vibration *extra_vibe;
    int vibe_count;
    double vibe_multiplier;
    int motion;
    double motiontime;
    Vibration horiz_vibe, vert_vibe;
  };  

  InsertKeyStep *current_step;
};
