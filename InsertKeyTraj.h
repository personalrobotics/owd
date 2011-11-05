#include "ApplyForceTraj.h"
#include <gfe_owd_plugin/InsertKey.h>

class InsertKeyTraj : public ApplyForceTraj {
  public:
  InsertKeyTraj();
  ~InsertKeyTraj();

  static bool InsertKey(gfe_owd_plugin::InsertKey::Request &req,
			gfe_owd_plugin::InsertKey::Response &res);

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

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
    virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt) =0;
  };
  
  class InsertKeyStep8 : public InsertKeyStep {
  public:
    InsertKeyStep8();
    ~InsertKeyStep8();
    
    virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

    double total_shift;
    OWD::JointPos start_jointpos;
    R3 original_position;
    SO3 original_rotation;
  };  

  InsertKeyStep *current_step;
};
