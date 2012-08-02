#include "JSController.hh"
#include "Joint.hh"

using namespace OWD;

JSController::JSController(std::string controller_name) : 
  name(_name),
  _name(controller_name) {
  // register this instance
  children.push_back(this);

  // create the individual joint controllers
  //                                  Kp,  Kd,   Ki
  jcontrollers.push_back(JointCtrlPID(900, 10.0, 2.5));  // J1
  jcontrollers.push_back(JointCtrlPID(2500, 20.0, 5.0)); // J2
  jcontrollers.push_back(JointCtrlPID(600, 10.0, 2.5));  // J3
  jcontrollers.push_back(JointCtrlPID(500,  2.5, 0.5));  // J4
  if (Joint::Jn > 4) {
    jcontrollers.push_back(JointCtrlPID( 40,  0.5, 0.5));  // J5
    jcontrollers.push_back(JointCtrlPID( 40,  0.5, 0.5));  // J6
    jcontrollers.push_back(JointCtrlPID( 16,  0.16, 0.1)); // J7
  }
  active.resize(jcontrollers.size(),true);
}

JSController::~JSController() {
  for (std::vector<JSController *>::iterator it=children.begin(); it!=children.end(); ++it) {
    if (*it == this) {
      children.erase(it);
      return;
    }
  }
}

std::vector<double> JSController::evaluate(std::vector<double> q_target,
					   std::vector<double> q,
					   double dt) {
  if (q_target.size() != jcontrollers.size()) {
    throw "q_target array size does not match number of controllers";
  }
  if (q.size() != jcontrollers.size()) {
    throw "q array size does not match number of controllers";
  }
  std::vector<double> torques(jcontrollers.size(), 0);
  for (unsigned int i=0; i<jcontrollers.size(); ++i) {
    if (active[i]) {
      torques[i]=jcontrollers[i].evaluate(q_target[i], q[i], dt);
    }
  }
  return torques;
}

void JSController::set_gains(unsigned int joint, std::vector<double> gains) {
  if (joint < jcontrollers.size()) {
    // might throw an error which will have to be caught by the
    // next level up.
    jcontrollers[joint].set_gains(gains);
  } else {
    throw "Index exceeds range";
  }
}

std::vector<double> JSController::get_gains(unsigned int joint) {
  if (joint < jcontrollers.size()) {
    // might throw an error which will have to be caught by the
    // next level up.
    return jcontrollers[joint].get_gains();
  } else {
    throw "Index exceeds range";
  }
}

void JSController::activate(unsigned int joint) {
  if (joint < jcontrollers.size()) {
    active[joint]=true;
  } else {
    throw "joint index exceeds number of controllers";
  }
}

void JSController::suppress(unsigned int joint) {
  if (joint < jcontrollers.size()) {
    active[joint]=false;
  } else {
    throw "joint index exceeds number of controllers";
  }
}

void JSController::reset(unsigned int j) {
  if (j<jcontrollers.size()) {
    jcontrollers[j].reset();
  } else {
    throw "Index out of range";
  }
}

void JSController::run(unsigned int j) {
  if (j<jcontrollers.size()) {
    jcontrollers[j].run();
  } else {
    throw "Index out of range";
  }
}

void JSController::stop(unsigned int j) {
  if (j<jcontrollers.size()) {
    jcontrollers[j].stop();
  } else {
    throw "Index out of range";
  }
}

JSController *JSController::find_controller(std::string controller_name) {
  for (std::vector<JSController *>::iterator it=children.begin(); it!=children.end(); ++it) {
    if ((*it)->_name == controller_name) {
      return *it;
    }
  }
  return NULL;
}

int JSController::DOF() {
  return jcontrollers.size();
}
  
std::vector<JSController *> JSController::children;


