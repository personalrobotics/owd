/***********************************************************************

  Copyright 2012 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#ifndef JSCONTROLLER_HH
#define JSCONTROLLER_HH

#include <vector>
#include "JointCtrlPID.hh"

namespace OWD {

  class JSController {
  public:
    
    JSController(std::string name);
    ~JSController();
    
    virtual std::vector<double> evaluate(std::vector<double> q_target,
					 std::vector<double> q,
					 double dt);
    virtual void set_gains(unsigned int joint, std::vector<double> gains);
    virtual std::vector<double> get_gains(unsigned int joint);
    virtual void activate(unsigned int joint);
    virtual void suppress(unsigned int joint);
    virtual void reset(unsigned int j);
    virtual void run(unsigned int j);
    virtual void stop(unsigned int j);
    virtual int DOF();

    static JSController *find_controller(std::string name);
    const std::string &name;

  private:
    std::vector<JointCtrlPID> jcontrollers;
    std::vector<bool> active;
    std::string _name;
    static std::vector<JSController *> children;

  };

};

  
#endif // JSCONTROLLER_HH
