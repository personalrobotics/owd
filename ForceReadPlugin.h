/***********************************************************************

Author:        Kevin Knott (SRI), 2011

Description:   This OWD Plugin publishes the feed forward predicted 
               force reading of the end effector based upon joint torques. 

Note:          You are free to modify and reuse this code without
               restrictions.
 
**********************************************************************/

#ifndef FORCEREADPLUGIN_H
#define FORCEREADPLUGIN_H

#include <openwam/Plugin.hh>
#include <ros/ros.h>
#include <pr_msgs/ForceRead.h>

class ForceReadPlugin : public OWD::Plugin {
 public:

  ForceReadPlugin();
  ~ForceReadPlugin();

  virtual void Publish();

  private:
  ros::Publisher force_read_pub;
};
 

#endif // FORCEREADPLUGIN_H
