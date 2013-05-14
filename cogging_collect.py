#!/usr/bin/env python

import roslib; roslib.load_manifest('owd_plugins')

import threading
import rospy
import genpy
from time import sleep
from pantilt.msg import WAMState
from owd_msgs.srv import AddTrajectory
from owd_msgs.srv import SetStiffness
from owd_msgs.srv import SetGains
from owd_msgs.msg import PIDgains
from owd_msgs.msg import JointTraj, Joints
from owd_plugins.srv import CoggingComp

class CC:
    def __init__(self):
        self.startangle=0
        self.stopangle=2.2
        self.valid_ws=False
        try:
            self.ws_sub = rospy.Subscriber("/head/owd/wamstate",
                                           WAMState,
                                           callback=self.ws_cb,
                                           queue_size=1)
            self.stiff = rospy.ServiceProxy('/head/owd/SetStiffness', SetStiffness)
            self.gains = rospy.ServiceProxy('/head/owd/SetGains', SetGains)
            self.traj = rospy.ServiceProxy('/head/owd/AddTrajectory', AddTrajectory)
            self.cogtraj = rospy.ServiceProxy('/head/owd/CoggingComp', CoggingComp)
        except rospy.ServiceException, e:
            print "A ROS connection has failed: %s"%e
        
    def ws_cb(self,data):
        self.jpos = data.positions
        self.target_jpos = data.target_positions
        self.state = data.state
        self.ws_time = data.header.stamp
        self.valid_ws=True

    def wait_for_traj(self,addtime):
        while ((self.ws_time < addtime) or
               (self.state != 1)):
            sleep(0.5)

    def collect_one(self,angle):
        print "Turning on PID gains"
        if (self.gains(joint=1,gains=PIDgains(kp=2.25,kd=0.08,ki=0.005)) is None):
            print "Could not SetGains"
            return(False)
        print "Holding position"
        if (self.stiff(1) is None):
            print "Could not SetStiffness 1"
            return(False)
        sleep(1)
        self.valid_ws = False
        while (not self.valid_ws):
            print "Waiting for a WAMState message"
            sleep(1)
        new_jpos = (angle,self.target_jpos[1])
        print "Moving to next angle"
        traj_response=self.traj(JointTraj(positions=[Joints(self.target_jpos),Joints(new_jpos)],blend_radius=[0,0],options=0,id="cogtraj"))
        if (traj_response is None):
            print "Could not move to requested angle"
            return(False)
        if (not traj_response.ok):
            print "AddTrajectory was rejected: %s"%traj_response.reason
            return(False)
        self.wait_for_traj(traj_response.time_added)
        print "Relaxing gains"
        if (self.gains(joint=1,gains=PIDgains(kp=0,kd=0,ki=0)) is None):
            print "Could not SetGains"
            return(False)
        sleep(1)
        self.lastangle = self.jpos[0]
        print "Collecting data starting at %f"%self.lastangle
        cogtraj_response = self.cogtraj(1,0.09, 0.003, 0.5, 100)
        if (cogtraj_response is None):
            print "Could not initiate cogging trajectory"
            return(False)
        print "Saving data"
        f=open("cogging.dat",'a')
        if (f is None):
            print "Could not save cogging data to cogging.dat"
            return(False)
        f.write(genpy.message.strify_message(cogtraj_response.data))
        f.close()
        print "Releasing position"
        if (self.stiff(0) is None):
            print "Could not SetStiffness 0"
            return(False)
        return(True)

    def collect(self):
        angle=0.03
        if (not self.collect_one(angle)):
            print "Failed at angle 0"
            return(False)
        while (self.lastangle < 6.3/3):
            if (not self.collect_one(self.lastangle + 0.09/3)):
                print "Failed at angle %f"%self.lastangle
                return(False)
        return(True)
        
if __name__ == '__main__':
    rospy.init_node('cogging_collect')
    CC_obj = CC()
    spin_thread=threading.Thread(name="spin",target=rospy.spin)
    spin_thread.start()
    if (not CC_obj.collect()):
        exit( -1)
    rospy.signal_shutdown("collection complete")
    spin_thread.join()
    exit(0)
