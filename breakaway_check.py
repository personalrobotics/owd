#!/usr/bin/env python

import roslib; roslib.load_manifest('gfe_owd_plugin')

import rospy
from pr_msgs.msg import BHState
from pr_msgs.srv import *
import sys

def handstate_cb(data):
    print "Hand positions: %(f1)1.2f %(f1)1.2f %(f3)1.2f %(f4)1.2f"%{'f1':data.positions[0],'f2':data.positions[1],'f3':data.positions[2],'f4':data.positions[3]}, "state: %d"%data.state

 
def breakaway_check():
    rospy.init_node('breakaway_check')
    # rospy.Subscriber("/right/bhd/handstate", BHState, handstate_cb)
    rospy.wait_for_service('/right/bhd/SetHandTorque')
    rospy.wait_for_service('/right/bhd/MoveHand')
    rospy.wait_for_service('/right/owd/SetStiffness')
    try:
        mv = rospy.ServiceProxy('/right/bhd/MoveHand',MoveHand)
    except rospy.ServiceException, e:
        print "Subscribing to /right/bhd/MoveHand failed: %s"%e
        sys.exit(1)
    try:
        rh = rospy.ServiceProxy('/right/bhd/ResetHand',ResetHand)
    except rospy.ServiceException, e:
        print "Subscribing to /right/bhd/ResetHand failed: %s"%e
        sys.exit(1)
    try:
        sht = rospy.ServiceProxy('/right/bhd/SetHandTorque',SetHandTorque);
    except rospy.ServiceException, e:
        print "Subscribing to /right/bhd/SetHandTorque failed: %s"%e
        sys.exit(1)
    try:
        ss = rospy.ServiceProxy('/right/owd/SetStiffness',SetStiffness);
    except rospy.ServiceException, e:
        print "Subscribing to /right/owd/SetStiffness failed: %s"%e
        sys.exit(1)

    print "Make sure hand is in a clear space with the palm facing up."
    print "Press enter to hold the arm position and reset the hand."
    cmd=raw_input()
    ss(1)
    rh()
    print "Please place the drill battery in the palm and hit enter"
    cmd=raw_input()
    sht(2200,1100)
    mv(1,[1.1,1.1,1.1,0])
    torque=600
    while (torque < 2200):
        print "Press enter to test grasp with torque of ",torque
        cmd=raw_input()
        sht(torque,100)
        mv(1,[3,3,3,0])
        #print "Which fingers, if any, broke away? Enter single digits in a row (like '13', or enter"
        #cmd=raw_input()
        #print "got string ", cmd
        torque += 100


#    rospy.spin()
 
if __name__ == '__main__':
     breakaway_check()

