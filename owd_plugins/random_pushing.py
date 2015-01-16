import roslib; roslib.load_manifest('gfe_owd_plugin')
import rospy
from numpy import *
from openravepy import *
from time import sleep
import subprocess
import sys
import os
import random
from gfe_owd_plugin.srv import AddWSTraj
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

# To run:
#
# python -i -c "import random_pushing as rp; he=rp.HerbEnv(False); rp.reset(he)"
#
#  rp.push(he,"right")

import TransformMatrix
import TSR

def CdSerialize1DArray(arr):
   return '%s'%(' '.join(' %.5f'%(arr[f]) for f in range(0,size(arr))))

def CdRosSrvCall(service,*args):
   s = 'rosservice call ' + service
   for arg in args:
      s += ' \'' + str(arg) + '\''
   print s
   os.system(s)

def CdTrajstopperTrigLoc():
   p = subprocess.Popen(['rosservice','call','/trajstopper/triggered_location'],stdout=subprocess.PIPE)
   p.wait()
   lines = p.stdout.readlines()
   if len(lines) != 2:
      print '### Error: bad response from trajstopper.'
      raise RuntimeError("Planner Failed")
   line = lines[1]
   if not line.startswith('reason: ') or not line.endswith('\n'):
      print '### Error: bad response from trajstopper.'
      raise RuntimeError("Planner Failed")
   line = line[8:-1]
   if line == 'Not Idle!':
      return False
   return [float(s) for s in line.split()]

class HerbEnv():
   
   def __init__(self,is_real):
      
      self.is_real = is_real
      
      RaveInitialize()
      self.env = Environment() # create openrave environment
      self.env.SetViewer('qtcoin') # attach viewer (optional)
      
      # Load Herb
      self.env.Load('/pr/data/ormodels/robots/herb2_padded.robot.xml')
      herb = self.env.GetRobots()[0]
      
      # Create Herb controller
      self.env.LockPhysics(True)
      herb_control = RaveCreateController(self.env,"Herb2Controller")
      herb.SetController(herb_control, range(herb.GetDOF()), 0 )
      herb.GetController().SendCommand("init leftarm rightarm") # include segway for real robot
      herb.GetController().SendCommand("trajectoryblends 1")
      self.env.LockPhysics(False)

      sleep(4)

      # Create problems attached to the env, attach them to herb
      self.env.LockPhysics(True)
      self.prob_cbirrt = RaveCreateProblem(self.env,'CBiRRT')
      self.env.LoadProblem(self.prob_cbirrt, herb.GetName())
      self.prob_traj = RaveCreateProblem(self.env,'Trajectory')
      self.env.LoadProblem(self.prob_traj, herb.GetName())
      self.prob_manip = RaveCreateProblem(self.env,'Manipulation')
      self.env.LoadProblem(self.prob_manip, herb.GetName())
      self.env.LockPhysics(False)
      

def reset(he):
   env = he.env
   herb = env.GetRobots()[0]
   herb_control = herb.GetController()
   prob_cbirrt = he.prob_cbirrt
   prob_traj = he.prob_traj

   # Reset arms in simulation
   if not he.is_real:
      home = array([
         4.3, -1.8, -0.8, 2.4, -3.2, 0.0, 0.0952,   0.0, 0.0, 0.0, 0.0,
         0.9, -1.9,  0.7, 2.4, -1.6, 0.0, 0.8,      0.0, 0.0, 0.0, 0.0
         ])
      herb.SetActiveDOFs(range(22))
      resp = prob_cbirrt.SendCommand('RunCBiRRT smoothingitrs 250 jointgoals %d %s'%(herb.GetActiveDOF(),CdSerialize1DArray(home)))
      if int(resp) != 1:
         print "Planner failed"
         raise RuntimeError("Planner Failed")
      prob_traj.SendCommand('ExecuteBlendedTrajectory trajfile cmovetraj.txt execute 1 blend_radius 0.2 blend_attempts 4 blend_step_size 0.05 linearity_threshold 0.1')
      while not herb_control.IsDone():
         sleep(0.5)
      herb.SetActiveDOFValues(home) #for the hands
   

def push(he, arm):

   env = he.env
   herb = env.GetRobots()[0]
   herb_control = herb.GetController()
   prob_cbirrt = he.prob_cbirrt
   prob_traj = he.prob_traj
   prob_manip = he.prob_manip
   is_real = he.is_real
   try:
      addwstraj = rospy.ServiceProxy('/right/owd/AddWSTraj',AddWSTraj)
   except rospy.ServiceException, e:
      print "Subscribing to AddWSTraj failed: %s"%e

   #raw_input("################# Press Enter to start planning ...\n")

   #arm = 'left'

   if arm == 'right':
      dofrange = range(0,7)
      manipid = 0
      handlen = 0.1625
   elif arm == 'left':
      dofrange = range(11,18)
      manipid = 1
      handlen = 0.1825
   else:
      raise RuntimeError("Which Arm?")

   # Set arm info
   herb.SetActiveDOFs(dofrange)
   herb.SetActiveManipulator(manipid)

   # Get original dofs
   dofs_orig = herb.GetActiveDOFValues()

   # get the current endpoint pose
   current_ep = herb.GetManipulators()[manipid].GetEndEffectorTransform() 
   print "currently at: ", current_ep

   # pick a random target 15cm away
   push_dist = 0.15
   rx = random.random() - 0.5
   ry = random.random() - 0.5
   rz = random.random() - 0.5
   rl = sqrt (rx*rx + ry*ry + rz*rz)
   rx = rx / rl
   ry = ry / rl
   rz = rz / rl
   target_ep = current_ep
   target_ep[0][3] += rx*push_dist
   target_ep[1][3] += ry*push_dist
   target_ep[2][3] += rz*push_dist
   print "target: ", target_ep

   # try to find an IK solution for the target pose
#   manip = herb.GetManipulators()[manipid]
#   solutions = manip.FindIKSolutions(array(target_ep),
#                                     IkFilterOptions.CheckEnvCollisions)
#   if solutions is None or len(solutions) == 0: # if found, then break
#      print 'Found no ik solutions for the target tm, terminating.'
#      return 0,"Found no ik solutions for the target tm, terminating.",[]
   
#   print 'found %d solutions:'%len(solutions)
#   print 'using solution ',solutions[0]

   soln = prob_manip.SendCommand('LiftArmGeneralIK exec 0 minsteps 50 maxsteps 50 direction %f %f %f'%(rx,ry,rz))
   if len(soln.split()) == 7:
      q_start = herb.GetActiveDOFValues()
      q_end = [float(soln.split()[j]) for j in range(7)]
      
      # send a WSTraj to the arm to get there
#      raw_input("Planning complete: press Enter to move ...\n")
      
      # translate the target from herb coordinates to arm coordinates
      newpose = Pose(position=Point(x=rz*push_dist,y=-ry*push_dist,z=rx*push_dist),
                     orientation=Quaternion(x=0,y=0,z=0,w=1))
      res = addwstraj(endpoint_change=newpose,
                      starting_config=q_start,
                      ending_config=q_end,
                      max_linear_velocity=0.1,
                      max_angular_velocity=0.01,
                      min_accel_time=1.0)
      
      if (res.ok):
         print "success"
      else:
         print "failure: %s"%res.reason
   else:
      print "could not create plan"


