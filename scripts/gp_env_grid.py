#!/usr/bin/python
import numpy as np
import rospy
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *
import time
import math
import std_msgs.msg
import actionlib
import gp_gazebo.msg
from global_var import GRID,envList
import global_var

# DEFINING THE OBSTACLE COORDINATES

GOAL_STATE_X = GRID
GOAL_STATE_Y = GRID
GOAL_STATE_Z = 0

class agentAction(object):
	_feedback = gp_gazebo.msg.agentFeedback()
	_result = gp_gazebo.msg.agentResult()

	def __init__(self,name):

		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name,gp_gazebo.msg.agentAction,execute_cb=self.execute, auto_start = False)
		self._as.start()
		print "Action server started"
		global_var.delta_t = 1

	def execute(self,goal):

		# sigma * np.random.randn(...) + mu	
		action = goal.action

		#print action
		mu = 0
		sigma = 0.15
		noise = sigma * np.random.randn() + mu
		state = global_var.current_state_for_grid_world_reference
		#noise = 0
		a = int(round(state[0] + action[0] * global_var.delta_t + noise))
		b = int(round(state[1] + action[1] * global_var.delta_t + noise)) 
		#actualState = state + action + randn(mean=0,sigma=0.1)
		currentState  = (max(min(a,GRID),-GRID), max(min(b,GRID),-GRID),0)
		global_var.current_state_for_grid_world_reference = currentState
		#print 'CS' + str(currentState)
		# print currentState
		if ((GOAL_STATE_X == currentState[0]) and (GOAL_STATE_Y == currentState[1])):
			print "********************* YOU Reached the goal ******************"
			#self._feedback.terminal = True
			self._result.reward = 50 
			self._result.terminal = False

		else: 
		#self._feedback.terminal = False
			self._result.reward= -1
			self._result.terminal = False
			#print "1 unit movement"
		
		self._result.state.insert(0, currentState[0])
		self._result.state.insert(1, currentState[1])
		self._result.state.insert(2, currentState[2])

		self._as.set_succeeded(self._result)
		self._result.state[:]=[]
	

if __name__ == '__main__':
    try:
		rospy.init_node('environment_action', anonymous=True)
		#rospy.spin()
		
		#set the publisher for sending the goals
		# initialize the subscriber node
		# subcribe to the mavros State
		
		state = rospy.wait_for_message("mavros/state",State)
		rospy.wait_for_service("mavros/cmd/arming");
		print " Arming service available"
		try:
		    armingClient = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		except rospy.ServiceException, e:
		    print "Service call failed: %s"%e

		rospy.wait_for_service("mavros/set_mode");
		print " SetMode service available"
		try:
		    setModeClient = rospy.ServiceProxy('mavros/set_mode', SetMode)
		except rospy.ServiceException, e:
		    print "Service call failed: %s"%e

		rospy.wait_for_service("mavros/cmd/takeoff");
		print " Takeoff service available"
		try:
		    takeoffClient = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
		except rospy.ServiceException, e:
		    print "Service call failed: %s"%e

		while not rospy.is_shutdown() and not state.connected:
		    state = rospy.wait_for_message("mavros/state",State)
		
		
		agentAction(envList[0])
		rospy.spin()
    except rospy.ROSInterruptException:
        pass
