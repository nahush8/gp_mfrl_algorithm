#!/usr/bin/python

import numpy as np
import rospy
import time
import random
import math
import std_msgs.msg
from collections import deque

import matplotlib.pyplot as plt
import actionlib
import matplotlib.pyplot as plt

import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *

import update_gp_new
import planner_new
import gp_gazebo.msg
from global_var import initialTrainingEpisodes, GRID, current_state_for_grid_world_reference,envList
import global_var
#import update_transition_class
#currentState = 0
action_value = (0,0,0)

old_state = (-GRID, -GRID, 0)
next_state_3d = (0,0,0)

record = []
states_2d = [ (i , j, 0) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1)]
states_3d = [ (i , j, k) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1) for k in xrange(-GRID,GRID+1,1)]

recordCounter = 0

global_var.sigmaDictX = {}
global_var.sigmaDictY = {}
global_var.sigmaDictZ = {}

global_var.delta_t = 1
 
currentEnv = envList[0]
transition = update_gp_new.update_transition_class()
updateObj = planner_new.gprmax()

#fig = plt.figure(2)
#plt.ion()
# for x in xrange(-GRID, GRID + 1, global_var.delta_t):
#     for y in xrange(-GRID, GRID + 1, global_var.delta_t):
#         plt.scatter(x,y, marker='o', s=100, color='red')

def currentStates(currentEnvironmet):
    global states_2d
    global states_3d

    if currentEnvironmet == 'grid':
        return states_2d
    else:   
        return states_3d

def check(curr, currentEnvironment):
    return curr in currentStates(envList[envList.index(currentEnvironment)-1])


def agent_client():

    global action_value

    global old_state
    global next_state

    global record
    global currentEnv
    global transition
    global updateObj
    global recordCounter

    sigma_sum_threshX = [0.5, 1.0]
    sigmaThreshX = [0.1, 0.2]

    sigma_sum_threshY = [.5, 1.0]
    sigmaThreshY = [0.1, 0.2]

    sigma_sum_threshZ = [0.5, 1.0]
    sigmaThreshZ = [0.1, 0.2]
    
    #TODO Make the actions more generic like a 3D vector. 
    actionList = [(0,1,0),(0,-1,0),(1,0,0),(-1,0,0),(0,0,1),(0,0,-1)]

    devQueueX = deque([], 5)
    devQueueY = deque([], 5)
    devQueueZ = deque([], 5)

    #set the publisher for sending the goals
    action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
    print "action client init"
    #r = rospy.Rate(20)
    action_client.wait_for_server()
    # Some Random Number
    '''
    Initialize T
    '''
    # Inti
    current_state_for_grid_world_reference = old_state
    for j in range (0,initialTrainingEpisodes):
        action_value = actionList_2d[random.randint(0,3)]

        goal = gp_gazebo.msg.agentGoal(action=action_value)
        action_client.send_goal(goal,done_cb= done)
        #action_client.send_goal(goal)
        #print "GOAL SENT --> " + str(goal) 
        action_client.wait_for_result()

    T = transition.upDate_transition(record,currentStates(currentEnv),currentEnv) 
    U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
    policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
    old_state = (-GRID, -GRID, 0)
    list_of_samples_gathered = []


    '''
    #GP-MFRL Algorithm
    '''
    tracking = 1
    f = open("reward_gp_single_mfrl.txt", "w")
    samples_in_second_simulator = 0
    while True:
    	#print tracking
    	tracking += 1
        print '\n' +  str(old_state)
        '''
        Change to optimal actions once planner is implemented
        '''
        if currentEnv == 'grid':
            #Actions with no movement in Z direction
            action_value = actionList[random.randint(0,3)]
        else
            action_value = actionList[random.randint(0,5)]


        if (envList.index(currentEnv) > 0 and 
            check(old_state, currentEnv) and 
            global_var.sigmaDictX.get((int(old_state[0]),action_value[0]), 99) > sigmaThreshX[envList.index(currentEnv)-1] and 
            global_var.sigmaDictY.get((int(old_state[1]),action_value[1]),99) > sigmaThreshY[envList.index(currentEnv)-1] and
            global_var.sigmaDictZ.get((int(old_state[2]),action_value[2]), 99) > sigmaThreshZ[envList.index(currentEnv)-1]:
            

            currentEnv = envList[envList.index(currentEnv)-1]
            print "*************** PREVIOUS TRANSITION ***************"        
            devQueueX = deque([], 5)
            devQueueY = deque([], 5)
            devQueueZ = deque([], 5)
            # New action client init
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()
        
        no_of_samples = 0
        if (sum(devQueueX) < sigma_sum_threshX[envList.index(currentEnv)] and 
            sum(devQueueY) < sigma_sum_threshY[envList.index(currentEnv)]) and
            sum(devQueueZ) < sigma_sum_threshZ[envList.index(currentEnv)]) and 
            len(devQueueY) > 4 and 
            envList.index(currentEnv) < len(envList) - 1:  

            currentEnv = envList[envList.index(currentEnv) + 1]
            print '++++++++NEXT Transition+++++++'
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()
            devQueueX = deque([], 5)
            devQueueY = deque([], 5)
            devQueueZ = deque([], 5)
  
        if envList.index(currentEnv) < len(envList):
            samples_in_second_simulator += 1
            print samples_in_second_simulator

            if samples_in_second_simulator % 25 == 0 : 
                U = updateObj.value_iteration (T, currentStates(currentEnv), currentEnv)
                policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
                # while old_state != Goal_state
                # reward_in_second_simulator += reward_dynmaics(old_state, actionValue, currentEnv) 
                f.write( str(U[(-8,-8)])  )
                f.write('\n')
                f.flush()
                print "WRITTEN TO A FILE"

        # no_of_samples += 1

        goal = gp_gazebo.msg.agentGoal(action = action_value)
        action_client.send_goal(goal, done_cb= done)
        action_client.wait_for_result()
        currSigmaX = global_var.sigmaDictX.get((int(next_state[0]),action_value[0]),999)
        currSigmaY = global_var.sigmaDictY.get((int(next_state[1]),action_value[1]),999)
        currSigmaZ = global_var.sigmaDictZ.get((int(next_state[2]),action_value[2]),999)

        devQueueX.appendleft(currSigmaX)
        devQueueY.appendleft(currSigmaY)
        devQueueZ.appendleft(currSigmaZ)

        recordCounter = recordCounter + 1

        if recordCounter  == 5:
            T = transition.upDate_transition(record,currentStates(currentEnv))
                # U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
                # policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)        
            recordCounter = 0
                #plt.quiver(next_state[0],next_state[1],actionValue[0],actionValue[1])
                #plt.scatter(next_state[0],next_state[1], marker='o', s=500, color='blue')
                #plt.scatter(old_state[0],old_state[1], marker='o', s=100, color='red')
                #plt.pause(0.001)
        # list_of_samples_gathered.append(no_of_samples)        
        # print  'Samples gathered in\t' + str(envList.index(currentEnv)) + '\t is \t' + str(no_of_samples)
        # FIND THE SIGMA VALUE and ADD

        if next_state == (GRID, GRID, GRID):
            tmp = U
            U = updateObj.value_iteration (T, currentStates(currentEnv), currentEnv)
            difference, maximum = 0, -999
            for key in U:
                difference = max(difference, abs(U[key] - tmp[key]))
                if U[key] > maximum : maximum = U[key]
                
            if difference < 0.05 * abs(maximum) and envList.index(currentEnv) == 1: break

            
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()

            T = transition.upDate_transition(record,currentStates(currentEnv))
            policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
        # print "==========="
        # print currentEnv
        # print "==========="	
    '''
    #END OF ALGORITHM
    '''
    print policy
    f.close()

			
def done(integer,result):
    global action_value
    global old_state
    global next_state
    global record
    action_value_append = ()

    if integer == 3:
        if result.terminal == False:
            if result.reward != 0:
                next_state = (int(result.state[0]),int(result.state[1]),int(result.state[2]))
                #next_state = (max(min(result.state[0],GRID),-GRID) , max(min(result.state[1],GRID),-GRID))
                velocity =  ((next_state[0] - old_state[0])/global_var.delta_t, (next_state[1] - old_state[1])/global_var.delta_t,(next_state[2] - old_state[2])/global_var.delta_t)
                record.append( [old_state, action_value, velocity] )
                # print record
                old_state = next_state
                global_var.current_state_for_grid_world_reference = old_state


if __name__ == '__main__':
    try:
        rospy.init_node('agent', anonymous=True)
        agent_client()
        #rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"