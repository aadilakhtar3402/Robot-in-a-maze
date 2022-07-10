#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import sys


def move_turtle(turns):
    rospy.init_node('topic_publisher')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(1)
    move = Twist()

    
    for i in range(len(turns)):
        if (turns[i] == 2):
            t0 = rospy.get_rostime().secs
            while (t0 + 16 >= rospy.get_rostime().secs):
                move.angular.z = (np.pi-0.3)/16
                pub.publish(move)
                rate.sleep()
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()

        elif (turns[i] == 1):
            t0 = rospy.get_rostime().secs
            while (t0 + 8 >= rospy.get_rostime().secs):
                move.angular.z = (np.pi-0.01*(t0-rospy.get_rostime().secs))/16
                pub.publish(move)
                rate.sleep()
            move.angular.z = -0.1
            pub.publish(move)
            rate.sleep()
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()

        elif (turns[i] == -1):
            t0 = rospy.get_rostime().secs
            while (t0 + 8 >= rospy.get_rostime().secs):
                move.angular.z =  -(np.pi-0.01*(t0-rospy.get_rostime().secs))/16
                pub.publish(move)
                rate.sleep()
            move.angular.z = 0.1
            pub.publish(move)
            rate.sleep()
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()

        t0 = rospy.get_rostime().secs
        while t0 + 5 >= rospy.get_rostime().secs:
            move.linear.x = 0.178
            pub.publish(move)
            rate.sleep()
        move.linear.x = 0.0
        pub.publish(move)
        rate.sleep()



def Qpath(initial):
    n = 66

    P = np.array([[0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, -1, -1, 0, -1, -1, -1],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, -1, 0, 0, 0, 0, 0, 0],
                [0, -1, 0, -1, -1, 0, -1, 0],
                [0, -1, 0, 0, 0, 0, -1, 0],
                [0, -1, -1, 0, 0, 0, -1, 0],
                [0, 0, 0, 0, 0, 0, -1, 0]])

    R = np.full((66,66),-1)

    for k in range(1,65):
        i = int((k-1)/8)
        j = (k-1)%8
        if (P[i,j] == 0):
            if (i-1>= 0 and P[i-1,j] == 0):
                R[k,k-8] = 0
            if (i+1 < 8 and P[i+1,j] == 0):
                R[k,k+8] = 0
            if (j-1 >= 0 and P[i,j-1] == 0):
                R[k,k-1] = 0
            if (j+1 < 8 and P[i,j+1] == 0):
                R[k,k+1] = 0

    R[0,1] = 0
    R[1,0] = 100
    R[64, 65] = 100
    R[65,64] = 0


    gamma = 0.8

    Q = np.zeros([66,66])

    def avlacts(state):
        return np.where(R[state,:] >= 0)[0]

    def update(gamma, state, nxt_state):
        m = 0
        acts = avlacts(nxt_state)
        for i in range (acts.size):
            m = max(m, Q[nxt_state, acts[i]])
        Q[state,nxt_state] = R[state, nxt_state] + gamma*m




    for i in range(10000):
        cur_state = np.random.randint(n)
        acts = avlacts(cur_state)
        if (acts.size > 0):
            nxt_state = acts[np.random.randint(acts.size)]
            update(gamma, cur_state, nxt_state)


    initial_state = initial 

    cur_state = initial_state
    steps = [initial_state,]
    while(cur_state != 0 and cur_state != 65):
        acts = avlacts(cur_state)
        nxt_state = acts[0]
        for i in range(acts.size):
            if (Q[cur_state,acts[i]] > Q[cur_state,nxt_state]):
                nxt_state = acts[i]
        steps.append(nxt_state)
        cur_state = nxt_state

    #print(Q/np.max(Q) * 100,'\n')
    #print(steps)
    return steps


def directions(steps):
    turns = []
    dir = 11
    for i in range(len(steps)-1):
        if (steps[i] == 1):
            if (steps[0] == 1):
                turns.append(1)
            else :
                turns.append(-1)
            break
        if (steps[i] == 64):
            if (steps[0] == 64):
                turns.append(0)
            else :
                turns.append(1)
            break
        if (dir == 11):
            if (steps[i]+1 == steps[i+1]):
                turns.append(0)
                dir = 11
                continue
            if (steps[i]-1 == steps[i+1]):
                turns.append(2)
                dir = 12
                continue
            if (steps[i]+8 == steps[i+1]):
                turns.append(-1)
                dir = 22
                continue
            if (steps[i]-8 == steps[i+1]):
                turns.append(1)
                dir = 21
                continue

        if (dir == 21):
            if (steps[i]+1 == steps[i+1]):
                turns.append(-1)
                dir = 11
                continue
            if (steps[i]-1 == steps[i+1]):
                turns.append(1)
                dir = 12
                continue
            if (steps[i]+8 == steps[i+1]):
                turns.append(2)
                dir = 22
                continue
            if (steps[i]-8 == steps[i+1]):
                turns.append(0)
                dir = 21
                continue

        if (dir == 12):
            if (steps[i]+1 == steps[i+1]):
                turns.append(2)
                dir = 11
                continue
            if (steps[i]-1 == steps[i+1]):
                turns.append(0)
                dir = 12
                continue
            if (steps[i]+8 == steps[i+1]):
                turns.append(1)
                dir = 22
                continue
            if (steps[i]-8 == steps[i+1]):
                turns.append(-1)
                dir = 21
                continue
        
        if (dir == 22):
            if (steps[i]+1 == steps[i+1]):
                turns.append(1)
                dir = 11
                continue
            if (steps[i]-1 == steps[i+1]):
                turns.append(-1)
                dir = 12
                continue
            if (steps[i]+8 == steps[i+1]):
                turns.append(0)
                dir = 22
                continue
            if (steps[i]-8 == steps[i+1]):
                turns.append(2)
                dir = 21
                continue    

    return turns


steps = Qpath(43)
turns = directions(steps)
move_turtle(turns)
