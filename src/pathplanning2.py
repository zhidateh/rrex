#!/usr/bin/env python

"""

Path tracking simulation with LQR speed and steering control

author Atsushi Sakai (*Atsushi_twi)

"""
import rospy
from rrex.msg import FusedPose as FusedPose
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
from numpy.linalg import multi_dot
import pathtracking
import pigpio
	

# LQR parameter
Q = np.eye(5)
R = np.eye(2)

# parameters
dt = 0.1  # time tick[s]
L = 0.48  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

show_animation = True

state_x,state_y,state_hdg,state_vel = 0.0,0.0,0.0,0.0

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self):
            """
            Update the state of the vehicle.

            Stanley Control uses bicycle model.

            :param acceleration: (float) Acceleration
            :param delta: (float) Steering
            """

            self.x = state_x
            self.y = state_y
            self.yaw = state_hdg
            self.v = state_vel
            
            #delta = np.clip(delta, -max_steer, max_steer)

            #self.x += self.v * np.cos(self.yaw) * dt
            #self.y += self.v * np.sin(self.yaw) * dt
            #self.yaw += self.v / L * np.tan(delta) * dt
            #self.yaw = normalize_angle(self.yaw)
            #self.v += acceleration * dt



'''
def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state
'''

def pi_2_pi(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    Xn = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = multi_dot([(A.T) , X , A]) - multi_dot([( A.T) , X , B , la.inv(R + multi_dot([( B.T) , X , B])) , (B.T) , X , A]) + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = multi_dot([ la.inv(multi_dot([B.T , X , B]) + R)  \
                    , multi_dot([B.T , X , A]) ])

    eigs = la.eig(A -  multi_dot([ B , K ]))

    return K, X, eigs[0]


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = sp[ind]

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0
    # print(A)

    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((5, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    ustar = multi_dot([-K , x])

    # calc steering input

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi(ustar[0, 0])

    # calc accel input
    ai = ustar[1, 0]

    delta = ff + fb

    return delta, ind, e, th_e, ai


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):


    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile

def readWaypointTxt(path):
    with open(path,'r') as f :
        cx,cy,cyaw,ck,s = [],[],[],[],[]
        for row in f:
            #print row.split(',')[1]
            #print '\n'
            cx.append(float(row.split(',')[0]))
            cy.append(float(row.split(',')[1]))
            cyaw.append(float(row.split(',')[2]))
            ck.append(float(row.split(',')[3]))
            s.append(float(row.split(',')[4]))


    return cx,cy,cyaw,ck,s

def ppm_callback(data):
    global state_x,state_y,state_hdg,state_vel
    
    state_x = data.x
    state_y = data.y
    state_hdg = data.hdg
    state_vel = data.vel
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.y)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.hdg)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.vel)


def main():
    
    
    rospy.init_node('ppm_node', anonymous=True)
    
    rospy.Subscriber("/loc_node/fusedpose", FusedPose, ppm_callback)
    rate = rospy.Rate(20)


    pi = pigpio.pi()
    
    cx,cy,cyaw,ck,s = readWaypointTxt("/home/pi/catkin_ws/src/rrex/config/path.txt")
    goal = [cx[-1], cy[-1]]

    target_speed = 5.0 / 3.6  # simulation parameter km/h -> m/s

    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    T = 500.0  # max simulation time
    goal_dis = 0.15
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while not rospy.is_shutdown() and T >= time:
        dl, target_ind, e, e_th, ai = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, sp)
        
        state.update()
        #state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt
        rate.sleep()
        
        
        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.plot(cy, cx, "-r", label="course")
            plt.plot(y, x, "ob", label="trajectory")
            plt.plot(cy[target_ind], cx[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)
            
            
    if 0:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(cy, cx, "-r", label="target course")
        plt.plot(y, x, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("y[m]")
        plt.ylabel("x[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()


if __name__ == '__main__':
    main()

