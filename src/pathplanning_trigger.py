#!/usr/bin/env python

"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import rospy
from rrex.msg import FusedPose as FusedPose
import numpy as np
import matplotlib.pyplot as plt
import sys
import math
import pathtracking
import pigpio
import time	
    

fps= 30.0
k = 0.0  # control gain
#Kp = 45.0  # speed propotional gain
dt = 0.1  # [s] time difference
L = 0.48  # [m] Wheel base of vehicle
max_steer = np.radians(45.0)  # [rad] max steering angle

show_animation = True

state_x,state_y,state_hdg,state_vel,left_wheel_vel,right_wheel_vel = 0.0,0.0,0.0,0.0,0.0,0.0

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    #def update(self, acceleration, delta):
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


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d
    
    #clip
    delta = np.clip(delta, -max_steer, max_steer)

    return delta, current_target_idx


def normalize_angle(angle):
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


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
   # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    closest_error = min(d)
    target_idx = d.index(closest_error)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      - np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


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
    global state_x,state_y,state_hdg,state_vel, left_wheel_vel, right_wheel_vel
    
    state_x         = data.x
    state_y         = data.y
    state_hdg       = data.hdg
    state_vel       = data.vel
    #state_x += state_vel * math.cos(data.hdg)*(1.0/fps)
    #state_y += state_vel * math.sin(data.hdg)*(1.0/fps)
    
    left_wheel_vel  = data.vel_l
    right_wheel_vel = data.vel_r

    #print normalize_angle((left_wheel_vel - right_wheel_vel) *fps/0.48)


class PathPlanning:
    def __init__(self):
        print "Initialising ..."

        rospy.init_node('ppm_node', anonymous=True)
        
        rospy.Subscriber("/loc_node/fusedpose", FusedPose, ppm_callback)

        self.rate = rospy.Rate(fps)


        self.pi = pigpio.pi()
        
        #Initialise PFM
        #25 ,9
        #R: reverse if < 1440 , forward if >1580 
        #L: reverse if > 1580 , forward if <1420 
        #left pin, right pin, frequency, L_command , R_command
        self.servoControl = pathtracking.ServoControl(self.pi,25,9,500,1500,1510)

        #  target course
        self.cx,self.cy,self.cyaw,self.ck,self.s = readWaypointTxt("/home/pi/catkin_ws/src/rrex/config/path.txt")

        self.target_speed = 0.5   # [m/s]

        # Initial state
        self.state = State(x=-0.0, y=0.0, yaw=np.radians(0.0), v=0.0)

        self.last_idx = len(self.cx) - 1
        self.x = [self.state.x]
        self.y = [self.state.y]
        self.yaw = [self.state.yaw]
        self.v = [self.state.v]
        self.t = [0.0]
        self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
        self.last_dist = float('Inf')

        self.start_time = time.time()

        self.interupt = False
        print "Starting ..."
        print "Waiting for trigger..."
        
        self.v_r = [0.0]
        self.v_l = [0.0]
        # and max_simulation_time >= time
    
    
    def trigger(self):
        print "Triggered! "
        while not rospy.is_shutdown() and self.last_idx >= self.target_idx:
            try:        
                self.state.update()

                self.di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)

                self.servoControl.pid_control(self.target_speed,left_wheel_vel,right_wheel_vel,self.di)
                
                #servoControl.L_command -= pid_control(target_speed, left_wheel_vel)
                #servoControl.R_command += pid_control(target_speed, right_wheel_vel)
                
                self.servoControl.run(True,fps)

                #print left_wheel_vel, "\t",  right_wheel_vel, " , " , di
                #print state.yaw , state_hdg
                #time += dt
                #print (ai)
                
                dist = math.hypot(self.state.x - self.cx[-1],self.state.y - self.cy[-1])
                if( dist <= self.last_dist):
                    self.last_dist = dist
                else: 
                    break

                if(self.interupt == True): 
                    break


                self.x.append(self.state.x)
                self.y.append(self.state.y)
                self.yaw.append(self.state.yaw)
                self.v.append(self.state.v)
                self.t.append(time.time() - self.start_time)

                self.v_r.append(right_wheel_vel)
                self.v_l.append(left_wheel_vel)
                
                #print right_wheel_vel, v_r[-1]
                self.rate.sleep()

                if 0:  # pragma: no cover
                    plt.cla()
                    plt.plot(cy, cx, ".r")
                    plt.plot(y, x, "-b")
                    plt.plot(cy[target_idx], cx[target_idx], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                    plt.pause(0.001)
                    
            except KeyboardInterrupt:
                self.interupt = True
                print "\n............"
                print("Interrupted")
                print "............"
                self.servoControl.run(False,30)
                time.sleep(0.05)
                self.servoControl.pwm.cancel()
                self.servoControl.pi.stop()
                self.pi.stop()

    
        if not self.interupt:

            self.servoControl.run(False,30)

            #STWcontrol.run(0,0,0,0)
            time.sleep(0.05)
            self.servoControl.pwm.cancel()
            self.servoControl.pi.stop()
            self.pi.stop()
        
        
        # Test
        assert self.last_idx >= self.target_idx, "Cannot reach goal"


    def plot(self):
        plt.subplot(1,1,1)
        plt.plot(self.cy, self.cx, ".r", label="course")
        plt.plot(self.y, self.x, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("y[m]")
        plt.ylabel("x[m]")
        plt.axis("equal")
        plt.grid(True)

        #plt.subplots(1)
        #plt.plot(t, [iv * 3.6 for iv in v], "-r")
        #plt.plot(t,v_r, "-r")
        #plt.plot(t,v_l, "-g")
        #plt.plot(t,v, "-k")
        #plt.plot([t[0],t[-1]], [target_speed,target_speed], "-b")
        #plt.xlabel("Time[s]")
        #plt.ylabel("Speed[km/h]")
        #plt.grid(True)

        #plt.subplots(1)
        #plt.plot(t, [ math.atan2(iy,ix) for ix,iy in zip(x,y)], "-r")
        #plt.plot(self.t,self.yaw, "-r")
        plt.subplot(3,1,1)
        plt.plot(self.t,self.v_r, "-r")
        plt.plot(self.t,self.v_l, "-g")
        plt.plot(self.t,self.v, "-k")
        plt.plot([self.t[0],self.t[-1]], [self.target_speed,self.target_speed], "-b")
        
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[ms-1]")
        plt.subplot(3,1,2)
        plt.plot(self.t,self.yaw)
        
        plt.xlabel("Time[s]")
        plt.ylabel("Yaw[rad]")
        plt.subplot(3,1,3)
        plt.plot(self.cy, self.cx, ".r", label="course")
        plt.plot(self.y, self.x, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("y[m]")
        plt.ylabel("x[m]")
        plt.axis("equal")
        
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    #main()
    pathPlanning = PathPlanning()
    pathPlanning.trigger()
    pathPlanning.plot()
