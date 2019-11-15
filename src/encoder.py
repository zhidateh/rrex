#!/usr/bin/env python 
# license removed for brevity

import rospy 
from rrex.msg import Encoder as Encoder
import encoder_interface
import time
import sys 
import os
import math

class DeadReckoningPosition:
	def __init__(self,pi,L_GPIOA,L_GPIOB,R_GPIOA,R_GPIOB):
		self.state_x =0
		self.state_y = 0
		self.heading = 0
		self.yaw = 0
		
		self.Left_vel =0
		self.Right_vel =0
		self.vel = 0
		
		self.Left_ds =0
		self.Right_ds =0
		
		self.Left_encoder_pinA = L_GPIOA
		self.Left_encoder_pinB = L_GPIOB

		self.Right_encoder_pinA = R_GPIOA
		self.Right_encoder_pinB = R_GPIOB

		self.radius = 0.1016/2
		self.speedratio = 0.76518
		self.tick2theta = 0.3055
		#self.tick2theta = 0.3550


		self.Left_ds = 0
		self.Right_ds = 0

		self.Left_pos = 0
		self.Left_lastpos = 0

		self.Right_pos = 0
		self.Right_lastpos = 0

		self.Left_encoderValue =0
		self.Left_lastencoderValue = 0

		self.start = time.time()
		self.start2 = time.time()
		self.pi = pi
		self.Left_decoder = encoder_interface.decoder(self.pi, self.Left_encoder_pinA , self.Left_encoder_pinB, self.Left_callback)
		self.Right_decoder = encoder_interface.decoder(self.pi, self.Right_encoder_pinA , self.Right_encoder_pinB, self.Right_callback)
		
		

	def run(self):

		self.Left_dtheta = (self.tick2theta*math.pi/180) * (self.Left_pos - self.Left_lastpos) 
		self.Right_dtheta = (self.tick2theta*math.pi/180) * (self.Right_pos - self.Right_lastpos) 

		dt = time.time() - self.start

		self.Left_rpm = self.speedratio *self.Left_dtheta / dt
		self.Left_vel = self.Left_rpm * self.radius
		
		self.Right_rpm = self.speedratio* self.Right_dtheta / dt
		self.Right_vel = self.Right_rpm * self.radius
		
		self.start = time.time()

		
		self.Left_ds += self.Left_dtheta * self.radius;
		self.Right_ds += self.Right_dtheta * self.radius;
		
		self.Left_lastpos, self.Right_lastpos = self.Left_pos, self.Right_pos

		self.getDRPosition()


	def Left_callback(self,way):

		self.Left_pos += way
		

	def Right_callback(self,way):
		
		self.Right_pos -= way
		

	def getDRPosition(self):
		dyaw= 0
		self.vel = 0.5*(self.Left_vel + self.Right_vel)
		
		#robot wheel separation = 0.48m
		dt = time.time() - self.start2
		dyaw = (self.Left_vel-self.Right_vel) *dt / 0.48
		self.start2 = time.time() 
		
		dx = self.vel*math.cos(dyaw)*dt
		dy = self.vel*math.sin(dyaw)*dt		
		
		dx,dy = self.rotationMatrix(dx,dy, self.heading)
		
		
		if(self.vel < 0):
			dy *= -1
		
		self.state_x += dx
		self.state_y += dy
		self.heading += dyaw
		self.yaw = dyaw
				
		#Normalize an angle to heading range [-pi, pi].
		if self.heading > math.pi:
			self.heading -= 2.0 * math.pi
		elif self.heading < -math.pi:
			self.heading += 2.0 * math.pi

			
			
	def rotationMatrix(self,x,y,angle):
		_x = x*math.cos(angle) - y*math.sin(angle)
		_y = x*math.sin(angle) + y*math.cos(angle)
		return _x,_y


def talker(DRposition):
	pub = rospy.Publisher('/encoder_node/encoder',Encoder, queue_size = 10)
	rospy.init_node('encoder',anonymous = True)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		pub.publish(DRposition.state_x,\
					DRposition.state_y,\
					DRposition.vel,\
					DRposition.heading,\
					DRposition.Left_vel,\
					DRposition.Right_vel)
			#20Hz
		DRposition.run()	
		#rospy.loginfo("State X  [m]:\t\t%.5f" % DRposition.state_x)
		#rospy.loginfo("State Y  [m]:\t\t%.5f" % DRposition.state_y)
		#rospy.loginfo("Heading  [m]:\t\t%.5f" % DRposition.heading)
		rospy.loginfo("Velocity [m/s]:\t\t%.5f" % DRposition.vel)
		rospy.loginfo("Left Velocity [m/s]:\t\t%.5f" % DRposition.Left_vel)
		rospy.loginfo("Right Velocity [m/s]:\t\t%.5f" % DRposition.Right_vel)
		
		
		#rospy.loginfo("Left rpm [m/s]:\t\t%.5f" % DRposition.Left_rpm)
		#rospy.loginfo("Right rpm [m/s]:\t\t%.5f" % DRposition.Right_rpm)
		rospy.loginfo("-----------------------------------------------")
		rospy.loginfo( "")
		rate.sleep()
				

		
if __name__ == '__main__':
	try: 
		import encoder
		import pigpio
	
		pi = pigpio.pi()
		DRposition = encoder.DeadReckoningPosition(pi,22,23,17,18)
		
		talker(DRposition)
		
	except rospy.ROSInterruptException:
		DRposition.Left_decoder.cancel()
		DRposition.Right_decoder.cancel()
		DRposition.pi.stop()
		pi.stop()
		pass 

