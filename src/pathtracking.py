import servo_interface
import time
import math
import numpy as np 

Kp_l = 4.0  # speed propotional gain
Kp_r = 5.0

#_time = time.tim()

class ServoControl:
	
	def __init__(self,pi,L_GPIO,R_GPIO,frequency,L_command, R_command):
		self.pi = pi
		self.Left_servo = L_GPIO
		self.Right_servo = R_GPIO
		self.L_command = L_command
		self.R_command = R_command
		self.seq =0
		self.start = time.time()
		
		self.pwm = servo_interface.PWM(pi)
		if frequency % 400:
			self.pwm.set_frequency(frequency)
		else:
			self.pwm.set_cycle_time(1000000.0/frequency)
		
		

	def run(self, start,frequency):
		self.seq  += 1
		if(start == False):
			self.L_command = 0
			self.R_command = 0
		
		
		if(self.seq%2):
			
			self.pwm.set_pulse_start_in_fraction(self.Left_servo, 0)
			self.pwm.set_pulse_start_in_fraction(self.Right_servo, 0)

			self.pwm.set_pulse_length_in_micros(self.Left_servo, self.L_command)
			self.pwm.set_pulse_length_in_micros(self.Right_servo, self.R_command)
		else:

			self.pwm.set_pulse_start_in_fraction(self.Right_servo, 0)
			self.pwm.set_pulse_start_in_fraction(self.Left_servo, 0)
		
			self.pwm.set_pulse_length_in_micros(self.Right_servo, self.R_command)
			self.pwm.set_pulse_length_in_micros(self.Left_servo, self.L_command)	


		self.pwm.update()
		
		time.sleep(1.0/frequency)
		

	def pid_control(self,target_speed, Left_vel, Right_vel, di):
		#di = 0.0
		#v = 0.5 (left _vel + right _vel )
		#yaw = 0.12 * dt * (left _vel - right _vel )
		dt = time.time() - self.start 
		self.start = time.time()
		L_target_speed = target_speed + di * 0.24 / dt
		R_target_speed = target_speed - di * 0.24 / dt


		L_target_speed = np.clip(L_target_speed, 0.0 , target_speed*1.2)
		R_target_speed = np.clip(R_target_speed, 0.0 , target_speed*1.2)
		
		#print L_target_speed, R_target_speed, target_speed, di

		#L_target_speed = target_speed
		#R_target_speed = target_speed
		

		
		
		self.L_command -= Kp_l * (L_target_speed - Left_vel)
		self.R_command += Kp_r * (R_target_speed - Right_vel)

		#self.L_command = max(

		#self.L_command = np.clip(self.L_command, 1300 , 1700)
		#self.R_command = np.clip(self.R_command, 1300 , 1700)

		print self.L_command,self.R_command
		
		#print target_speed, dt,di
		#print "L target speed: " , L_target_speed 
		#print "R target speed: " , R_target_speed 
		#print "di    		 : " , self.translate(Right_vel,0,0,0,10) 
		#print " ----------------------------------- "
		
		

		
	def translate(self, value, leftMin, leftMax, rightMin, rightMax):
		# Figure out how 'wide' each range is
		leftSpan = leftMax - leftMin
		rightSpan = rightMax - rightMin
	
		if (leftSpan == 0):
			valueScaled = 0
		else:
			# Convert the left range into a 0-1 range (float)
			valueScaled = float(leftMax - value) / float(leftSpan)


		# Convert the 0-1 range into a value in the right range.
		return rightMin + (valueScaled * rightSpan)
		
		
if __name__ == "__main__": 
	import pigpio
	import pathtracking
	import sys
	
	pi = pigpio.pi()
	
	#25 ,9
	#left pin, right pin, frequency
	servoControl = pathtracking.ServoControl(pi,25,9,500)
	#print STWcontrol.pwm.get_frequency()
	
	start = time.time()
	while(time.time() - start < 3):
		try:
				
			#left start, left length, right start, right length
			#L:1468 -- start forwarding 1536 ++ start reversing /////////// midpoint 1502
			#R:1544 ++ start forwarding 1476 -- start reversing /////////// midpoint 1510
			
			servoControl.run(0,1536,0,1476)

		except KeyboardInterrupt:
			print "\n............"
			print("Interrupted")
			print "............"
			servoControl.run(0,0,0,0)
			servoControl.pwm.cancel()
			time.sleep(0.05)
			STWcontrol.pi.stop()
			pi.stop()
			sys.exit()
			
	print "\n............"
	print("Interrupted")
	print "............"

	servoControl.run(0,0,0,0)
	time.sleep(0.05)
	servoControl.pwm.cancel()
	servoControl.pi.stop()
	pi.stop()
	sys.exit()

