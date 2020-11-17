'''
code on vision controlled robot arm using openCV for background subtraction and object location
written on the 13th of November 2020

'''
from imutils.video import VideoStream
import imutils
import numpy as np
import cv2
import math
import RPi.GPIO as GPIO
import time

#link lengths
L1=5.5
L2=7.5

#link angles
theta1=0
theta2=0
phi=0

#setting GPIO pins

IR_sensor = 4  # IR sensor to detect when an object enter the field of view 

#Stepper motor variables

#encoder for detecting stepper motor home position
elbow_encoder = 9
lower_arm_encoder = 5

# stepper Last postion variables
lift_height = 200
lift_stepper_LastPos = 0
elbow_stepper_LastPos = 0
lower_arm_stepper_LastPos = 0

#stepper direction and step pins
lift_stepper_dir_pin = 18
lift_stepper_step_pin = 19

elbow_stepper_dir_pin = 20
elbow_stepper_step_pin = 15

lower_arm_stepper_dir_pin = 16
lower_arm_stepper_step_pin = 21

#steps to move for each pos to move
stepsPerRevolution_elbow = 0
stepsPerRevolution_lower_arm = 0

# end effector motor variables
gripper_suction_a = 12
gripper_suction_b = 10

#setting up GPIO pins

GPIO.setmode(GPIO.BCM)
GPIO.setup(lift_stepper_dir_pin, GPIO.OUT)
GPIO.setup(lift_stepper_step_pin, GPIO.OUT)
GPIO.setup(elbow_stepper_dir_pin, GPIO.OUT)
GPIO.setup(elbow_stepper_step_pin, GPIO.OUT)
GPIO.setup(lower_arm_stepper_dir_pin, GPIO.OUT)
GPIO.setup(lower_arm_stepper_step_pin, GPIO.OUT)
GPIO.setup(gripper_suction_a, GPIO.OUT)
GPIO.setup(gripper_suction_b, GPIO.OUT)
GPIO.setup(elbow_encoder, GPIO.INPUT)
GPIO.setup(lower_arm_encoder, GPIO.INPUT)
GPIO.setup(IR_sensor, GPIO.INPUT)


# function to take the stepper motor to it's home position

def take_stepper_to_home():
    
    if GPIO.input(elbow_encoder) == 1:
        elbow_stepper_LastPos = 90
        
    else:
        
        GPIO.output(elbow_stepper_dir_pin, True)
        
        for angle in range(90):
            
            GPIO.output(elbow_stepper_step_pin, True)
            time.sleep(3)
            GPIO.output(elbow_stepper_step_pin, False)
            time.sleep(3)
    
    if GPIO.input(lower_arm_encoder) == 1:
        
        lower_arm_stepper_LastPos = 90
        
        	#break
    
    else:
        
        GPIO.output(lower_arm_stepper_dir_pin, True)
        for angle in range(90):
            
            GPIO.output(lower_arm_stepper_step_pin, True)
            time.sleep(3)
            GPIO.output(lower_arm_stepper_step_pin, False)
            time.sleep(3)
            
        
        	#break

def inverseKinematics(x,y):

	try:
        
		theta2 =math.acos(((x**2) + (y**2)-(L1**2)-(L2**2)) / (2 * L1 * L2))
		
		if x < 0 and y < 0:

			theta2 = (-1) * theta2
		    	theta1 = math.atan(x/y) - math.atan((L2 * math.sin(theta2)) / (L1 + L2 * math.cos(theta2)))
		    	theta2 = theta2 * 180/math.pi
		    	theta1 = theta1 * 180/math.pi
				# Angles adjustment depending in which quadrant the final tool coordinate x,y is
		if x >= 0 and y >= 0:     # 1st quadrant
		    	theta1 = 90 -theta1


		if x < 0 and y > 0:       # 2nd quadrant
		    	theta1 = 90 - theta1


		if x < 0 and y < 0:       # 3rd quadrant
		    	theta1 = 270 - theta1
		    	phi = 270 - theta1 - theta2
		    	phi = (-1) * phi
		    		
		if x > 0 and y < 0:       # 4th quadrant
		    	theta1 = -90 - theta1

		if x < 0 and y == 0:
		    	theta1 = 270 + theta1
		        
			# Calculate "phi" angle so gripper is parallel to the X axis

		phi = 90 + theta1 + theta2

		phi = (-1) * phi

			# Angles adjustments depending in which quadrant the final tool coordinate x,y is
		if x < 0 and y < 0:
		    	phi = 270 - theta1 - theta2

		if abs(phi) > 165:
		    	phi = 180 + phi

		theta1 = math.ceil(theta1)
		print("theta1: ",theta1)
		theta2 = math.ceil(theta2)
		print("theta2: ",theta2)
		phi = math.ceil(phi)
		print("phi: ",phi)
		
		stepsPerRevolution_elbow = theta1
		stepsPerRevolution_lower_arm = theta2
		
			# driving arm up the rail
			
		GPIO.output(lift_stepper_dir_pin, False)
		for angle in range(lift_height):
		    	GPIO.output(lift_stepper_step_pin, True)
		    	time.sleep(3)
		    	GPIO.output(lift_stepper_step_pin, False)
		    	time.sleep(3)
		lift_stepper_LastPos = lift_height

		# positioning the elbow link
		
		if stepsPerRevolution_elbow <= 90:
		    	GPIO.output(elbow_stepper_dir_pin, True)
		    	for angle in range(stepsPerRevolution_elbow):
		        	GPIO.output(elbow_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(elbow_stepper_step_pin, False)
		        	time.sleep(3)
		    	elbow_stepper_LastPos = stepsPerRevolution_elbow
			#break
			
		 elif stepsPerRevolution_elbow > 90:
		    	GPIO.output(elbow_stepper_dir_pin, False)
		    	for angle in range(stepsPerRevolution_elbow - 90):
		        	GPIO.output(elbow_stepper_step_pin, True)
		        	time.sleep(3)
				GPIO.output(elbow_stepper_step_pin, False)
				time.sleep(3)
		    	elbow_stepper_LastPos = stepsPerRevolution_elbow
				#break
			
		
		# positioning the lower arm link for picking

		if stepsPerRevolution_lower_arm <= 90:
		    	GPIO.output(lower_arm_stepper_dir_pin, True)
		    	for angle in range(stepsPerRevolution_lower_arm):
		        	GPIO.output(lower_arm_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(lower_arm_stepper_step_pin, False)
		        	time.sleep(3)
		    	lower_arm_stepper_LastPos = stepsPerRevolution_lower_arm
				#break

		elif stepsPerRevolution_elbow > 90:
		    	GPIO.output(elbow_stepper_dir_pin, False)
		    	for angle in range(stepsPerRevolution_elbow):
		        	GPIO.output(lower_arm_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(lower_arm_stepper_step_pin, False)
		        	time.sleep(3)
		    	lower_arm_stepper_LastPos = stepsPerRevolution_lower_arm
				#break        
			
			# activating gripper

		GPIO.output(gripper_suction_a, True)
		GPIO.output(gripper_suction_a, False)

			##########################################################
			#                                                        #
			# driving down the guide rails                           #
			#                                                        #
			##########################################################

			# driving arm up the rail to lift off the object

		GPIO.output(lift_stepper_dir_pin, True)
		for angle in range(lift_height):
		    	GPIO.output(lift_stepper_step_pin, True)
		    	time.sleep(3)
		    	GPIO.output(lift_stepper_step_pin, False)
		  	time.sleep(3)
		lift_stepper_LastState = lift_height    # recording last height of the assembly

		if elbow_stepper_LastState <= 90:
		    	GPIO.output(elbow_stepper_dir_pin, True)
		    	for angle in range(elbow_stepper_LastState + 100):
		        	GPIO.output(elbow_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(elbow_stepper_step_pin, False)
		        	time.sleep(3)
		    	elbow_stepper_LastState = elbow_stepper_LastState + 100
		
		elif elbow_stepper_LastState > 90:
		    	GPIO.output(elbow_stepper_dir_pin, False)
		    	for angle in range(elbow_stepper_LastState - 100):
		        	GPIO.output(elbow_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(elbow_stepper_step_pin, False)
		        	time.sleep(3)
		    	elbow_stepper_LastState = elbow_stepper_LastState - 100

			# positioning lower arm link for dropping object

		if lower_arm_stepper_LastState <= 90:

		    	GPIO.output(lower_arm_stepper_dir_pin, True)
		    	for angle in range(lower_arm_stepper_LastState + 50):
		        	GPIO.output(lower_arm_stepper_step_pin, True)
		        	time.sleep(3)
		        	GPIO.output(lower_arm_stepper_LastState, False)
		        	time.sleep(3)
		    	lower_arm_stepper_LastState = lower_arm_stepper_LastState + 50
		    		
		elif lower_arm_stepper_LastState > 90:
			GPIO.output(lower_arm_stepper_dir_pin, False)
			for angle in range(lower_arm_stepper_LastState - 50):
		            	GPIO.output(lower_arm_stepper_step_pin, True)
		            	time.sleep(3)
		            	GPIO.output(lower_arm_stepper_step_pin, False)
		            	time.sleep(3)
		    	lower_arm_stepper_LastState = lower_arm_stepper_LastState - 50

		# driving arm down the rail to drop

		GPIO.output(lift_stepper_dir_pin, False)
		for angle in range(lift_height):
		    	GPIO.output(lift_stepper_step_pin, True)
		    	time.sleep(3)
		    	GPIO.output(lift_stepper_step_pin, False)
		    	time.sleep(3)
		    
		lift_stepper_LastState = lift_height

			# deactivating gripper for releasing the object

		GPIO.output(gripper_suction_a, True)
		GPIO.output(gripper_suction_a, False)

			# driving arm again up the rail to go to home

		GPIO.output(lift_stepper_dir_pin, True)
		for angle in range(lift_height):
		    	GPIO.output(lift_stepper_step_pin, True)
		    	time.sleep(3)
		    	GPIO.output(lift_stepper_step_pin, False)
		    	time.sleep(3)
		    
		lift_stepper_LastState = lift_height

			# returning the robot arm to home state

		take_stepper_to_home()
	except:

		print("no new object in view")


cap = VideoStream(src=0).start()   #start webcam for frame capturing

cm_to_pixel = 11.3 /640.0

while True:

	frame=cap.read()

    	gray_image1=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    	cv2.imshow("background",gray_image1)
    
    	k=cv2.waitKey(1)      #waits 1sec to check for key press
	
	print("capturing background, do not move the camera....")
	
	time.sleep(30)
	
	break

    	#if k==27:             #27 is the keyboard number for the escape key

        	#break

    	while True:

		if GPIO.input(IR_sensor) == 1:  # if the sensor picks up an object, it captures the frame and gets the difference

			ret,frame=cap.read()

			gray_image2=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	    
			cv2.imshow("foreground",gray_image2)

			difference=np.absolute(np.array(np.int16(gray_image1)-np.int16(gray_image2)))

			difference[difference>255]=255

			difference=np.uint8(difference)

			cv2.imshow('difference',difference)

			BW=difference

			BW[BW<=100]=0

			BW[BW>100]=1

			#calculating the x coordinate

			column_sums = np.matrix(np.sum(BW,0))

			column_numbers = np.matrix(np.arange(640))

			column_mult = np.multiply(column_sums,column_numbers)

			total = np.sum(column_mult)

			total_total = np.sum(np.sum(BW))

			column_location = total/total_total

			x_location = column_location * cm_to_pixel
			#print(x_location)

			#calculating the y coordinate

			row_sums = np.matrix(np.sum(BW,1))

			rows_sums = row_sums.transpose()

			row_numbers = np.matrix(np.arange(480))

			row_mult = np.multiply(row_sums,row_numbers)

			total = np.sum(row_mult)

			total_total = np.sum(np.sum(BW))

			row_location = total/total_total

			y_location = row_location * cm_to_pixel
		
			print(x_location, y_location)

			inverseKinematics(x_location, y_location)

			k=cv2.waitKey(1)      #waits 1sec to check for key press

		if k == 27:             #27 is the keyboard number for the escape key

		    break

cap.release()

cv2.destroyAllWindows()
