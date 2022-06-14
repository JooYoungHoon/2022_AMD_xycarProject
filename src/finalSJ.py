#####################
# 2017430020 2017430039 신승철 주영훈
# 6월 13일
# 신호등 정보 완료, 차선 주행은 일단 됨, 장애물 해야함
#############################################

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int64, Int32
import time
import sys
import os
import signal

# 변수 설정
image = np.empty(shape=[0])
bridge = CvBridge()
distance = []
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
start_time = 0.0
check = 0
index = 0 
# ar viewer
arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0
ar_viewer_id = -1
leftColor = None
rightColor = None
timeCount = None

def signal_handler(sig, frame):
	os.system('killall -9 python rosout')
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 콜백 함수
def img_callback(data):
	global image
	image = bridge.imgmsg_to_cv2(data, "bgr8")  

def lidar_callback(data):
	global distance
	distance = data.ranges

def ar_viewer_callback(data):
	global arData, ar_viewer_id, roll, pitch, yaw
	for i in data.markers:
		arData["DX"] = i.pose.pose.position.x
		arData["DY"] = i.pose.pose.position.y
		arData["DZ"] = i.pose.pose.position.z

		arData["AX"] = i.pose.pose.orientation.x
		arData["AY"] = i.pose.pose.orientation.y
		arData["AZ"] = i.pose.pose.orientation.z
		arData["AW"] = i.pose.pose.orientation.w
		(roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

		ar_viewer_id = i.id

def lTrafficLightCallback(data):
	global leftColor
	leftColor = data.data

def rTrafficLightCallback(data):
	global rightColor
	rightColor = data.data

def trafficLightCountCallback(data):
	global timeCount
	timeCount = data.data

# PID 제어
def PID(input_data, kp, ki, kd):
	global start_time, end_time, prev_error, i_error
	end_time = time.time()
	dt = end_time - start_time
	start_time = end_time

	error = 320 - input_data
	derror =  error - prev_error

	p_error = kp * error
	i_error = i_error + ki * error * dt
	d_error = kd * derror / dt

	output = p_error + i_error + d_error
	prev_error = error

	if output > 50:
		output = 50
	elif output < -50:
		output = -50

	return -output

# 장애물 회피 구현
def scan_obstacles():
	global check, distance
	# if check < 1:
	okL = 0
	okR = 0
	for degree in range(120, 170):	        
		if distance[degree] <= 0.4:
			okL += 1
			print("lidar Lcalled")
	for degree in range(190, 240):
		if distance[degree] <=0.4:
			okR += 1
			print("lidar Rcalled")

	if okL > 2:
		check += 1
		print("obstacles left")
		drive(50, 5)
		time.sleep(1)
		drive(-30, 5)
		time.sleep(0.5)
		
	elif okR > 2:
		check += 1
		print("obstacles right")
		drive(-50, 3)
		time.sleep(1.0)
		drive(30, 5)
		time.sleep(0.5)
		drive(0, 4)

# 주차 함수
def parking():
	global pub, motor_msg

	motor_msg = xycar_motor()

	for i in range(12):
		motor_msg.angle = -50
		motor_msg.speed = -3
		pub.publish(motor_msg)
		time.sleep(0.1)
	for i in range(8):
		motor_msg.angle = 30
		motor_msg.speed = -3
		pub.publish(motor_msg)
		time.sleep(0.1)
	for i in range(5):
		motor_msg.angle = -40
		motor_msg.speed = 3
		pub.publish(motor_msg)
		time.sleep(0.1)
	for i in range(5):
		motor_msg.angle = 40
		motor_msg.speed = 3
		pub.publish(motor_msg)
		time.sleep(0.1)

# publish xycar_motor msg
def drive(Angle, Speed): 
	global pub

	msg = xycar_motor()
	msg.angle = Angle
	msg.speed = Speed

	pub.publish(msg)

# draw lines
def draw_lines(img, lines):
	global Offset
	for line in lines:
		x1, y1, x2, y2 = line[0]
		color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
		img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
	return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
	center = (lpos + rpos) / 2

	cv2.rectangle(img, (lpos - 5, 15 + offset),
		(lpos + 5, 25 + offset),
		(0, 255, 0), 2)
	cv2.rectangle(img, (rpos - 5, 15 + offset),
		(rpos + 5, 25 + offset),
		(0, 255, 0), 2)
	cv2.rectangle(img, (center-5, 15 + offset),
		(center+5, 25 + offset),
		(0, 255, 0), 2)    
	cv2.rectangle(img, (315, 15 + offset),
		(325, 25 + offset),
		(0, 0, 255), 2)
	return img

# left lines, right lines
def divide_left_right(lines):
	global Width

	low_slope_threshold = 0
	high_slope_threshold = 10

	# calculate slope & filtering with threshold
	slopes = []
	new_lines = []

	for line in lines:
		x1, y1, x2, y2 = line[0]

	if x2 - x1 == 0:
		slope = 0
	else:
		slope = float(y2-y1) / float(x2-x1)

	if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
		slopes.append(slope)
		new_lines.append(line[0])

	# divide lines left to right
	left_lines = []
	right_lines = []

	for j in range(len(slopes)):
		Line = new_lines[j]
		slope = slopes[j]

		x1, y1, x2, y2 = Line

		if (slope < 0) and (x2 < Width/2 - 10):
		left_lines.append([Line.tolist()])
		elif (slope > 0) and (x1 > Width/2 + 10):
		right_lines.append([Line.tolist()])

	return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
	# sum of x, y, m
	x_sum = 0.0
	y_sum = 0.0
	m_sum = 0.0

	size = len(lines)
	if size == 0:
		return 0, 0

	for line in lines:
		x1, y1, x2, y2 = line[0]

		x_sum += x1 + x2
		y_sum += y1 + y2
		m_sum += float(y2 - y1) / float(x2 - x1)

	x_avg = x_sum / (size * 2)
	y_avg = y_sum / (size * 2)
	m = m_sum / size
	b = y_avg - m * x_avg

	return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
	global Width, Height
	global Offset, Gap

	m, b = get_line_params(lines)

	if m == 0 and b == 0:
		if left:
			pos = 0
		if right:
			pos = Width
	else:
		y = Gap / 2
		pos = (y - b) / m

		b += Offset
		x1 = (Height - b) / float(m)
		x2 = ((Height/2) - b) / float(m)

		cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

	return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
	global Width
	global Offset, Gap

# gray
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

	# blur
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

	# canny edge
	low_threshold = 60
	high_threshold = 70
	edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

	# HoughLinesP
	roi = edge_img[Offset : Offset+Gap, 0 : Width]
	all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

	# divide left, right lines
	if all_lines is None:
		return 0, 640
	left_lines, right_lines = divide_left_right(all_lines)

	# get center of lines
	frame, lpos = get_line_pos(frame, left_lines, left=True)
	frame, rpos = get_line_pos(frame, right_lines, right=True)

	# draw lines
	frame = draw_lines(frame, left_lines)
	frame = draw_lines(frame, right_lines)
	frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
				
	# draw rectangle
	frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

	# show image
	cv2.imshow('calibration', frame)

	return lpos, rpos

def start():
	global pub
	global image, distance, check, index
	global leftColor, rightColor, timeCount
	global ar_viewer_id, roll, pitch, yaw, arData
	global cap
	global Width, Height

	rospy.init_node('auto_drive')
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

	lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
	arviewer_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_viewer_callback, queue_size = 1)
	lTrafficLightSubscriber = rospy.Subscriber('Left_color', String, lTrafficLightCallback, queue_size = 1)
	rTrafficLightSubscriber = rospy.Subscriber('Right_color', String, rTrafficLightCallback, queue_size = 1)
	trafficLightCountSubscriber = rospy.Subscriber('time_count', Int64, trafficLightCountCallback, queue_size = 1)
	print "---------- Xycar A2 v1.0 ----------"

	rospy.sleep(3)

	while True:
		while not image.size == (640*480*3):
			continue
		print("check"+str(check) "index"+str(index))
		print("AR id: "+str(ar_viewer_id))
		print("Roll: "+str(roll))
		print("Pitch: "+str(pitch))
		print("Yaw: "+str(yaw))
		print(str(arData))
		print("\n")
		print(str(leftColor)+str(rightColor)+str(timeCount))

		lpos, rpos = process_image(image)

		if lpos == 0:
			angle = -50
			speed = 4
		elif rpos == 640:
			angle = 50
			speed = 4
		else:
			center = (lpos + rpos) / 2
			angle = PID(center, 0.38, 0.0005, 0.15)
			speed = 5

		if ar_viewer_id not in range(0, 7):
			print("just drive")
			drive(angle, speed)

		if check <2:
			print("Wow OB")
			scan_obstacles()

		if ar_viewer_id == 0 and index == 0 and arData["DZ"] <= 0.5:
			t = time.time()
			current_time = time.time()-t
			while current_time <= 10:
				lpos, rpos = process_image(image)
				center = (lpos + rpos) / 2
				angle = PID(center, 0.38, 0.0005, 0.15)
				speed = 2		
				drive(angle, speed)
				current_time = time.time() - t
				print(current_time)
			index = 1
			ar_viewer_id = -1

		elif 

		elif ar_viewer_id == 2 and arData["DZ"] <= 0.5 and index == 1:
			if 
			if lpos == 0:
				drive(-50, 4)

			elif rpos == 640:
				drive(50, 4)
			else:
				center = (lpos + rpos) / 2
				angle = PID(center, 0.38, 0.0005, 0.15)
				speed = 4
				drive(angle, speed)
			

		elif ar_viewer_id == 4 and arData["DZ"] <= 0.5 index:
			

		elif ar_viewer_id == 6:
			drive(0, 0)
			#parking()


		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	rospy.spin()

if __name__ == '__main__':

	i_error = 0.0
	prev_error = 0.0
	start_time = time.time()

	start()


