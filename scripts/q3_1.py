#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import math

#Definições Globais 

#TARGETs [X,Y]
target0 = np.array([-5.0,  -12.0])
target1 = np.array([-1.0,  -9.0]) 
target2 = np.array([2.5,  -7.0]) 
target3 = np.array([1.0,  0.0]) 
target4 = np.array([-3.5,  -0.5]) 
target5 = np.array([-4.5,  6.0])
target6 = np.array([-0.50,  10.0])
target7 = np.array([7.5,  12.0])
gx = target0[0]
gy = target0[1]

# Distância mínima para chegar ao alvo
min_distance = 0.5

#Funções

odometry_msg = Odometry()
velocity = Twist()

def odometry_callback(data):
	global odometry_msg
	odometry_msg = data

def para_andar():
	velocity.linear.x = 0.0
	pub.publish(velocity) 
	
def move_frente():
	velocity.linear.x = 0.5
	pub.publish(velocity)  

def para_rodar():
	velocity.angular.z = 0.0
	pub.publish(velocity) 

def roda(velocidade):
	velocity.angular.z = velocidade
	pub.publish(velocity)

def localizacao():
	global px,py
	px = odometry_msg.pose.pose.position.x
	py = odometry_msg.pose.pose.position.y

#Função Principal

if __name__ == "__main__": 
	# Node
	rospy.init_node("controle_stage_node", anonymous=False)  

	# Subscribers
	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	# Publishers
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub_pos = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
  
	rate = rospy.Rate(10) #10hz

	while not rospy.is_shutdown():

		localizacao()

		move_frente()
		
		distance = math.sqrt((px-gx)**2 + (py-gy)**2)   

		print("=================================")
		print("Alvo: (%.2f, %.2f)" % (gx, gy))
		print("Posição: (%.2f, %.2f)" % (px, py)) 
		print("Distância: %.2f" % distance)
		print("=================================")

		if((distance < 0.5) and (gx == target0[0]) and (gy == target0[1])):
			roda(0.1)
			gx = target1[0]
			gy = target1[1]

		elif((distance < 0.5) and (gx == target1[0]) and (gy == target1[1])):
			roda(-0.1)
			gx = target2[0]
			gy = target2[1]
		
		elif((distance < 0.5) and (gx == target2[0]) and (gy == target2[1])):
			roda(0.15)
			gx = target3[0]
			gy = target3[1]
		
		elif((distance < 0.5) and (gx == target3[0]) and (gy == target3[1])):
			para_rodar()
			gx = target4[0]
			gy = target4[1]
		
		elif((distance < 0.5) and (gx == target4[0]) and (gy == target4[1])):
			roda(-0.15)
			gx = target5[0]
			gy = target5[1]

		elif((distance < 0.5) and (gx == target5[0]) and (gy == target5[1])):
			roda(0.04)
			gx = target6[0]
			gy = target6[1]

		elif((distance < 0.5) and (gx == target6[0]) and (gy == target6[1])):
			roda(-0.07)
			gx = target7[0]
			gy = target7[1]

		elif((distance < min_distance) and (gx == target7[0]) and (gy == target7[1])):
			para_andar()
			para_rodar()
			print("Chegoou")

		rate.sleep()