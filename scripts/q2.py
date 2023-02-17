#!/usr/bin/env python3

import rospy
from os import system
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import math

#Definições Globais

#TARGETs [X,Y] - Mude os Target para conseguir ir
target1 = np.array([0.0,  0.0])
target2 = np.array([0.0,  0.0])
target2 = np.array([0.0,  0.0])
gx = target1[0]
gy = target1[1]

# Distância mínima para chegar ao alvo
min_distance = 1.0

px = 0.0
py = 0.0
pz = 0.0
ox = 0.0
oy = 0.0
oz = 0.0
ang = 0.0

kp = 3.0
ka = 8.0
kb = -0.1;

anteriorB = 0.0;
anteriorHip = 0.0;
anteriorVA = 0.0;

theta = 0.0

mat1 = np.zeros((3,3));

odometry_msg = Odometry()
velocity = Twist()

#Funções

def odometry_callback(data):
	global odometry_msg
	odometry_msg = data
	
def move_frente():
	velocity.linear.x = 0.5
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def para():
	velocity.linear.x = 0.0
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def Localizacao():
	global px,py
	px = odometry_msg.pose.pose.position.x
	py = odometry_msg.pose.pose.position.y
	pz = odometry_msg.pose.pose.position.z
	ox = odometry_msg.pose.pose.orientation.x
	oy = odometry_msg.pose.pose.orientation.y
	oz = odometry_msg.pose.pose.orientation.z
	ow = odometry_msg.pose.pose.orientation.w
		
	(roll, pitch, yaw) = euler_from_quaternion ([ox, oy, oz, ow])
	ang = yaw

def ModeloCinematico():
	global px, py
	global anteriorHip, anteriorB, anteriorVA

	# Calculo das distâncias entre a posição atual e posiçaõ desejada.
	dx = (gx-px)-0.05
	dy = (gy-py)-0.05
	hip = math.sqrt( (dx*dx) + (dy*dy) )

	# Calculo dos ângulos de orientação para movimentação do robô na direção correta.
	alpha = -theta + math.atan2(dx,dy)
	beta = -theta-alpha

	# Cálculo das coordenadas polares 
	mat1[0][0] = (-kp * hip * math.cos(alpha))
	mat1[0][0] = mat1[0][0] + (mat1[0][0]-anteriorHip)

	pa = (kp*math.sin(alpha) - ka*alpha - kb*beta)
	pb = (-kp * math.sin(alpha))

	# Condição para determinar para fazer com que o robô siga o trajeto definido pelo modelo cinemático.
	# No entanto, seguir este trajeto gera oscilações.
	if( pb <= (anteriorB ) ):				# retornar
		mat1[1][0] = (-pa) 
		mat1[2][0] = (-pb)
	else:
		mat1[1][0] = (pa)
		mat1[2][0] = pb

	anteriorHip = hip
	anteriorB = pb
		
	# Cálculo dos dados da entrada de controle.
	# Velocidade linear e angular.
	v = kp*mat1[0][0]
	va = (ka*mat1[1][0] + kb*mat1[2][0])

	# Condições para determinar quais sinais devem ser aplicados às velocidade.
	# É definido se o robô deve se deslocar para direita, para esquerda, para frente ou para trás.
	if( abs(dx)<0.5 and abs(dy)<0.5 ):
		velocity.linear.x = 0
		velocity.angular.z = 0
	elif( px<0 and py>0 ):			
		print("Quadrante 1")
		velocity.linear.x = v/2
		velocity.angular.z = (va+anteriorVA)/2 					
	elif( px>0 and py>0 ):				
		print("Quadrante 2")
		velocity.linear.x = v/2
		velocity.angular.z = (va+anteriorVA)/1.9 		
	elif( px<=0 and py<=0 ):			
		print("Quadrante 3")
		velocity.linear.x = -v/2
		velocity.angular.z = (va+anteriorVA)/1.8		
	elif( px>0 and py<0 ):		
		print("Quadrante 4")
		velocity.linear.x = -v/2
		velocity.angular.z = (va+anteriorVA)/2			

	anteriorVA=-va

	pub.publish(velocity)  

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
      
		Localizacao()
		
		distance = math.sqrt((px-gx)**2 + (py-gy)**2) 
		
		print("=================================")
		print("Alvo: (%.2f, %.2f)" % (gx, gy))
		print("Posição: (%.2f, %.2f)" % (px, py)) 
		print("Distância: %.2f" % distance)
		print("=================================")

		ModeloCinematico()

		if( distance < min_distance ):
			para()

		rate.sleep()

