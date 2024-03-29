#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import math

#Definições Globais 

#TARGETs [X,Y]
target0 = np.array([-8.0, -12.0])
target1 = np.array([-3.0, -7.0]) 
target2 = np.array([1.0, 0.0]) 
target3 = np.array([4.0, 7.0]) 
target4 = np.array([8.0,  12.0]) 
target5 = np.array([10.0, 10.0])
target6 = np.array([10.0, 10.0])
target7 = np.array([10.0, 10.0])
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

#Função para parar de andar
def para_andar():
	velocity.linear.x = 0.0
	pub.publish(velocity) 

#Função para andar pra frente	
def move_frente():
	velocity.linear.x = 0.5
	pub.publish(velocity)  

#Função para parar de girar
def para_rodar():
	velocity.angular.z = 0.0
	pub.publish(velocity) 

#Função para girar, recebo o valor por parametro pra verificar se o giro é pra direita ou esquerda
def roda(velocidade):
	velocity.angular.z = velocidade
	pub.publish(velocity)

#Função para localização
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
		
		#Chama funções de localização e de mover para frente, calcula distancia e mostra os dados em tempo real

		localizacao()

		move_frente()
		
		distance = math.sqrt((px-gx)**2 + (py-gy)**2)   

		print("=================================")
		print("Alvo: (%.2f, %.2f)" % (gx, gy))
		print("Posição: (%.2f, %.2f)" % (px, py)) 
		print("Distância: %.2f" % distance)
		print("=================================")

		#Ifs para verificar a distancia de pontos alvos e setar novos pontos ao chegar neles
		if((distance < 0.5) and (gx == target0[0]) and (gy == target0[1])):
			roda(0.1)
			gx = target1[0]
			gy = target1[1]

		elif((distance < 0.5) and (gx == target1[0]) and (gy == target1[1])):
			roda(-0.048)
			gx = target2[0]
			gy = target2[1]
		
		elif((distance < 0.5) and (gx == target2[0]) and (gy == target2[1])):
			roda(0.05)
			gx = target3[0]
			gy = target3[1]
		
		elif((distance < 0.5) and (gx == target3[0]) and (gy == target3[1])):
			roda(-0.075)
			gx = target4[0]
			gy = target4[1]
		
		#Mostrar quando chegar no ponto alvo final e parar de andar
		elif((distance < 0.5) and (gx == target4[0]) and (gy == target4[1])):
			para_andar()
			para_rodar()
			print("Chegoou")

		rate.sleep()