#!/usr/bin/env python
# coding: utf-8

# In[4]:


#!/usr/bin/env python
from __future__ import division
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from homogeneous_matrix import *
import pickle

class TurtleBotNode:

    def __init__(self): #Criado o publisher e o subscriber

	rospy.init_node('turtle_bot_basic_controller', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.callback_update_position)
        self.velocity_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        self.PosicaoX = 0
        self.PosicaoY = 0
        self.yaw_deg = 0
        self.rate = rospy.Rate(10)

    def callback_update_position(self, data): #gera posição e orientação atual usando o subscriber
        self.PosicaoX = round(data.pose.pose.position.x, 4)
        self.PosicaoY = round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation

        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)

    def Formulacao(self): #Recebe as coordenadas, trata os dados, ajusta e movimenta.
	Cord = []
	Cords = []
	dados = open('Coordenadas.txt', "rb")
	Cords = pickle.load(dados)
	#print(Cords)
	
	aux = []
	aux3 = 0
	aux2 = 0
	for k in range(len(Cords)):
    		aux = []
    		for l in range(0, 2):
	        	if l == 1:
	        		Cords[k][l] = Cords[k][l] * -1
        
		        aux2 = (Cords[k][l]/40)
		        aux3 = aux2
		        aux.append(aux3)
	    	Cord.append(aux)
	
   	#print(">>>>>>>>>>>", Cord)
	for x in range(len(Cord)):
	    if x == 0:
		continue
	    Cord[x][1] = Cord[x][1] - Cord[0][1]
	Cord.pop(0)
	
	for i in range(len(Cord)):
    
		ObjetivoX = Cord[i][0]
		ObjetivoY = Cord[i][1]

		#controlamos angulo e movimento
		PID_Rotacao = Controlador_PID(60.0, 0.076, 1.35, 0.5) 
		PID_Distancia = Controlador_PID(0.1, 0.1, 0.6, 0.5) 

		distancia = math.sqrt(math.pow((ObjetivoX - self.PosicaoX), 2) + math.pow((ObjetivoY - self.PosicaoY), 2))
		twist = Twist()
		
		while distancia >= 0.2: #Valor Ideal
		    	
		    Teta = math.atan2(ObjetivoY - self.PosicaoY, ObjetivoX - self.PosicaoX)
		    angulo = math.degrees(Teta)		    
		    ErroAngular= angulo - self.yaw_deg

		    if (ErroAngular > 20):
		    	    while (ErroAngular > 20 ):
		
				Teta = math.atan2(ObjetivoY - self.PosicaoY, ObjetivoX - self.PosicaoX)
				angulo = math.degrees(Teta)
				
				twist.linear.x = 0.2 #Valor Ideal

				if angulo < 180:
					twist.angular.z = 0.8 #Valor Ideal
				else:
					twist.angular.z = -0.8 #Valor Ideal

				self.velocity_publisher.publish(twist)
				self.rate.sleep()

				ErroAngular= angulo - self.yaw_deg
		 
		    distancia = math.sqrt(math.pow((ObjetivoX - self.PosicaoX), 2) + math.pow((ObjetivoY - self.PosicaoY), 2))
		    ErroAngular= angulo - self.yaw_deg
		    SaidaAngulo = PID_Rotacao.AtualizaErro(ErroAngular)
		    SaidaDistancia = PID_Distancia.AtualizaErro(distancia)
                    
		    twist.linear.x = SaidaDistancia
		    twist.angular.z = SaidaAngulo
		    self.velocity_publisher.publish(twist)

		print("TURTLEBOT CHEGOU A COORDENADA:", ObjetivoX, ObjetivoY)
	
	print("O TURTLEBOT FINALIZOU O TRAJETO!")
	print("TRAJETO PERCORRIDO: ", Cord)
        
	twist.linear.x = 0
        twist.angular.z = 0    
	self.velocity_publisher.publish(twist)
        
class Controlador_PID:

    def __init__(self, P, I, D, limitador):

        self.ErroP = P
        self.ErroI = I
        self.ErroD = D
        self._limitador = limitador
        self._ErroAnterior = 0.0

    def AtualizaErro(self, erro):

        SaidaErroP = erro * self.ErroP

        ErroDif = erro - self._ErroAnterior
        SaidaErroD = self.ErroD * ErroDif

        ErroInt = erro + self._ErroAnterior
        SaidaErroI = self.ErroI * ErroInt

        self._ErroAnterior = erro

        ErroTotal = SaidaErroP + SaidaErroD + SaidaErroI

        if ErroTotal > self._limitador:
            ErroTotal = self._limitador
        elif ErroTotal < (-self._limitador):
            ErroTotal = (-self._limitador)

        return ErroTotal
			   
def shutdown_callback():
    print("------------------- End.")
    
if __name__ == "__main__":

    robo = TurtleBotNode()
    robo.Formulacao()
    rospy.on_shutdown(shutdown_callback)
    rospy.spin()


# In[ ]:




