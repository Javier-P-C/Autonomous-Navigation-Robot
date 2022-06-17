#!/usr/bin/env python

#NODO MAESTRO

# Importamos todas las librerias necesarias
import rospy, sys
import math
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32

class Master():
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		# Declaramos suscriptores
		self.signals_sub = rospy.Subscriber("signal_topic",String,self.signal_cb)
		self.line_sub = rospy.Subscriber("line_topic",Int32, self.line_cb)
		self.traffic_light_sub = rospy.Subscriber("traffic_light_topic",String,self.light_cb)

		# Declaramos publicadores
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

		# Variables
		self.line_vel = Twist()
		self.val = 0.0
		self.vel = Twist()
		self.signal = ""
		self.light = ""
		self.banderatr = 0
		self.banderaga = 0
		self.theta = 0.0
		self.x = 0.0
		self.y = 0.0
		self.count_GA = 0
		self.count_TR = 0
		
		# Establecemos la velocidad de ejecucion
		rate = 35 #Hz
		ra = rospy.Rate(rate)  
        
		while not rospy.is_shutdown():
			# Imprimimos la info que llega de los otros nodos
			print("Luz: "+str(self.light)+" Senal: "+str(self.signal) + str(self.banderatr))
			# Si detectamos semaforo verde u otro color que no sea rojo, entramos a la maquina de estados
			if self.light == "Green" or self.light == "Other":
				# Cuentas para cumplir go ahead interrumpiendo el seguidor de linea
				if self.count_GA != 0:
					self.count_GA += 1
					self.vel.linear.x = 0.1
					self.vel.angular.z = -0.002
					if self.count_GA >= rate * 6:
						self.count_GA = 0
						print("Fin GoAHead")
				# Cuentas para cumplir turn right interrumpiendo el seguidor de linea
				elif self.count_TR != 0:
					if self.count_TR < rate * 6:
						self.vel.linear.x = 0.1
						self.vel.angular.z = 0.0
					elif self.count_TR >= rate *6 and self.count_TR <rate *9:
						self.vel.linear.x = 0.0
						self.vel.angular.z = -0.1
					elif self.count_TR >= rate *9:
						self.vel.linear.x = 0.1
						self.vel.angular.z = 0.0
					self.count_TR +=1
					if self.count_TR >= rate * 10:
						self.count_TR = 0
						print("Fin TurnRight")
                # Si hay senal de go ahead avanzamos sin seguir la linea
				elif self.signal == "GoAhead":
					self.count_GA = 1
					self.banderaga = 0
					self.vel.linear.x = 0.0
					self.vel.angular.z = 0.0
					self.cmd_vel_pub.publish(self.vel)
					self.vel.linear.x = 0.1
					self.vel.angular.z = -0.002
				# Si hay senal de turn right giramos a la derecha sin seguir la linea
				elif self.signal == "TurnRight":
					self.count_TR = 1
					self.banderatr = 0
					self.vel.linear.x = 0.0
					self.vel.angular.z = 0.0
				# Si hay senal de stop paramos y el codigo acaba
				elif self.signal == "Stop":
					self.vel.linear.x = 0.1
					rospy.sleep(3)
					self.vel.linear.x = 0.0
					self.vel.angular.z = 0.0
					self.cleanup()
					break
               	# Si no se detectan senales aplicamos un controlador P para seguir la linea     
				elif(self.banderatr == 0):
					kw = 0.001
					self.vel.linear.x = 0.08
					# Si hay seña de no limit subimos la velocidad por 1.2 segundos
					if self.signal == "NoLimit":
						self.vel.linear.x = 0.18
						self.cmd_vel_pub.publish(self.vel)
						rospy.sleep(1.2)
					# Control P
					self.vel.angular.z = kw*(90 - self.val)
					self.line_vel.angular.z = self.vel.angular.z
					self.line_vel.linear.x = self.vel.linear.x
            # Si se detecta semaforo rojo se detiene todo movimiento       
			elif self.light == "Red" :
				# Se activan las banderas de las senales para no perder la instruccion
				if  self.signal == "TurnRight":
					self.banderatr = 1
				if  self.signal == "GoAhead":
					self.banderaga = 1
				self.vel.linear.x = 0.0
				self.vel.angular.z = 0.0
			# Publicamos la velocidad
			self.cmd_vel_pub.publish(self.vel)
			ra.sleep() 
    # Obtenemos la senal detecada por la red   
	def signal_cb(self, signal):
		self.signal = signal.data
	
	# Obtenemos el punto mas oscuro del seguidor de linea		
	def line_cb(self, val):
		self.val = val.data
	
	# Obtenemos el estado del semaforo  
	def light_cb(self, light):
		self.light = light.data
	
	# Funcion de cierre
	def cleanup(self):
		self.vel.linear.x = 0.0
		self.vel.angular.z = 0.0
		print("LISTO UWU")
		self.cmd_vel_pub.publish(self.vel)

if __name__ == '__main__':
    rospy.init_node('Master_node', anonymous=True)
    Master()   
