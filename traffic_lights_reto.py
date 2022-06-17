#!/usr/bin/env python

# TRAFFIC LIGHTS

# Importamos todas las librerias necesarias
import imghdr
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

bridge = CvBridge()
class TrafficLights():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        image_topic = "/video_source/raw"
        
        # Declaramos suscriptores
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        
        # Declaramos publicadores
        self.light_pub = rospy.Publisher("traffic_light_topic", String, queue_size=1)
        
        # Establecemos la velocidad de ejecucion
        ra = rospy.Rate(10) #10Hz  
        
        # Variables
        self.bandera = 0
        self.light = ""

        while not rospy.is_shutdown():
            if self.bandera: 
                # Obtenemos la imagen
                imagen = self.cv_image
                # Damos un nuevo tamano
                resize = cv2.resize(imagen,(640,360))
                # Seleccionamos el area de interes
                self.img = resize[0:180,0:320]
                # Funcion para detectar color
                self.colors(self.img)
                # Detectamos si hay verde en nuestra imagen
                if (cv2.countNonZero(self.frame_thresholdGreen) > 30):
                    self.light = "Green" 
                # Detectamos si hay rojo en ambas mascaras (limitado para evitar detecciones erroneas)
                elif (cv2.countNonZero(self.frame_thresholdRed) > 50 and cv2.countNonZero(self.frame_thresholdRed) < 400) or (cv2.countNonZero(self.frame_thresholdRed2) > 100 and cv2.countNonZero(self.frame_thresholdRed2) < 200):
                    self.light = "Red" 
                # De lo contrario se envia otro
                else:
                    self.light = "Other"
                print(self.light)      
                # Publicamos el color detectado
                self.light_pub.publish(self.light)
            ra.sleep() 

    def callback(self,data):
        # Convertimos imagen de ros a cv2
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bandera = 1
    
    # Funcion de cierre
    def cleanup(self):
        print('Done')

    def colors(self, img):
        # Aplicamos un filtro para reducir ruido
        flt_img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10,7,21)  
        # Convertimos imagen a HSV 
        hsv_img = cv2.cvtColor(flt_img, cv2.COLOR_BGR2HSV)

        # Mascara para detectar rojo 1
        self.frame_thresholdRed = cv2.inRange(hsv_img, (2, 10, 180) , (30, 255, 255))
        # Mascara para detectar verde
        self.frame_thresholdGreen = cv2.inRange(hsv_img, (70, 22, 120) , (90, 255, 255))
        # Mascara para detectar rojo 2
        self.frame_thresholdRed2 = cv2.inRange(hsv_img, (150, 30, 120) , (180, 255, 255))
        
        # Aplicamos funcion de threshold binario invertido a las 3 capas
        (T,self.frameRed) = cv2.threshold(self.frame_thresholdRed,127,255,cv2.THRESH_BINARY_INV)
        (T,self.frameGreen) = cv2.threshold(self.frame_thresholdGreen,127,255,cv2.THRESH_BINARY_INV)
        (T,self.frameRed2) = cv2.threshold(self.frame_thresholdRed2,127,255,cv2.THRESH_BINARY_INV)


if __name__ == '__main__':
    rospy.init_node('TrafficLights', anonymous=True)
    TrafficLights()
