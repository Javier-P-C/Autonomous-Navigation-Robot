#!/usr/bin/env python

# LINE FOLLOWER

# Importamos librerias necesarias
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

bridge = CvBridge()
class LineFollower():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        kw = 0.04
        image_topic = "/video_source/raw"
        
        # Declaramos suscriptores
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        
        # Declaramos publicadores
        self.line_pub = rospy.Publisher("line_topic", Int32, queue_size=1)   
        
        # Establecemos la velocidad de ejecucion
        ra = rospy.Rate(10) #10Hz 

        # Variables
        self.vel = Twist() 
        self.bandera = 0
        
        while not rospy.is_shutdown():  
            if self.bandera: 
                # Cambiamos imagen a escala de grises
                gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                width = gray.shape[1]
                height = gray.shape[0]
                print(gray.shape)
                # Ajustamos a un nuevo tamaño
                img_resizedd = cv.resize(gray, (320,180), interpolation = cv.INTER_AREA)
                # Aplicamos un threshoold binario para resaltar la linea
                (T, img_resized) = cv.threshold(img_resizedd, 100, 255, cv.THRESH_BINARY)
                
                # Filtramos con gauss, erode y dilate para mejorar la deteccion de la linea
                kernel = np.ones((4,4), np.uint8)
                gaus_img = cv.GaussianBlur(img_resized, (5,5),0)
                img_erosion = cv.erode(gaus_img, kernel, iterations=4) #quita los blancos
                img_dilation = cv.dilate(img_erosion , kernel, iterations=4) #aumenta los blancos
                
                # Obtenemos informacion de la imagen
                x_len = img_dilation.shape[1]
                y_len = img_dilation.shape[0]
                crop_percent = int(x_len*0.22)
		
                # Seleccionamos el area de interes
                imageOut = img_dilation[90:height,70:250]
                # Detectamos el punto mas oscuro
                data = np.array(imageOut)
                data2 = data.sum(axis=0)
                datoMin = np.min(data2)
                val = np.where(data2==datoMin)[0][0]

                # Centramos el punto
                if val > 80 and val < 100:
                    val = 90
                
                # Publicamos la coordenada de dicho punto
                self.line_pub.publish(val)
                print("pixel mas oscuro: " + str(val))


            ra.sleep() 
        cv.destroyAllWindows() 

    def callback(self,data):
        # Convertimos imagen de ros a cv2
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bandera = 1
    
    # Funcion de cierre
    def cleanup(self):
        print('Done')

if __name__ == '__main__':
    rospy.init_node('LineFollower', anonymous=True)
    LineFollower()
