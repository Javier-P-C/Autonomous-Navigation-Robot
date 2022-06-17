#!/usr/bin/env python3

# CNN

# Importamos todas las librerias necesarias
from time import sleep
from matplotlib import image
import rospy
from tensorflow.keras.models import load_model
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
import cv2

class Signals2():
    def __init__ (self):
        rospy.on_shutdown(self.cleanup)
        self.bridge_object = CvBridge()
        self.image_received = 0
        rute = "/video_source/raw"
        
        # Declaramos suscriptores
        image_sub = rospy.Subscriber(rute, Image, self.image_cb)

        # Declaramos publicadores
        self.signal_pub = rospy.Publisher("signal_topic", String, queue_size=1)

	    # Variables
        self.detected = False

        # Establecemos la velocidad de ejecucion
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.image_received:
                # Deteccion de senal
                self.ia_signal()
            r.sleep()

    def image_cb(self, ros_image):
        # Convertimos imagen de ros a cv2
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            self.image_received = 1
        except CvBridgeError as e:
            print("Error",e)

    # Detectamos que senal es
    def ia_signal(self):
        self.detected = False

        # Abrimos imagen y obtenemos informacion
        img = self.cv_image
        height = img.shape[0]
        width = img.shape[1]
        # Nuevp ancho y altura
        height_y = height/2
        width_x = width/2
        # Seleccionamos area de interes
        img = img[0:int(height_y),int(width_x):int(width)]
        # Pasamos a escala de grises
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Aplicamos un filtro blur
        blur_img = cv2.medianBlur(gray_img,5)
        # Detectamos circulos
        circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,1,20,param1=100,param2=27,minRadius=10,maxRadius=50)
        # En caso de detectar circulos
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # Obtenemos informacion
            data_circle = circles[0]
            arr = data_circle[0]
            x = arr[0]
            y = arr[1]
            r = arr[2]

            # Ajustamos para el futuro corte
            desp = r * 2.4
            orig_x = x - desp/2
            orig_y = y - desp/2
            final_x = orig_x + desp
            final_y = orig_y + desp

            # Cortamos nueva area de interes
            self.CNN_img = img[int(orig_y):int(final_y),int(orig_x):int(final_x)]
            # Llamamos a la red nueronal convolucional
            self.RedCNN()
            self.detected = True
        # Comparamos la senal de stop con un xml de una senal de stop
        stop_sign = cv2.CascadeClassifier(r"/home/neri/Neutron/cascade_stop_sign.xml")
        stop_sign_scaled = stop_sign.detectMultiScale(gray_img, 1.3, 5)
        band = np.size(stop_sign_scaled)
        # En caso de ser stop
        if band != 0:
            # Establecemos cordenadas
            stop_cord = stop_sign_scaled[0]
            stop_orig_x = stop_cord[0]
            stop_final_x = stop_orig_x + 30
            stop_orig_y = stop_cord[1] 
            stop_final_y = stop_orig_y + 30
            # Cortamos el area de interes
            self.CNN_img = img[stop_orig_y:stop_final_y,stop_orig_x:stop_final_x]
            # Enviamos que la senal si es un stop
            self.signal_pub.publish("Stop")
            print("Stop")
            self.detected = True
            sleep(1)

        # Si no hay senal detectada
        if self.detected == False:
            self.signal_pub.publish("no signal")

    # Red neuronal convolucional
    def RedCNN(self):
        # Cargamos el modelo
        modeloCNN2 = load_model(r"/home/neri/Neutron/chulada.h5")
        # Establecemos tamano de imagen
        TAMANO_IMG = 72
        try:
            # Cambiamos tamano y a escala de grises
            neu_img = cv2.resize(self.CNN_img, (TAMANO_IMG,TAMANO_IMG))
            neu_img = cv2.cvtColor(neu_img, cv2.COLOR_BGR2GRAY)
            neu_img = neu_img.reshape(1,TAMANO_IMG, TAMANO_IMG,1)
            # Hacemos prediccion de la senal
            resultado = modeloCNN2.predict(neu_img)
            # Obtenemos el nombre de la senal
            signals = {0: "Stop",
            1:"NoLimit",
            2:"TurnRight",
            3:"GoAhead"}
            # Enviamos la senal detectada
            load = signals[np.argmax(resultado)]
            print(load)
            self.signal_pub.publish(load)
        except:
            print("no pude toy chiquito")

    # Funcion de cierre
    def cleanup(self):
        print('Done')

if __name__ == "__main__":
    rospy.init_node("image_signal", anonymous=True)
    print("AAAAAAAAAAAA")
    Signals2()
