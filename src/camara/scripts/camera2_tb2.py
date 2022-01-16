#!/usr/bin/env python
import rospy
import cv2
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8

pub = rospy.Publisher('qr',Int8,queue_size = 5)  #Nodo que publica en topic qr

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback) #Nodo que se suscribe al topic de la camara el tb2

  def callback(self,data):
    bridge = CvBridge() #Objeto de la clase CvBridge
    dato = 0 #Inicializacion de la variable dato
    
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") #Conversion imagen mensaje de ROS a imagen OpenCV
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image

    barcodes = pyzbar.decode(image) #Libreria pyzbar que busca los qr de la imagen. Metodo decode.
    
    for barcode in barcodes:
        # Extraccion de la localizacion del qr a traves de una caja roja
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # Trnasformacion de la data del qr de byte object a string 
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # Dibujo del contenido del qr
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
        if barcodes:
            dato = barcodes[0][0] #Coger solo el dato data del qr
            
    dato = int(dato)
    pub.publish(dato) #Publicar en el topic
    
    #Se muestra la imagen en una ventana de opencv 
    cv2.imshow("Image", image)
    cv2.waitKey(3)

def main():

  camera_1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
