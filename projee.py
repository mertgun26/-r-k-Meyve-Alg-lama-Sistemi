import pyfirmata
import time
from pyfirmata import Arduino, SERVO
from time import sleep  
import numpy as np
import cv2
import pyfirmata
import time
from tkinter import *
from threading import Thread


webcam = cv2.VideoCapture(1)
port='COM5'
pin = 8
kirmizi = 2
yesil = 4
mavi = 3
board = Arduino(port)


 
board.digital[pin].mode = SERVO
 
def rotateServo(pin, angle):
        board.digital[pin].write(angle)
        sleep(0.015) 
  
metin = "bos"
while(1):       
    
    _, goruntu = webcam.read()
    goruntu1 = cv2.cvtColor(goruntu, cv2.COLOR_BGR2GRAY)
    goruntu2 = goruntu1
    goruntu1 = cv2.GaussianBlur(goruntu1, (5,5), 0)
    ret, thresh1 = cv2.threshold(goruntu1, 205, 255, cv2.THRESH_BINARY)
    ret, thresh3 = cv2.threshold(goruntu1, 160, 255, cv2.THRESH_BINARY)
    thresh3 = cv2.GaussianBlur(thresh3, (5,5), 0)	
    genis = cv2.dilate(thresh3, None, iterations = 1)
    genis2 = cv2.dilate(thresh1, None, iterations = 0)
    kontur,_ = cv2.findContours(genis, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    kontur2,_ = cv2.findContours(genis2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    fark = cv2.absdiff(thresh3, thresh1)
    ret, bos = cv2.threshold(goruntu1, 255, 255, cv2.THRESH_BINARY)	
		
    fark_resim = cv2.absdiff(thresh1, thresh3)
    fark_sayı = cv2.countNonZero(fark_resim)
   
    hsvFrame = cv2.cvtColor(goruntu, cv2.COLOR_BGR2HSV)
  
    
    kirmizi_min = np.array([136, 87, 111], np.uint8)
    kirmizi_max = np.array([180, 255, 255], np.uint8)
    kirmizi_filtre = cv2.inRange(hsvFrame, kirmizi_min, kirmizi_max)

    mavi_min = np.array([80,150,2], np.uint8)
    mavi_max = np.array([130,255,255], np.uint8)
    mavi_filtre = cv2.inRange(hsvFrame, mavi_min, mavi_max)
  

    sari_min = np.array([0, 150, 150], np.uint8)
    sari_max = np.array([105, 255, 255], np.uint8)
    sari_filtre = cv2.inRange(hsvFrame, sari_min, sari_max)
    
    yesil_min = np.array([50, 52, 72], np.uint8)
    yesil_max = np.array([95, 250, 150], np.uint8)
    yesil_filtre = cv2.inRange(hsvFrame, yesil_min, yesil_max)
  
    
    
      
    
    kernal = np.ones((5, 5), "uint8")
      
    
    kirmizi_filtre = cv2.dilate(kirmizi_filtre, kernal)
    res_kirmizi = cv2.bitwise_and(goruntu, goruntu,mask = kirmizi_filtre)

    mavi_filtre = cv2.dilate(mavi_filtre, kernal)
    res_mavi = cv2.bitwise_and(goruntu, goruntu,mask = mavi_filtre)
      
    
    yesil_filtre = cv2.dilate(yesil_filtre, kernal)
    res_yesil = cv2.bitwise_and(goruntu, goruntu,mask = yesil_filtre)

    sari_filtre = cv2.dilate(sari_filtre, kernal)
    res_sari = cv2.bitwise_and(goruntu, goruntu,mask = sari_filtre)
      
    cv2.putText(bos, metin,(10, 100), cv2.FONT_HERSHEY_SIMPLEX, 5.0, (255, 0, 0), 3)  	    
    	    
    if(fark_sayı > 4000):
        rotateServo(pin, 60)
        
        metin = "curuk"
        time.sleep(2)	
    
   
    #kirmizi
    contours, hierarchy = cv2.findContours(kirmizi_filtre,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            
            rotateServo(pin, 180)
            
            x, y, w, h = cv2.boundingRect(contour)
            goruntu = cv2.rectangle(goruntu, (x, y),(x + w, y + h),(0, 0, 255), 2)
            cv2.putText(goruntu, "kirmizi", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
            metin= "kirmizi"

    #mavi
    contours, hierarchy = cv2.findContours(mavi_filtre,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            
            rotateServo(pin, 120)
            
            x, y, w, h = cv2.boundingRect(contour)
            goruntu = cv2.rectangle(goruntu, (x, y),(x + w, y + h),(0, 0, 255), 2)
            cv2.putText(goruntu, "kirmizi", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
            metin= "mavi"

    #sari
    contours, hierarchy = cv2.findContours(sari_filtre,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            
            rotateServo(pin, 0)
            x, y, w, h = cv2.boundingRect(contour)
            goruntu = cv2.rectangle(goruntu, (x, y),(x + w, y + h),(0, 0, 255), 2)
            cv2.putText(goruntu, "sari", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
            metin= "sari"

              
    
    # yesil
    contours, hierarchy = cv2.findContours(yesil_filtre,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            
            rotateServo(pin, 120)
            x, y, w, h = cv2.boundingRect(contour)
            goruntu = cv2.rectangle(goruntu, (x, y),(x + w, y + h),(0, 255, 0), 2)
            cv2.putText(goruntu, "yesil", (x, y),cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 0))
            metin = "yesil"              
    
    resim = np.hstack((thresh1,thresh3))
    resim1 = np.hstack((bos,goruntu2))
    resim1 = cv2.resize(resim1, (1360,400))          
    resim = cv2.resize(resim, (1360,400))
    resim2 = np.vstack((resim,resim1))
    cv2.imshow('Vertical Appended', resim2)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    