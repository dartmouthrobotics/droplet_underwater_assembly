# import the opencv library 
import cv2 
import numpy as np
  
# define a video capture object 
vid = cv2.VideoCapture(0) 

color= [255, 0, 255]

def binary_from_image(image, color):
    pass
  
while(True): 
    _, frame = vid.read() 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    # Range for upper range
    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    
    mask2 = cv2.inRange(hsv,lower_red,upper_red)
    mask1 = mask1+mask2

    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

  
vid.release() 
cv2.destroyAllWindows() 
