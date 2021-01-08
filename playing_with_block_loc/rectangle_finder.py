# import the opencv library 
import cv2 
import numpy as np
  
# define a video capture object 
vid = cv2.VideoCapture(1) 

if not vid.isOpened():
    raise IOError("Cannot open webcam")

color= [255, 0, 255]

def binary_from_image(image, color):
    pass
  
while(True): 
    _, frame = vid.read() 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("original", frame)

    lower_red = np.array([(10.0 / 360.0) * 255.0,160,170])
    upper_red = np.array([(50.0 / 360.0) * 255.0,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    res = cv2.bitwise_and(frame,frame, mask=mask1)

    _, _, gray = cv2.split(res)
    kernel = np.ones((5,5),np.uint8)
    #gray = cv2.GaussianBlur(gray, (3,3), 5.5)
    gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
    gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    gray = cv2.morphologyEx(gray, cv2.MORPH_DILATE, kernel)
    cv2.imshow("gray", gray)

    #thresh1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 7, 2)
    ret,thresh1 = cv2.threshold(gray,107,255,cv2.THRESH_BINARY_INV)

    cv2.imshow("segmented", thresh1)

    threshold = 100
    canny_output = cv2.Canny(thresh1, threshold, threshold * 3)

    _, contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    filtered_contours = []

    for contour in contours:
        rect = cv2.minAreaRect(contour) 
        area = rect[1][0] * rect[1][1]

        if area > 200.0: 
            filtered_contours.append(contour)

    contour_image = np.zeros(gray.shape, dtype=np.uint8)

    cv2.drawContours(contour_image, filtered_contours, -1, 255, 2)

    # Set our filtering parameters 
    # Initialize parameter settiing using cv2.SimpleBlobDetector 
    params = cv2.SimpleBlobDetector_Params() 
      
    # Set Area filtering parameters 
    params.filterByArea = True
    params.minArea = 70
      
    # Set Circularity filtering parameters 
    params.filterByCircularity = True 
    params.minCircularity = 0.7
      
    # Set Convexity filtering parameters 
    params.filterByConvexity = False
    params.minConvexity = 0.2
          
    # Set inertia filtering parameters 
    params.filterByInertia = False 
    params.minInertiaRatio = 0.00001
      
    # Create a detector with the parameters 
    detector = cv2.SimpleBlobDetector_create(params) 
          
    # Detect blobs 
    keypoints = detector.detect(thresh1) 
      
    # Draw blobs on our image as red circles 
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(frame, keypoints, blank, (0, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow("blobs", blobs)

    cv2.waitKey(1)
      
vid.release() 
cv2.destroyAllWindows() 
