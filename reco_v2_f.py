import cv2
import numpy as np

# This function dedect the object's vertices.
# If the the number of vertices is suitable to the predefine number of vertices,
# the function will highlight the contours.

def finding_shape(img , contour):
    approx  = cv2.approxPolyDP(contour , 0.01*cv2.arcLength(contour , True) , True)
    if 5 > len(approx) > 2:
        cv2.drawContours(img, [approx], 0,(255,0,0),4)
        cv2.putText( img , "object has been detected" , (50,50) ,font, 1 , (255,255,255))

# This function dedect the object in the space.
# The function draw a rectangle on the dedeted object. 

def object_position(img, contour):
    x ,y, w ,h = cv2.boundingRect(contour)
    print(f"({x},{y})")
    cv2.rectangle(img , (x,y) , (x+w , y+h ) , (0,0,255))
    cv2.putText(img, 'follow', (x - 60, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)


font = cv2.FONT_HERSHEY_SIMPLEX
lower = np.array([30,100,20])
upper = np.array([83,255,255])

video = cv2.VideoCapture(0)

while True:
    _ , img = video.read()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image , lower , upper)

    contours, hierarchy =cv2.findContours(mask , cv2.RETR_TREE , cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)!=0:
        for contour in contours:
            if cv2.contourArea(contour) > 300:
                finding_shape(img , contour)
                object_position(img , contour)
                
            
    cv2.imshow("mask" , mask)
    cv2.imshow("webcam" , img)

    cv2.waitKey(1)
