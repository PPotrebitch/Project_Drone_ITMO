import cv2
import time
HImage = None
WImage = None

def points3(image): 
    XY=[]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #cv2.imshow("1", blurred)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    cv2.imshow("1", thresh)
    print("-")
    for i in range(9,-1,-1):
        obrezimage = thresh[(HImage//10)*i:(HImage//10)*(i+1), 0:WImage]  
        contours=cv2.findContours(obrezimage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        #cv2.drawContours(image,contours,0,(255,0,0),5)
        cv2.imshow("18", obrezimage)
        if len(contours) > 1:
            #print(i)
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            cv2.rectangle(image,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            XY.append([xx, yy]) 
    time.sleep(0.04)
    return XY,image#!/usr/bin/env python3



input_video_path = 'IMG_box/Video_from_drone_101.mp4'

cap = cv2.VideoCapture(input_video_path)

ret, frame = cap.read()
HImage, WImage, _ = frame.shape

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        xy, image= points3(frame)
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
    else:
        break

cap.release()
cv2.destroyAllWindows()