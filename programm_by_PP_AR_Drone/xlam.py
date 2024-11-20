import cv2
import time
import numpy as np

HImage = None
WImage = None
    
def points(image): #äëÿ ñïëîøíîé äîðîãè
    XY=[]
    cv2.imshow("1", cv2.inRange(image,(15,15,15),(60,60, 60)))
    #cv2.imshow("1", cv2.inRange(image,(15,15,15),(60,60, 60)))
    for i in range(9,-1,-1):
        obrezimage = image[(HImage//10)*i:(HImage//10)*(i+1), 0:WImage]  
        BW=cv2.inRange(obrezimage,(15,15,15),(60,60, 60))
        contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        if len(contours) > 1:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            cv2.rectangle(image,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            XY.append([xx, yy]) 
    time.sleep(0.01)
    return XY,image#!/usr/bin/env python3





def points2(image): #äëÿ ñïëîøíîé äîðîãè
    XY=[]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #cv2.imshow("1", blurred)
    (T, threshInv) = cv2.threshold(blurred, 70, 255,
	cv2.THRESH_BINARY_INV)
    cv2.imshow("1", threshInv)
    print("-")
    for i in range(9,-1,-1):
        obrezimage = image[(HImage//10)*i:(HImage//10)*(i+1), 0:WImage]  
        gray = cv2.cvtColor(obrezimage, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        (T, threshInv) = cv2.threshold(blurred, 70, 255,0)
        #print(T)
        #threshInv= cv2.inRange(obrezimage,(15,15,15),(60,60, 60))
        contours=cv2.findContours(threshInv, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        cv2.drawContours(image,contours,0,(255,0,0),5)
        cv2.imshow("18", threshInv)
        if len(contours) > 1:
            print(i)
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            cv2.rectangle(image,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            XY.append([xx, yy]) 
    time.sleep(0.01)
    return XY,image#!/usr/bin/env python3

    
def points3(image): #äëÿ ñïëîøíîé äîðîãè
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
        else:
            XY.append([None, None]) 
    time.sleep(0.008)
    return XY,image#!/usr/bin/env python3
# image = cv2.imread("image.mp4", 1)
# if __name__ == '__main__':
#     image= points(image)
#     cv2.imshow("1", image)
#     cv2.waitKey(3)


def cross(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #cv2.imshow("1", blurred)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    contours=cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if len(contours) > 1:
            #print(i)
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xn = x
            centerx = x + w//2

            obrezimage1 = thresh[ x:x+3, :]
            contours=cv2.findContours(obrezimage1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if len(contours) > 1:
            #print(i)
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                y1 = y+h//2

            obrezimage2 = thresh[ x+w-3:x+w, :]
            contours=cv2.findContours(obrezimage2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if len(contours) > 1:
            #print(i)
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                y2 = y+h//2
            centery = (y1+y2)//2
    print(centerx,centery)             




def points4(image): #äëÿ ñïëîøíîé äîðîãè
    XY=[]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #cv2.imshow("1", blurred)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    #cv2.ellipse(thresh,(WImage//2, HImage//2), (WImage//4, HImage//4), 0, 0, 360, (0, 255, 0), 3, 3, -1)
    #cv2.ellipse(thresh,(250, 150), (80, 20), 5, 0, 360, (0, 255, 0), -1)
    w_3c = np.full_like(thresh, fill_value=(255))
    center = (thresh.shape[1]//2, thresh.shape[0]//2)
    radius = int(min(center) * .7)
    zeros = np.zeros_like(thresh[:,:], dtype='uint8')
    cv2.circle(zeros, center, radius, 255, thickness=cv2.FILLED)
    masked = cv2.bitwise_and(thresh, w_3c, mask=zeros)
    contour=cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour=contour[0]
    if len(contour)>0:
        masked = masked
    else:
        masked = thresh
    cv2.imshow("1", masked)
    #print("-")
    #cv2.rectangle(image,((+(HImage//2 - radius)),(WImage//2 - radius)), (((HImage//2 - radius))+2, (WImage//2 - radius)+2),(0,0,255), 5)  

    for i in range(9,-1,-1):
        obrezimage = masked[((radius//5)*i+(HImage//2 - radius)):((radius//5)*(i+1)+(HImage//2 - radius)), 0:WImage]
        #cv2.rectangle(image,(((radius//5)*i+(HImage//2 - radius)),(WImage//2 - radius)), (((radius//5)*i+(HImage//2 - radius))+2, (WImage//2 - radius)+2),(0,0,255), 5)  
        contours=cv2.findContours(obrezimage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        print(i)
        contours=contours[0]
        #cv2.drawContours(image,contours,0,(255,0,0),5)
        cv2.imshow("18", obrezimage)
        if len(contours) > 1:
            #print(i)
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            cv2.rectangle(image,(xx,yy), (xx+2, yy+2),(255,0,0), 5)
            XY.append([xx, yy])
        else:
            XY.append([None, None])
    #time.sleep(0.008)
    return XY,image

def cross(img): # алгоритм для распознования крестика для стабилизации дрона 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #cv2.imshow("1", blurred)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    contours=cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if len(contours) > 1:
            #print(i)
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xn = x
            centerx = x + w//2

            obrezimage1 = thresh[ x:x+3, :]
            contours=cv2.findContours(obrezimage1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if len(contours) > 1:
            #print(i)
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                y1 = y+h//2

            obrezimage2 = thresh[ x+w-3:x+w, :]
            contours=cv2.findContours(obrezimage2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if len(contours) > 1:
            #print(i)
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                y2 = y+h//2
            centery = (y1+y2)//2
    return centerx, centery

def cross(img):
    frame1=cv2.inRange(img,(0,0,0),(17,17,17))#пороги цвета от мин до макс (синий зелёный красный)
    frame1[0][0]=1
    y1=0
    y2=0
    contours=cv2.findContours(frame1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        cenerx = x+w//2
    x1=x
    x2=x+w
    contours=cv2.findContours(frame1[:,x1:x1+3], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        y1 = y+h//2
    contours=cv2.findContours(frame1[:,x2-3:x2], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        y2 = y+h//2
    cenery = (y1+y2)//2
    print(cenerx,cenery,y1,y2)

input_video_path = '/home/tori/catkin_ws/src/dlf-solution/scripts/image.mp4'

cap = cv2.VideoCapture(input_video_path)

ret, frame = cap.read()
HImage, WImage, _ = frame.shape
nach =time.time()
c=0
while(cap.isOpened()):
    ret, frame = cap.read()
    #print(frame, ret)
    if ret:
        xy, image= points4(frame)
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        c+=1
    else:
        break
con = time.time()
#print("nach",nach)
print('it', (con - nach)/c)
cap.release()
cv2.destroyAllWindows()