import time
import threading
import cv2
import numpy as np
from PIL import Image
import RPi.GPIO as GPIO
import serial
import numpy as np
import subprocess

output = subprocess.run(['v4l2-ctl', '--list-devices'], stdout = subprocess.PIPE).stdout.decode('utf-8')
idxCam1 = int(output[output.index("USB 2.0 Camera")+62:output.index("USB 2.0 Camera")+63])
idxCam2 = int(output[output.index("mmal service 16.1")+56:output.index("mmal service 16.1")+57])
start_time = time.time()

#start cameras
cam1 = cv2.VideoCapture(idxCam1)
cam2 = cv2.VideoCapture(idxCam2)

print("O");

#color HSV bounds
cam1_RED_LOWER_0 = np.array([0, 150, 10])
cam1_RED_UPPER_0 = np.array([5, 255, 255])
cam1_RED_LOWER_1 = np.array([165, 150, 10])
cam1_RED_UPPER_1 = np.array([180, 255, 255])

cam1_GREEN_LOWER = np.array([45, 170, 10])
cam1_GREEN_UPPER = np.array([80, 255, 255])

cam1_YELLOW_LOWER = np.array([15, 200, 50])
cam1_YELLOW_UPPER = np.array([45, 255, 255])


cam2_RED_LOWER_0 = np.array([0, 130, 100])
cam2_RED_UPPER_0 = np.array([5, 255, 255])
cam2_RED_LOWER_1 = np.array([160, 130, 100])
cam2_RED_UPPER_1 = np.array([180, 255, 255])

cam2_GREEN_LOWER = np.array([60, 40, 10])
cam2_GREEN_UPPER = np.array([100, 255, 255])

cam2_YELLOW_LOWER = np.array([10, 10, 25])
cam2_YELLOW_UPPER = np.array([45, 255, 255])


#colors
def colorRec(index, redLower0, redUpper0, redLower1, redUpper1, greenLower, greenUpper, yellowLower, yellowUpper):
    #read image
    if index == 1:
        v, img = cam1.read()
        while (v == False):
            v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
        while (v == False):
            v, img = cam2.read()
    
    if (index == 1):
        cv2.imwrite("test.jpg", img)
    else:
        cv2.imwrite("test3.jpg", img)
    
    #convert to hsv
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #set pixel colors into red, green, yellow, and white
    red_mask1 = cv2.inRange(img_hsv, redLower0, redUpper0)
    red_mask2 = cv2.inRange(img_hsv, redLower1, redUpper1)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    img[red_mask > 0] = [0, 0, 255]

    green_mask = cv2.inRange(img_hsv, greenLower, greenUpper)
    img[green_mask > 0] = [0, 255, 0]
    
    yellow_mask = cv2.inRange(img_hsv, yellowLower, yellowUpper)
    img[yellow_mask > 0] = [0, 255, 255]
    
    img[(red_mask == 0) & (yellow_mask == 0) & (green_mask == 0)] = 255
    
    #make borders white so contours can be touching edges
    img[0:480,0:1] = 255
    img[479:480,0:640] = 255
    img[0:1,0:640] = 255
    img[0:480,639:640] = 255
    
    if (index == 1):
        cv2.imwrite('test2.jpg', img)
    else:
        cv2.imwrite('test4.jpg', img)
    
    #apply contours
    blur = cv2.blur(img,(7,7))

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray,250,255,cv2.THRESH_BINARY_INV)

    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #if (index == 1):
        #cv2.imwrite('test3.jpg', thresh)
    #contours must meet area requirements and have 4 sides
    for c in contours:
        area = cv2.contourArea(c, False)
        if area > 20000:
            #print('color area requirement met')
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03*peri, True)
            if len(approx) == 4:
                #print('color 4 sides requirement met')
                #crop image within contour
                M = cv2.moments(c)
                centerX = int(M['m10']/M['m00'])
                centerY = int(M['m01']/M['m00'])
                
                img = img[centerY-70:centerY+70, centerX-70:centerX+70]
                
                #average color in cropped image
                average = img.mean(axis=0).mean(axis=0)
                #print(average)

                #colors
                green = [0, 255, 0]
                yellow = [0, 255, 255]
                red = [0, 0, 255]
                white = [255, 255, 255]
                
                #find closest color value
                colors = np.array([green, yellow, red, white])
                distances = np.sqrt(np.sum((colors-average)**2, axis=1))
                idx = np.where(distances==np.amin(distances))
                closest = colors[idx]
                #print(closest)
                if (closest == green).all():
                    callInterrupt(threading.currentThread().getName() + "g\n")
                    print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
                elif (closest == yellow).all():
                    callInterrupt(threading.currentThread().getName() + "y\n")
                    print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
                elif (closest == red).all():
                    callInterrupt(threading.currentThread().getName() + "r\n")
                    print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
                elif (closest == white).all():
                    pass


#letters
def letterRec(index):
    #read image
    if index == 1:
        v, img = cam1.read()
        while (v == False):
            v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
        while (v == False):
            v, img = cam2.read()
    
    #if (index == 1):
        #cv2.imwrite('test.jpg', img)
    
    #color bounds
    BLACK_UPPER = 85
    
    #convert to grayscale and threshold darkest pixels
    blur = cv2.blur(img,(7,7))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,BLACK_UPPER,255,cv2.THRESH_BINARY_INV)
    #if (index == 1):
        #cv2.imwrite('test2.jpg', thresh)
    
    #find contours
    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    #contours must be fully in the image, meet area requirements, and have a certain height:width ratio
    for c in contours:
        x, y, w1, h1 = cv2.boundingRect(c)
        if x>0 and x+w1<640 and y>0 and y+h1<480:
            area = w1*h1
            if area > 15000:
                #print('letter area requirement met')
                rect = cv2.minAreaRect(c)
                cx, cy = rect[0]
                w, h = rect[1]
                angle = rect[2]
                area = w*h
                if 1 <= h/w <= 1.6 or 1 <= w/h <= 1.6:
                    #rotate image so letter is upright
                    #print('letter ratio requirement met')
                    #print(int(cy-h/2)-5, int(cy+h/2)+5, int(cx-w/2)-5, int(cx+w/2)+5)
                    #print(int(cy-w/2)-5, int(cy+w/2)+5, int(cx-h/2)-5, int(cx+h/2)+5)
                    if w<h:
                        rot = cv2.getRotationMatrix2D((cx, cy), angle, 1)
                        thresh = cv2.warpAffine(thresh, rot, (640, 480))
                        thresh = thresh[max(0, int(cy-h/2)-5): int(cy+h/2)+5, max(0, int(cx-w/2)-5): int(cx+w/2)+5]
                    else:
                        rot = cv2.getRotationMatrix2D((cx, cy), angle+90, 1)
                        thresh = cv2.warpAffine(thresh, rot, (640, 480))
                        thresh = thresh[max(0, int(cy-w/2)-5): int(cy+w/2)+5, max(0, int(cx-h/2)-5): int(cx+h/2)+5]
                    
                    #divide image in half horizontally, in thirds vertically
                    rows, cols = thresh.shape

                    left_thresh = thresh[0: rows, 0: int(cols/2)]
                    right_thresh = thresh[0: rows, int(cols/2): cols]

                    top_thresh = thresh[0: int(rows/3), 0: cols]
                    mid_thresh = thresh[int(rows/3): int(2*rows/3), 0: cols]
                    bot_thresh = thresh[int(2*rows/3): rows, 0: cols]

                    #if (index == 1):
                        #cv2.imwrite('left.jpg', left_thresh)
                        #cv2.imwrite('right.jpg', right_thresh)
                        #cv2.imwrite('top.jpg', top_thresh)
                        #cv2.imwrite('mid.jpg', mid_thresh)
                        #cv2.imwrite('bot.jpg', bot_thresh)
                    
                    #find and count the number of contours in each section
                    contoursL, hierarchy = cv2.findContours(left_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursR, hierarchy = cv2.findContours(right_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    contoursT, hierarchy = cv2.findContours(top_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursM, hierarchy = cv2.findContours(mid_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursB, hierarchy = cv2.findContours(bot_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    L = []
                    R = []
                    T = []
                    M = []
                    B = []

                    for c in contoursL:
                        area = cv2.contourArea(c, False)
                        L.append(area)
                    for c in contoursR:
                        area = cv2.contourArea(c, False)
                        R.append(area)
                    for c in contoursT:
                        area = cv2.contourArea(c, False)
                        T.append(area)
                    for c in contoursM:
                        area = cv2.contourArea(c, False)
                        M.append(area)
                    for c in contoursB:
                        area = cv2.contourArea(c, False)
                        B.append(area)
                    
                    #ignore contours that are too small
                    while min(L) < max(L)/4:
                        L.remove(min(L))
                    while min(R) < max(R)/4:
                        R.remove(min(R))
                    while min(T) < max(T)/4:
                        T.remove(min(T))
                    while min(M) < max(M)/4:
                        M.remove(min(M))
                    while min(B) < max(B)/4:
                        B.remove(min(B))
                    
                    #print(len(L), len(R), len(T), len(M), len(B))
                    #return result
                    if len(L) == 2 and len(R) == 2:
                        callInterrupt(threading.currentThread().getName() + "s\n")
                        print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                    elif len(T) == 2 and len(M) == 2 and len(B) == 1:
                        callInterrupt(threading.currentThread().getName() + "u\n")
                        print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")
                    elif len(T) == 1 and len(M) == 2 and len(B) == 2:
                        callInterrupt(threading.currentThread().getName() + "u\n")
                        print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")
                    elif len(T) == 2 and len(M) == 1 and len(B) == 2:
                        callInterrupt(threading.currentThread().getName() + "h\n")
                        print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
                    else:
                        pass


#run color recognition
#colorRec(1, cam1_RED_LOWER_0, cam1_RED_UPPER_0, cam1_RED_LOWER_1, cam1_RED_UPPER_1, cam1_GREEN_LOWER, cam1_GREEN_UPPER, cam1_YELLOW_LOWER, cam1_YELLOW_UPPER)
#colorRec(2, cam2_RED_LOWER_0, cam2_RED_UPPER_0, cam2_RED_LOWER_1, cam2_RED_UPPER_1, cam2_GREEN_LOWER, cam2_GREEN_UPPER, cam2_YELLOW_LOWER, cam2_YELLOW_UPPER)

#run letter recognition
#letterRec(1)
#letterRec(2)

def callInterrupt(data):
    try:
        ser.write(data.encode()) #encode the data so that it can be put through the serial connectoin
        time.sleep(10) #wait 10 seconds
    except:
        print("ERROR")

def camWrapper(index, redLower0, redUpper0, redLower1, redUpper1, greenLower, greenUpper, yellowLower, yellowUpper):
    while True:
        colorRec(index, redLower0, redUpper0, redLower1, redUpper1, greenLower, greenUpper, yellowLower, yellowUpper)
        print(threading.currentThread().getName())

def camWrapper2(index):
    while True:
        letterRec(index)
        print(threading.currentThread().getName())

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    except:
        print('failed to connect')
        time.sleep(60)
    finally:
        ser.write('start\n'.encode())

    t1 = threading.Thread(target = camWrapper, args = (1, cam1_RED_LOWER_0, cam1_RED_UPPER_0, cam1_RED_LOWER_1, cam1_RED_UPPER_1, cam1_GREEN_LOWER, cam1_GREEN_UPPER, cam1_YELLOW_LOWER, cam1_YELLOW_UPPER), name = "1")
    t2 = threading.Thread(target = camWrapper, args = (2, cam2_RED_LOWER_0, cam2_RED_UPPER_0, cam2_RED_LOWER_1, cam2_RED_UPPER_1, cam2_GREEN_LOWER, cam2_GREEN_UPPER, cam2_YELLOW_LOWER, cam2_YELLOW_UPPER), name = "2")
    t3 = threading.Thread(target = camWrapper2, args = [1], name = "3")
    t4 = threading.Thread(target = camWrapper2, args = [2], name = "4")
    
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    while True: #make the main thread sleep
        time.sleep(60)


cam1.release()
cam2.release()

#print("--- %s seconds ---" % (time.time() - start_time))

