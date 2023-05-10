import time
import threading
import cv2
import numpy as np
import pytesseract
from PIL import Image
import RPi.GPIO as GPIO
import serial
import numpy as np

start_time = time.time()

#start cameras
cam1 = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(1)

#color HSV bounds
cam1_RED_LOWER_0 = np.array([0, 140, 75])
cam1_RED_UPPER_0 = np.array([10, 255, 255])
cam1_RED_LOWER_1 = np.array([165, 140, 75])
cam1_RED_UPPER_1 = np.array([180, 255, 255])

cam1_GREEN_LOWER = np.array([65, 90, 50])
cam1_GREEN_UPPER = np.array([90, 255, 255])

cam1_YELLOW_LOWER = np.array([10, 125, 125])
cam1_YELLOW_UPPER = np.array([35, 255, 255])


cam2_RED_LOWER_0 = np.array([0, 70, 25])
cam2_RED_UPPER_0 = np.array([10, 255, 255])
cam2_RED_LOWER_1 = np.array([165, 70, 25])
cam2_RED_UPPER_1 = np.array([180, 255, 255])

cam2_GREEN_LOWER = np.array([75, 65, 10])
cam2_GREEN_UPPER = np.array([100, 255, 255])

cam2_YELLOW_LOWER = np.array([10, 70, 25])
cam2_YELLOW_UPPER = np.array([45, 255, 255])


#colors
def colorRec(index, redLower0, redUpper0, redLower1, redUpper1, greenLower, greenUpper, yellowLower, yellowUpper):
    #read image
    if index == 1:
        v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
    
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
    
    #apply contours
    blur = cv2.blur(img,(7,7))

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray,250,255,cv2.THRESH_BINARY_INV)

    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    #contours must meet area requirements and have 4 sides
    for c in contours:
        area = cv2.contourArea(c, False)
        if area > 20000:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*peri, True)
            if len(approx) == 4:
                #crop image within contour
                M = cv2.moments(c)
                centerX = int(M['m10']/M['m00'])
                centerY = int(M['m01']/M['m00'])
                
                img = img[centerY-70:centerY+70, centerX-70:centerX+70]
                
                #average color in cropped image
                average = img.mean(axis=0).mean(axis=0)

                #colors
                green = [0, 255, 0]
                yellow = [0, 255, 255]
                red = [0, 0, 255]
                white = [255, 255, 255]
                
                #find closest color value
                colors = np.array([green, yellow, red, white])
                distances = np.sqrt(np.sum((colors-average)**2, axis=1))
                idx = np.where(distances==np.amin(distances))
                closest = colors[idx].tolist()

                if (closest == green):
                    print(f"cam {index}: green")
                elif (closest == yellow):
                    print(f"cam {index}: yellow")
                elif (closest == red):
                    print(f"cam {index}: red")
                elif (closest == white):
                    print(f"cam {index}: nothing")


#letters
def letterRec(index):
    #read image
    if index == 1:
        v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
    
    #color bounds
    BLACK_LOWER = 0
    BLACK_UPPER = 50
    
    #convert to grayscale and threshold darkest pixels
    blur = cv2.blur(img,(7,7))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
    
    #find contours
    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    #contours must be fully in the image, meet area requirements, and have a certain height:width ratio
    for c in contours:
        x, y, w1, h1 = cv2.boundingRect(c)
        if x>0 and x+w1<640 and y>0 and y+h1<480:
            area = w1*h1
            if area > 20000:
                rect = cv2.minAreaRect(c)
                cx, cy = rect[0]
                w, h = rect[1]
                angle = rect[2]
                area = w*h
                if 1 <= h/w <= 1.6 or 1 <= w/h <= 1.6:
                    #rotate image so letter is upright
                    if w<h:
                        rot = cv2.getRotationMatrix2D((cx, cy), angle, 1)
                        thresh = cv2.warpAffine(thresh, rot, (640, 480))
                        thresh = thresh[int(cy-h/2)-5: int(cy+h/2)+5, int(cx-w/2)-5: int(cx+w/2)+5]
                    else:
                        rot = cv2.getRotationMatrix2D((cx, cy), angle+90, 1)
                        thresh = cv2.warpAffine(thresh, rot, (640, 480))
                        thresh = thresh[int(cy-w/2)-5: int(cy+w/2)+5, int(cx-h/2)-5: int(cx+h/2)+5]

                    ud = cv2.rotate(thresh, cv2.ROTATE_180)
                    target = pytesseract.image_to_string(thresh, lang='eng', boxes=False, config='--psm 10 --oem 3 -c tessedit_char_whitelist=hHsSuU')
                    target2 = pytesseract.image_to_string(ud, lang='eng', boxes=False, config='--psm 10 --oem 3 -c tessedit_char_whitelist=hHsSuU')
                    if (target == "H" or targe2 == "H"):
                        print(f"tesscam {index}: H")
                    elif (target == "S" or targe2 == "S"):
                        print(f"tesscam {index}: S")
                    elif (target == "U" or targe2 == "U"):
                        print(f"tesscam {index}: U")
                    else:
                        print(f"tesscam {index}: nothing")
                    
                    #divide image in half horizontally, in thirds vertically
                    rows, cols = thresh.shape

                    left_thresh = thresh[0: rows, 0: int(cols/2)]
                    right_thresh = thresh[0: rows, int(cols/2): cols]

                    top_thresh = thresh[0: int(rows/3), 0: cols]
                    mid_thresh = thresh[int(rows/3): int(2*rows/3), 0: cols]
                    bot_thresh = thresh[int(2*rows/3): rows, 0: cols]
                    
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
                    
                    #return result
                    if len(L) == 2 and len(R) == 2:
                        print(f"cam {index}: S")
                    elif len(T) == 2 and len(M) == 2 and len(B) == 1:
                        print(f"cam {index}: U")
                    elif len(T) == 1 and len(M) == 2 and len(B) == 2:
                        print(f"cam {index}: U")
                    elif len(T) == 2 and len(M) == 1 and len(B) == 2:
                        print(f"cam {index}: H")
                    else:
                        print(f"cam {index}: nothing")


#run color recognition
colorRec(1, cam1_RED_LOWER_0, cam1_RED_UPPER_0, cam1_RED_LOWER_1, cam1_RED_UPPER_1, cam1_GREEN_LOWER, cam1_GREEN_UPPER, cam1_YELLOW_LOWER, cam1_YELLOW_UPPER)
colorRec(2, cam2_RED_LOWER_0, cam2_RED_UPPER_0, cam2_RED_LOWER_1, cam2_RED_UPPER_1, cam2_GREEN_LOWER, cam2_GREEN_UPPER, cam2_YELLOW_LOWER, cam2_YELLOW_UPPER)

#run letter recognition
letterRec(1)
letterRec(2)

print("--- %s seconds ---" % (time.time() - start_time))
