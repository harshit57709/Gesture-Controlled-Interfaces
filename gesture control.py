from detecting_hands import DetectHand
import cv2
import mediapipe as mp
import numpy as np
import math
import numpy
import autopy
from autopy.mouse import Button
import keyboard
import time


screen_width , screen_height = autopy.screen.size()
smoothening_factor = 4.5
prev_x, prev_y = 0,0
curr_x, curr_y = 0,0
print(screen_width , screen_height )
detector = DetectHand(min_detection_confidence = 0.7)
start_point = (150, 100)
end_point   = (500, 300)
cap = cv2.VideoCapture(0)
while(cap.isOpened()):
    '''resolution of cam = 480*640
    resolution of screem = 1920*1080'''
    success, frame = cap.read()

    image = detector.detect_hands(frame, draw = 0)
    cv2.rectangle(image,start_point, end_point, (0,0,255), 2)
    length = detector.get_coordinates(image, draw = 0)
    cam_height,cam_width,  n = image.shape
    #gets tips of index and middle fingers_up

    if(len(detector.finger_tips) > 0):
        desired_tips = detector.finger_tips[0:3]

        distance0_1 = detector.find_distance(desired_tips[0][1:], desired_tips[2][1:])
        #dicide the mode of operation based on how many fingers are raised

    fingers_up = detector.fingers_up(image)


    if(len(fingers_up) > 0):
        detector.mark_points(image,fingers_up)
        idx, x1, y1 = desired_tips[1]
        #convert cooridantes since resolution isnt same
        if((start_point[0]<x1 <end_point[0]) and (start_point[1]<y1 <end_point[1])):
            cv2.rectangle(image,start_point, end_point, (0,255,0), 2)
            x1_internp, y1_internp = np.interp(x1, (start_point[0], end_point[0]), (0, screen_width)), np.interp(y1, (start_point[1], end_point[1]), (0, screen_height))

            #right click funcationality

            if(len(fingers_up) == 2 and detector.right_clicked == 0):
                autopy.mouse.click(Button.RIGHT)
                detector.right_clicked = 1;
                continue
            if(len(fingers_up) == 1):
                detector.right_clicked = 0;
            #-----------------------------------------------------------------
            if(distance0_1 < 65):
                curr_x = prev_x + (x1_internp - prev_x)/smoothening_factor
                curr_y = prev_y + (y1_internp - prev_y)/smoothening_factor
                detector.mode =0
                autopy.mouse.move(curr_x, curr_y)
                prev_x , prev_y = curr_x, curr_y
        # if in clicking mode, do a click based upon distance between the fingers ,,,,,,,,,mode is introdiced s oas to register onlu one click
            elif(detector.mode == 0):
                autopy.mouse.click()
                detector.mode =1
        #when outside the box and one finger is raised switch to left right key control
        elif(start_point[0]>x1):
            keyboard.press_and_release("left")
        elif(end_point[0]<x1):
            print("right")



        '''fixed required - respomse to movement isnt good on edges'''  #--fixed (a tracking region is introduced)
        '''fixed required - multiple clikcs registered'''                #--fixed (a mode is introduced)
        '''fixed required - the movement is way to shaky'''             #--fixed (movement is smoothened by a specific smoothening factor)
    cv2.imshow("hands", image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
