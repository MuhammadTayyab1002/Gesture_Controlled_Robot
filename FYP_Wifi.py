import mediapipe as mp
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector
import time
from serial import Serial
import socket
ESP_IP = '10.4.5.94'
ESP_PORT = 8888
#Getting Models


detector=HandDetector(detectionCon=0.8,maxHands=1)
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
mp_pose=mp.solutions.pose
mp_Hand=mp.solutions.hands


#myserial = Serial('COM7',9600)
cap = cv2.VideoCapture(0)
hands=mp_Hand.Hands(static_image_mode=False,max_num_hands=2,min_detection_confidence=0.8)
# Initiate holistic model
def calculate_angle(a,b,c):
    a=np.array(a)
    b=np.array(b)
    c=np.array(c)
    radians=np.arctan2(c[1]-b[1],c[0]-b[0])-np.arctan2(a[1]-b[1],a[0]-b[0])
    angle =np.abs(radians*180/np.pi)
    if angle >180:
        angle=360-angle
    angle=int(angle)
    return angle
def calculate_angle_1(a,b,c):
    a=np.array(a)
    b=np.array(b)
    c=np.array(c)
    radians=np.arctan2(c[1]-b[1],c[0]-b[0])-np.arctan2(a[1]-b[1],a[0]-b[0])
    angle =np.abs(radians*180/np.pi)
    angle=int(angle)
    return angle
def pad_zeros(arr):
    send=""
    for i in range(len(arr)):
        if len(str(arr[i])) < 3:
            send = send+str(arr[i]).zfill(3)
        else:
            send=send+str(arr[i])
    return send
def finger_angle(val):
    if val==1:
        angle = 0
    elif val==0:
        angle=90
    return angle
Thumb_angle=0
Index_angle=0
Middle_angle=0
Ring_angle=0
Pinky_angle=0
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
   ## myserial.write(str(0).encode())
    
    while cap.isOpened():
        t = time.time()
        ret, frame = cap.read()
        # Recolor Feed
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Make Detections
        results = holistic.process(image)
        results_1= hands.process(image) 
        #  pose_landmarks, left_hand_landmarks, right_hand_landmarks
        
        # Recolor image back to BGR for rendering
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        ##
        
        #extract landmarks
        try:
            landmarks=results.pose_landmarks.landmark
            landmarks_1=results.right_hand_landmarks.landmark
#        print(landmarks)
        except:
            pass

        # Right hand
       # mp_drawing.draw_landmarks(image, results_1.handLandmarks, mp_Hand.HAND_CONNECTIONS)

        # Pose Detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
        #Frame
        cv2.imshow('Gesture Control Robot', cv2.flip(image,1))
        if results_1.multi_hand_landmarks:
            for handLandmarks in results_1.multi_hand_landmarks:
                lmList, bbox = detector.findHands(image, handLandmarks)
                if lmList:
              
                    fingers = detector.fingersUp(lmList[0])
                    # Detect open or closed fingers
                    # 1 - Open finger, 0 - Closed finger
                   # COnverting Finger angles
                    Thumb_angle=finger_angle(fingers[0])
                    Index_angle=finger_angle(fingers[1])
                    Middle_angle=finger_angle(fingers[2])
                    Ring_angle=finger_angle(fingers[3])
                    Pinky_angle=finger_angle(fingers[4])
                    # Example output format
                    # fingers = [0, 1, 0, 1, 0]  # Thumb, Index, Middle, Ring, Pinky
           
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        #Calculating Angle
        
       
        
        left_middle = [landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].y]
        shoulder_R = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
        shoulder_L = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
        elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
        wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
        hip_joint = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]    

        shoulder_angle_UD=calculate_angle(hip_joint, shoulder_R, elbow)
        shoulder_angle_SW=calculate_angle_1(shoulder_L, shoulder_R, elbow)
        shoulder_angle_SW=270-shoulder_angle_SW
        elbow_angle=calculate_angle(shoulder_R, elbow,wrist)
        elbow_angle=180-elbow_angle
        wrist_angle=calculate_angle(elbow,wrist,left_middle)
        
        
        
        #Assiging Data to array
        data=[Thumb_angle,Index_angle,Middle_angle,Ring_angle,Pinky_angle,shoulder_angle_UD,elbow_angle,shoulder_angle_SW]
        Send=pad_zeros(data)
        message = Send
        print(message)
        t1 = time.time()
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the ESP
        sock.connect((ESP_IP, ESP_PORT))


        # Send the message to the ESP
        sock.sendall(str(message).encode())
        time.sleep(0.8)
       # myserial.write(str(Send).encode())
        elapsed1 = time.time() - t1
        print('Sending Time = ',elapsed1)
        elapsed = time.time() - t
        print('TOC = ',elapsed)

        


cap.release()
cv2.destroyAllWindows()