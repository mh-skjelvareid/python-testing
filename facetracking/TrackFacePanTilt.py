# -*- coding: utf-8 -*-
"""
Detect a face in a video stream and track it using a pan-tilt camera, keeping
the face in the center of the image.

The code relies on a Haar cascade face detector, and the pan-tilt platform 
is controlled by serial communication with an Arduino.

@author: Martin H. Skjelvareid, UiT
"""

# %% Imports
import cv2
import serial
import time
import numpy as np

# %% Set fixed parameters
panGain = 0.02          # Parameter for adjusting servo pan position
tiltGain = 0.035         # Parameter for adjusting servo tilt position
panLimits = (0,170)     # Min / max pan angle
tiltLimits = (65,150)   # Min / max tilt angle
desFacePos = (0.5,0.6)  # Desired face center, relative 

cascPath = 'haarcascade_frontalface_default.xml'    # Face detection file
scaleFactor = 1.15      # Difference between scales used for detection
minNeighbors = 5        # Lower number: Higher sensitivity (?), higher chance of error
minSize = (60, 60)      # Minimum face size [pixels]
maxSize = (350,350)     # Maximum face size [pixels]
relayWaitPeriod = 5;    # How long to wait between activating relay
faceCloseThreshold = 0.17;   # Relative size of face vs screen considered "close"
noFacesResetTime = 10.0;
defaultPanAngle = 90.0;
defaultTiltAngle = 110.0;

# %% Initialize variables / objects
panAngle = defaultPanAngle
tiltAngle = defaultTiltAngle
video_capture = cv2.VideoCapture(0)
faceCascade = cv2.CascadeClassifier(cascPath)
arduino = serial.Serial('/dev/ttyACM0', 115200)   # create serial object named arduino
refTime = time.perf_counter()
noFaceCounter = time.perf_counter()

# %% Methods for changing camera angle
def updateServoPos(panAngle,tiltAngle):
    """ Generate bytestrings for updating servo angles """
    panAngle = np.clip(panAngle,panLimits[0],panLimits[1])
    tiltAngle = np.clip(tiltAngle,tiltLimits[0],tiltLimits[1])
      
    sendServoPos('P' + str(int(round(panAngle))) + '\n')
    sendServoPos('T' + str(int(round(tiltAngle))) + '\n')
    
def sendServoPos(posString):
    """ Write bytestring to arduino and print response """
    arduino.write(posString.encode())
    response = arduino.readline() 
    print(response.decode('ascii').rstrip())    
    
    
# %% Initial code (run once)
time.sleep(2)                               # Let serial connection be established
updateServoPos(panAngle,tiltAngle)  # Set original tilt

# %% Create window
window = cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Video', 900,900)

# %% Main loop
try:
    while True:
        # Capture frame-by-frame, convert to greyscale for face detection
        ret, frame = video_capture.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (frameHeight,frameWidth,nChannels) = frame.shape
        
        # Detect faces
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=scaleFactor,
            minNeighbors=minNeighbors,
            minSize=minSize,
            maxSize=maxSize,
            flags=cv2.CASCADE_SCALE_IMAGE)
        
        # Process detected faces
        if not (len(faces)==0):                 # In any faces detected
            # Find positions of faces relative to image center
            faceCenterX = faces[:,0] + faces[:,2]/2     # left edge + half width
            faceCenterY = faces[:,1] + faces[:,3]/2     # lower edge + half height
            xOffset = faceCenterX - frameWidth*(1-desFacePos[0]);
            yOffset = faceCenterY - frameHeight*(1-desFacePos[1]);
            rOffset = np.sqrt(xOffset**2 + yOffset**2)  # Radius from image center
            
            # Find face closest to center, calculate position error
            index_rOffsetMin = np.argmin(rOffset)
            xError = xOffset[index_rOffsetMin]
            yError = yOffset[index_rOffsetMin]
            
            # Update camera angle to reduce x and y error
            panAngle -= panGain*xError;
            tiltAngle -= tiltGain*yError;        
            updateServoPos(panAngle,tiltAngle)  
            
            # Trigger relay if face is close enough
            if (faces[index_rOffsetMin,2] > frameWidth*faceCloseThreshold):
                if time.perf_counter() > refTime + relayWaitPeriod:
                    refTime = time.perf_counter()
                    sendServoPos('R2\n')    # Send trigger code
    
            # Draw rectangle(s) around face(s)
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
            # Reset "no faces" counter
            noFaceCounter = time.perf_counter()
            
        # Reset camera angle if "no faces" timeout
        if time.perf_counter() > noFaceCounter + noFacesResetTime:
            updateServoPos(defaultPanAngle,defaultTiltAngle)
    
        # Display the resulting frame (with or without faces)
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    #  Clean up
    video_capture.release()
    cv2.destroyAllWindows()
    arduino.close()
