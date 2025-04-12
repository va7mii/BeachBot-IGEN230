import os
import sys
import argparse
import glob
import time

import cv2
import numpy as np
from ultralytics import YOLO

import serial
import time
import threading

import socket

#Serial information
port = 'COM5'
baudrate = 115200
URL = "http://192.168.137.186" # for camera web server

#ESP Wifi Info
ESP32_IP = "192.168.137.73"
PORT = 23  # Match the port you used in ESP32 code

send_interval = 1 # amount of time video streams until it sends it to the ESP32

def send_command(command):
    try:
        # Create a TCP/IP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ESP32_IP, PORT))
        
        # Send command (must be bytes)
        s.sendall(command.encode())  # e.g., "1" or "LED ON"
        
        # Optionally read response
        #response = s.recv(1024)
        #print("ESP32 says:", response.decode())
        
        s.close()
    except Exception as e:
        print("Error:", e)

# Reading serial thread
def read_from_esp32(ser):
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"[ESP32] {line}")
        except Exception as e:
            print(f"[Read error] {e}")
            break

    

# Time keeping to slow down sending stuff from computer to ESP32
counter = 0
start_time = time.time();






# Define and parse user input arguments

parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--debug', help='Activate COM port to ESP32 via USB to read Serial commandss',
                    default=False)
parser.add_argument('--webcam', help='Use webcam instead of ESP32Cam for debugging',
                    default=False)

args = parser.parse_args()




# Parse user inputs
model_path = args.model
min_thresh = args.thresh
user_res = args.resolution
debug_mode = args.debug
webcam_mode = args.webcam


#Connect to USB Serial if debug mode activated
if (debug_mode):
    # Start serial with ESP32
    ser = serial.Serial(port, baudrate,dsrdtr=None, timeout=1)
    ser.setRTS(False)
    ser.setDTR(False)
    print("Connected to ESP32 on COM5.")

    # Start the reader thread (comment it when USB not connect to ES)
    reader_thread = threading.Thread(target=read_from_esp32, args=(ser,), daemon=True)
    reader_thread.start()

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])



if (webcam_mode):
    cap = cv2.VideoCapture(0)
else:
    cap = cv2.VideoCapture(URL + ":81/stream")

#Using ESP-CAM
#cap = cv2.VideoCapture("http://192.168.1.56:81/stream")

# Set camera or video resolution if specified by user
if user_res:
    ret = cap.set(3, resW)
    ret = cap.set(4, resH)

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# Begin inference loop

frame_counter = 0;
sum_objects = 0;

while True:

    t_start = time.perf_counter()
    
    # If source is a USB camera, grab frame from camera
    ret, frame = cap.read()
    if (frame is None) or (not ret):
        print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')
        break
    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame,(resW,resH))

    # Run inference on frame
    results = model(frame, verbose=False)

    # Extract results
    detections = results[0].boxes

    # Initialize variable for basic object counting example
    object_count = 0

    # Go through each detection and get bbox coords, confidence, and class
    for i in range(len(detections)):

        # Get bounding box coordinates
        # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
        xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
        xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
        xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

        # Get bounding box class ID and name
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]

        # Get bounding box confidence
        conf = detections[i].conf.item()

        # Draw box if confidence threshold is high enough
        if conf > 0.5:

            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

            

            # Basic example: count the number of objects in the image
            object_count = object_count + 1

    # Calculate and draw framerate (if using video, USB, or Picamera source)
    cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
    
    # Display detection results
    cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
    cv2.imshow('YOLO detection results',frame) # Display image



    # Write object detection info to ESP32 via Serial

    # Sends a "moving average" of number of objects detected over a certain period, to take account to objects "flashng for a sec"
    current_time = time.time()

    if (current_time - start_time >= send_interval and (frame_counter != 0)):
        start_time = time.time()

        print("sum" + str(sum_objects) +",  ")
        print("frame_counter" + str(frame_counter) + ",  ")

        confidence = round((sum_objects / frame_counter), 6)

        print("confidence" + str(confidence) + "\n")
        cmd = str(confidence)
        send_command(cmd+"\n") # Send the command
        

        #reset counter and sum
        frame_counter = 0
        sum_objects = 0
    else:
        frame_counter += 1
        sum_objects += object_count

    #     cmd = str(object_count)
    #     send_command(cmd) #Send the command



    # If inferencing on individual images, wait for user keypress before moving to next image. Otherwise, wait 5ms before moving to next frame.
    key = cv2.waitKey(5)
    
    if key == ord('q') or key == ord('Q'): # Press 'q' to quit
        break
    elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
        cv2.waitKey()
    elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
        cv2.imwrite('capture.png',frame)
    
    # Calculate FPS for this frame
    t_stop = time.perf_counter()
    frame_rate_calc = float(1/(t_stop - t_start))

    # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
    if len(frame_rate_buffer) >= fps_avg_len:
        temp = frame_rate_buffer.pop(0)
        frame_rate_buffer.append(frame_rate_calc)
    else:
        frame_rate_buffer.append(frame_rate_calc)

    # Calculate average FPS for past frames
    avg_frame_rate = np.mean(frame_rate_buffer)




# Clean up
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
cap.release()
cv2.destroyAllWindows()
