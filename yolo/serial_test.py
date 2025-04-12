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


import serial
import threading

# Parse user inputs
model_path = "yolo11n.pt"
img_source = "usb0"
#min_thresh = args.thresh
user_res = "320x240"

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

def write_to_esp32(ser):
    print("Type commands to send to ESP32. Type 'exit' to quit.")
    while True:
        try:
            cmd = input(">>> ")
            if cmd.lower() == "exit":
                break
            ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Write error] {e}")
            break
        
if __name__ == '__main__':
    try:
        ser = serial.Serial('COM5', 115200, timeout=1)
        print("Connected to ESP32 on COM5.")
        
        # Start the reader thread
        reader_thread = threading.Thread(target=read_from_esp32, args=(ser,), daemon=True)
        reader_thread.start()

        # Main thread handles writing
        write_to_esp32(ser)

        ser.close()
        print("Connection closed.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")