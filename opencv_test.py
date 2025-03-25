
import cv2

# Open the default camera (0 = first camera device)
#cap = cv2.VideoCapture(0)

# Try the web-hosed ESP32-Cam
URL = "http://192.168.1.56"
cap = cv2.VideoCapture(URL + ":81/stream")ch

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the resulting frame
    cv2.imshow('Video Stream', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture when done
cap.release()
cv2.destroyAllWindows()
