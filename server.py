import cv2
import serial
import time

# Initialize serial communication
SERIAL_PORT = 'COM6'  # Change this to the port your microcontroller is connected to
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Give the serial connection some time to initialize

# Initialize the webcam
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Load the Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Get the camera frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
center_x, center_y = frame_width // 2, frame_height // 2

try:
    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Convert the frame to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the frame
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=1, minSize=(100, 100))

        # Process the first detected face (if any)
        if len(faces) > 0:
            x, y, w, h = faces[0]  # Coordinates and size of the detected face
            face_center_x = x + w // 2
            face_center_y = y + h // 2

            # Calculate the offset from the center
            offset_x = face_center_x - center_x
            offset_y = center_y - face_center_y  # Invert y-axis to match Cartesian coordinates

            # Send the offset to the microcontroller
            data = f"{offset_x},{offset_y}\n"
            ser.write(data.encode())
            print(data)

            # Draw a rectangle around the face and indicate the center
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)
        else:
            # No face detected, send neutral position (0,0)
          ser.write(b"0,0\n")
          print("0,0")

        # Draw the center of the frame
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # Display the frame
        cv2.imshow('Face Tracking', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
