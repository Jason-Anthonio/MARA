from ultralytics import YOLO
import cv2 as cv
from collections import Counter

model = YOLO('yolo11n.pt')

capture = cv.VideoCapture(0) #use 1 for external webcam, but it's quite laggy for usb 3.0, or could be from the load displaying yolo visuals
capture.set(cv.CAP_PROP_FRAME_WIDTH, 1280)  # Set the width
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 720)  # Set the height

while True:
    ret, frame = capture.read()  # Read a frame from the webcam
    results = model(frame, conf=0.7)  
    display = results[0].plot()  # Get the display image with bounding boxes
    c = results[0].boxes.cls
    cc = c.numpy()
    ccc = results[0].names

    class_counts = Counter(ccc[cc[i]] for i in range(len(cc))) #counts the number of same ids
    print("Detected objects:", dict(class_counts)) #displays the each id detected and the number of them
    print("Box Positions:", results[0].boxes.xyxy) 
    
    for box in results[0].boxes.xyxy:
        x1, y1, x2, y2 = map(int, box.cpu().numpy())
        centerx = x1 + (x2 - x1) // 2
        centery = y1 + (y2 - y1) // 2
        cv.line(display, (centerx, 0), (centerx, 720), (255, 255, 255), 2)
        cv.line(display, (0, centery), (1280, centery), (255, 255, 255), 2)
        coordinates = f"({centerx}, {centery})"
        cv.putText(display, coordinates, (centerx + 100, centery - 25), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv.imshow("YOLO Object Detector", display) 

    if cv.waitKey(1) & 0xFF == ord('q'): #only turns off window when Q is pressed
        break