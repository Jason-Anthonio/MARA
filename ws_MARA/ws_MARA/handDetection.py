import cv2
import mediapipe as mp
import time

def run_hand_tracking():
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

    cap = cv2.VideoCapture(0) 
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n--- HAND TRACKING ---")
    print("1. Point with your index finger to target.")
    print("2. Make a tight fist and hold for 2 seconds to confirm.")

    hover_coords = None
    fist_start_time = None
    CONFIRM_TIME = 2.0  # Seconds required to hold the fist

    while cap.isOpened():
        success, img = cap.read()
        if not success: break
        
        img = cv2.flip(img, 1)
        h, w, c = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        status_text = "WAITING FOR POINTING POSE"
        status_color = (0, 0, 255) # Red

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                
                # Finger states (y-axis is inverted in OpenCV, smaller y is "higher")
                index_up = hand_lms.landmark[8].y < hand_lms.landmark[6].y
                index_down = hand_lms.landmark[8].y > hand_lms.landmark[6].y
                middle_down = hand_lms.landmark[12].y > hand_lms.landmark[10].y
                ring_down = hand_lms.landmark[16].y > hand_lms.landmark[14].y
                pinky_down = hand_lms.landmark[20].y > hand_lms.landmark[18].y

                # Strict logic
                is_pointing = index_up and middle_down and ring_down and pinky_down
                is_fist = index_down and middle_down and ring_down and pinky_down

                if is_pointing:
                    fist_start_time = None 
                    
                    x_min, y_min = w, h
                    x_max, y_max = 0, 0
                    depth_z = hand_lms.landmark[0].z

                    for lm in hand_lms.landmark:
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        x_min, x_max = min(x_min, cx), max(x_max, cx)
                        y_min, y_max = min(y_min, cy), max(y_max, cy)

                    center_x = int((x_min + x_max) / 2)
                    center_y = int((y_min + y_max) / 2)
                    
                    hover_coords = (center_x, center_y, depth_z)

                    mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
                    cv2.rectangle(img, (x_min-20, y_min-20), (x_max+20, y_max+20), (0, 255, 0), 2)
                    
                    coord_text = f"X:{center_x} Y:{center_y} Z:{depth_z:.2f}"
                    cv2.putText(img, coord_text, (x_min, y_min-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    status_text = "TARGETING - MAKE A FIST TO LOCK"
                    status_color = (0, 255, 255) # Yellow

                elif is_fist and hover_coords:
                    if fist_start_time is None:
                        fist_start_time = time.time()
                    
                    elapsed_time = time.time() - fist_start_time
                    
                    bar_width = int(400 * (elapsed_time / CONFIRM_TIME))
                    cv2.rectangle(img, (50, 400), (50 + bar_width, 430), (0, 255, 0), -1)
                    cv2.rectangle(img, (50, 400), (450, 430), (255, 255, 255), 2)
                    
                    status_text = f"LOCKING IN... {elapsed_time:.1f}s"
                    status_color = (0, 255, 0) # Green

                    if elapsed_time >= CONFIRM_TIME:
                        cap.release()
                        cv2.destroyAllWindows()
                        print(f"\n[System: Coordinates Locked via fist gesture at {hover_coords}]")
                        return hover_coords
                else:
                    fist_start_time = None

        cv2.putText(img, f"STATUS: {status_text}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.imshow("Pose Sensitive Tracker", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return hover_coords