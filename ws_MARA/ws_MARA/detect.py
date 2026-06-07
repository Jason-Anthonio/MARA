from ultralytics import YOLO
import cv2 as cv
import time
import math

model = YOLO('yolo11n.pt') 

def run_vision_system(target_object=None):
    capture = cv.VideoCapture(0) 
    
    capture.set(cv.CAP_PROP_BUFFERSIZE, 1)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    final_counts = {}
    final_found = False
    final_coords = None

    print(f"\n--- YOLO SEARCHING FOR: '{target_object.upper()}' ---")

    # --- Lock & Timeout Variables ---
    LOCK_TIME = 2.0  
    STABILITY_RADIUS = 50  
    SEARCH_TIMEOUT = 10.0  # Gives up if target isn't found in 10 seconds
    
    lock_start_time = None
    last_stable_coords = None
    search_start_time = time.time()

    while True:
        ret, frame = capture.read()
        if not ret: break

        results = model(frame, conf=0.7)
        display = results[0].plot()

        current_counts = {}
        target_found_this_frame = False
        
        best_coords_this_frame = None
        best_y_bottom = 0  
        best_area = 0
        LEVEL_THRESHOLD = 40  

        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            class_name = results[0].names[class_id]
            current_counts[class_name] = current_counts.get(class_name, 0) + 1

            if target_object and (target_object in class_name or class_name in target_object):
                target_found_this_frame = True
                
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                area = (x2 - x1) * (y2 - y1)
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                y_bottom = y2 

                if best_coords_this_frame is None:
                    best_y_bottom = y_bottom
                    best_area = area
                    best_coords_this_frame = (cx, cy)
                else:
                    if abs(y_bottom - best_y_bottom) < LEVEL_THRESHOLD:
                        if area > best_area:
                            best_y_bottom = y_bottom
                            best_area = area
                            best_coords_this_frame = (cx, cy)
                    elif y_bottom > best_y_bottom:
                        best_y_bottom = y_bottom
                        best_area = area
                        best_coords_this_frame = (cx, cy)

        # --- The Timeout Logic ---
        if not target_found_this_frame and last_stable_coords is None:
            elapsed_search = time.time() - search_start_time
            cv.putText(display, f"SEARCHING... TIMEOUT IN {SEARCH_TIMEOUT - elapsed_search:.1f}s", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if elapsed_search > SEARCH_TIMEOUT:
                print(f"\n[System: Timeout. '{target_object}' not found in workspace.]")
                break
        else:
            # If we see it, reset the search timeout so we have time to lock
            search_start_time = time.time()

        # --- Spatial Auto-Lock Logic ---
        if target_found_this_frame and best_coords_this_frame:
            cx, cy = best_coords_this_frame

            if last_stable_coords is None:
                last_stable_coords = (cx, cy)
                lock_start_time = time.time()
            else:
                drift_distance = math.hypot(cx - last_stable_coords[0], cy - last_stable_coords[1])

                if drift_distance <= STABILITY_RADIUS:
                    elapsed_time = time.time() - lock_start_time
                    
                    bar_width = int(400 * (elapsed_time / LOCK_TIME))
                    cv.rectangle(display, (50, 400), (50 + bar_width, 430), (0, 255, 0), -1)
                    cv.rectangle(display, (50, 400), (450, 430), (255, 255, 255), 2)
                    cv.putText(display, f"STABLE: LOCKING IN... {elapsed_time:.1f}s", (50, 380), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    if elapsed_time >= LOCK_TIME:
                        print(f"\n[System: Target locked automatically at {best_coords_this_frame}]")
                        final_counts = current_counts
                        final_found = target_found_this_frame
                        final_coords = best_coords_this_frame
                        break
                else:
                    last_stable_coords = (cx, cy)
                    lock_start_time = time.time()
                    cv.putText(display, "TARGET MOVING - HOLD STILL", (50, 380), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv.putText(display, f"TRACKING: X:{cx} Y:{cy}", (50, 80), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            cv.circle(display, (cx, cy), 8, (0, 0, 255), -1)
            cv.line(display, (cx-20, cy), (cx+20, cy), (0, 255, 0), 2)
            cv.line(display, (cx, cy-20), (cx, cy+20), (0, 255, 0), 2)

        else:
            last_stable_coords = None
            lock_start_time = None

        cv.imshow("Vision Subsystem", display)

        if cv.waitKey(1) & 0xFF == ord('q'):
            final_counts = current_counts
            final_found = target_found_this_frame
            final_coords = best_coords_this_frame
            break

    capture.release()
    cv.destroyAllWindows()
    return final_counts, final_found, final_coords