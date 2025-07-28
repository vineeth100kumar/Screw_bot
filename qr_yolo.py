import cv2
import socket
import threading
import time
import numpy as np
from pyzbar.pyzbar import decode
from ultralytics import YOLO

# --- CONFIGURATION ---
RASPBERRY_PI_IP = "192.168.231.39"
MJPEG_PORT = 8090
TCP_QR_PORT = 5001

REAL_QR_WIDTH_CM = 5.0
FOCAL_LENGTH_PIXELS = 700.0
SEND_DISTANCE_THRESHOLD_CM = 12.0

laptop_detected_qr_data = "Waiting for QR..."
qr_detection_lock = threading.Lock()

# --- TCP CLIENT THREAD ---
def tcp_qr_client_thread():
    global laptop_detected_qr_data
    print(f"Attempting to connect to TCP QR server on Pi at {RASPBERRY_PI_IP}:{TCP_QR_PORT}")

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((RASPBERRY_PI_IP, TCP_QR_PORT))
                sock.settimeout(5)
                print("Connected to TCP QR server on Pi.")

                while True:
                    with qr_detection_lock:
                        if laptop_detected_qr_data and \
                           "Waiting" not in laptop_detected_qr_data and \
                           "NO_QR" not in laptop_detected_qr_data and \
                           "TOO_FAR" not in laptop_detected_qr_data:
                            data_to_send = laptop_detected_qr_data
                        else:
                            data_to_send = "NO_QR_FROM_LAPTOP"

                    sock.sendall(f"{data_to_send}\n".encode('utf-8'))

                    received_buffer = b''
                    while True:
                        part = sock.recv(1)
                        if not part:
                            raise ConnectionResetError("Socket connection broken")
                        if part == b'\n':
                            break
                        received_buffer += part

                    received_ack = received_buffer.decode('utf-8').strip()
                    print(f"Received ACK from Pi: {received_ack}")
                    time.sleep(0.5)

        except Exception as e:
            print(f"TCP QR Client Error: {e}. Reconnecting...")
            time.sleep(3)

# --- VIDEO + QR + HUMAN DETECTION LOOP ---
def main_video_display_loop():
    global laptop_detected_qr_data

    mjpeg_url = f"http://{RASPBERRY_PI_IP}:{MJPEG_PORT}/stream.mjpg"
    print(f"Connecting to: {mjpeg_url}")
    cap = cv2.VideoCapture(mjpeg_url)

    if not cap.isOpened():
        print("Failed to open MJPEG stream.")
        return

    # Video recorder
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('recorded_feed.avi', fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))

    # Load YOLOv5 nano
    print("Loading YOLOv5n model from Ultralytics...")
    model = YOLO("yolov5n.pt")  # Automatically downloads if not present

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame. Reconnecting...")
                cap.release()
                time.sleep(2)
                cap = cv2.VideoCapture(mjpeg_url)
                continue

            decoded_objects = decode(frame)
            detected_this_frame_status = "NO_QR"
            current_estimated_distance_cm = None
            estimated_distance_display = "N/A"
            qr_data = None

            # --- HUMAN DETECTION USING YOLO ---
            results = model.predict(source=frame, verbose=False)
            person_detected = False
            detections = results[0].boxes
            if detections is not None:
                for box in detections:
                    cls_id = int(box.cls)
                    class_name = model.names[cls_id]
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy[:4])

                    if class_name == 'person':
                        person_detected = True
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame, "HUMAN", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    else:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                        cv2.putText(frame, class_name, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # --- QR DETECTION ---
            if decoded_objects:
                obj = decoded_objects[0]
                qr_data = obj.data.decode("utf-8")
                points = obj.polygon

                if len(points) == 4:
                    qr_x = int(sum([p.x for p in points]) / 4)
                    qr_y = int(sum([p.y for p in points]) / 4)
                    cv2.circle(frame, (qr_x, qr_y), 5, (255, 0, 0), -1)

                    for j in range(4):
                        pt1 = (int(points[j].x), int(points[j].y))
                        pt2 = (int(points[(j + 1) % 4].x), int(points[(j + 1) % 4].y))
                        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                qr_pixel_width = obj.rect.width
                if qr_pixel_width > 0:
                    current_estimated_distance_cm = (FOCAL_LENGTH_PIXELS * REAL_QR_WIDTH_CM) / qr_pixel_width
                    estimated_distance_display = f"{current_estimated_distance_cm:.2f} cm"

                if current_estimated_distance_cm is not None and current_estimated_distance_cm < SEND_DISTANCE_THRESHOLD_CM:
                    detected_this_frame_status = qr_data
                else:
                    detected_this_frame_status = "QR_TOO_FAR"

                label = f"{qr_data}, Dist: {estimated_distance_display}"
                cv2.putText(frame, label, (obj.rect.left, obj.rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # --- FINAL DECISION BASED ON HUMAN PRESENCE ---
            if person_detected:
                detected_this_frame_status = "stop"
            elif not decoded_objects:
                detected_this_frame_status = "NO_QR"

            # --- SEND TO TCP ---
            with qr_detection_lock:
                if laptop_detected_qr_data != detected_this_frame_status:
                    laptop_detected_qr_data = detected_this_frame_status
                    print(f"Status update: {laptop_detected_qr_data}")

            # --- DISPLAY INFO ---
            overlay = f"Status: {laptop_detected_qr_data}"
            if decoded_objects:
                overlay += f" | Dist: {estimated_distance_display}"
            text_color = (0, 255, 0) if "stop" not in overlay and "NO_QR" not in overlay else (0, 0, 255)
            cv2.putText(frame, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

            # Timestamp
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Record and show
            out.write(frame)
            cv2.imshow("Live Feed", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("Video stream closed.")

# --- MAIN ---
if __name__ == '__main__':
    qr_send_thread = threading.Thread(target=tcp_qr_client_thread, daemon=True)
    qr_send_thread.start()
    time.sleep(1)
    main_video_display_loop()
    print("Laptop client finished.")
