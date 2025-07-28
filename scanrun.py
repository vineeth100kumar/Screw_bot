import io
import time
import threading
import json
import os
import socket
import socketserver
import RPi.GPIO as GPIO
from http import server
from PIL import Image
import picamera2
import pickup
import drop
# === CONFIG ===
CAMERA_RESOLUTION = (640, 480)
CAMERA_FRAMERATE = 15
MJPEG_PORT = 8090
TCP_QR_PORT = 5001
ROUTE_FOLDER = "recorded_routes"

# === GLOBAL FLAG ===
route_executed = False  # Flag to prevent multiple QR activations
pause_event = threading.Event()
busy_processing = False

# === GPIO SETUP ===
motors = {
    1: {'pinA': 17, 'pinB': 18},
    2: {'pinA': 22, 'pinB': 23},
    3: {'pinA': 24, 'pinB': 25},
    4: {'pinA': 4, 'pinB': 27}
}

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for m in motors.values():
        GPIO.setup(m['pinA'], GPIO.OUT)
        GPIO.setup(m['pinB'], GPIO.OUT)
        GPIO.output(m['pinA'], GPIO.LOW)
        GPIO.output(m['pinB'], GPIO.LOW)

def motor_forward(m):
    GPIO.output(motors[m]['pinA'], GPIO.HIGH)
    GPIO.output(motors[m]['pinB'], GPIO.LOW)

def motor_backward(m):
    GPIO.output(motors[m]['pinA'], GPIO.LOW)
    GPIO.output(motors[m]['pinB'], GPIO.HIGH)

def stop_motor(m):
    GPIO.output(motors[m]['pinA'], GPIO.LOW)
    GPIO.output(motors[m]['pinB'], GPIO.LOW)

def stop_all():
    for m in motors:
        stop_motor(m)

def move_all_forward(dur=None):
    for m in motors: motor_forward(m)
    if dur: time.sleep(dur); stop_all()

def move_all_backward(dur=None):
    for m in motors: motor_backward(m)
    if dur: time.sleep(dur); stop_all()

def pivot_left(dur=None):
    motor_forward(1)
    motor_forward(3)
    motor_backward(2)
    motor_backward(4)
    if dur: time.sleep(dur); stop_all()

def pivot_right(dur=None):
    motor_backward(1)
    motor_backward(3)
    motor_forward(2)
    motor_forward(4)
    if dur: time.sleep(dur); stop_all()

def replicate_route(route):
    print(f"Executing route with {len(route)} steps...")
    for step in route:
        action = step["action"]
        dur = step["duration"]
        print(f"Step: {action} for {dur}s")

        start_time = time.time()
        while time.time() - start_time < dur:
            if pause_event.is_set():
                print("Execution paused due to obstacle...")
                stop_all()
                while pause_event.is_set():
                    time.sleep(0.5)
                print("Obstacle cleared. Resuming...")
                start_time = time.time()  # Reset start time for current step
            if action == "forward":
                move_all_forward(0.1)
            elif action == "backward":
                move_all_backward(0.1)
            elif action == "pivot_left":
                pivot_left(0.1)
            elif action == "pivot_right":
                pivot_right(0.1)
            elif action == "stop":
                stop_all()
                break
        stop_all()
        time.sleep(1.0)
    stop_all()
    print("Route execution complete.")



# === CAMERA STREAMING ===
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def set_frame(self, frame_data):
        with self.condition:
            self.frame = frame_data
            self.condition.notify_all()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            content = f"""<html><head><title>Rover Cam</title></head>
            <body><h1>Camera View</h1>
            <img src="/stream.mjpg" width="{CAMERA_RESOLUTION[0]}" height="{CAMERA_RESOLUTION[1]}" /></body></html>""".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    if frame:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(frame))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
            except Exception as e:
                print(f"Client disconnected: {e}")
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def camera_stream_loop(picam2, output):
    interval = 1.0 / CAMERA_FRAMERATE
    while True:
        try:
            buffer = picam2.capture_buffer("main")
            image = Image.frombytes("RGBX", CAMERA_RESOLUTION, buffer, "raw", "RGBX")
            with io.BytesIO() as b:
                image.save(b, format="jpeg", quality=80)
                output.set_frame(b.getvalue())
            time.sleep(interval)
        except Exception as e:
            print(f"Camera error: {e}")
            break

# === QR TCP SERVER ===
class QRDataHandler(socketserver.BaseRequestHandler):
    def handle(self):
        global route_executed
        global busy_processing
        print(f"Connection from {self.client_address}")
        try:
            while True:
                buf = b''
                while True:
                    part = self.request.recv(1)
                    if not part:
                        return
                    if part == b'\n':
                        break
                    buf += part

                qr_data = buf.decode().strip()
                print(f"QR Received: {qr_data}")
                safe_label = "".join(c for c in qr_data if c.isalnum() or c in ('_', '-')).strip()
                
                if safe_label == "stop":
                    print("Obstacle detected. Pausing movement.")
                    pause_event.set()
                    continue

                elif pause_event.is_set() and safe_label != "stop":
                    print("Obstacle cleared. Resuming movement.")
                    pause_event.clear()
                    continue

                if busy_processing:
                    print("Currently executing a route. Ignoring QR:", qr_data)
                    continue

                self.request.sendall(f"ACK_RECEIVED: {qr_data}\n".encode())            

                if not route_executed:
                    filepath = os.path.join(ROUTE_FOLDER, f"{safe_label}.json")
                    if os.path.exists(filepath):
                        busy_processing = True
                        pickup.pickup()
                        with open(filepath) as f:
                            route = json.load(f)
                        replicate_route(route)
                        print("Route execution complete. No more QR codes will be processed.")
                        drop.pickup()
                        filepath1 = os.path.join(ROUTE_FOLDER, f"{safe_label}_reverse.json")
                        with open(filepath1) as f:
                            route = json.load(f)
                        replicate_route(route)
                        reverse_replicate_route(route)
                        busy_processing = False
                        route_executed = True 
                    else:
                        print(f"Route file not found: {filepath}")

        except Exception as e:
            print(f"QR TCP Error: {e}")
        finally:
            self.request.close()


class QR_TCP_Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

# === MAIN ===
if __name__ == "__main__":
    output = StreamingOutput()
    setup_gpio()

    try:
        print("Starting PiCamera2...")
        picam2 = picamera2.Picamera2()
        config = picam2.create_video_configuration(main={"size": CAMERA_RESOLUTION, "format": "XBGR8888"},
                                                   controls={"FrameRate": CAMERA_FRAMERATE})
        picam2.configure(config)
        picam2.start()

        # Camera thread
        cam_thread = threading.Thread(target=camera_stream_loop, args=(picam2, output), daemon=True)
        cam_thread.start()

        # MJPEG server
        mjpeg_server = StreamingServer(('', MJPEG_PORT), StreamingHandler)
        threading.Thread(target=mjpeg_server.serve_forever, daemon=True).start()
        print(f"Camera Stream at http://<pi_ip>:{MJPEG_PORT}/stream.mjpg")

        # QR TCP server
        qr_server = QR_TCP_Server(('', TCP_QR_PORT), QRDataHandler)
        threading.Thread(target=qr_server.serve_forever, daemon=True).start()
        print(f"QR TCP Server listening on port {TCP_QR_PORT}")

        print("System ready. Awaiting QR commands... Press Ctrl+C to stop.")
        while not route_executed:
            time.sleep(1)

        print("Shutting down QR listener (route already executed)...")
        qr_server.shutdown()

        # Optional: keep camera stream alive or shut it down here
        print("System complete. Camera stream will continue. Press Ctrl+C to exit.")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        stop_all()
        GPIO.cleanup()
        try:
            picam2.stop()
        except: pass
