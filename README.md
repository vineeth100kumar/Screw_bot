# Screw Bot – Mobile Pick-and-Place Robot for Workshops

Screw Bot is a compact robotic system designed to automate the retrieval and placement of small components like screws in interior design workshops. It combines a mobile base, a 3-axis robotic arm, and real-time video streaming with computer vision for efficient, semi-autonomous operation.

## Features

- **3-Axis Robotic Arm**  
  Built using MG995 and SG90 servos for precise pick-and-place control using acrylic and 3D-printed parts.

- **Mobile Base (2WD)**  
  Driven by 4 DC motors through an L298N motor driver for maneuverability in indoor workshop settings.

- **Raspberry Pi Zero 2 W Control Unit**  
  Runs all control logic, video streaming, and CV modules.

- **Real-Time Video Streaming**  
  MJPEG video stream from Raspberry Pi Camera for remote monitoring.

- **Computer Vision**  
  - **Human Detection**: YOLOv5 detects "person" class to trigger safety stop.
  - **QR Code Detection**: Pyzbar detects and locates QR tags for bin identification.
  
- **Battery Operated**  
  Powered by dual 21700 Li-ion cells with TP4056 charging and protection circuit.

## System Architecture

1. Live video stream from Raspberry Pi.
2. External client processes each frame.
3. QR codes and humans are detected.
4. Based on logic:
   - If human detected → send `stop`
   - If QR detected and close → send `QR_DATA`
   - Else → send `NO_QR` or `QR_TOO_FAR`
5. Command sent to Raspberry Pi via TCP socket.

## Hardware Used

- Raspberry Pi Zero 2 W
- 4x DC Motors
- L298N H-Bridge Driver
- 2x MG995 and 2x SG90 Servo Motors
- Raspberry Pi Camera Module
- 2x 21700 Li-ion Batteries (8000mAh)
- TP4056 Charging Module
- Acrylic Chassis & 3D-Printed Arm Links

## Software Requirements

- Python 3.8+
- OpenCV
- Pyzbar
- torch + YOLOv5
- MJPEG Streamer / Flask
- socket / TCP networking


## Applications

- Automating small item handling in workshops
- Embedded robotics and computer vision research
- Educational tool for robotics prototyping

## Developers

- Vineeth Kumar  
- Mustafa Patwari  

