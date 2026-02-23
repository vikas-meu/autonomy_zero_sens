# 🤖 Vision-Based Zero Sensor Autonomous Robot

Autonomous Path Following using **ESP32 + OpenCV + ArUco Markers**
No ultrasonic. No IR. No encoders. **Zero onboard sensors.**

---

## 🚀 Project Overview

This project demonstrates a fully autonomous robot that navigates a drawn path using **external computer vision instead of onboard sensors**.

The robot itself has:

* ❌ No ultrasonic sensors
* ❌ No IR sensors
* ❌ No encoders
* ❌ No IMU

All sensing and decision-making is handled externally using:

* OpenCV
* ArUco marker detection
* WiFi communication
* Overhead camera

The ESP32 acts only as a motor controller that receives speed commands from a Python script.

---

## 🧠 How It Works

1. An overhead camera monitors the workspace.
2. An ArUco marker is attached to the robot.
3. OpenCV detects:

   * Robot position (X, Y)
   * Robot orientation (angle)
4. User draws a path on screen.
5. The system computes heading error.
6. Motor speeds are sent via WiFi to ESP32.
7. The robot follows the path autonomously.

---

## 🛠 Hardware Requirements

* ESP32 DevKit V1
* TB6612FNG Motor Driver
* 2 DC Motors + Wheels
* Robot Chassis
* 5–12V Motor Battery
* Overhead Camera (Webcam / Phone Camera)
* Printed ArUco Marker (DICT_4X4_50, ID 0, 5.7cm x 5.7cm)

---

## 🔌 Wiring

### TB6612 → ESP32

| TB6612 Pin | ESP32 GPIO      |
| ---------- | --------------- |
| PWMA       | 25              |
| AIN1       | 26              |
| AIN2       | 27              |
| PWMB       | 22              |
| BIN1       | 16              |
| BIN2       | 21              |
| STBY       | 13              |
| VCC        | 3.3V            |
| GND        | GND             |
| VM         | Motor Battery + |

⚠ Important:

* Share ground between ESP32 and motor battery.
* Add capacitor across VM for stability.

---

## 💻 Software Requirements

* Arduino IDE (for ESP32 upload)
* Python 3.8+
* OpenCV
* NumPy
* Requests

Install Python dependencies:

```bash
pip install opencv-python numpy requests
```

---

## 📂 Project Structure

```
zero-sensor-robot/
│
├── esp32_code/
│   └── esp32_robot.ino
│
├── vision_control/
│   └── path_following.py
│
├── LICENSE
└── README.md
```

---

## 📡 Setup Instructions

### 1️⃣ Upload ESP32 Code

* Open Arduino IDE
* Select ESP32 board
* Enter your WiFi credentials
* Upload code
* Note the IP address shown in Serial Monitor

---

### 2️⃣ Update Python Script

Set:

```python
ESP_IP = "192.168.x.x"
```

---

### 3️⃣ Run Vision System

```bash
python path_following.py
```

* Draw path using mouse
* Click **Follow**
* Robot starts autonomous navigation

---

## ⚙ Control Logic

* Differential drive model
* Lookahead-based path following
* Real-time heading correction
* Motor speed range: -255 to 255

All navigation decisions happen on the computer.

ESP32 only executes received commands.

---

## 🎯 Features

✔ Zero onboard sensors
✔ Vision-based localization
✔ Real-time WiFi control
✔ Interactive path drawing
✔ Clean and low-cost hardware
✔ Easy to extend

---

## 📸 Demo

(Add your images or GIFs here)

Suggested sections:

* Robot hardware image
* Camera setup image
* ArUco detection screenshot
* Path drawing interface
* Robot following path

---

## 🔬 Future Improvements

* PID tuning interface
* Multi-robot tracking
* Obstacle detection using vision
* Swarm coordination
* ROS2 integration
* ML-based trajectory planning

---

## 📜 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.

You are free to:

* Use
* Modify
* Distribute

But you must:

* Keep it under GPL
* Provide source code if distributing
* Give proper credit

See the `LICENSE` file for full details.

---

## 👨‍💻 Author

Vikas Singh Thakur
Robotics Developer | Vision Systems | Autonomous Systems

If you found this project useful, consider starring ⭐ the repository.


#I AM VIKAS 
#I LOVE ROBOTICS ❤️✨🍀
