/*
 * Project Name: Vision-Based Zero Sensor Robot
 * Copyright (C) 2026 Vikas Singh Thakur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "";
const char* password = "";

WebServer server(80);

// Motor pins – fixed PWMA to 25 (GPIO35 is input-only, can't use for PWM/output)
#define PWMA  25   // ← CORRECTED: was 35 (invalid), now 25 (safe for PWM)
#define AIN1  26
#define AIN2  27
#define PWMB  18
#define BIN1  19
#define BIN2  21
#define STBY  13

#define PWM_FREQ      1000
#define PWM_RESOLUTION 8

int motorSpeed = 255;  // Max speed 0–255 (increased from 200 for full range)

// ===== HTML PAGE with larger joystick pad =====
String webpage = R"====(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
body { text-align:center; font-family:Arial; background:#111; color:white; }
h1 { font-size:40px; }
p { font-size:20px; }
#joystick { width:400px; height:400px; background:#333; border-radius:50%; position:relative; margin:auto; cursor:crosshair; }
#thumb { width:50px; height:50px; background:red; border-radius:50%; position:absolute; transform:translate(-50%,-50%); }
</style>
</head>
<body>
<h1>Keyboard Control</h1>
<p>Click anywhere on this page first!</p>
<p>W = Forward</p>
<p>S = Backward</p>
<p>A = Left</p>
<p>D = Right</p>
<p>Space = Stop</p>
<h2>Joystick Control</h2>
<p>Click/touch and drag on the circle below for smooth direction/speed control.</p>
<div id="joystick"><div id="thumb"></div></div>
<script>
let activeKeys = new Set();
let currentLA = 0, currentRA = 0;
let lastLA = 0, lastRA = 0;
let maxSpeed = 255;
let rampFactor = 0.2;  // Smoothing factor for exponential ramp
let expCurve = 1.5;    // Exponential curve for speed (fine control near center)
let joystickActive = false;
let joystickDX = 0, joystickDY = 0;

document.addEventListener('keydown', function(event) {
  let key = event.key.toLowerCase();
  if(['w','s','a','d',' '].includes(key)) activeKeys.add(key);
});

document.addEventListener('keyup', function(event) {
  let key = event.key.toLowerCase();
  activeKeys.delete(key);
});

let joystick = document.getElementById('joystick');
let thumb = document.getElementById('thumb');
let centerX = 200, centerY = 200;  // Center of 400x400 pad
let radius = 200;

// Mouse events
joystick.addEventListener('mousedown', () => joystickActive = true);
document.addEventListener('mouseup', () => { joystickActive = false; resetThumb(); });
joystick.addEventListener('mousemove', updateJoystickMouse);

// Touch events (for mobile)
joystick.addEventListener('touchstart', () => joystickActive = true);
joystick.addEventListener('touchend', () => { joystickActive = false; resetThumb(); });
joystick.addEventListener('touchmove', updateJoystickTouch);

function updateJoystickMouse(e) {
  if(joystickActive) {
    let rect = joystick.getBoundingClientRect();
    let x = e.clientX - rect.left;
    let y = e.clientY - rect.top;
    updateJoystick(x, y);
  }
}

function updateJoystickTouch(e) {
  if(joystickActive && e.touches.length > 0) {
    e.preventDefault();
    let touch = e.touches[0];
    let rect = joystick.getBoundingClientRect();
    let x = touch.clientX - rect.left;
    let y = touch.clientY - rect.top;
    updateJoystick(x, y);
  }
}

function updateJoystick(x, y) {
  let dx = x - centerX;
  let dy = y - centerY;
  let dist = Math.sqrt(dx*dx + dy*dy);
  if(dist > radius) {
    dx = (dx / dist) * radius;
    dy = (dy / dist) * radius;
    dist = radius;
  }
  thumb.style.left = (centerX + dx) + 'px';
  thumb.style.top = (centerY + dy) + 'px';
  joystickDX = dx;
  joystickDY = dy;
}

function resetThumb() {
  thumb.style.left = centerX + 'px';
  thumb.style.top = centerY + 'px';
  joystickDX = 0;
  joystickDY = 0;
}

// Update and send motor speeds every 50ms
setInterval(() => {
  let fwd = 0, turn = 0;
  
  // Keyboard input
  if(activeKeys.has('w')) fwd += 1;
  if(activeKeys.has('s')) fwd -= 1;
  if(activeKeys.has('a')) turn -= 1;
  if(activeKeys.has('d')) turn += 1;
  if(activeKeys.has(' ')) fwd = 0, turn = 0;
  
  // Joystick input (overrides keyboard if active)
  let jDist = Math.sqrt(joystickDX**2 + joystickDY**2) / radius;
  if(jDist > 0.05) {  // Deadzone
    let normX = joystickDX / radius;
    let normY = joystickDY / radius;  // Positive Y = forward (down on screen)
    let mag = Math.pow(jDist, expCurve);  // Exponential speed curve
    fwd = mag * normY;
    turn = mag * normX;
  }
  
  // Compute targets and clamp
  let targetLA = fwd - turn;
  let targetRA = fwd + turn;
  let maxMag = Math.max(Math.abs(targetLA), Math.abs(targetRA));
  if(maxMag > 1) {
    targetLA /= maxMag;
    targetRA /= maxMag;
  }
  
  // Exponential smooth ramp to target
  currentLA += (targetLA - currentLA) * rampFactor;
  currentRA += (targetRA - currentRA) * rampFactor;
  
  // Zero if small
  if(Math.abs(currentLA) < 0.01) currentLA = 0;
  if(Math.abs(currentRA) < 0.01) currentRA = 0;
  
  // Compute speeds
  let speedLA = Math.round(currentLA * maxSpeed);
  let speedRA = Math.round(currentRA * maxSpeed);
  
  // Send only if changed
  if(speedLA !== lastLA || speedRA !== lastRA) {
    fetch(`/set?la=${speedLA}&ra=${speedRA}`);
    lastLA = speedLA;
    lastRA = speedRA;
  }
}, 50);
</script>
</body>
</html>
)====";

void setup() {
  Serial.begin(115200);
  
  // Set motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH);  // Enable TB6612 driver
  
  // Configure PWM channels (ESP32 LEDC style)
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Define web server routes
  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  
  server.on("/F", forward);
  server.on("/B", backward);
  server.on("/L", left);
  server.on("/R", right);
  server.on("/S", stopMotors);
  server.on("/set", handleSet);  // New route for joystick/precise control
  
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
}

// ===== Motor Control Helper =====
void setMotors(int la, int ra) {
  // Motor A (assuming left)
  if (la > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, la);
  } else if (la < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, -la);
  } else {
    ledcWrite(PWMA, 0);
  }
  
  // Motor B (assuming right)
  if (ra > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, ra);
  } else if (ra < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, -ra);
  } else {
    ledcWrite(PWMB, 0);
  }
}

// ===== Handler for /set?la=X&ra=Y =====
void handleSet() {
  int la = server.arg("la").toInt();
  int ra = server.arg("ra").toInt();
  setMotors(la, ra);
  server.send(200, "text/plain", "OK");
}

// ===== Legacy Movement Functions (full speed for single presses) =====
void forward() {
  setMotors(motorSpeed, motorSpeed);
  server.send(200, "text/plain", "OK");
}

void backward() {
  setMotors(-motorSpeed, -motorSpeed);
  server.send(200, "text/plain", "OK");
}

void left() {
  setMotors(-motorSpeed, motorSpeed);
  server.send(200, "text/plain", "OK");
}

void right() {
  setMotors(motorSpeed, -motorSpeed);
  server.send(200, "text/plain", "OK");
}

void stopMotors() {
  setMotors(0, 0);
  server.send(200, "text/plain", "OK");
}
