/*
  ============================================================
  ESP32 Wi-Fi Controlled Robot Car (AP Mode, Tank Steering, PWM Speed)
  + QUADRATURE ENCODER SUPPORT (live readout on webpage)
  ============================================================

  Hardware:
    - ESP32 DevKit V1 (30-pin)
    - 2 x L293D Motor Drivers
    - 4 x DC Geared Motors WITH ENCODERS (A/B channels)

  Motor Wiring:
    Driver 1:
      OUT1 & OUT2 -> Motor 1 (Front Left)   | IN1 = D13, IN2 = D16, EN = D18
      OUT3 & OUT4 -> Motor 4 (Front Right)  | IN3 = D14, IN4 = D27, EN = D19
    Driver 2:
      OUT1 & OUT2 -> Motor 3 (Rear Right)   | IN1 = D33, IN2 = D32, EN = D4
      OUT3 & OUT4 -> Motor 2 (Rear Left)    | IN3 = D26, IN4 = D25, EN = D2

  *** IMPORTANT ***
  The 8 encoder pin #defines below are left BLANK on purpose.
  You must fill in your chosen GPIO numbers before this sketch
  will compile. Search for "FILL IN YOUR PIN HERE" below.

  Tip: pick pins that are NOT input-only (avoid 34,35,36,39 for
  channel A since they have no internal pull-up and can't be used
  with INPUT_PULLUP cleanly). Good free choices on a 30-pin DevKit
  after the motor pins above are taken: 5, 17, 21, 22, 23, 15, 12,
  34, 35, 36, 39 (last four as B-only/read-only if needed).

  Layout:
        Front
    M1          M4
    M2          M3
        Rear

  Wi-Fi:
    ESP32 creates its own Access Point (AP Mode).
    SSID: ESP32_CAR
    PASS: 12345678
    Connect your phone to this Wi-Fi, then open http://192.168.4.1

  Libraries Used:
    - WiFi.h      (built-in ESP32 core library, used for AP mode)
    - WebServer.h (built-in ESP32 core library, synchronous web server)

  Notes on PWM:
    - Uses analogWrite() on the EN pins, which on modern ESP32 Arduino
      core (2.0.4+) is automatically mapped to the internal LEDC PWM
      peripheral. No extra ledcSetup() calls are needed.
    - Direction is set by the IN pins. Speed is set by PWM on the EN pin.

  Notes on Encoders:
    - Each motor's channel A pin triggers an interrupt on RISING edge.
    - Channel B's level at that instant tells us the direction (this is
      simple "X1" quadrature decoding -> 1 count per pulse, not the full
      4x resolution, but it's plenty for a hobby robot speed/direction
      readout).
    - If a motor's count goes DOWN when it should be going UP (or vice
      versa), just swap that motor's A and B wires (or A/B pin numbers
      in the #defines below).
    - Counts are cumulative and only reset on boot or via the "Reset
      Encoders" button on the webpage / GET request to /resetEncoders.

  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------------
// Wi-Fi Access Point Credentials
// ---------------------------------------------------------------
const char* ssid     = "ESP32_CAR";
const char* password = "12345678";

// ---------------------------------------------------------------
// Create Web Server object on port 80 (standard HTTP port)
// ---------------------------------------------------------------
WebServer server(80);

// ---------------------------------------------------------------
// Motor Direction Pin Definitions
// ---------------------------------------------------------------

// Motor 1 - Front Left (Driver 1, OUT1/OUT2)
#define M1_IN1 13
#define M1_IN2 12
#define M1_EN  18

// Motor 4 - Front Right (Driver 1, OUT3/OUT4)
#define M4_IN3 14
#define M4_IN4 27
#define M4_EN  19

// Motor 3 - Rear Right (Driver 2, OUT1/OUT2)
#define M3_IN1 33
#define M3_IN2 32
#define M3_EN  4

// Motor 2 - Rear Left (Driver 2, OUT3/OUT4)
#define M2_IN3 26
#define M2_IN4 25
#define M2_EN   2

// ---------------------------------------------------------------
// Encoder Pin Definitions
// ---------------------------------------------------------------
// *** FILL IN YOUR PIN HERE for every line below before compiling ***

// Motor 1 - Front Left Encoder
#define M1_ENC_A  35   // <-- FILL IN YOUR PIN HERE
#define M1_ENC_B  34   // <-- FILL IN YOUR PIN HERE

// Motor 2 - Rear Left Encoder
#define M2_ENC_A  21   // <-- FILL IN YOUR PIN HERE
#define M2_ENC_B  5   // <-- FILL IN YOUR PIN HERE

// Motor 3 - Rear Right Encoder
#define M3_ENC_A  15   // <-- FILL IN YOUR PIN HERE
#define M3_ENC_B   39  // <-- FILL IN YOUR PIN HERE

// Motor 4 - Front Right Encoder
#define M4_ENC_A   22  // <-- FILL IN YOUR PIN HERE
#define M4_ENC_B    23 // <-- FILL IN YOUR PIN HERE

// ---------------------------------------------------------------
// Global Speed Variable (0-255), controlled by webpage slider
// ---------------------------------------------------------------
int motorSpeed = 255; // default = full speed

// ---------------------------------------------------------------
// Encoder Counters (updated inside ISRs, so must be volatile)
// ---------------------------------------------------------------
volatile long encM1Count = 0;
volatile long encM2Count = 0;
volatile long encM3Count = 0;
volatile long encM4Count = 0;

// ---------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------
void setupMotorPins();
void setupEncoders();
void forward();
void backward();
void left();
void right();
void stopMotors();
void handleRoot();
void handleForward();
void handleBackward();
void handleLeft();
void handleRight();
void handleStop();
void handleSpeed();
void handleEncoders();
void handleResetEncoders();
void handleNotFound();
void IRAM_ATTR isrM1();
void IRAM_ATTR isrM2();
void IRAM_ATTR isrM3();
void IRAM_ATTR isrM4();

// ---------------------------------------------------------------
// HTML Webpage
// Mobile-friendly layout with large touch buttons + speed slider
// + live encoder readout polled every 300ms.
// ---------------------------------------------------------------
const char htmlPage[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 Robot Car</title>
  <style>
    * {
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }
    body {
      margin: 0;
      padding: 0;
      background-color: #121212;
      font-family: Arial, Helvetica, sans-serif;
      color: #ffffff;
      text-align: center;
      user-select: none;
    }
    h1 {
      margin: 15px 0;
      font-size: 24px;
      color: #00e6e6;
    }
    h2 {
      margin: 10px 0 5px 0;
      font-size: 16px;
      color: #888888;
      font-weight: normal;
      letter-spacing: 1px;
    }
    .control-grid {
      display: grid;
      grid-template-columns: 100px 100px 100px;
      grid-template-rows: 100px 100px 100px;
      gap: 12px;
      justify-content: center;
      margin: 20px auto;
    }
    .btn {
      width: 100px;
      height: 100px;
      font-size: 36px;
      border: none;
      border-radius: 16px;
      background-color: #1e1e1e;
      color: #00e6e6;
      box-shadow: 0 4px 6px rgba(0,0,0,0.5);
      touch-action: manipulation;
    }
    .btn:active {
      background-color: #00e6e6;
      color: #121212;
    }
    .stop-btn {
      background-color: #4a0000;
      color: #ff4d4d;
    }
    .stop-btn:active {
      background-color: #ff4d4d;
      color: #121212;
    }
    .empty {
      background: none;
      box-shadow: none;
    }
    .speed-box {
      margin: 20px auto;
      width: 80%;
      max-width: 320px;
    }
    .speed-box label {
      font-size: 16px;
      color: #aaaaaa;
    }
    input[type=range] {
      width: 100%;
    }
    #status {
      margin-top: 10px;
      font-size: 16px;
      color: #aaaaaa;
    }
    .encoder-grid {
      display: grid;
      grid-template-columns: 100px 100px 100px;
      grid-template-rows: auto auto;
      gap: 12px;
      justify-content: center;
      margin: 15px auto;
      max-width: 320px;
    }
    .enc-box {
      background-color: #1e1e1e;
      border-radius: 12px;
      padding: 10px 0;
      font-family: 'Courier New', monospace;
    }
    .enc-label {
      font-size: 12px;
      color: #888888;
    }
    .enc-value {
      font-size: 20px;
      color: #00e6e6;
      font-weight: bold;
    }
    .reset-btn {
      margin-top: 15px;
      padding: 10px 20px;
      background-color: #1e1e1e;
      color: #ff9900;
      border: 1px solid #ff9900;
      border-radius: 8px;
      font-size: 14px;
    }
    .reset-btn:active {
      background-color: #ff9900;
      color: #121212;
    }
  </style>
</head>
<body>

  <h1>ESP32 Wi-Fi Robot Car</h1>

  <div class="control-grid">
    <div class="empty"></div>
    <button class="btn" ontouchstart="sendCmd('forward')" onclick="sendCmd('forward')">&#8679;</button>
    <div class="empty"></div>

    <button class="btn" ontouchstart="sendCmd('left')" onclick="sendCmd('left')">&#8678;</button>
    <button class="btn stop-btn" ontouchstart="sendCmd('stop')" onclick="sendCmd('stop')">&#9632;</button>
    <button class="btn" ontouchstart="sendCmd('right')" onclick="sendCmd('right')">&#8680;</button>

    <div class="empty"></div>
    <button class="btn" ontouchstart="sendCmd('backward')" onclick="sendCmd('backward')">&#8681;</button>
    <div class="empty"></div>
  </div>

  <div class="speed-box">
    <label for="speedSlider">Speed: <span id="speedValue">255</span></label>
    <input type="range" id="speedSlider" min="0" max="255" value="255" oninput="updateSpeed(this.value)">
  </div>

  <h2>ENCODER COUNTS</h2>
  <div class="encoder-grid">
    <div class="empty"></div>
    <div class="enc-box">
      <div class="enc-label">M1 (FL)</div>
      <div class="enc-value" id="enc1">0</div>
    </div>
    <div class="enc-box">
      <div class="enc-label">M4 (FR)</div>
      <div class="enc-value" id="enc4">0</div>
    </div>

    <div class="empty"></div>
    <div class="enc-box">
      <div class="enc-label">M2 (RL)</div>
      <div class="enc-value" id="enc2">0</div>
    </div>
    <div class="enc-box">
      <div class="enc-label">M3 (RR)</div>
      <div class="enc-value" id="enc3">0</div>
    </div>
  </div>
  <button class="reset-btn" onclick="resetEncoders()">Reset Encoders</button>

  <div id="status">Ready</div>

  <script>
    // Sends a movement command to the ESP32 instantly using fetch()
    function sendCmd(cmd) {
      document.getElementById('status').innerText = 'Sending: ' + cmd;
      fetch('/' + cmd)
        .then(response => {
          if (response.ok) {
            document.getElementById('status').innerText = 'Command: ' + cmd;
          } else {
            document.getElementById('status').innerText = 'Error sending command';
          }
        })
        .catch(error => {
          document.getElementById('status').innerText = 'Connection error';
        });
    }

    // Sends updated speed value to the ESP32 as the slider moves
    function updateSpeed(val) {
      document.getElementById('speedValue').innerText = val;
      fetch('/speed?value=' + val)
        .catch(error => {
          document.getElementById('status').innerText = 'Speed update error';
        });
    }

    // Polls /encoders every 300ms and updates the live readout
    function updateEncoders() {
      fetch('/encoders')
        .then(response => response.json())
        .then(data => {
          document.getElementById('enc1').innerText = data.m1;
          document.getElementById('enc2').innerText = data.m2;
          document.getElementById('enc3').innerText = data.m3;
          document.getElementById('enc4').innerText = data.m4;
        })
        .catch(error => {
          // Stay quiet on transient poll failures
        });
    }
    setInterval(updateEncoders, 300);

    // Resets all 4 encoder counters back to zero
    function resetEncoders() {
      fetch('/resetEncoders')
        .then(() => {
          document.getElementById('status').innerText = 'Encoders reset';
        })
        .catch(error => {
          document.getElementById('status').innerText = 'Reset error';
        });
    }
  </script>

</body>
</html>
)rawliteral";

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting ESP32 Robot Car...");

  // Configure all motor control pins as OUTPUT
  setupMotorPins();

  // Configure encoder pins and attach interrupts
  setupEncoders();

  // Make sure motors are stopped at boot
  stopMotors();

  // -------------------------------------------------------------
  // Start ESP32 in Access Point (AP) Mode
  // The ESP32 itself becomes a Wi-Fi hotspot.
  // No router or internet connection is required.
  // -------------------------------------------------------------
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP); // Should print 192.168.4.1

  // -------------------------------------------------------------
  // Define Web Server Routes (URL Endpoints)
  // -------------------------------------------------------------
  server.on("/", handleRoot);                 // Main webpage
  server.on("/forward", handleForward);       // Move forward
  server.on("/backward", handleBackward);     // Move backward
  server.on("/left", handleLeft);             // Turn left
  server.on("/right", handleRight);           // Turn right
  server.on("/stop", handleStop);             // Stop all motors
  server.on("/speed", handleSpeed);           // Update motor speed
  server.on("/encoders", handleEncoders);     // Get live encoder counts (JSON)
  server.on("/resetEncoders", handleResetEncoders); // Zero all encoder counts
  server.onNotFound(handleNotFound);          // 404 handler

  // Start the web server
  server.begin();
  Serial.println("HTTP server started.");
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
void loop() {
  // Continuously handle incoming HTTP client requests.
  // This must be called repeatedly for WebServer.h to function.
  server.handleClient();
}

// ---------------------------------------------------------------
// Configure all motor driver pins as OUTPUT
// ---------------------------------------------------------------
void setupMotorPins() {
  // Direction pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M4_IN3, OUTPUT);
  pinMode(M4_IN4, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  // Enable (PWM) pins
  pinMode(M1_EN, OUTPUT);
  pinMode(M4_EN, OUTPUT);
  pinMode(M3_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT);
}

// ---------------------------------------------------------------
// Configure encoder pins as INPUT_PULLUP and attach interrupts
// Channel A triggers the interrupt; channel B's level at that
// instant determines direction (simple X1 quadrature decoding).
// ---------------------------------------------------------------
void setupEncoders() {
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);

  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  pinMode(M3_ENC_A, INPUT_PULLUP);
  pinMode(M3_ENC_B, INPUT_PULLUP);

  pinMode(M4_ENC_A, INPUT_PULLUP);
  pinMode(M4_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isrM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isrM2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isrM3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENC_A), isrM4, RISING);
}

// ---------------------------------------------------------------
// ENCODER INTERRUPT SERVICE ROUTINES
// Triggered on every RISING edge of each motor's channel A.
// If B is HIGH at that instant -> count up, else count down.
// (If a motor counts the wrong way, swap its A/B pins or wires.)
// ---------------------------------------------------------------
void IRAM_ATTR isrM1() {
  if (digitalRead(M1_ENC_B) == HIGH) encM1Count++;
  else encM1Count--;
}

void IRAM_ATTR isrM2() {
  if (digitalRead(M2_ENC_B) == HIGH) encM2Count++;
  else encM2Count--;
}

void IRAM_ATTR isrM3() {
  if (digitalRead(M3_ENC_B) == HIGH) encM3Count++;
  else encM3Count--;
}

void IRAM_ATTR isrM4() {
  if (digitalRead(M4_ENC_B) == HIGH) encM4Count++;
  else encM4Count--;
}

// ---------------------------------------------------------------
// MOTOR CONTROL FUNCTIONS
// ---------------------------------------------------------------
// Convention used for every motor:
//   Forward  -> first IN pin HIGH, second IN pin LOW, EN = PWM speed
//   Reverse  -> first IN pin LOW,  second IN pin HIGH, EN = PWM speed
//   Stop     -> both IN pins LOW, EN = 0
// ---------------------------------------------------------------

// Move robot FORWARD (all 4 motors spin forward)
void forward() {
  // Front Left (M1) forward
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, motorSpeed);

  // Rear Left (M2) forward
  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
  analogWrite(M2_EN, motorSpeed);

  // Rear Right (M3) forward
  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);
  analogWrite(M3_EN, motorSpeed);

  // Front Right (M4) forward
  digitalWrite(M4_IN3, HIGH);
  digitalWrite(M4_IN4, LOW);
  analogWrite(M4_EN, motorSpeed);

  Serial.println("Action: FORWARD");
}

// Move robot BACKWARD (all 4 motors spin in reverse)
void backward() {
  // Front Left (M1) reverse
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_EN, motorSpeed);

  // Rear Left (M2) reverse
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, HIGH);
  analogWrite(M2_EN, motorSpeed);

  // Rear Right (M3) reverse
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);
  analogWrite(M3_EN, motorSpeed);

  // Front Right (M4) reverse
  digitalWrite(M4_IN3, LOW);
  digitalWrite(M4_IN4, HIGH);
  analogWrite(M4_EN, motorSpeed);

  Serial.println("Action: BACKWARD");
}

// Turn robot LEFT (tank steering: left side reverse, right side forward)
void left() {
  // Left motors (M1 Front Left, M2 Rear Left) -> REVERSE
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_EN, motorSpeed);

  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, HIGH);
  analogWrite(M2_EN, motorSpeed);

  // Right motors (M3 Rear Right, M4 Front Right) -> FORWARD
  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);
  analogWrite(M3_EN, motorSpeed);

  digitalWrite(M4_IN3, HIGH);
  digitalWrite(M4_IN4, LOW);
  analogWrite(M4_EN, motorSpeed);

  Serial.println("Action: LEFT");
}

// Turn robot RIGHT (tank steering: left side forward, right side reverse)
void right() {
  // Left motors (M1 Front Left, M2 Rear Left) -> FORWARD
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, motorSpeed);

  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
  analogWrite(M2_EN, motorSpeed);

  // Right motors (M3 Rear Right, M4 Front Right) -> REVERSE
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);
  analogWrite(M3_EN, motorSpeed);

  digitalWrite(M4_IN3, LOW);
  digitalWrite(M4_IN4, HIGH);
  analogWrite(M4_EN, motorSpeed);

  Serial.println("Action: RIGHT");
}

// STOP all motors (direction pins LOW, EN pins set to 0)
void stopMotors() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, 0);

  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, LOW);
  analogWrite(M2_EN, 0);

  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  analogWrite(M3_EN, 0);

  digitalWrite(M4_IN3, LOW);
  digitalWrite(M4_IN4, LOW);
  analogWrite(M4_EN, 0);

  Serial.println("Action: STOP");
}

// ---------------------------------------------------------------
// WEB SERVER REQUEST HANDLERS
// ---------------------------------------------------------------

// Serves the main HTML control page
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Handles "/forward" request -> drives robot forward
void handleForward() {
  forward();
  server.send(200, "text/plain", "Forward");
}

// Handles "/backward" request -> drives robot backward
void handleBackward() {
  backward();
  server.send(200, "text/plain", "Backward");
}

// Handles "/left" request -> turns robot left
void handleLeft() {
  left();
  server.send(200, "text/plain", "Left");
}

// Handles "/right" request -> turns robot right
void handleRight() {
  right();
  server.send(200, "text/plain", "Right");
}

// Handles "/stop" request -> stops all motors
void handleStop() {
  stopMotors();
  server.send(200, "text/plain", "Stop");
}

// Handles "/speed?value=XXX" request -> updates global motor speed
void handleSpeed() {
  if (server.hasArg("value")) {
    int newSpeed = server.arg("value").toInt();
    // Constrain to valid PWM range 0-255
    if (newSpeed < 0) newSpeed = 0;
    if (newSpeed > 255) newSpeed = 255;
    motorSpeed = newSpeed;
    Serial.print("Speed updated to: ");
    Serial.println(motorSpeed);
    server.send(200, "text/plain", "Speed set");
  } else {
    server.send(400, "text/plain", "Missing value parameter");
  }
}

// Handles "/encoders" request -> returns live encoder counts as JSON
// Example response: {"m1":120,"m2":-45,"m3":118,"m4":-50}
void handleEncoders() {
  // Snapshot the volatile counters quickly to avoid them changing
  // mid-read (interrupts can fire between reads otherwise).
  noInterrupts();
  long c1 = encM1Count;
  long c2 = encM2Count;
  long c3 = encM3Count;
  long c4 = encM4Count;
  interrupts();

  String json = "{";
  json += "\"m1\":" + String(c1) + ",";
  json += "\"m2\":" + String(c2) + ",";
  json += "\"m3\":" + String(c3) + ",";
  json += "\"m4\":" + String(c4);
  json += "}";

  server.send(200, "application/json", json);
}

// Handles "/resetEncoders" request -> zeroes all 4 encoder counters
void handleResetEncoders() {
  noInterrupts();
  encM1Count = 0;
  encM2Count = 0;
  encM3Count = 0;
  encM4Count = 0;
  interrupts();

  Serial.println("Encoders reset to 0");
  server.send(200, "text/plain", "Encoders reset");
}

// Handles unknown routes (404 Not Found)
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}
