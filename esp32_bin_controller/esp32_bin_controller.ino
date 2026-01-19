/*
 * ESP32 Waste Bin Controller
 * Receives waste classification via HTTP from ESP32-CAM
 * Controls stepper motor (bin rotation) and servo (lid)
 * 
 * Waste Categories:
 *   - metal: Position 0° (home)
 *   - plastic: Position 120°
 *   - paper: Position 240°
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ==================== WiFi Configuration ====================
const char* ssid = "YOUR_WIFI_SSID";          // Change this to your WiFi SSID
const char* password = "YOUR_WIFI_PASSWORD";   // Change this to your WiFi Password

// ==================== GPIO Pin Configuration ====================
// Stepper Motor (28BYJ-48 with ULN2003 Driver)
#define STEPPER_IN1  13   // IN1 on ULN2003
#define STEPPER_IN2  18   // IN2 on ULN2003
#define STEPPER_IN3  14   // IN3 on ULN2003
#define STEPPER_IN4  19   // IN4 on ULN2003

// Servo Motor (for lid control)
#define SERVO_PIN    26

// LED Indicator (optional - uses built-in LED)
#define LED_PIN      2

// Buzzer
#define BUZZER_PIN   23

// ==================== Motor Configuration ====================
// 28BYJ-48 has 2048 steps per revolution (with gear reduction)
#define STEPS_PER_REV  2048
#define STEP_DELAY_MS  2  // Delay between steps (lower = faster, min ~1-2ms)

// Servo angles
#define LID_CLOSED  0
#define LID_OPEN    90

// Swing steps for each waste type (from default position)
// 28BYJ-48: 2048 steps = 360°, so 1264 steps = ~222°
// Metal: swing right
// Plastic: swing left
// Paper: no swing
#define METAL_SWING_STEPS      1264
#define PLASTIC_SWING_STEPS   -1264
#define PAPER_SWING_STEPS      0

// ==================== Stepper Sequence ====================
// Half-step sequence for smoother operation
const int stepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};
int stepIndex = 0;

// ==================== Objects ====================
Servo lidServo;
WebServer server(80);

// ==================== State Variables ====================
int currentPosition = 0;  // 0=metal, 1=plastic, 2=paper
bool isBusy = false;

// Manual control state
int currentStepPosition = 0;  // Track absolute position in steps from home
bool isRotating = false;       // Is motor currently rotating manually
int rotationDirection = 0;     // 1 = right, -1 = left, 0 = stopped


// Position mapping for waste types
enum WasteType {
  METAL = 0,
  PLASTIC = 1,
  PAPER = 2,
  UNKNOWN = -1
};

// ==================== Function Prototypes ====================
void handleRoot();
void handleSort();
void handleStatus();
void handleTest();
void handleManualStart();
void handleManualStop();
void handleManualGoto();
void handleManualHome();
WasteType parseWasteType(String type);
void stepMotor(int steps);
void setStepperPins(int s1, int s2, int s3, int s4);
void disableStepper();
void openLid();
void closeLid();
void blinkLED(int times);
void beep(int times);

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Waste Bin Controller ===");

  // Initialize stepper pins
  pinMode(STEPPER_IN1, OUTPUT);
  pinMode(STEPPER_IN2, OUTPUT);
  pinMode(STEPPER_IN3, OUTPUT);
  pinMode(STEPPER_IN4, OUTPUT);
  disableStepper();  // Start with motor off
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize servo
  lidServo.attach(SERVO_PIN);
  lidServo.write(LID_CLOSED);
  delay(500);

  // Connect to WiFi
  // Set Static IP
  IPAddress local_IP(192, 168, 0, 201);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Blink while connecting
  }
  
  digitalWrite(LED_PIN, HIGH);  // Solid LED when connected
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Setup HTTP endpoints
  server.on("/", handleRoot);
  server.on("/sort", HTTP_GET, handleSort);
  server.on("/status", handleStatus);
  server.on("/test", handleTest);
  
  // Manual control endpoints
  server.on("/manual/start", HTTP_GET, handleManualStart);
  server.on("/manual/stop", HTTP_GET, handleManualStop);
  server.on("/manual/goto", HTTP_GET, handleManualGoto);
  server.on("/manual/home", HTTP_GET, handleManualHome);
  
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("\nReady to receive waste classification!");
  Serial.println("Endpoints:");
  Serial.println("  /sort?type=metal");
  Serial.println("  /sort?type=plastic");
  Serial.println("  /sort?type=paper");
  Serial.println("  /test?steps=100  (test stepper)");
  Serial.println("  /status");

  // Test each pin individually
  Serial.println("\n=== WIRING TEST ===");
  Serial.println("Watch the LEDs on ULN2003:");
  
  Serial.println("Testing IN1 (GPIO13) - D1 should light up...");
  digitalWrite(STEPPER_IN1, HIGH);
  delay(1000);
  digitalWrite(STEPPER_IN1, LOW);
  
  Serial.println("Testing IN2 (GPIO12) - D2 should light up...");
  digitalWrite(STEPPER_IN2, HIGH);
  delay(1000);
  digitalWrite(STEPPER_IN2, LOW);
  
  Serial.println("Testing IN3 (GPIO14) - D3 should light up...");
  digitalWrite(STEPPER_IN3, HIGH);
  delay(1000);
  digitalWrite(STEPPER_IN3, LOW);
  
  Serial.println("Testing IN4 (GPIO27) - D4 should light up...");
  digitalWrite(STEPPER_IN4, HIGH);
  delay(1000);
  digitalWrite(STEPPER_IN4, LOW);
  
  Serial.println("=== TEST COMPLETE ===");
  Serial.println("All 4 LEDs should have lit up in sequence.");
  Serial.println("If not, check the wiring for the missing LEDs.\n");

  blinkLED(3);  // Ready signal
}

// ==================== Main Loop ====================
void loop() {
  server.handleClient();
  
  // Handle continuous manual rotation
  if (isRotating && !isBusy) {
    // Step one step in the current direction
    stepIndex += rotationDirection;
    if (stepIndex >= 8) stepIndex = 0;
    if (stepIndex < 0) stepIndex = 7;
    
    setStepperPins(
      stepSequence[stepIndex][0],
      stepSequence[stepIndex][1],
      stepSequence[stepIndex][2],
      stepSequence[stepIndex][3]
    );
    
    // Track position
    currentStepPosition += rotationDirection;
    
    delay(2);  // STEP_DELAY_MS
  } else if (!isRotating && !isBusy) {
    // Turn off coils when not rotating to save power
    static unsigned long lastDisable = 0;
    if (millis() - lastDisable > 100) {
      disableStepper();
      lastDisable = millis();
    }
  }
}

// ==================== HTTP Handlers ====================
// ==================== HTTP Handlers ====================
void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <title>Waste Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {
      --primary: #2563eb;
      --success: #10b981;
      --danger: #ef4444;
      --bg: #f3f4f6;
      --card: #ffffff;
      --text: #1f2937;
      --text-sec: #6b7280;
      --border: #e5e7eb;
    }
    body { font-family: system-ui, -apple-system, sans-serif; background: var(--bg); color: var(--text); padding: 20px; margin: 0; display: flex; flex-direction: column; align-items: center; }
    h1 { color: var(--primary); font-weight: 700; letter-spacing: -0.5px; margin-bottom: 24px; font-size: 1.5rem; }
    
    .card { background: var(--card); border-radius: 16px; padding: 24px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05); width: 100%; max-width: 400px; margin-bottom: 20px; border: 1px solid var(--border); box-sizing: border-box; }
    h3 { margin: 0 0 16px 0; font-size: 0.9rem; text-transform: uppercase; color: var(--text-sec); letter-spacing: 0.05em; font-weight: 600; }
    
    .status-row { display: flex; justify-content: space-between; margin-bottom: 12px; border-bottom: 1px solid var(--border); padding-bottom: 12px; }
    .status-row:last-child { border: none; margin: 0; padding: 0; }
    .label { font-size: 0.9rem; color: var(--text-sec); }
    .value { font-weight: 600; font-family: monospace; font-size: 1.1rem; }
    
    .btn { display: block; width: 100%; padding: 16px; border: none; border-radius: 12px; font-weight: 600; cursor: pointer; transition: all 0.2s; text-align: center; text-decoration: none; font-size: 1rem; color: white; margin-bottom: 12px; box-sizing: border-box; box-shadow: 0 1px 2px rgba(0,0,0,0.05); }
    .btn:active { transform: translateY(1px); box-shadow: none; }
    
    .btn-metal { background: #64748b; }
    .btn-plastic { background: var(--primary); }
    .btn-paper { background: #fff; color: #7f1d1d; border: 2px solid #fecaca; }
    .btn-home { background: var(--text); }
    
    .manual-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .arrow-btn { background: #f9fafb; color: var(--text); border: 2px solid var(--border); font-size: 1.5rem; line-height: 1; padding: 20px; }
    .arrow-btn:active { background: #e5e7eb; }
  </style>
</head>
<body>
  <h1>System Control Panel</h1>
  
  <!-- Status Card -->
  <div class="card">
    <div class="status-row">
      <span class="label">System Status</span>
      <span class="value">)";
  html += String(isBusy ? "PROCESSING" : "READY");
  html += R"(</span>
    </div>
    <div class="status-row">
      <span class="label">Motor Position</span>
      <span class="value"><span id="stepPos">)";
  html += String(currentStepPosition);
  html += R"(</span> steps</span>
    </div>
  </div>

  <!-- Manual Control -->
  <div class="card">
    <h3>Manual Override</h3>
    <div class="manual-grid">
      <button class="btn arrow-btn" id="leftBtn">&larr;</button>
      <button class="btn arrow-btn" id="rightBtn">&rarr;</button>
    </div>
  </div>

  <!-- Calibration -->
  <div class="card">
    <h3>Calibration Targets</h3>
    <a href="/manual/goto?type=metal" class="btn btn-metal">Metal Position</a>
    <a href="/manual/goto?type=plastic" class="btn btn-plastic">Plastic Position</a>
    <a href="/manual/goto?type=paper" class="btn btn-paper">Paper Position</a>
    <div style="margin-top: 16px;"></div>
    <a href="/manual/home" class="btn btn-home">Return Home</a>
  </div>

  <!-- Sort Testing -->
  <div class="card">
    <h3>Test Sorting Cycle</h3>
    <a href="/sort?type=metal" class="btn btn-metal">Sort Metal</a>
    <a href="/sort?type=plastic" class="btn btn-plastic">Sort Plastic</a>
    <a href="/sort?type=paper" class="btn btn-paper">Sort Paper</a>
  </div>

  <script>
    const leftBtn = document.getElementById('leftBtn');
    const rightBtn = document.getElementById('rightBtn');
    
    // Universal Event Binder for Touch/Mouse
    const bindBtn = (btn, dir) => {
        const start = (e) => { 
            if (e.cancelable) e.preventDefault(); 
            fetch('/manual/start?dir=' + dir); 
            btn.style.background = '#e5e7eb';
        };
        const stop = (e) => { 
            if (e.cancelable) e.preventDefault(); 
            fetch('/manual/stop'); 
            btn.style.background = '';
        };
        
        btn.addEventListener('mousedown', start);
        btn.addEventListener('touchstart', start);
        
        btn.addEventListener('mouseup', stop);
        btn.addEventListener('mouseleave', stop);
        btn.addEventListener('touchend', stop);
    };
    
    bindBtn(leftBtn, 'left');
    bindBtn(rightBtn, 'right');
    
    // Live update loop
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
          document.getElementById('stepPos').textContent = data.stepPosition;
      });
    }, 500);
  </script>
</body>
</html>
)";
  server.send(200, "text/html", html);
}

void handleTest() {
  int steps = 100;
  if (server.hasArg("steps")) {
    steps = server.arg("steps").toInt();
  }
  
  Serial.print("Test: Moving ");
  Serial.print(steps);
  Serial.println(" steps");
  
  stepMotor(steps);
  
  server.send(200, "text/html", "<h1>Moved " + String(steps) + " steps</h1><a href='/'>Back</a>");
}

void handleSort() {
  if (isBusy) {
    server.send(503, "application/json", "{\"status\":\"busy\",\"message\":\"Bin is currently processing\"}");
    return;
  }

  if (!server.hasArg("type")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing 'type' parameter\"}");
    return;
  }

  String wasteTypeStr = server.arg("type");
  wasteTypeStr.toLowerCase();
  
  WasteType wasteType = parseWasteType(wasteTypeStr);
  
  if (wasteType == UNKNOWN) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Unknown waste type: " + wasteTypeStr + "\"}");
    return;
  }

  // Send immediate response
  server.send(200, "application/json", "{\"status\":\"ok\",\"type\":\"" + wasteTypeStr + "\",\"position\":" + String(wasteType) + "}");

  // Process the waste
  isBusy = true;
  Serial.println("\n>>> Received: " + wasteTypeStr);
  
  // Different beep patterns for each waste type
  if (wasteType == METAL) beep(1);           // Metal: 1 beep
  else if (wasteType == PLASTIC) beep(2);    // Plastic: 2 beeps
  else if (wasteType == PAPER) beep(3);      // Paper: 3 beeps
  
  // Swing to correct position based on waste type
  int swingSteps = 0;
  if (wasteType == METAL) swingSteps = METAL_SWING_STEPS;
  else if (wasteType == PLASTIC) swingSteps = PLASTIC_SWING_STEPS;
  else if (wasteType == PAPER) swingSteps = PAPER_SWING_STEPS;
  
  // Swing to bin
  if (swingSteps != 0) {
    Serial.print("Swinging ");
    Serial.print(swingSteps);
    Serial.println(" steps...");
    stepMotor(swingSteps);
  }
  
  // Open lid
  openLid();
  
  // Wait for waste to drop
  delay(2000);
  
  // Close lid
  closeLid();
  
  // Return to default position
  if (swingSteps != 0) {
    Serial.println("Returning to default position...");
    stepMotor(-swingSteps);  // Swing back
  }
  
  isBusy = false;
  Serial.println(">>> Processing complete");
}

void handleStatus() {
  String json = "{";
  json += "\"status\":\"" + String(isBusy ? "busy" : "ready") + "\",";
  json += "\"position\":" + String(currentPosition) + ",";
  json += "\"position_name\":\"" + String(currentPosition == 0 ? "metal" : (currentPosition == 1 ? "plastic" : "paper")) + "\",";
  json += "\"stepPosition\":" + String(currentStepPosition) + ",";
  json += "\"isRotating\":" + String(isRotating ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ==================== Manual Control Handlers ====================
void handleManualStart() {
  if (isBusy) {
    server.send(503, "text/plain", "Bin is busy");
    return;
  }
  
  String dir = server.arg("dir");
  if (dir == "left") {
    rotationDirection = -1;
    isRotating = true;
    Serial.println("Manual rotation: LEFT");
    server.send(200, "text/plain", "Rotating left");
  } else if (dir == "right") {
    rotationDirection = 1;
    isRotating = true;
    Serial.println("Manual rotation: RIGHT");
    server.send(200, "text/plain", "Rotating right");
  } else {
    server.send(400, "text/plain", "Invalid direction. Use ?dir=left or ?dir=right");
  }
}

void handleManualStop() {
  isRotating = false;
  rotationDirection = 0;
  disableStepper();  // Turn off coils
  Serial.println("Manual rotation: STOPPED");
  Serial.print("Current position: ");
  Serial.print(currentStepPosition);
  Serial.println(" steps");
  server.send(200, "text/plain", "Stopped");
}

void handleManualGoto() {
  if (isBusy || isRotating) {
    server.send(503, "text/plain", "Bin is busy");
    return;
  }
  
  String typeStr = server.arg("type");
  typeStr.toLowerCase();
  
  int targetSteps = 0;
  if (typeStr == "metal") {
    targetSteps = METAL_SWING_STEPS;
  } else if (typeStr == "plastic") {
    targetSteps = PLASTIC_SWING_STEPS;
  } else if (typeStr == "paper") {
    targetSteps = PAPER_SWING_STEPS;
  } else {
    server.send(400, "text/plain", "Invalid type");
    return;
  }
  
  int stepsToMove = targetSteps - currentStepPosition;
  Serial.print("Going to ");
  Serial.print(typeStr);
  Serial.print(" position (");
  Serial.print(stepsToMove);
  Serial.println(" steps)");
  
  stepMotor(stepsToMove);
  currentStepPosition = targetSteps;
  
  server.send(200, "text/plain", "Moved to " + typeStr);
}

void handleManualHome() {
  if (isBusy || isRotating) {
    server.send(503, "text/plain", "Bin is busy");
    return;
  }
  
  int stepsToHome = -currentStepPosition;
  Serial.print("Returning home (");
  Serial.print(stepsToHome);
  Serial.println(" steps)");
  
  stepMotor(stepsToHome);
  currentStepPosition = 0;
  
  server.send(200, "text/plain", "Returned home");
}


// ==================== Helper Functions ====================
WasteType parseWasteType(String type) {
  if (type == "metal") return METAL;
  if (type == "plastic") return PLASTIC;
  if (type == "paper") return PAPER;
  return UNKNOWN;
}

void setStepperPins(int s1, int s2, int s3, int s4) {
  digitalWrite(STEPPER_IN1, s1);
  digitalWrite(STEPPER_IN2, s2);
  digitalWrite(STEPPER_IN3, s3);
  digitalWrite(STEPPER_IN4, s4);
}

void disableStepper() {
  digitalWrite(STEPPER_IN1, LOW);
  digitalWrite(STEPPER_IN2, LOW);
  digitalWrite(STEPPER_IN3, LOW);
  digitalWrite(STEPPER_IN4, LOW);
}

void stepMotor(int steps) {
  int direction = (steps > 0) ? 1 : -1;
  steps = abs(steps);
  
  Serial.print("Stepping: ");
  Serial.print(steps);
  Serial.println(direction > 0 ? " forward" : " backward");
  
  for (int i = 0; i < steps; i++) {
    stepIndex += direction;
    if (stepIndex >= 8) stepIndex = 0;
    if (stepIndex < 0) stepIndex = 7;
    
    setStepperPins(
      stepSequence[stepIndex][0],
      stepSequence[stepIndex][1],
      stepSequence[stepIndex][2],
      stepSequence[stepIndex][3]
    );
    
    delay(STEP_DELAY_MS);
  }
  
  // Turn off coils to prevent heating
  disableStepper();
}

void openLid() {
  Serial.println("Opening lid...");
  lidServo.write(LID_OPEN);
  delay(500);
}

void closeLid() {
  Serial.println("Closing lid...");
  lidServo.write(LID_CLOSED);
  delay(500);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}
