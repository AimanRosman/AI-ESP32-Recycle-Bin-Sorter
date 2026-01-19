/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <WasteSorter2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "esp_http_server.h"
#include <ESPmDNS.h>

// ==================== WiFi Configuration ====================
const char* ssid = "YOUR_WIFI_SSID";          // Change this to your WiFi SSID
const char* password = "YOUR_WIFI_PASSWORD";   // Change this to your WiFi Password

// ==================== Bin Controller Configuration ====================
// IP address of the ESP32 bin controller (check Serial output of bin controller)
const char* binControllerIP = "192.168.0.201";  // Static IP of bin controller
const int binControllerPort = 80;

// Confidence threshold for sending classification (0.0 to 1.0)
const float CONFIDENCE_THRESHOLD = 0.6;

// ==================== IR Sensor Configuration ====================
// IR sensor for object detection trigger (active LOW - 0 when object detected)
#define IR_SENSOR_PIN  15  // Change to your IR sensor pin
#define IR_ACTIVE_LOW  true  // Set to false if your sensor is active HIGH

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

// ==================== Detection Confirmation ====================
// Require multiple consecutive detections to confirm waste type
#define REQUIRED_CONFIRMATIONS 3
String lastDetectedClass = "";
int confirmationCount = 0;

// ==================== Dashboard Statistics ====================
int metalCount = 0;
int plasticCount = 0;
int paperCount = 0;
String currentDetection = "None";
float currentConfidence = 0;
String lastSentType = "None";
unsigned long lastSentTime = 0;

// Web server
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void startCameraServer();

// ==================== Dashboard HTML ====================
// ==================== Dashboard HTML ====================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AI Vision Monitor</title>
    <style>
        :root {
            --bg: #f8fafc;
            --surface: #ffffff;
            --primary: #3b82f6;
            --success: #10b981;
            --text-main: #0f172a;
            --text-sec: #64748b;
            --border: #e2e8f0;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); color: var(--text-main); padding: 20px; }
        
        .header { margin-bottom: 24px; border-bottom: 1px solid var(--border); padding-bottom: 16px; text-align: center; }
        h1 { font-size: 1.5rem; font-weight: 700; color: var(--text-main); letter-spacing: -0.5px; }
        
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); gap: 24px; max-width: 1000px; margin: 0 auto; }
        
        .card { background: var(--surface); border-radius: 16px; padding: 24px; box-shadow: 0 1px 3px 0 rgba(0,0,0,0.1), 0 1px 2px -1px rgba(0,0,0,0.1); border: 1px solid var(--border); }
        h2 { font-size: 0.875rem; text-transform: uppercase; letter-spacing: 0.05em; color: var(--text-sec); margin: 0 0 16px 0; font-weight: 600; display: flex; align-items: center; justify-content: space-between; }
        
        .cam-container { width: 100%; border-radius: 12px; overflow: hidden; background: #000; position: relative; aspect-ratio: 4/3; }
        .camera-feed { width: 100%; height: 100%; object-fit: contain; display: block; }
        
        .detect-box { text-align: center; padding: 32px 16px; background: #f0f9ff; border-radius: 12px; border: 1px solid #bae6fd; margin-bottom: 24px; }
        .detect-intro { font-size: 0.875rem; color: #0369a1; font-weight: 500; margin-bottom: 8px; }
        .detect-main { font-size: 2.5rem; font-weight: 800; color: #0284c7; line-height: 1; text-transform: uppercase; }
        .conf-label { font-size: 0.875rem; color: #38bdf8; margin-top: 8px; font-weight: 500; }
        
        .progress-container { margin-top: 20px; }
        .progress-header { display: flex; justify-content: space-between; margin-bottom: 8px; font-size: 0.875rem; color: var(--text-sec); font-weight: 500; }
        .bar-bg { height: 8px; background: #e2e8f0; border-radius: 4px; overflow: hidden; }
        .bar-fill { height: 100%; background: var(--success); width: 0%; transition: width 0.3s ease-out; }
        
        .stat-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; }
        .stat-box { background: #f8fafc; padding: 16px; border-radius: 12px; text-align: center; border: 1px solid var(--border); }
        .stat-val { font-size: 1.5rem; font-weight: 700; color: var(--text-main); line-height: 1; margin-bottom: 4px; }
        .stat-lbl { font-size: 0.75rem; color: var(--text-sec); font-weight: 600; text-transform: uppercase; }
        
        .footer-status { margin-top: 24px; padding-top: 16px; border-top: 1px solid var(--border); display: flex; align-items: center; gap: 8px; font-size: 0.875rem; color: var(--text-sec); }
        .status-dot { width: 8px; height: 8px; background: var(--success); border-radius: 50%; }
        
        @media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="header">
        <h1>AI Vision Monitor 2.0</h1>
    </div>
    
    <div class="grid">
        <!-- Camera Feed Card -->
        <div class="card">
            <h2>Live Feed <span style="font-size:0.7em; opacity:0.6">ESP32-CAM</span></h2>
            <div class="cam-container">
                <img class="camera-feed" id="stream" src="">
            </div>
            <div class="footer-status">
                <div class="status-dot"></div>
                <span id="lastSent" style="white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">System Active</span>
            </div>
        </div>
        
        <!-- Analysis Card -->
        <div class="card">
            <h2>Real-time Analysis</h2>
            
            <div class="detect-box">
                <div class="detect-intro">DETECTED OBJECT</div>
                <div class="detect-main" id="detection">IDLE</div>
                <div class="conf-label" id="confidence">Waiting...</div>
            </div>
            
            <div class="progress-container">
                <div class="progress-header">
                    <span>Confirmation Lock</span>
                    <span id="confirmText">0/3</span>
                </div>
                <div class="bar-bg">
                    <div class="bar-fill" id="confirmBar"></div>
                </div>
            </div>
            
            <h2 style="margin-top: 32px">Session Statistics</h2>
            <div class="stat-grid">
                <div class="stat-box">
                    <div class="stat-val" id="metalCount">0</div>
                    <div class="stat-lbl">Metal</div>
                </div>
                <div class="stat-box">
                    <div class="stat-val" id="plasticCount">0</div>
                    <div class="stat-lbl">Plastic</div>
                </div>
                <div class="stat-box">
                    <div class="stat-val" id="paperCount">0</div>
                    <div class="stat-lbl">Paper</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Use timestamp to prevent cache for snapshots
        function updateSnapshot() {
            var img = document.getElementById('stream');
            img.src = '/snapshot?' + new Date().getTime();
        }
        setInterval(updateSnapshot, 1000); // 1Hz refresh for stability
        updateSnapshot();
        
        // Fetch data
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('detection').textContent = data.detection;
                    
                    // Dynamic coloring for text
                    const det = document.getElementById('detection');
                    const type = data.detection.toLowerCase();
                    if(type === 'metal') det.style.color = '#eab308'; // yellow-600
                    else if(type === 'plastic') det.style.color = '#3b82f6'; // blue-500
                    else if(type === 'paper') det.style.color = '#ef4444'; // red-500
                    else det.style.color = '#94a3b8'; // slate-400
                    
                    document.getElementById('confidence').textContent = 
                        data.detection === 'None' ? 'Waiting for object...' : 
                        data.confidence.toFixed(1) + '% Confidence';
                        
                    // Update bars
                    document.getElementById('confirmBar').style.width = (data.confirmation / 3 * 100) + '%';
                    document.getElementById('confirmText').textContent = data.confirmation + '/3';
                    
                    // Update stats
                    document.getElementById('metalCount').textContent = data.metal;
                    document.getElementById('plasticCount').textContent = data.plastic;
                    document.getElementById('paperCount').textContent = data.paper;
                    
                    if (data.lastSent !== 'None') {
                        document.getElementById('lastSent').textContent = 'Last: ' + data.lastSent + ' (' + data.lastSentAgo + 's ago)';
                    }
                })
                .catch(e => console.log(e));
        }
        setInterval(updateStatus, 500);
        updateStatus();
    </script>
</body>
</html>
)rawliteral";

// Snapshot handler - takes single frame (doesn't conflict with inference)
static esp_err_t snapshot_handler(httpd_req_t *req) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    
    return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DASHBOARD_HTML, strlen(DASHBOARD_HTML));
}

static esp_err_t status_handler(httpd_req_t *req) {
    char json[256];
    unsigned long ago = (millis() - lastSentTime) / 1000;
    snprintf(json, sizeof(json), 
        "{\"detection\":\"%s\",\"confidence\":%.1f,\"confirmation\":%d,\"metal\":%d,\"plastic\":%d,\"paper\":%d,\"lastSent\":\"%s\",\"lastSentAgo\":%lu}",
        currentDetection.c_str(), currentConfidence * 100, confirmationCount,
        metalCount, plasticCount, paperCount, lastSentType.c_str(), ago);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, strlen(json));
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL };
    httpd_uri_t snapshot_uri = { .uri = "/snapshot", .method = HTTP_GET, .handler = snapshot_handler, .user_ctx = NULL };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &snapshot_uri);
    }
    
    Serial.println("Camera server started!");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP());
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
    
    // Initialize IR sensor pin
    pinMode(IR_SENSOR_PIN, INPUT);
    Serial.println("IR sensor initialized on GPIO " + String(IR_SENSOR_PIN));
    
    // Connect to WiFi
    // Set Static IP
    IPAddress local_IP(192, 168, 0, 200);
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
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Start mDNS
    if (MDNS.begin("wastesorter-cam")) {
        Serial.println("mDNS responder started: http://wastesorter-cam.local");
    }
    
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    // Start web server
    startCameraServer();

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Send waste type to bin controller via HTTP
* @param      wasteType  The classified waste type (metal, plastic, contaminated)
* @return     true if successful
*/
bool sendToBinController(const char* wasteType) {
    if (WiFi.status() != WL_CONNECTED) {
        ei_printf("WiFi not connected!\r\n");
        return false;
    }
    
    HTTPClient http;
    String url = "http://" + String(binControllerIP) + ":" + String(binControllerPort) + "/sort?type=" + String(wasteType);
    
    ei_printf("Sending to bin controller: %s\r\n", url.c_str());
    
    http.begin(url);
    http.setTimeout(5000);  // 5 second timeout
    
    int httpCode = http.GET();
    
    if (httpCode > 0) {
        ei_printf("HTTP Response: %d\r\n", httpCode);
        String response = http.getString();
        ei_printf("Response: %s\r\n", response.c_str());
        http.end();
        return httpCode == 200;
    } else {
        ei_printf("HTTP Error: %s\r\n", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    // Check if object is detected by IR sensor
    bool objectDetected = digitalRead(IR_SENSOR_PIN);
    if (IR_ACTIVE_LOW) objectDetected = !objectDetected;  // Invert for active LOW sensors
    
    if (!objectDetected) {
        // No object detected - skip inference and wait
        currentDetection = "None";
        currentConfidence = 0;
        delay(100);  // Small delay to avoid busy loop
        return;
    }
    
    // Object detected! Run inference
    ei_printf("\n>>> Object detected by IR sensor, starting inference...\r\n");

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    
    // Find the best detection (highest confidence)
    float maxConfidence = 0;
    const char* bestLabel = nullptr;
    
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
        
        // Track highest confidence detection
        if (bb.value > maxConfidence) {
            maxConfidence = bb.value;
            bestLabel = bb.label;
        }
    }
    
    // Process best detection if confidence is above threshold
    if (bestLabel != nullptr && maxConfidence >= CONFIDENCE_THRESHOLD) {
        ei_printf("\n>>> Detected: %s (confidence: %.2f%%)\r\n", bestLabel, maxConfidence * 100);
        
        // Update dashboard
        currentDetection = String(bestLabel);
        currentConfidence = maxConfidence;
        
        // Check if it's a valid waste type
        if (strcmp(bestLabel, "metal") == 0 || 
            strcmp(bestLabel, "plastic") == 0 || 
            strcmp(bestLabel, "paper") == 0) {
            
            // Check if same class detected consecutively
            if (lastDetectedClass == String(bestLabel)) {
                confirmationCount++;
                ei_printf(">>> Confirmation: %d/%d\r\n", confirmationCount, REQUIRED_CONFIRMATIONS);
            } else {
                // Different class detected, reset counter
                lastDetectedClass = String(bestLabel);
                confirmationCount = 1;
                ei_printf(">>> New detection, starting confirmation: 1/%d\r\n", REQUIRED_CONFIRMATIONS);
            }
            
            // Only send if we have enough confirmations
            if (confirmationCount >= REQUIRED_CONFIRMATIONS) {
                ei_printf(">>> CONFIRMED: %s - Sending to bin controller!\r\n", bestLabel);
                
                if (sendToBinController(bestLabel)) {
                    ei_printf(">>> Successfully sent to bin controller!\r\n");
                    
                    // Update statistics
                    if (strcmp(bestLabel, "metal") == 0) metalCount++;
                    else if (strcmp(bestLabel, "plastic") == 0) plasticCount++;
                    else if (strcmp(bestLabel, "paper") == 0) paperCount++;
                    
                    lastSentType = String(bestLabel);
                    lastSentTime = millis();
                    
                    // Reset confirmation count after successful send
                    confirmationCount = 0;
                    lastDetectedClass = "";
                    currentDetection = "None";
                    currentConfidence = 0;
                    
                    // Add delay to wait for bin to process
                    delay(5000);
                } else {
                    ei_printf(">>> Failed to send to bin controller\r\n");
                }
            }
        }
    } else if (result.bounding_boxes_count == 0) {
        // No detection - keep waiting, don't reset confirmation streak
        currentDetection = "None";
        currentConfidence = 0;
        // Keep lastDetectedClass and confirmationCount unchanged
        // so we continue the streak when object is detected again
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    
    // Find the best prediction
    float maxConfidence = 0;
    int maxIndex = -1;
    
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
        
        if (result.classification[i].value > maxConfidence) {
            maxConfidence = result.classification[i].value;
            maxIndex = i;
        }
    }
    
    // Send to bin controller if confidence is above threshold
    if (maxIndex >= 0 && maxConfidence >= CONFIDENCE_THRESHOLD) {
        const char* detectedClass = ei_classifier_inferencing_categories[maxIndex];
        ei_printf("\n>>> Detected: %s (confidence: %.2f%%)\r\n", detectedClass, maxConfidence * 100);
        
        // Check if it's a valid waste type (metal, plastic, or contaminated)
        if (strcmp(detectedClass, "metal") == 0 || 
            strcmp(detectedClass, "plastic") == 0 || 
            strcmp(detectedClass, "contaminated") == 0) {
            
            // Check if same class detected consecutively
            if (lastDetectedClass == String(detectedClass)) {
                confirmationCount++;
                ei_printf(">>> Confirmation: %d/%d\r\n", confirmationCount, REQUIRED_CONFIRMATIONS);
            } else {
                // Different class detected, reset counter
                lastDetectedClass = String(detectedClass);
                confirmationCount = 1;
                ei_printf(">>> New detection, starting confirmation: 1/%d\r\n", REQUIRED_CONFIRMATIONS);
            }
            
            // Only send if we have enough confirmations
            if (confirmationCount >= REQUIRED_CONFIRMATIONS) {
                ei_printf(">>> CONFIRMED: %s - Sending to bin controller!\r\n", detectedClass);
                
                if (sendToBinController(detectedClass)) {
                    ei_printf(">>> Successfully sent to bin controller!\r\n");
                    // Reset confirmation count after successful send
                    confirmationCount = 0;
                    lastDetectedClass = "";
                    // Add delay to wait for bin to process
                    delay(5000);
                } else {
                    ei_printf(">>> Failed to send to bin controller\r\n");
                }
            }
        }
    } else {
        // No confident detection - reset confirmation
        if (confirmationCount > 0) {
            ei_printf("\n>>> Detection lost, resetting confirmation\r\n");
            confirmationCount = 0;
            lastDetectedClass = "";
        }
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    free(snapshot_buf);

}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
