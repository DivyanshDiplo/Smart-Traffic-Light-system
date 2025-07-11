# SMART TRAFFIC LIGHT SYSTEM
## INTRODUCTION

Traffic congestion is a major issue, especially in densely populated urban areas like Delhi, Gurgaon, and other parts of the NCR region. While several factors contribute to this problem—such as narrow roads, inadequate infrastructure, malfunctioning traffic lights, and high population density—one of the key reasons is the use of outdated traffic lights with fixed transition timings. These conventional systems fail to adapt to real‑time traffic conditions, often resulting in unbalanced green and red light durations across intersections.

To address this challenge, I have developed a **SMART TRAFFIC LIGHT SYSTEM** that dynamically manages traffic flow using basic, cost‑effective, and reliable components. This system is designed to respond in real time, improving traffic efficiency, reducing unnecessary delays and providing green coridors to emergency vehicles.


### Hardwares used
    - ESP32-CAM Modules (4x) along with OV2640 Cameras (4x)
    - HC-SR04 Ultrasonic Sensors (4x)
    - Red, Green and Yellow LEDS
    - 5V Power Supply
    - BreadBoards
    - Resistors (220 Ohms)
    - Jumper Wires (male-to-male and male-to-female)
    - USB-to-TTL Converter (if your ESP32-CAM dosen't have a built in USB port)

### Softwares Used
    - Arduino IDE
    - Jupyter notebook
    - YoloV3 model (download 3 Yolov3 model files which will be discussed later)
    - flask (use it to communicate with esp32. Yes we are handling the computations locally and not using cloud as this is a prototyping stage)

    

## WorkFlow
### 1) Arduino Code 
*First We have to build the circuit and configure the ESP32-CAM modules.* 
**Prerequisites** for Arduino IDE ->

1) Install the ESP32 board in your Arduino IDE
2) Install the following libraries from the Library Manager
3) ArduinoJson by Benoit Blanchon
4) HTTPClient (usually comes with the ESP32 core)

This code is for a single ESP32 unit. You will flash this same code onto all four ESP32 devices, changing only the DIRECTION identifier for each one.

**ESP32 Code (C++ code)**

```
    #include <WiFi.h>
    #include <HTTPClient.h>
    #include <ArduinoJson.h>
    #include "esp_camera.h"
    
    // --- CONFIGURATION ---
    // WIFI CREDENTIALS
    const char* ssid = "YOUR_WIFI_SSID";
    const char* password = "YOUR_WIFI_PASSWORD";
    
    // SERVER DETAILS (Your computer's IP where Jupyter is running)
    const char* serverUrl = "http://192.168.1.10:5000/update_traffic"; // Replace with your PC's IP address
    
    // UNIQUE IDENTIFIER FOR THIS ESP32 UNIT
    // Change this for each of the 4 units: "north", "south", "east", "west"
    const char* DIRECTION = "north";
    
    // ULTRASONIC SENSOR PINS
    #define TRIGGER_PIN 12
    #define ECHO_PIN 13
    
    // ESP32-CAM (AI-Thinker Model) PIN DEFINITION
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
    
    void setup() {
      Serial.begin(115200);
    
      // Initialize Ultrasonic Sensor
      pinMode(TRIGGER_PIN, OUTPUT);
      pinMode(ECHO_PIN, INPUT);
    
      // Connect to Wi-Fi
      WiFi.begin(ssid, password);
      Serial.print("Connecting to WiFi...");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nConnected to WiFi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    
      // Initialize Camera
      camera_config_t config;
      config.ledc_channel = LEDC_CHANNEL_0;
      config.ledc_timer = LEDC_TIMER_0;
      config.pin_d0 = Y2_GPIO_NUM;
      config.pin_d1 = Y3_GPIO_NUM;
      config.pin_d2 = Y4_GPIO_NUM;
      config.pin_d3 = Y5_GPIO_NUM;
      config.pin_d4 = Y6_GPIO_NUM;
      config.pin_d5 = Y7_GPIO_NUM;
      config.pin_d6 = Y8_GPIO_NUM;
      config.pin_d7 = Y9_GPIO_NUM;
      config.pin_xclk = XCLK_GPIO_NUM;
      config.pin_pclk = PCLK_GPIO_NUM;
      config.pin_vsync = VSYNC_GPIO_NUM;
      config.pin_href = HREF_GPIO_NUM;
      config.pin_sscb_sda = SIOD_GPIO_NUM;
      config.pin_sscb_scl = SIOC_GPIO_NUM;
      config.pin_pwdn = PWDN_GPIO_NUM;
      config.pin_reset = RESET_GPIO_NUM;
      config.xclk_freq_hz = 20000000;
      config.pixel_format = PIXFORMAT_JPEG;
      
      // Use QVGA for faster processing, or VGA for better quality
      config.frame_size = FRAMESIZE_VGA; // 640x480
      config.jpeg_quality = 12; // 0-63 lower numbers are higher quality
      config.fb_count = 1;
    
      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
      }
    }
    
    void loop() {
      // Capture a photo
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        delay(1000);
        return;
      }
    
      // Get distance from ultrasonic sensor
      long duration;
      int distance;
      digitalWrite(TRIGGER_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGGER_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH);
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2
    
      Serial.printf("Direction: %s, Distance: %d cm\n", DIRECTION, distance);
      
      // Send data to the server
      sendDataToServer(fb->buf, fb->len, distance);
    
      // Return the frame buffer to be reused
      esp_camera_fb_return(fb);
    
      // Wait for the next cycle
      delay(10000); // Send data every 10 seconds
    }
    
    void sendDataToServer(const uint8_t * image_buffer, size_t image_len, int distance) {
      HTTPClient http;
      
      // Construct the multipart/form-data request
      String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
      String start_request = "--" + boundary + "\r\n";
      start_request += "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n";
      start_request += "Content-Type: image/jpeg\r\n\r\n";
      
      String end_request = "\r\n--" + boundary + "\r\n";
      end_request += "Content-Disposition: form-data; name=\"direction\"\r\n\r\n";
      end_request += String(DIRECTION) + "\r\n";
      end_request += "--" + boundary + "\r\n";
      end_request += "Content-Disposition: form-data; name=\"distance\"\r\n\r\n";
      end_request += String(distance) + "\r\n";
      end_request += "--" + boundary + "--\r\n";
    
      size_t total_len = start_request.length() + image_len + end_request.length();
    
      http.begin(serverUrl);
      http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
      
      // Send the request with the image data
      int httpCode = http.sendRequest("POST", (uint8_t *)start_request.c_str(), start_request.length());
      http.getStream().write(image_buffer, image_len);
      httpCode = http.sendRequest("POST", (uint8_t *)end_request.c_str(), end_request.length());
    
      if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
          Serial.println("Received response:");
          Serial.println(payload);
          
          // Parse the JSON response
          DynamicJsonDocument doc(1024);
          deserializeJson(doc, payload);
          
          int ns_green = doc["ns_green"];
          int ew_green = doc["ew_green"];
          
          Serial.printf("Updated Timers -> NS Green: %d, EW Green: %d\n", ns_green, ew_green);
          // ** ADD YOUR CODE HERE TO CONTROL THE TRAFFIC LIGHT LEDs **
          // For example: controlLEDs(ns_green, ew_green);
          
        } else {
          Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
      } else {
        Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
    
      http.end();
    }
```


## 2) Python Server and ML Processing (Jupyter Notebook)
This code sets up a Flask server inside your Jupyter Notebook. It creates an endpoint to receive data from the ESP32s, processes it with YOLOv3, calculates timers, and sends them back.

Download YOLOv3 files:

    -yolov3.weights
    -yolov3.cfg
    -coco.names
You can get these files from the official YOLO website or various GitHub repositories. Place them in the same directory as your Jupyter Notebook.
Install Dependencies-
```
pip install flask opencv-python numpy
```

## 3) Import Libraries and Initialize
```
import cv2
import numpy as np
import time
from flask import Flask, request, jsonify
import threading
```

## 4) Global State Variables
This dictionary will store the latest data from each direction

```
traffic_data = {
    'north': {'vehicle_count': 0, 'distance': 999, 'timestamp': 0},
    'south': {'vehicle_count': 0, 'distance': 999, 'timestamp': 0},
    'east': {'vehicle_count': 0, 'distance': 999, 'timestamp': 0},
    'west': {'vehicle_count': 0, 'distance': 999, 'timestamp': 0}
}
```

## 5) YOLOv3 Model Setup
```
# Load YOLOv3
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
# Get the names of the output layers by finding their indices in the list of all layer names
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]


with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Vehicle classes in COCO dataset
vehicle_classes = ["car", "motorbike", "bus", "truck"]

print("YOLOv3 Model Loaded Successfully.")
```

## 6) Helper Function for Vehicle Counting

```
def get_vehicle_count_yolo(image):
    """
    Takes an image as input and returns the count of detected vehicles using YOLOv3.
    """
    height, width, channels = image.shape
    
    # Create a blob from the image and perform a forward pass of YOLO
    blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    class_ids = []
    confidences = []
    boxes = []
    
    # Process the outputs
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            if confidence > 0.5 and classes[class_id] in vehicle_classes:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                
    # Apply Non-Max Suppression to remove redundant overlapping boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
    return len(indexes) if isinstance(indexes, np.ndarray) else 0
```

## 7) Adaptive Timer Calculation Logic
```
def calculate_adaptive_green_times(current_traffic_data):
    """
    Calculates ns_green and ew_green based on the latest traffic data.
    The ultrasonic sensor data can be used to estimate queue length.
    A lower distance means a longer queue.
    """
    # Define timing constraints
    MIN_GREEN = 15
    MAX_GREEN = 60
    YELLOW_TIME = 3
    ALL_RED_TIME = 2
    CYCLE_LENGTH = 120
    
    available_green_time = CYCLE_LENGTH - 2 * (YELLOW_TIME + ALL_RED_TIME)

    # Calculate demand scores for each direction (combining vehicle count and queue length)
    demand_scores = {}
    for direction, data in current_traffic_data.items():
        # Weight vehicle count more heavily
        count_score = data['vehicle_count'] * 2.0
        
        # Inversely score distance (lower distance = higher score)
        # Assuming max distance is 300cm.
        queue_score = max(0, (300 - data['distance']) / 100.0) 
        
        demand_scores[direction] = count_score + queue_score

    # Calculate combined demand for opposing directions
    ns_demand = demand_scores['north'] + demand_scores['south']
    ew_demand = demand_scores['east'] + demand_scores['west']
    total_demand = ns_demand + ew_demand
    
    if total_demand == 0:
        ns_green = available_green_time / 2
        ew_green = available_green_time / 2
    else:
        # Proportional allocation
        ns_ratio = ns_demand / total_demand
        ns_green = MIN_GREEN + (ns_ratio * (available_green_time - 2 * MIN_GREEN))
        ew_green = MIN_GREEN + ((1 - ns_ratio) * (available_green_time - 2 * MIN_GREEN))

    # Apply min/max constraints
    ns_green = max(MIN_GREEN, min(ns_green, MAX_GREEN))
    ew_green = max(MIN_GREEN, min(ew_green, MAX_GREEN))
    
    return {
        'ns_green': int(ns_green),
        'ew_green': int(ew_green)
    }
```

## 8) Flask Web Server
```
app = Flask(__name__)

@app.route('/update_traffic', methods=['POST'])
def update_traffic():
    global traffic_data
    
    # Get data from the request
    direction = request.form.get('direction')
    distance = int(request.form.get('distance', 999))
    image_file = request.files.get('image')
    
    if not all([direction, image_file]):
        return jsonify({"error": "Missing data"}), 400

    # Convert image file to OpenCV format
    image_stream = image_file.read()
    np_arr = np.frombuffer(image_stream, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Get vehicle count using YOLOv3
    vehicle_count = get_vehicle_count_yolo(image)
    
    print(f"Received data from {direction}: Vehicle Count = {vehicle_count}, Distance = {distance}cm")

    # Update the global state
    traffic_data[direction] = {
        'vehicle_count': vehicle_count,
        'distance': distance,
        'timestamp': time.time()
    }

    # Calculate the new timer states using the most recent data from all directions
    updated_timers = calculate_adaptive_green_times(traffic_data)
    
    print(f"Calculated Timers: {updated_timers}")
    
    # Return the updated timers to the ESP32
    return jsonify(updated_timers)
```

## 9) Running Flask APP

```
# This allows the Jupyter notebook to remain responsive.
def run_app():
    # Use '0.0.0.0' to make the server accessible on your local network
    app.run(host='0.0.0.0', port=5000)

flask_thread = threading.Thread(target=run_app)
flask_thread.daemon = True
flask_thread.start()

print("Flask server is running on port 5000. Your ESP32s should connect to this machine's IP address.")
```

