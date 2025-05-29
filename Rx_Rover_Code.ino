// ========== Libaries ==========
#include <SPI.h>          // SPI library for nRF24 radio
#include <nRF24L01.h>     // nRF24L01 radio library
#include <RF24.h>         // RF24 radio control library
#include <Servo.h>        // Servo control library
#include <DHT.h>          // Temperature and humidity sensor
#include <TinyGPS++.h>    // GPS parsing library

// ========== CONSTANTS ==========
const int MAX_INPUT_LENGTH = 30;                // Max length for serial input buffer
const unsigned long SERIAL_CHECK_INTERVAL = 10; // How often to check serial port (ms)
const unsigned long ULTRASONIC_INTERVAL = 100;  // How often to check for obstacles (ms)
const int angleMin = 0, angleMax = 180;         // Servo angle limits for arm
const int gripperMin = 30, gripperMax = 160;    // Gripper servo angle limits
const int deadZone = 30;                        // Deadzone for joystick inputs
const unsigned long updateInterval = 20;        // How often to update radio (ms)
const unsigned long tempSendInterval = 10000;   // How often to send temperature (ms)

// ========== OBJECTS ==========
TinyGPSPlus gps;                                // GPS parser object
RF24 radio(7, 8);                               // Radio object (CE, CSN pins)
DHT dhtSensor(45, DHT22);                       // Temperature sensor object
Servo movementServoLX, movementServoRY, movementServoRX, gripperServo, steeringServo; // Servo objects

// ========== PIN DEFINITIONS ==========
const int pinLX = 11, pinRY = 12, pinRX = 13;   // Servo control pins
const int pinGripper = 10, pinSteering = 9;     // Gripper and steering servo pins
const int AIA = 5, AIB = 6, BIA = 2, BIB = 3;   // Motor control pins
const int trans_pin = 46;                       // Ultrasonic trigger pin
const int recv_pin = 44;                        // Ultrasonic echo pin

// ========== VARIABLES ==========
String input = "";                              // Buffer for serial input
int angleLX = 95, angleRY = 12, angleRX = 45;   // Initial servo positions
int gripperAngle = 90, steeringAngle = 75;      // Initial gripper and steering positions
// Timers for non-blocking operations
unsigned long lastSerialCheck = 0;              
unsigned long lastUpdate = 0; 
unsigned long lastSignal = 0; 
unsigned long lastTempSend = 0; 
unsigned long lastUltrasonicCheck = 0;


// ========== DATA STRUCTURES ==========
// Structure for incoming radio control data
struct ControlData {
  int rxVal, ryVal, lxVal, lyVal;      // Joystick values (0-1023)
  bool mode, mode2;                    // Mode switches
} data;

// Structure for outgoing acknowledgement data
struct AckPayload {
  float temperature;                   // Current temperature
  float latitude;                      // GPS latitude
  float longitude;                     // GPS longitude
} ack;

// ========== FUNCTION PROTOTYPES ==========
void checkGPS();                       // Check for new GPS data
void processMessage(String msg);       // Process incoming serial messages
void sendTemperature();                // Send temperature over serial
void handleInput();                    // Handle incoming control data
void checkObstacle();                  // Check for obstacles with ultrasonic sensor
int updateAngleWithDeadzone(int angle, int value, int direction, int minVal = angleMin, int maxVal = angleMax); // Update servo angle with deadzone
void driveWithJoystick(int throttle);  // Control vehicle motors
void steerWithJoystick(int input);     // Control steering servo
void STOP();                           // Stop all motors

// ========== SETUP ==========
void setup() {
  // Initialize serial ports
  Serial.begin(9600);                  // Debug serial
  Serial1.begin(9600);                 // Communication with other systems
  Serial2.begin(9600);                 // GPS serial
  
  dhtSensor.begin();                   // Start temperature sensor
  
  // Attach servos to their pins
  movementServoLX.attach(pinLX);
  movementServoRY.attach(pinRY);
  movementServoRX.attach(pinRX);
  gripperServo.attach(pinGripper);
  steeringServo.attach(pinSteering);

  // Setup ultrasonic sensor pins
  pinMode(trans_pin, OUTPUT);          // Trigger is output
  pinMode(recv_pin, INPUT);            // Echo is input

  // Motor setup
  pinMode(AIA, OUTPUT); pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT); pinMode(BIB, OUTPUT);

  // Radio setup
  const byte address[6] = "00010";     // Radio address
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_HIGH);      // High power level
  radio.setDataRate(RF24_1MBPS);       // 1Mbps data rate
  radio.enableAckPayload();            // Enable acknowledgement payloads
  radio.startListening();              // Start listening for controller

  Serial.println("System initialized");
}

// ========== MAIN LOOP ==========
void loop() {
  // Non-blocking serial communication check
  if (millis() - lastSerialCheck >= SERIAL_CHECK_INTERVAL) {
    lastSerialCheck = millis();
    while (Serial1.available() && input.length() < MAX_INPUT_LENGTH) {
      char c = Serial1.read();
      if (c == '\n') {                // End of message
        processMessage(input);        // Process complete message
        input = "";                   // Reset buffer
      } else {
        input += c;                   // Add character to buffer
      }
    }
  }

  checkGPS();                         // Check for new GPS data

  // Non-blocking ultrasonic obstacle check
  if (millis() - lastUltrasonicCheck >= ULTRASONIC_INTERVAL) { 
    lastUltrasonicCheck = millis();
    checkObstacle();
  }

  // Radio communication handling
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();
    
    if (radio.available()) {          // New data available
      radio.read(&data, sizeof(data));// Read control data
      lastSignal = millis();          // Update last signal time

      // Prepare acknowledgement data
      ack.temperature = dhtSensor.readTemperature();
      radio.writeAckPayload(1, &ack, sizeof(ack));

      handleInput();                  // Process control data
    }

    // Safety feature - stop if signal lost
    if (millis() - lastSignal > 2000) {
      Serial.println("⚠️ No signal - STOPPING");
      STOP();
    }
  }

  // Periodic temperature reporting
  if (millis() - lastTempSend >= tempSendInterval) {
    lastTempSend = millis();
    sendTemperature();
  }
}

// ========== ULTRASONIC FUNCTION ==========
void checkObstacle() {
  // Only active in car mode (data.mode = false)
  if (data.mode) return;

  // Send ultrasonic pulse
  digitalWrite(trans_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trans_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trans_pin, LOW);

  // Measure echo pulse duration with timeout (30ms = ~5m max range)
  float duration = pulseIn(recv_pin, HIGH, 30000);
  float distance_cm = duration * 0.0343 / 2;  // Convert to distance in cm

  // Stop motors if obstacle closer than 20cm (and measurement is valid)
  if (distance_cm > 0 && distance_cm < 20) {
    STOP();
  }
}
///////////////////////////////

void checkGPS() {
  // Read and parse all available GPS data
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {  // Parse GPS data
      if (gps.location.isValid()) {    // Check if location is valid
        // Update acknowledgement data with current position
        ack.latitude = gps.location.lat();
        ack.longitude = gps.location.lng();
        
        // Print position to debug serial
        Serial.print("GPS: ");
        Serial.print(ack.latitude, 6);
        Serial.print(",");
        Serial.println(ack.longitude, 6);
      }
    }
  }
}

void processMessage(String msg) {
  Serial.print("[MSG] Received: "); Serial.println(msg);
  
  // Check for location message (starts with 'L')
  if (msg.startsWith("L")) {
    msg.remove(0, 1);                  // Remove 'L' prefix
    int separator = msg.indexOf(';');  // Find coordinate separator
    
    if (separator == -1) {
      Serial.println("ERROR: Missing separator");
      return;
    }

    // Parse coordinates
    float latitude = msg.substring(0, separator).toFloat();
    float longitude = msg.substring(separator+1).toFloat();

    // Validate coordinates
    if (latitude == 0.0 || longitude == 0.0) {
      Serial.println("ERROR: Invalid coordinates");
      return;
    }

    // Print received coordinates
    Serial.print("Coordinates received: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);
  }
}

// ========== TEMPERATURE REPORTING ==========
void sendTemperature() {
  // Send current temperature over serial
  Serial1.print("Temp: ");
  Serial1.print(ack.temperature, 1);
  Serial1.println(" C");
}

// ========== INPUT HANDLING ==========
void handleInput() {
  if (!data.mode) {
    // Car mode - control vehicle movement
    driveWithJoystick(data.lyVal);  // Left Y controls throttle
    steerWithJoystick(data.rxVal);  // Right X controls steering
  } else {
    // Crane mode - control robotic arm
    // Update servo angles based on joystick inputs
    angleLX = updateAngleWithDeadzone(angleLX, data.lxVal, -1);
    angleRX = updateAngleWithDeadzone(angleRX, data.rxVal, 1);
    angleRY = updateAngleWithDeadzone(angleRY, data.ryVal, 1);
    gripperAngle = updateAngleWithDeadzone(gripperAngle, data.lyVal, 1, gripperMin, gripperMax);

    // Write new angles to servos
    movementServoLX.write(angleLX);
    movementServoRY.write(angleRY);
    movementServoRX.write(angleRX);
    gripperServo.write(gripperAngle);
  }
}

// ========== SERVO CONTROL ==========
int updateAngleWithDeadzone(int angle, int value, int direction, int minVal, int maxVal) {
  // Only update angle if joystick is outside deadzone
  if (abs(value - 512) > deadZone) {
    // Map joystick position to speed (1-10)
    int speed = map(abs(value - 512), deadZone, 512, 1, 10);
    // Update angle based on direction and joystick position
    angle += (value > 512) ? speed * direction : -speed * direction;
    // Constrain angle to specified limits
    angle = constrain(angle, minVal, maxVal);
  }
  return angle;
}

// ========== VEHICLE CONTROL ==========
void driveWithJoystick(int throttle) {
  int center = 512;   // Joystick center position
  int dead = 30;      // Deadzone size

  // Forward movement
  if (throttle > center + dead) {
    int power = map(throttle, center + dead, 1023, 0, 255);
    analogWrite(AIA, power); analogWrite(AIB, 0);
    analogWrite(BIA, power); analogWrite(BIB, 0);
  } 
  // Reverse movement
  else if (throttle < center - dead) {
    int power = map(throttle, center - dead, 0, 0, 255);
    analogWrite(AIA, 0); analogWrite(AIB, power);
    analogWrite(BIA, 0); analogWrite(BIB, power);
  } 
  // Stop if in deadzone
  else {
    STOP();
  }
}

void steerWithJoystick(int input) {
  int center = 512;   // Joystick center position
  int dead = 30;      // Deadzone size
  // Calculate target steering angle (82 is center)
  int target = (abs(input - center) < dead) ? 82 : map(input, 0, 1023, 45, 120);

  // Smoothly move steering to target position
  if (steeringAngle < target) {
    steeringAngle = min(steeringAngle + 8, target);
  } else if (steeringAngle > target) {
    steeringAngle = max(steeringAngle - 8, target);
  }
  steeringServo.write(steeringAngle);
}

// ========== SAFETY FUNCTION ==========
void STOP() {
  // Stop all motors by setting all control pins low
  analogWrite(AIA, 0);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, 0);
}
