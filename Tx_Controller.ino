// ========== LIBRARIES ==========
#include <SPI.h>                     // SPI library for nRF24 radio
#include <nRF24L01.h>                // nRF24L01 radio library
#include <RF24.h>                    // SRF24 radio control library
#include <Wire.h>                    // I2C library 
#include <Adafruit_GFX.h>            // Graphics library for display
#include <Adafruit_SSD1306.h>        // OLED display library

// ========== DISPLAY CONFIGURATION ==========
#define SCREEN_WIDTH 128                                                     // Defines the width of the OLED-display screen 
#define SCREEN_HEIGHT 64                                                     // Defines the height of the OLED-display screen
#define OLED_RESET -1                                                        // Reset pin (the display doesn't have a reset pin so this automates the reset function)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);    // Creates an object display from the class Adafruit_SSD1306 with parameters mentioned above 

// ========== JOYSTICK PINS ==========
#define RX A0                        // Right joysticks X-axes
#define RY A1                        // Right joysticks Y-axes
#define LX A2                        // Left joysticks X-axes
#define LY A3                        // Left joysticks Y-axes

// ========== BUTTONS ==========
#define BUTTON1 2                   // Toggles between Crane / Car mode
#define BUTTON2 5                   // (Optional for future use)

// ========== RADIO CONFIGURERATIONS ==========
RF24 radio(7, 8);                   // Radio object (CE, CSN pins)
const byte address[6] = "00010";    // Radio communication address

// ====== DATA STRUCTS ======
struct ControlData {                // Structure for control data being sent to receive
  int rxVal;
  int ryVal;
  int lxVal;
  int lyVal;
  bool mode;
  bool mode2;
};

struct AckPayload {                 //Structure for acknowledgement data received from rover 
  float temperature;
  float latitude;  
  float longitude; 
};

ControlData data;                   // variable that contains the struct controlData
AckPayload ack;                     // variable that contains the struct AckPayload

// ========== GLOBAL VARIABLES ==========
bool lastButton1 = HIGH;               // variable set to high. This compares to variable BUTTON1
bool lastButton2 = HIGH;               // variable set to high. This compares to variable BUTTON2

unsigned long lastSend = 0;            // Last time data was sent
const unsigned long sendInterval = 20; // Time between each data transmission in milliseconds

// ========== SETUP FUNCTION ==========
void setup() {
  Serial.begin(9600);                  // Initialize serial

  pinMode(BUTTON1, INPUT_PULLUP);      // Configure buttons with internal pull-up resistors
  pinMode(BUTTON2, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);   // Configures OLED-display with different methods
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();

  radio.begin();                      // Configures radiomodule with different methods
  radio.openWritingPipe(address);     // Set transmitter address
  radio.setPALevel(RF24_PA_HIGH);     // High power level for better range
  radio.setDataRate(RF24_1MBPS);      // 1Mbps data rate
  radio.enableAckPayload();           // Enable receiving acknowledgement data
  radio.stopListening();              // Set as transmitter

  Serial.println("TX klar - Crane + Car");
  radio.printDetails();               // Print radio configuration
}

// ========== MAIN LOOP ==========
void loop() {
  if (millis() - lastSend >= sendInterval) {    // Checks time since last data transmission
    lastSend = millis();

    // Read joystick values
    data.rxVal = analogRead(RX);
    data.ryVal = analogRead(RY);
    data.lxVal = analogRead(LX);
    data.lyVal = analogRead(LY);

    // MODE toggle (Crane/Car)
    bool currentButton1 = digitalRead(BUTTON1);
    if (currentButton1 == LOW && lastButton1 == HIGH) {
      data.mode = !data.mode;
      delay(300); // Debounce
    }
    lastButton1 = currentButton1;

    // MODE2 (NOT USED)
    bool currentButton2 = digitalRead(BUTTON2);
    if (currentButton2 == LOW && lastButton2 == HIGH) {
      data.mode2 = !data.mode2;
      delay(300);
    }
    lastButton2 = currentButton2;

    // Send data to RX
    radio.write(&data, sizeof(data));

    // Check for incoming acknowledgement data
    if (radio.isAckPayloadAvailable()) {
      radio.read(&ack, sizeof(ack));
    }

    // ========== UPDATE DISPLAY ==========
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Mode: ");
    display.setTextSize(2);            
    display.setCursor(30, 0);          
    display.print(ack.latitude, 6);    
    display.print(",");                
    display.println(ack.longitude, 6); 
    display.setTextSize(2);
    display.setCursor(60, 0);
    if (data.mode) {
      display.println("CRANE");
    } else {
      display.println("CAR");
    }

    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print("Temp:");
    display.print(ack.temperature, 1);
    display.print("C");
    display.display();

    // ========== DEBUG OUTPUT ==========
    Serial.print("MODE: "); Serial.print(data.mode ? "CRANE" : "CAR");
    Serial.print(" | RX: "); Serial.print(data.rxVal);
    Serial.print(" | RY: "); Serial.print(data.ryVal);
    Serial.print(" | LX: "); Serial.print(data.lxVal);
    Serial.print(" | LY: "); Serial.print(data.lyVal);
    Serial.print(" | TEMP: "); Serial.println(ack.temperature, 1);
  }
}
