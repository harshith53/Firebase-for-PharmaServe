#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>  // Include the Servo library
#include <Wire.h>   // I2C communication
#include <LiquidCrystal_I2C.h>  // LCD library
#include <FirebaseESP8266.h>  // Include Firebase library

// Define GPIO pins for LED, Buzzer, Motion Detection, and Servo
#define LED_PIN      0    // D3 (GPIO 0) for LED
#define BUZZER_PIN   2    // D4 (GPIO 2) for Buzzer
#define BUZZER_PIN2  14   // D5 (GPIO 14) for Buzzer 2
#define MOTION_PIN   12   // D6 (GPIO 12) for IR Motion Sensor
#define SERVO_PIN    13   // D7 (GPIO 13) for Servo motor

// Blynk Authentication Token
#define BLYNK_AUTH_TOKEN ""

// Wi-Fi credentials
const char* ssid = "";         // Your Wi-Fi SSID
const char* password = "";     // Your Wi-Fi password

// Firebase credentials
#define FIREBASE_HOST ""  // Firebase database URL
#define FIREBASE_AUTH ""  // Firebase database secret or authentication token

// Firebase object
FirebaseData firebaseData;

// Create Servo object
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define states for devices
bool ledState = LOW;
bool buzzerState = LOW;
bool buzzerState2 = LOW;
bool irActive = false;

// Setup Blynk virtual pins for LED, Buzzer, Servo, and the new combined control
BlynkTimer timer;

// Debounce variables
unsigned long lastMotionTime = 0;
const unsigned long debounceDelay = 500;  // 500ms debounce delay

// Function to update LED state via Blynk
BLYNK_WRITE(V1) {
  ledState = param.asInt();
  digitalWrite(LED_PIN, ledState);
}

// Function to update Buzzer state via Blynk
BLYNK_WRITE(V2) {
  buzzerState = param.asInt();
  digitalWrite(BUZZER_PIN, buzzerState);
}

// Function to update Servo position via Blynk
BLYNK_WRITE(V3) {
  int servoPosition = param.asInt();
  myServo.write(servoPosition);  // Move the servo to the specified angle
}

// Function to update both LED and Buzzer state together via Blynk (using V4)
BLYNK_WRITE(V4) {
  int state = param.asInt();  // Get the state (either 0 or 1)
  ledState = state;
  buzzerState = state;
  digitalWrite(LED_PIN, ledState);
  digitalWrite(BUZZER_PIN, buzzerState);
}

// Setup Blynk, Firebase, and connect to Wi-Fi
void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  // Set LED, Buzzer, and Motion pins as inputs/outputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUZZER_PIN2, OUTPUT);
  pinMode(MOTION_PIN, INPUT);

  // Set initial states of the devices to OFF
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUZZER_PIN2, LOW);

  // Initialize servo motor
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // Initial position at 90 degrees

  // Initialize the LCD with 16 columns and 2 rows
  lcd.init();
  delay(500);  // Delay to ensure LCD initializes properly
  lcd.backlight();  // Turn on the backlight
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  // Connect to Wi-Fi
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password, "blynk.cloud", 80);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Connect to Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
}

void loop() {
  Blynk.run();  // Run Blynk

  // Check the motion sensor state
  int motionState = digitalRead(MOTION_PIN);
  unsigned long currentTime = millis();

  if (motionState == LOW && (currentTime - lastMotionTime > debounceDelay)) {
    if (!buzzerState2) {
      digitalWrite(BUZZER_PIN2, HIGH);  // Turn on Buzzer if not controlled by Blynk
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion Detected");
    Serial.println("Motion Detected");

    irActive = true;
    lastMotionTime = currentTime;
  } else if (motionState == HIGH && irActive) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    Serial.println("System Ready");

    irActive = false;
  }

  // Firebase logic to control LED and Buzzer together
  if (Firebase.getBool(firebaseData, "/control/led_buzzer")) {
    bool state = firebaseData.boolData();
    ledState = state;
    buzzerState = state;
    digitalWrite(LED_PIN, ledState);
    digitalWrite(BUZZER_PIN, buzzerState);
    Serial.print("Firebase LED & Buzzer State: ");
    Serial.println(state);
  } else {
    Serial.print("Firebase read failed: ");
    Serial.println(firebaseData.errorReason());
  }

  // Override IR sensor behavior with Blynk control
  digitalWrite(LED_PIN, ledState);
  digitalWrite(BUZZER_PIN, buzzerState);
  digitalWrite(BUZZER_PIN2, buzzerState2);
}
