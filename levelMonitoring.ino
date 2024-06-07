#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include <Firebase_Arduino_WiFiNINA.h>

// WiFi network details
const char* wifiSSID = "Archit"; // Replace with your WiFi SSID
const char* wifiPassword = "tag271004"; // Replace with your WiFi password

// IFTTT Webhook details
const char* iftttServer = "maker.ifttt.com";
const char* iftttEventName = "sanitizer_low"; // Replace with your IFTTT event name
const char* iftttKey = "b1vpuiVCmd93sDPeGuHUc1"; // Replace with your IFTTT key

// Firebase details
#define FIREBASE_HOST "monitorrefill-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "URrUmiRqYRrVuW5cxgAYilKpAApITLjNg00WfOLs"

// Pin assignments
const int trigPin = 5;  // TRIG pin for the ultrasonic sensor
const int echoPin = 18; // ECHO pin for the ultrasonic sensor
const int ledPin = 13;  // LED pin

WiFiClient wifiClient;
HttpClient httpClient = HttpClient(wifiClient, iftttServer, 80);
FirebaseData firebaseData;

unsigned long ledOnTime = 0; // To track when the LED was turned on
bool ledCurrentlyOn = false; // To track the current state of the LED

void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(wifiSSID, wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected to WiFi!");
}

void sendIFTTTNotification() {
    Serial.println("Sending IFTTT notification...");
    httpClient.beginRequest();
    httpClient.get("/trigger/" + String(iftttEventName) + "/with/key/" + String(iftttKey));
    httpClient.endRequest();
    int statusCode = httpClient.responseStatusCode();
    String response = httpClient.responseBody();
    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);
}

long measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1;
    return distance;
}

void setup() {
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(ledPin, OUTPUT);

    connectToWiFi();

    // Initialize Firebase with WiFi credentials
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, wifiSSID, wifiPassword);
    Firebase.reconnectWiFi(true);
}

void loop() {
    long distance = measureDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 15) {
        sendIFTTTNotification();
    }

    if (Firebase.getBool(firebaseData, "/LEDstate")) {
        bool ledState = firebaseData.boolData();
        if (ledState && !ledCurrentlyOn) {
            ledOnTime = millis();
            ledCurrentlyOn = true;
            digitalWrite(ledPin, HIGH);
        }
        if (ledState && ledCurrentlyOn && (millis() - ledOnTime >= 5000)) {
            // Turn off the LED and update Firebase
            ledCurrentlyOn = false;
            digitalWrite(ledPin, LOW);
            if (Firebase.setBool(firebaseData, "/LEDstate", false)) {
                Serial.println("LED turned off and Firebase updated");
            } else {
                Serial.println("Failed to update Firebase");
            }
        }
    } else {
        digitalWrite(ledPin, LOW);
        ledCurrentlyOn = false;
    }

    delay(5000);
}
