#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoBLE.h>

// WiFi network details
const char* wifiSSID = "Archit"; // Replace with your WiFi SSID
const char* wifiPassword = "tag271004"; // Replace with your WiFi password

// MQTT Broker details
const char* mqttBroker = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttClientId = "mqttx_d51f1941";
const char* mqttTopic = "sensor/distance";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// BLE Service and Characteristic UUIDs
BLEService handService("19b10010-e8f2-537e-4f6c-d104768a1214");
BLEStringCharacteristic handCharacteristic("19b10011-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 32);

// Pin assignments
const int ledPin = 13;  // Pin for LED

void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(wifiSSID, wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected to WiFi!");
}

void messageReceived(int messageSize) {
    String topic = mqttClient.messageTopic();
    String messageContent = "";

    while (mqttClient.available()) {
        messageContent += (char)mqttClient.read();
    }

    Serial.println("Message received: '" + messageContent + "' on topic: '" + topic + "'");

    if (messageContent == "1") {
        digitalWrite(ledPin, HIGH); // Turn on LED
        Serial.println("LED turned ON");
    } else if (messageContent == "0") {
        digitalWrite(ledPin, LOW);  // Turn off LED
        Serial.println("LED turned OFF");
    }
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT broker");
    while (!mqttClient.connect(mqttBroker, mqttPort)) {
        Serial.print(".");
        delay(5000);
    }

    Serial.println("Connected to MQTT broker!");
    mqttClient.subscribe(mqttTopic); // Subscribe to the sensor/distance topic
    mqttClient.onMessage(messageReceived); // Set the message received callback
}

void setupBLE() {
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }
    BLE.setLocalName("HandSensor");
    BLE.setAdvertisedService(handService);
    handService.addCharacteristic(handCharacteristic);
    BLE.addService(handService);
    handCharacteristic.writeValue("0");
    BLE.advertise();
    Serial.println("BLE device active, waiting for connections...");
}

void setup() {
    Serial.begin(9600);
    connectToWiFi();
    connectToMQTT();

    pinMode(ledPin, OUTPUT); // Set LED pin as output
    digitalWrite(ledPin, LOW); // Ensure the LED is initially off
    setupBLE();
}

void loop() {
    if (!mqttClient.connected()) {
        connectToMQTT();
    }

    mqttClient.poll(); // Keep the connection alive

    BLEDevice central = BLE.central();
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());

        while (central.connected()) {
            if (handCharacteristic.written()) {
                String value = handCharacteristic.value();
                if (value == "1") {
                    digitalWrite(ledPin, HIGH);
                    Serial.println("LED turned ON via BLE");
                } else if (value == "0") {
                    digitalWrite(ledPin, LOW);
                    Serial.println("LED turned OFF via BLE");
                }
            }
        }
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}
