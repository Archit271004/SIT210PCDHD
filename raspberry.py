import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import bluetooth

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# MQTT setup
broker_address = "broker.emqx.io"
port = 1883
client_id = "mqttx_d51f1941"
topic = "sensor/distance"

mqtt_client = mqtt.Client(client_id)

# Bluetooth setup
serverMACAddress = 'ec:62:60:8f:5a:e6'  
port_bt = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

# Debug: Verify connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code %d\n", rc)

mqtt_client.on_connect = on_connect

def connect_mqtt():
    try:
        mqtt_client.connect(broker_address, port)
        mqtt_client.loop_start()
        return True
    except:
        return False

def connect_bluetooth():
    try:
        sock.connect((serverMACAddress, port_bt))
        return True
    except:
        return False

def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(2)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

mqtt_connected = connect_mqtt()
bt_connected = False

try:
    while True:
        dist = measure_distance()
        print(f"Distance: {dist} cm")
        if dist < 10:
            message = "1"
        else:
            message = "0"

        if mqtt_connected:
            result = mqtt_client.publish(topic, message)
            status = result[0]
            if status == 0:
                print(f"Sent {message} to topic {topic}")
            else:
                print(f"Failed to send message to topic {topic}, switching to Bluetooth")
                mqtt_client.loop_stop()
                mqtt_connected = False
                bt_connected = connect_bluetooth()
        elif bt_connected:
            try:
                sock.send(message)
                print(f"Sent {message} via Bluetooth")
            except:
                print("Failed to send message via Bluetooth, retrying MQTT")
                bt_connected = False
                mqtt_connected = connect_mqtt()
        else:
            mqtt_connected = connect_mqtt()
            if not mqtt_connected:
                bt_connected = connect_bluetooth()

        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
    if mqtt_connected:
        mqtt_client.loop_stop()
    if bt_connected:
        sock.close()
