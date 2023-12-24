from machine import Pin, SoftI2C, I2C
import network
import time
from umqtt.robust import MQTTClient
import sys
import dht
import utime
from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
from math import sqrt
from mpu6050 import init_mpu6050, get_mpu6050_data
from ssd1306 import SSD1306_I2C

i2c = SoftI2C(sda=Pin(16),  # Here, use your I2C SDA pin
                  scl=Pin(17),  # Here, use your I2C SCL pin
                  freq=400000)  # Fast: 400kHz, slow: 100kHz
sensor = MAX30102(i2c=i2c)  # DHT11 Sensor on Pin 4 of ESP32
if sensor.i2c_address not in i2c.scan():
    print("Sensor not found.")
elif not (sensor.check_part_id()):
    # Check that the targeted sensor is compatible
    print("I2C device ID not corresponding to MAX30102 or MAX30105.")
else:
    print("Sensor connected and recognized.")

print("Setting up sensor with default configuration.", '\n')
sensor.setup_sensor()

# It is also possible to tune the configuration parameters one by one.
# Set the sample rate to 400: 400 samples/s are collected by the sensor
sensor.set_sample_rate(400)
# Set the number of samples to be averaged per each reading
sensor.set_fifo_average(8)
# Set LED brightness to a medium value
sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)

# The readTemperature() method allows to extract the die temperature in °C    
print("Reading temperature in °C.", '\n')
print(sensor.read_temperature())

WIFI_SSID = 'abcxyz'
WIFI_PASSWORD = 'education.india'

mqtt_client_id = bytes('client_' + '12321', 'utf-8')  # Just a random client ID

ADAFRUIT_IO_URL = 'io.adafruit.com'
ADAFRUIT_USERNAME = 'amogh_jha'
ADAFRUIT_IO_KEY = 'aio_RCFB69hPrAbqI6AoD7uIbPvH6Bkb'

TEMP_FEED_ID = 'temp'
RED_FEED_ID = 'red'
IR_FEED_ID = 'ir'
STEPS_FEED_ID = 'steps'
HR_FEED_ID= 'hr'
SPO2_FEED_ID = 'spo2'
LOCATION_FEED_ID = 'location'

i2c_mpu = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
init_mpu6050(i2c_mpu)

steps = 0
last_step_time = utime.ticks_ms()  # Record last step time for cadence calculation
rumination_count = 0
rumination_threshold = 100  # Adjust this based on calibration

def connect_wifi():
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.connect(WIFI_SSID, WIFI_PASSWORD)
    if not wifi.isconnected():
        print('connecting..')
        timeout = 0
        while not wifi.isconnected() and timeout < 20:
            print(20 - timeout)
            timeout += 1
            time.sleep(1)
    if wifi.isconnected():
        print('connected')
    else:
        print('not connected')
        sys.exit()

connect_wifi()  # Connecting to WiFi Router

client = MQTTClient(client_id=mqtt_client_id,
                    server=ADAFRUIT_IO_URL,
                    user=ADAFRUIT_USERNAME,
                    password=ADAFRUIT_IO_KEY,
                    ssl=False)
try:
    client.connect()
except Exception as e:
    print('could not connect to MQTT server {}{}'.format(type(e).__name__, e))
    sys.exit()

def cb(topic, msg):  # Callback function
    print('Received Data:  Topic = {}, Msg = {}'.format(topic, msg))
    received_data = str(msg, 'utf-8')  #   Receiving Data
    if received_data == "0":
        led.value(0)
    if received_data == "1":
        led.value(1)

temp_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, TEMP_FEED_ID), 'utf-8')
red_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, RED_FEED_ID), 'utf-8')
ir_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, IR_FEED_ID), 'utf-8')
steps_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, STEPS_FEED_ID), 'utf-8')
hr_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, HR_FEED_ID), 'utf-8')
spo2_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, SPO2_FEED_ID), 'utf-8')
location_feed = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, LOCATION_FEED_ID), 'utf-8')

client.set_callback(cb)

last_sens_data_time = 0
sens_data_interval = 5000  # Interval in milliseconds (5 seconds)

WIDTH = 128
HEIGHT = 64
i2c_oled = I2C(1, scl=Pin(19), sda=Pin(18), freq=200000)
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c_oled)

while True:
    try:
        client.check_msg()
        sensor.check()

        # Check if it's time to send sensor data
        current_time = utime.ticks_ms()
        
        data = get_mpu6050_data(i2c_mpu)
        accel_x = data['accel']['x']
        accel_y = data['accel']['y']
        accel_z = data['accel']['z']
        gyro_x = data['gyro']['x']
        gyro_y = data['gyro']['y']
        gyro_z = data['gyro']['z']

        # Step counting logic
        total_acceleration = sqrt(accel_x**2 + accel_y**2 + accel_z**2)  # Calculate total acceleration
        if total_acceleration > 1.5:  # Adjust threshold based on calibration
            current_time = utime.ticks_ms()
            if current_time - last_step_time > 200:  # Ignore steps too close together
                steps += 1
                last_step_time = current_time

        # Rumination calculation (simplified example, adjust based on research)
        if sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2) > rumination_threshold:
            rumination_count += 1
        
        if utime.ticks_diff(current_time, last_sens_data_time) >= sens_data_interval:
            red_range = (6000, 11000)
            ir_range = (7000, 17000)
            heart_rate_range = (80, 120)
            spo2_range = (93.4, 99.6)
            
            temp = sensor.read_temperature()
            red = sensor.pop_red_from_storage()
            ir = sensor.pop_ir_from_storage()
            heart_rate_factor = (red - red_range[0]) / (red_range[1] - red_range[0]) + (ir - ir_range[0]) / (ir_range[1] - ir_range[0])
            heart_rate = heart_rate_range[0] + heart_rate_factor * (heart_rate_range[1] - heart_rate_range[0])
            
            spo2_factor = (ir - red) / (ir_range[1] - red_range[0])
            spo2 = spo2_range[0] + spo2_factor * (spo2_range[1] - spo2_range[0])
            
            time.sleep(12)
            
            testdata= {'latitude': '12', 'longitude': '34'}
                    
            client.publish(temp_feed,
                           bytes(str(temp), 'utf-8'),
                           qos=0)
            client.publish(steps_feed, bytes(str(steps), 'utf-8'),qos=0)
            client.publish(hr_feed, bytes(str(heart_rate), 'utf-8'),qos=0)
            client.publish(spo2_feed, bytes(str(spo2), 'utf-8'),qos=0)
            client.publish(location_feed, msg=str(testdata))
            print("Temp - ", str(temp))
            print("Red - ", str(red))
            print("IR - ", str(ir))
            print("Steps - ", str(steps))
            print("Heart Rate - ", str(heart_rate))
            print("SPO2 - ", str(spo2))
            print('Msg sent')
            last_sens_data_time = current_time
            
            oled.fill(0)
            oled.text("COW params", 0,0)
            oled.text(f"Temp: {temp}", 0,16)
            if heart_rate > 80:
                oled.text(f"Pulse: {heart_rate}",0,32)
            else:
                oled.text(f"Pulse not found.", 0, 32)
            if spo2 > 94:
                oled.text(f"SpO2: {spo2}", 0,48)
            else:
                oled.text("no oxygen", 0,48)
            oled.show()	
    except Exception as e:
        print('Exception:', e)
        client.disconnect()
        sys.exit()
