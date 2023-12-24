'''
NOTES:
1. Line 128-130
'''


def fahrenheit(value):
    return 1.8*value + 32


def main():
    # Imports
    from machine import SoftI2C, Pin, UART, I2C, Timer
    from micropyGPS import MicropyGPS
    from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
    from mpu6050 import init_mpu6050, get_mpu6050_data
    from ssd1306 import SSD1306_I2C
    from utime import ticks_diff, ticks_us, time
    from time import sleep
    from math import sqrt
    import sys
    import network
    from umqtt.robust import MQTTClient

    # CONSTANTS
    WIDTH = 128  # For OLED
    HEIGHT = 64  # For OLED
    # Constants for check_for_beat
    RATE_SIZE = 4
    rates = [0] * RATE_SIZE
    rateSpot = 0
    lastBeat = 0
    beatsPerMinute = 0
    beatAvg = 0
    # Constants for Adafruit
    WIFI_SSID = 'abcxyz'
    WIFI_PASSWORD = 'education.india'

    # Just a random client ID
    mqtt_client_id = bytes('client_'+'12321', 'utf-8')

    ADAFRUIT_IO_URL = 'io.adafruit.com'
    ADAFRUIT_USERNAME = 'amogh_key'
    ADAFRUIT_IO_KEY = 'aio_RCFB69hPrAbqI6AoD7uIbPvH6Bkb'

    TEMP_FEED_ID = 'temperature'
    IR_FEED_ID = 'ir'
    LOCATION_FEED_ID = 'location'
    RED_FEED_ID = 'red'
    SPO2_FEED_ID = 'spo2'
    STEPS_FEED_ID = 'steps'

    # VARIABLES
    compute_frequency = True
    t_start = ticks_us()  # Starting time of the acquisition
    samples_n = 0  # Number of samples that have been collected

    # Variables for step counting and rumination calculation
    steps = 0
    steps_threshold = 0

    # Variables for GPS
    TIMEZONE = 5
    my_gps = MicropyGPS(TIMEZONE)

    # DATA
    red = []
    ir = []
    times = [0]
    accelerations = [0]
    distances = [0, 0]

    # Adafruit Feeds
    temp_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, TEMP_FEED_ID), 'utf-8')
    ir_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, IR_FEED_ID), 'utf-8')
    location_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, LOCATION_FEED_ID), 'utf-8')
    red_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, RED_FEED_ID), 'utf-8')
    spo2_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, SPO2_FEED_ID), 'utf-8')
    steps_feed = bytes(
        '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, STEPS_FEED_ID), 'utf-8')

    # FUNCTIONS
    def convert(parts):
        if (parts[0] == 0):
            return None

        data = parts[0]+(parts[1]/60.0)
        # parts[2] contain 'E' or 'W' or 'N' or 'S'
        if (parts[2] == 'S'):
            data = -data
        if (parts[2] == 'W'):
            data = -data

        data = '{0:.6f}'.format(data)  # to 6 decimal places
        return str(data)

    def check_for_beat(irValue):
        import utime

        global lastBeat
        global beatsPerMinute
        global rateSpot
        global beatAvg

        if irValue > 50000:
            return False

        if irValue < 50000:
            # We sensed a beat!
            delta = utime.ticks_ms() - lastBeat
            lastBeat = utime.ticks_ms()

            beatsPerMinute = 60 / (delta / 1000.0)

            if 20 < beatsPerMinute < 255:
                # Store this reading in the array
                rates[rateSpot] = int(beatsPerMinute)
                rateSpot = (rateSpot + 1) % RATE_SIZE  # Wrap variable

                # Take average of readings
                beatAvg = sum(rates) // RATE_SIZE

            return True

        return False

    def spo2():
        red_dc = sum(red[-150:])/len(red[-150:])
        ir_dc = sum(ir[-150:])/len(ir[-150:])

        red_ac = abs(max(red[-150:]) - min(red[-150:]))
        ir_ac = abs(max(ir[-150:]) - min(ir[-150:]))

        R = (red_ac/red_dc)/(ir_ac/ir_dc)
        spot_reading = 110 - 25*R

        return spot_reading

    def debounce():
        pull()
        irq(rel(0))

    def connect_wifi():
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        
        wifi.connect(WIFI_SSID, WIFI_PASSWORD)
        if not wifi.isconnected():
            print('connecting..')
            timeout = 0
            while (not wifi.isconnected() and timeout < 10):
                print(10 - timeout)
                timeout = timeout + 1
        if (wifi.isconnected()):
            print('connected')
        else:
            print('not connected')
            sys.exit()

    def sens_data(data):
        client.publish(temp_feed,bytes(str(data[0]), 'utf-8'),qos=0)
        client.publish(ir_feed,bytes(str(data[1]), 'utf-8'),qos=0)
        client.publish(location,(data[4], data[5]),qos=0)
        client.publish(spo2_feed,bytes(str(data[2]), 'utf-8'),qos=0)
        client.publish(steps_feed,bytes(str(data[3]), 'utf-8'),qos=0)
    connect_wifi()

    # I2C software instance
    i2c_max = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)

    i2c_mpu = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
    i2c_oled = I2C(1, scl=Pin(19), sda=Pin(18), freq=200000)
    #gps_module = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
    
    # Adafruit
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

    def cb(topic, msg):                             # Callback function
        print('Received Data:  Topic = {}, Msg = {}'.format(topic, msg))
        recieved_data = str(msg, 'utf-8')  # Recieving Data

    #init_mpu6050(i2c_mpu)
    oled = SSD1306_I2C(WIDTH, HEIGHT, i2c_oled)
    max_sensor = MAX30102(i2c=i2c_max)  # An I2C instance is required

    max_sensor.setup_sensor()
    max_sensor.set_sample_rate(400)
    max_sensor.set_fifo_average(8)
    max_sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
    max_sensor.setPulseAmplitudeRed(0x0A)
    max_sensor.setPulseAmplitudeGreen(0)

    sleep(1)

    print("Starting data acquisition from RED & IR registers...", '\n')

    sleep(1)

    while True:
        max_sensor.check()

        if max_sensor.available():
            import math

            # STEPS ALGRORITHM:
            '''
            data = get_mpu6050_data(i2c_mpu)
            accel_x = data['accel']['x']
            accel_y = data['accel']['y']
            accel_z = data['accel']['z']
            gyro_x = data['gyro']['x']
            gyro_y = data['gyro']['y']
            gyro_z = data['gyro']['z']
            '''
            
            total_acceleration = 9.8 * math.sqrt(
                accel_x ** 2 + accel_y ** 2 + accel_z ** 2)
            times.append(utime.ticks_ms)
            accelerations.append(total_acceleration)

            dt = times[-1] - times[-2]
            # Average velocity
            v_avg = (accelerations[-1] + accelerations[-2]) / 2
            if v_avg < 40:
                d = distances[-2] + v_avg * dt
                distances.append(d)
                steps = d/steps_threshold

            # CODE FOR GPS MODULE
            '''
            length = gps_module.any()
            if length > 0:
                b = gps_module.read(length)
                for x in b:
                    msg = my_gps.update(chr(x))
            '''
            # Readings
            red_reading = max_sensor.pop_red_from_storage()
            ir_reading = max_sensor.pop_ir_from_storage()

            red.append(red_reading)
            ir.append(ir_reading)

            temp_reading = max_sensor.read_temperature()
            check_for_beat(ir_reading)
            hr_reading = beatsPerMinute
            spot_reading = spo2()
            
            '''
            latitude = convert(my_gps.latitude)
            longitude = convert(my_gps.longitude)
            t = my_gps.timestamp
            # t[0] => hours : t[1] => minutes : t[2] => seconds
            speed = my_gps.speed_string('kph')  # 'kph' or 'mph' or 'knot'
            '''

            redr, irr, tempr, hrr, spotr = str(red_reading), str(ir_reading), str(
                temp_reading), str(hr_reading), str(spot_reading)

            data = [temp_reading, hr_reading, spot_reading,
                    steps, latitude, longitude, speed]

            
            oled.fill(0)
            oled.text(f"Temp: {tempr}", 0, 0)
            oled.text(f"Heart Rate: {hrr}", 0, 16)
            oled.text(f"SPO2: {spotr}", 0, 32)
            oled.text(f"Steps: {steps}", 0, 48)
            oled.text(f"Latitude: {str(latitude)}", 0, 0)
            oled.text(f"Longitude: {str(longitude)}", 0, 16)
            oled.text(f"Speed: {speed} kmph", 0, 32)
            oled.show()

            # Code to publish data on Adafruit
            timer = Timer(0)
            timer.init(period=5000, mode=Timer.PERIODIC,
                       callback=sens_data(data))

            # Compute the real frequency at which we receive data
            if compute_frequency:
                if ticks_diff(ticks_us(), t_start) >= 999999:
                    f_HZ = samples_n
                    samples_n = 0
                    print("acquisition frequency = ", f_HZ)
                    t_start = ticks_us()
                else:
                    samples_n = samples_n + 1

if __name__ == '__main__':
    main()
