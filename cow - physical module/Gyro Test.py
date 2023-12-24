from machine import Pin, I2C
import utime
from mpu6050 import init_mpu6050, get_mpu6050_data
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
init_mpu6050(i2c)

# OLED setup
WIDTH = 128
HEIGHT = 64
i2c_oled = I2C(1, scl=Pin(19), sda=Pin(18), freq=200000)
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c_oled)

# Variables for step counting and rumination calculation
steps = 0
last_step_time = utime.ticks_ms()  # Record last step time for cadence calculation
rumination_count = 0
rumination_threshold = 100  # Adjust this based on calibration

while True:
    data = get_mpu6050_data(i2c)
    accel_x = data['accel']['x']
    accel_y = data['accel']['y']
    accel_z = data['accel']['z']
    gyro_x = data['gyro']['x']
    gyro_y = data['gyro']['y']
    gyro_z = data['gyro']['z']

    # Step counting logic
    total_acceleration = abs(accel_x) + abs(accel_y) + abs(accel_z)  # Calculate total acceleration
    if total_acceleration > 1.5:  # Adjust threshold based on calibration
        current_time = utime.ticks_ms()
        if current_time - last_step_time > 200:  # Ignore steps too close together
            steps += 1
            last_step_time = current_time

    # Rumination calculation (simplified example, adjust based on research)
    if abs(gyro_x) + abs(gyro_y) + abs(gyro_z) > rumination_threshold:
        rumination_count += 1

    # Display results
    oled.fill(0)
    oled.text(f"Steps: {steps}", 0, 0)
    oled.text(f"Rumination: {rumination_count}", 0, 16)
    oled.show()

    utime.sleep(0.5)
