import max30100 
import time
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

# OLED setup (Adjust your I2C pins if needed)
i2c_oled = I2C(1, scl=Pin(19), sda=Pin(18), freq=200000)
oled = SSD1306_I2C(128, 64, i2c_oled)

# MAX30100 sensor setup
mx30 = max30100.MAX30100() 

# Main loop
while True:
    mx30.refresh_temperature() 
    temp = mx30.get_temperature() 

    mx30.read_sensor() 

    # Display values on OLED
    oled.fill(0)  # Clear screen
    oled.text("COW Params", 0, 0) 
    oled.text(f"Temp: {temp:.2f} C", 0, 16)
    oled.text(f"IR: {mx30.ir}", 0, 32)
    oled.text(f"Red: {mx30.red}", 0, 48)
    oled.show()  
