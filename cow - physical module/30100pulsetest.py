import max30100
import time

mx30 = max30100.MAX30100()
while True:
    mx30.refresh_temperature()
    temp = mx30.get_temperature()

    mx30.read_sensor()

    print("Temperature: {:.2f} C".format(temp))  # Formatted temperature output
    print("IR: {}".format(mx30.ir))
    print("Red: {}".format(mx30.red))
    print("-----------------")  # Optional separator between readings

    time.sleep(1)  # Adjust delay as needed (1 second in this example) 

