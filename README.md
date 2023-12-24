# cow
physical extension to gocattle. uses sensors to determine data and predict diseases in cows.
3 computer vision models to determine if the cow has IBK, FMD or LSD

also has the code to the infamous cow project showcased at CRSE 2023-24 at DPS Ranchi.
The Cow Project had the following components:
- Max30102 - Tracks Heart Rate and Oxygen Saturation, the current algorithm to determine heart rate and SpO2 is not reliable, but Tracks RED and IR refractions accurately. uses photoreceptors for that.
- MPU6050 - xyz accelerometer and a gyroscope to measure steps walked by a cow and rumination 
percentage, can be used to calculate heat cycle. works on the principle of corealis acceleration.
- Neo 6M GPS - works half of the time (:P) but determines accurate location and windspeed through 24 different satellites.

how to use/remake:
0. components required: pico w, usb cable, all the sensors listed above, jumper wires, thonny ide, a computer hopefully and will to live
1. connect neo 6m tx to GP5
2. connect max30102 sda to GP16 and scl to GP17
3. connect oled 0.96 ssd1306 sda to GP18 and scl to GP19 (we use i2c bus 1)
4. connect mpu 6050 sda to GP20 and scl to GP21
5. install micropython (not the beta versions please, a stable, proper version) to your pico w, by pressing the bootset button
6. open thonny ide, go to tools, go to options, then go to interpreter, and select raspberry pi pico as interpreter, alternative go with RP2040 if you're using a clone 🤡
7. run the main.py file, voila! put your finger there and you see a totally accurate, not definitely faked reading through the max30102 on your oled.

were you counting? because I was. to initialize cow, you require only 7 steps. thala for a reason.


~~ Sarthak Sidhant

special thanks to Amogh

