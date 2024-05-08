import max30100
mx30 = max30100.MAX30100()
mx30.read_sensor()

# The latest values are now available via .ir and .red
print(mx30.ir, mx30.red)
