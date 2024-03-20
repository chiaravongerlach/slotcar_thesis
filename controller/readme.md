# Slot car controller and interfacing
This is the folder to store source code for the Arduino code for the slot car controller, as well as the accompanying test scripts for Python side. The Python code and the controller communicates with each other using serial communication. We need to send a single data point from the Python script to the Arduino, which is the angle position ($[0, 180]$). To ensure that we are only processing the correct data, the sent package is a tuple of 3 numbers, in the form of `(start, angle, stop)`. The `start` and `stop` values are predetermined by both sides and are used to check and make sure that the receiving package is in the correct format and correct data is received.

We are using unsigned byte in our package , so each value is within the range $[0, 255]$. Refer to [struct format](https://docs.python.org/3/library/struct.html#struct-format-strings) to see more information about how we package the array before sending to serial.

Python side:
```python
# send
ser.write(pack ('3B', 255, data, 255)) # send the value `data` to the Arduino, with start=127, stop=127

# receive
dat = ser.readline()
```
