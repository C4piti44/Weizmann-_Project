#https://www.collvy.com/blog/controlling-dc-motor-with-raspberry-pi
import rpi.gpio as io
import time

io.setwarnings(False)

io.setmode(io.BOARD)

io.setup(29, io.OUT) #purple
io.setup(31, io.OUT) #grey
io.setup(33, io.OUT) #brown
io.setup(32, io.OUT) #orange

pin_29 = io.PWM(29, 100)
pin_31 = io.PWM(31, 100)
pin_33 = io.PWM(33, 100)
pin_32 = io.PWM(32, 100)

pin_29.start(0)
pin_31.start(0)
pin_33.start(0)
pin_32.start(0)

pin_29.ChangedutyCycle(100)
time.sleep(3)
pin_29.stop()

pin_31.ChangedutyCycle(100)
time.sleep(3)
pin_31.stop()

pin_32.ChangedutyCycle(100)
time.sleep(3)
pin_32.stop()

pin_33.ChangedutyCycle(100)
time.sleep(3)
pin_33.stop()
