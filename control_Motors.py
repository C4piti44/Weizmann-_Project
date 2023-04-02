import RPi.GPIO as GPIO          
import time

# Right Motor
in1 = 5
in2 = 6
en_a = 25

# Left motor
in3 =12
in4 = 13
en_b = 4
 
    

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en_b,GPIO.OUT)

q=GPIO.PWM(en_a,100)
p=GPIO.PWM(en_b,100)

p.start(95)
q.start(95)

GPIO.output(in2,GPIO.HIGH)
GPIO.output(in3,GPIO.HIGH)
time.sleep(3)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)




GPIO.output(in1,GPIO.HIGH)
GPIO.output(in4,GPIO.HIGH)
time.sleep(3)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)




p.start(45)
q.start(45)
#turn right

GPIO.output(in1, GPIO.LOW)
GPIO.output(in3, GPIO.HIGH)
time.sleep(2)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)

# turn Left
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in3, GPIO.LOW)
time.sleep(2)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)

