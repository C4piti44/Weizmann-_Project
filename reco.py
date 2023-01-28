import cv2
import numpy as np
import RPi.GPIO as GPIO

# setup the pins on the raspberry pi
# Right Motor in1 is for forward and in2 is backward '''might not be true'''
in1 = 5
in2 = 6
en_a = 25

# Left Motor in3 is for forward and in4 is backward '''might not be true'''
in3 = 12
in4 = 13
en_b = 4

GPIO.setmode(GPIO.BCM)  # addressing the pins with their gpio number and not the physical pun number.
GPIO.setwarnings(False)  # normally the pins send notifications, False blocks it.

# setting up the pins
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en_a, GPIO.OUT)

GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en_b, GPIO.OUT)

q = GPIO.PWM(en_a, 90)  # determine the frequency which we gonna send the pulses to the raspberry
p = GPIO.PWM(en_b, 90)  # determine the frequency which we gonna send the pulses to the raspberry


# Turns the robot left
def turn_right():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)


# Turns the robot left
def turn_left():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)


# Drive forward
def move_forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)


# Drive backward
def move_backwards():
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in4, GPIO.HIGH)


# variables that help to calculate the linear function
min_area = 2500
max_area = 130000
max_velocity = 95
min_velocity = 0


# calculating the velocity accordingly to the area of the object
def velocity(object_area):
    if object_area < min_area:
        object_area = min_area
    elif object_area > max_area:
        object_area = max_area

    # calculate speed using inverted linear mapping
    speed = max_velocity - (object_area - min_area) * (max_velocity - min_velocity) / (max_area - min_area)
    return speed


# This function detect the object's vertices.
# If the the number of vertices is suitable to the predefine number of vertices,
# the function will highlight the contours.
def finding_shape(img, contour):
    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
    if 5 > len(approx) > 2:
        cv2.drawContours(img, [approx], 0, (255, 0, 0), 4)
        cv2.putText(img, "object has been detected", (50, 50), font, 1, (255, 255, 255))


# returning the center point of the object
def center_point(contour):
    return [cv2.boundingRect(contour)[0] + cv2.boundingRect(contour)[2] / 2,
            cv2.boundingRect(contour)[1] + cv2.boundingRect(contour)[3] / 2]


def get_coords(contour):
    return cv2.boundingRect(contour)


# This function detect the object in the space.
# The function draw a rectangle on the detect object.

def object_position(img, contour):
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255))
    cv2.putText(img, 'follow', (x - 60, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)


font = cv2.FONT_HERSHEY_SIMPLEX
lower = np.array([30, 100, 20])
upper = np.array([83, 255, 255])

video = cv2.VideoCapture(0)

while True:
    _, img = video.read()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower, upper)

    cv2.line(img, (500, 0), (500, 720), (255, 0, 0), 2)
    cv2.line(img, (780, 0), (780, 720), (255, 0, 0), 2)
    # cv2.line(img, (650, 0), (650, 720), (255, 0, 0), 2)
    # cv2.line(img, ((img.shape[1]) / 2 + 30, 0), ((img.shape[1]) / 2 + 30, img.shape[0]), (255, 0, 0), 2)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        for contour in contours:
            area = cv2.contourArea(contour)  # the area of the object
            if area > 300:
                finding_shape(img, contour)
                object_position(img, contour)
                coords = get_coords(contour)
                center_p = center_point(contour)
                velocity_of_the_robot = velocity(area)
                p.start(velocity_of_the_robot)
                q.start(velocity_of_the_robot)
                if (img.shape[1]) / 2 - 140 > center_p[0]:
                    q.start(45)
                    p.start(45)
                    turn_right()
                if (img.shape[1]) / 2 + 140 < center_p[0]:
                    q.start(45)
                    p.start(45)
                    turn_left()

    cv2.imshow("mask", mask)
    cv2.imshow("webcam", img)

    cv2.waitKey(1)
