import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

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
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in1, GPIO.HIGH)


# Turns the robot left
def turn_left():
    GPIO.output(in4, GPIO.HIGH)
    GPIO.output(in1, GPIO.LOW)


# Drive forward
def move_forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in4, GPIO.HIGH)


# Drive backward
def move_backwards():
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)


# variables to calculate the linear function
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
    return int(speed)


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


def main():
    font = cv2.FONT_HERSHEY_SIMPLEX
    lower = np.array([87, 80, 20])
    upper = np.array([105, 255, 255])

    video = cv2.VideoCapture(0)

    while True:
        _, img = video.read()
        image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, lower, upper)
        # cv2.line(img, (500, 0), (500, 720), (255, 0, 0), 2)
        # cv2.line(img, (780, 0), (780, 720), (255, 0, 0), 2)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            for contour in contours:
                area = cv2.contourArea(contour)  # the area of the object
                if area > 300:
                    #finding_shape(img, contour) -> it does nothing, i have no usage in it anymore
                    #object_position(img, contour) -> i don't have to open a window, it takes computation power.
                    coords = get_coords(contour)
                    center_p = center_point(contour)
                    if (img.shape[1]) / 2 - 180 > center_p[0]:
                        q.start(80)
                        p.start(80)
                        turn_left()
                    elif (img.shape[1]) / 2 + 180 < center_p[0]:
                        q.start(80)
                        p.start(80)
                        turn_right()
                    else:
                        velocity_of_the_robot = velocity(area)
                        p.start(velocity_of_the_robot)
                        q.start(velocity_of_the_robot)
                        move_forward()
                        
        cv2.imshow("mask", mask)
        cv2.imshow("webcam", img)

        cv2.waitKey(1)


if __name__ == '__main__':
    main()
