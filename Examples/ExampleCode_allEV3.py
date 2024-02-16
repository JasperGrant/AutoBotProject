#!/usr/bin/env python3

# Importing motor functions and output ports
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D

# Importing sensor functions and input ports
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor, TouchSensor

# Importing the button function to allow for easy turn off
from ev3dev2.button import Button

# Initializing variables
btn = Button()
color_sensor = ColorSensor(address = INPUT_1)
gyro_sensor = GyroSensor(address = INPUT_2)
ultrasonic_sensor = UltrasonicSensor(address = INPUT_3)
touch_sensor = TouchSensor(address = INPUT_4)

# Initializing robot so it can move. This uses "MoveTank" since we are using two motors for the wheels. However, you can activate a motor and use it
# for something other than movement. Check the API for the functions to do this. 
robot = MoveTank(OUTPUT_A, OUTPUT_D)

# This is sample code for the EV3 motors and sensors to run until a button on the EV3 is pressed
while not btn.any(): #exit loop upon button press
    
    #Color Sensor
    color_sensor.MODE_COL_REFLECT
    print("reflected light intensity: ", color_sensor.reflected_light_intensity) #Outputs number to the screen of the EV3

    #Gyroscope Sensor
    gyro_sensor.MODE_GYRO_G_A
    print("gyro angle and rate: ", gyro_sensor.angle_and_rate)

    #Ultrasonic Sensor
    ultrasonic_sensor.MODE_US_DIST_CM
    print("ultrasonic reading(cm): ", ultrasonic_sensor.distance_centimeters)

    #Touch Sensor
    touch_sensor.MODE_TOUCH
    print("Is touch sensor pressed(1 = yes, 0=no)?: ", touch_sensor.is_pressed)

    robot.on(left_speed=10, right_speed=10)
    