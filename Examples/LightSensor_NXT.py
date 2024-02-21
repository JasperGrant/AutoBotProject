#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import LightSensor
from ev3dev2.button import Button

btn = Button()
light_sensor = LightSensor(address = INPUT_1)

#This is sample code for the NXT light sensor
while not btn.any(): #exit loop upon button press
    light_sensor.MODE_REFLECT
    light_sensor.MODe
    light_sen_reading = light_sensor.reflected_light_intensity
    print(light_sen_reading) #Outputs number to the screen of the EV3
