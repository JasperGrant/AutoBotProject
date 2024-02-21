#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import InfraredSensor
from ev3dev2.button import Button

btn = Button()
infrared_sensor = InfraredSensor(address = INPUT_4)

#This is sample code for the NXT light sensor
while not btn.any(): #exit loop upon button press
    infrared_sensor.MODE_IR_PROX
    infrared_sensor_reading = infrared_sensor.proximity
    
    ''' Outputs the proximity reading from 0% to 100% where 100% is approximately 70cm.
    If the objecting being detected is dark it will give a a reading closer to 100% (farther away) because there is less reflection, if there is a bright 
    white object it will have more light reflection and be give a reading closer to 0% (nearer to the sensor).
    '''
    print("infrared proximity reading: ", infrared_sensor_reading) #Outputs number to the screen of the EV3
