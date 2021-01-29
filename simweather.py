#!/usr/bin/python3
# -*- coding:utf-8 -*-

import sys
import os
import time
import math
import random
import paho.mqtt.client as mqtt
import threading

# MAIN
def main():
    mqtt_lock = threading.Lock()
    TopicTemp1="/wetterstation/aussen/temperatur1"
    TopicPress1="/wetterstation/aussen/luftdruck1"
    TopicHum1="/wetterstation/aussen/luftfeuchtigkeit1"
    TopicLight="/wetterstation/aussen/beleuchtungsstaerke"
    step=0.0
    client = mqtt.Client()
    client.connect("localhost", 1883, 10)
    
    while(1==1):
        temp1=23.14+math.sin(step)*10.0
        temp2=14.14+math.sin(step+1.34)*7.0
        temp3=16.8+math.sin(2*step-0.34)*5.0
        temp4=10.09+math.sin(step)*math.cos(step)*13.0
        hum1=66.5+10*math.sin(2.5*step)
        hum2=23.7+20*math.sin(2.5*step)*math.cos(step*step)
        hum3=85.6+14*math.cos(1.5*step)
        hum4=99.2-15*math.fabs(math.sin(2.5*step))
        press1=1000+math.sin(step*3)*40;
        windspeed=35.6+10*step
        winddir=(1+math.sin(step))*180
        rain=5+random.randrange(100, 1000, 3)
        brightness=40000-math.sin(step)*10000
        client.publish(TopicTemp1,str(temp1))
        client.publish(TopicPress1,str(press1))
        client.publish(TopicHum1,str(hum1))
        client.publish(TopicLight,str(brightness))
        print("Schreibe: "+str(temp1)+" "+str(press1)+" "+str(hum1)+" "+str(brightness))
        time.sleep(5)
        step+=math.pi/50.0

if __name__ == '__main__':
    main()
