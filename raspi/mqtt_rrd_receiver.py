#!/usr/bin/python3 -u

import sys
import os
import time
import rrdtool
import math
import random
import paho.mqtt.client as mqtt
import datetime

TopicTemp="/wetterstation/temperatur"
TopicPress="/wetterstation/luftdruck"
TopicHum="/wetterstation/luftfeuchtigkeit"
TopicLight="/wetterstation/beleuchtungsstaerke"
TopicCounter=0
temp1=0
temp2=0
temp3=0
temp4=0
hum1=0
hum2=0
hum3=0
hum4=0
press1=0
press2=0
windspeed=0
winddir=0
rain=0
brightness=0

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(TopicTemp)
    client.subscribe(TopicPress)
    client.subscribe(TopicHum)
    client.subscribe(TopicLight)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # print(msg.topic+" "+str(msg.payload))
    global TopicCounter
    global temp1
    global temp2
    global temp3
    global temp4
    global hum1
    global hum2
    global hum3
    global hum4
    global press1
    global press2
    global windspeed
    global winddir
    global rain
    global brightness
    
    if (msg.topic==TopicTemp):
        temp1=float(msg.payload)
        TopicCounter=TopicCounter+1
    elif (msg.topic==TopicPress):
        press1=float(msg.payload)
        TopicCounter=TopicCounter+1
    elif (msg.topic==TopicHum):
        hum1=float(msg.payload)
        TopicCounter=TopicCounter+1
    elif (msg.topic==TopicLight):
        brightness=float(msg.payload)
        TopicCounter=TopicCounter+1
        
    if (TopicCounter==4):
        # print("Schreibe Zeile in die RRD-Datenbank:")
        TopicCounter=0
        update=('N:'+str(temp1)+':'+str(temp2)+':'+str(temp3)+':'+str(temp4)+':'+
            str(hum1)+':'+str(hum2)+':'+str(hum3)+':'+str(hum4)+':'+
            str(press1)+':'+str(press2)+':'+
            str(windspeed)+':'+str(winddir)+':'+str(rain)+':'+str(brightness))
        # print(update)
        
        # insert data into database
        try:
            rrdtool.update(
            "%s/weather.rrd" % (os.path.dirname(os.path.abspath(__file__))),
            update)
        except Exception as e:
            print("Fehler", e.__class__, "occurred: ",sys.exc_info()[0])

now = datetime.datetime.now()
print("MQTT->RRD receiver gestartet: " + now.strftime('%Y-%m-%d %H:%M:%S'))
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("raspiwetter", 1883, 10)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
