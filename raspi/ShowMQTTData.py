#!/usr/bin/python3
# -*- coding:utf-8 -*-
import re
from typing import NamedTuple

import paho.mqtt.client as mqtt
import time

INFLUXDB_ADDRESS = 'localhost'

MQTT_ADDRESS = 'localhost'
MQTT_USER = 'Oliver'
MQTT_PASSWORD = 'Oliver'
MQTT_TOPIC = '/wetterstation/+'
MQTT_REGEX = '/wetterstation/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'ShowMQTTData'

TopicTemp1="/wetterstation/aussen/temperatur"
TopicTemp2="/wetterstation/schuppen/temperatur"
TopicPress1="/wetterstation/aussen/luftdruck"
TopicHum1="/wetterstation/aussen/luftfeuchtigkeit"
TopicHum2="/wetterstation/schuppen/luftfeuchtigkeit"
TopicLight="/wetterstation/aussen/beleuchtungsstaerke"
TopicStatus="/wetterstation/aussen/status"
TopicLuefter="/wetterstation/aussen/luefterdrehzahl"

TopicCounter=0

class SensorData(NamedTuple):
    location: str
    measurement: str
    value: float

def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))
    client.subscribe(TopicTemp1)
    client.subscribe(TopicTemp2)
    client.subscribe(TopicPress1)
    client.subscribe(TopicHum1)
    client.subscribe(TopicHum2)
    client.subscribe(TopicLight)
    client.subscribe(TopicStatus)
    client.subscribe(TopicLuefter)
 

def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ':' + str(msg.payload))

def main():
    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    #mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()


if __name__ == '__main__':
    print('ShowMQTTData')
    main()
