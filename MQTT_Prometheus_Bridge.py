#!/usr/bin/python3
# -*- coding:utf-8 -*-
import re
from typing import NamedTuple

import paho.mqtt.client as mqtt
import time
from prometheus_client import start_http_server, Summary, Gauge
import logging

INFLUXDB_ADDRESS = 'localhost'

MQTT_ADDRESS = 'raspiwetter'
MQTT_USER = 'Oliver'
MQTT_PASSWORD = 'Oliver'
MQTT_TOPIC = '/wetterstation/+'
MQTT_REGEX = '/wetterstation/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'MQTT_Prometheus_Bridge'

TopicTemp1="/wetterstation/aussen/temperatur"
TopicTemp2="/wetterstation/schuppen/temperatur"
TopicPress1="/wetterstation/aussen/luftdruck"
TopicHum1="/wetterstation/aussen/luftfeuchtigkeit"
TopicHum2="/wetterstation/schuppen/luftfeuchtigkeit"
TopicLight="/wetterstation/aussen/beleuchtungsstaerke"
TopicStatus="/wetterstation/aussen/status"
TopicLuefter="/wetterstation/aussen/luefterdrehzahl"

TopicCounter=0
temp1=0
temp2=0
hum1=0
hum2=0
press1=0
brightness=0
cooler=0
status=""

prom_temp = Gauge("Temperatur", "Temperatur der Wetterstation", ['ort','sensorart'])
prom_hum = Gauge("Luftfeuchtigkeit", "Luftfeuchtigkeit der Wetterstation", ['ort','sensorart'])
prom_press = Gauge("Luftdruck", "Luftdruck der Wetterstation", ['ort','sensorart'])
prom_bright = Gauge("Beleuchtungsstaerke", "Beleuchtungsstaerke der Wetterstation", ['ort','sensorart'])
prom_cool = Gauge("Luefterdrehzal", "Drehzahl des Lüfters", ['ort','sensorart'])
prom_status = Gauge("Status", "Status der Wetterstation", ['ort','sensorart'])

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
    global temp1
    global temp2
    global hum1
    global hum2
    global press1
    global brightness
    global cooler
    global status
    print(msg.topic + ':' + str(msg.payload))
    try:
        if (msg.topic==TopicTemp1):
            temp1=float(msg.payload)
            prom_temp.labels(ort='Aussen', sensorart='SHT35').set(temp1)
        elif (msg.topic==TopicTemp2):
            temp2=float(msg.payload)
            prom_temp.labels(ort='Schuppen', sensorart='SHT35').set(temp2)
        elif (msg.topic==TopicPress1):
            press1=float(msg.payload)
            prom_press.labels(ort='Schuppen', sensorart='BMP280').set(press1)
        elif (msg.topic==TopicHum1):
            hum1=float(msg.payload)
            prom_hum.labels(ort='Aussen', sensorart='SHT35').set(hum1)
        elif (msg.topic==TopicHum2):
            hum2=float(msg.payload)
            prom_hum.labels(ort='Schuppen', sensorart='SHT35').set(hum2)
        elif (msg.topic==TopicLight):
            brightness=float(msg.payload)
            prom_bright.labels(ort='Aussen', sensorart='VEML7700').set(brightness)
        elif (msg.topic==TopicLuefter):
            cooler=float(msg.payload)
            prom_cool.labels(ort='Schuppen', sensorart='Luefter').set(cooler)
        elif (msg.topic==TopicStatus):
            status=msg.payload
            prom_status.labels(ort='Schuppen', sensorart='log').set(status)
    except Exception as e:
        logging.info("Fehler", e.__class__, "occurred: ",e)

def main():
    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    #mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    # Start up the server to expose the metrics.
    start_http_server(8000)
    mqtt_client.loop_forever()


if __name__ == '__main__':
    print('ShowMQTTData')
    main()


