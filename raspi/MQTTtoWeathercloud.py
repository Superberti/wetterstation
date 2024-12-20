#!/usr/bin/python3
# -*- coding:utf-8 -*-
import re
from typing import NamedTuple

import paho.mqtt.client as mqtt
import time
import requests
import json
import sys

INFLUXDB_ADDRESS = 'localhost'

MQTT_ADDRESS = 'localhost'
MQTT_USER = 'Oliver'
MQTT_PASSWORD = 'Oliver'
MQTT_TOPIC = '/wetterstation/+'
MQTT_REGEX = '/wetterstation/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'MQTTtoWeathercloud'

TopicTemp1="/wetterstation/aussen/temperatur"
#TopicTemp2="/wetterstation/schuppen/temperatur"
TopicPress1="/wetterstation/aussen/luftdruck"
TopicHum1="/wetterstation/aussen/luftfeuchtigkeit"
#TopicHum2="/wetterstation/schuppen/luftfeuchtigkeit"
#TopicLight="/wetterstation/aussen/beleuchtungsstaerke"
#TopicStatus="/wetterstation/aussen/status"
#TopicLuefter="/wetterstation/aussen/luefterdrehzahl"
TopicTemp_gwhs="/wetterstation/gwhs/temperatur"
TopicHum_gwhs="/wetterstation/gwhs/luftfeuchtigkeit"
#TopicVBatt_gwhs="/wetterstation/gwhs/vbatt"

CloudCounter=0
Temperatur=0
Luftfeuchtigkeit=0
Luftdruck=0
Temp_gwhs=0
Luft_gwhs=0
StartTime = time.time()

class SensorData(NamedTuple):
    location: str
    measurement: str
    value: float

def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))
    client.subscribe(TopicTemp1)
    #client.subscribe(TopicTemp2)
    client.subscribe(TopicPress1)
    client.subscribe(TopicHum1)
    #client.subscribe(TopicHum2)
    #client.subscribe(TopicLight)
    #client.subscribe(TopicStatus)
    #client.subscribe(TopicLuefter)
    client.subscribe(TopicTemp_gwhs)
    client.subscribe(TopicHum_gwhs)
    
def _parse_mqtt_message(topic, payload):
    match = re.match(MQTT_REGEX, topic)
    if match:
        location = match.group(1)
        measurement = match.group(2)
        if measurement == 'status':
            return None
        return SensorData(location, measurement, float(payload))
    else:
        print('Kein match:'+topic)
        return None
 

def on_message(client, userdata, msg):
    
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ':' + str(msg.payload))
    try:
        sensor_data = _parse_mqtt_message(msg.topic, msg.payload.decode('utf-8'))
        if sensor_data is not None:
            #print("SD.location:"+sensor_data.location)
            #print("SD.measurement:"+sensor_data.measurement)
            #print("SD.value:"+str(sensor_data.value))
            _send_sensor_data_to_weathercloud(sensor_data)
                    
    except Exception as e:
        print("Error", e.__class__, "occurred: ",e, file = sys.stderr)
    
def _send_sensor_data_to_weathercloud(sensor_data):
    global Temperatur
    global Luftdruck
    global Luftfeuchtigkeit
    global CloudCounter
    global Temp_gwhs
    global Luft_gwhs
    global StartTime
    # compose url for uplad

#http://api.weathercloud.net/set/wid/xxxxxxx/key/xxxxxxxxxxxxx/temp/210/tempin/233/chill/214/heat/247/hum/33/humin/29/wspd/30/wspdhi/30/wspdavg/21/wdir/271/wdiravg/256/bar/10175/rain/0/solarrad/1630/uvi/ 5/ver/1.2/type/201
#http://api.weathercloud.net/get/wid/xxxxxxxx/key/xxxxxxxxxxx/temp/48/tempin/233/dewin/110/chill/0/heat/48/dew/15/hum/79/humin/46/wspd/5/wspdhi/10/wspdavg/7/wdir/152/wdiravg/149/bar/10128/rain/0.0/solarrad/0/uvi/0/rainrate/0/thw/48/et/0/ver/1.7/type/201

    WCurl = "http://api.weathercloud.net/set/wid"
    WC_Id = "/33b4e95835c83043" #personal ID
    WC_key = "/8413c4dfa4dd2b58f123529ec781d17e" #personal key
    last = "/ver/1.2/type/201"
    # Gleitender Mittelwert wird mit 0.9*alt+0.1*neu berechnet
    SmoothFactor=0.1;
    
    if sensor_data.location == "aussen" and sensor_data.measurement == "temperatur":
        if Temperatur==0:
            Temperatur=sensor_data.value
        else:
            Temperatur=(1-SmoothFactor)*Temperatur+SmoothFactor*sensor_data.value
            
    if sensor_data.location == "aussen" and sensor_data.measurement == "luftfeuchtigkeit":
        if Luftfeuchtigkeit==0:
            Luftfeuchtigkeit=sensor_data.value
        else:
            Luftfeuchtigkeit=(1-SmoothFactor)*Luftfeuchtigkeit+SmoothFactor*sensor_data.value
            
    if sensor_data.location == "gwhs" and sensor_data.measurement == "temperatur":
        if Temp_gwhs==0:
            Temp_gwhs=sensor_data.value
        else:
            Temp_gwhs=(1-SmoothFactor)*Temp_gwhs+SmoothFactor*sensor_data.value
            
    if sensor_data.location == "gwhs" and sensor_data.measurement == "luftfeuchtigkeit":
        if Luft_gwhs==0:
            Luft_gwhs=sensor_data.value
        else:
            Luft_gwhs=(1-SmoothFactor)*Luft_gwhs+SmoothFactor*sensor_data.value
        
    if sensor_data.location == "aussen" and sensor_data.measurement == "luftdruck":
        if Luftdruck==0:
            Luftdruck=sensor_data.value
        else:
            Luftdruck=(1-SmoothFactor)*Luftdruck+SmoothFactor*sensor_data.value
            
        # Luftdruck wird immer zuletzt gesendet. Dann sind alle Werte aktuell
        TimePast=time.time()-StartTime
        if TimePast > 600: #nur alle 600s senden, mehr mag weathercloud nicht
            StartTime=time.time()
            req_line=(WCurl + WC_Id + "/key" + WC_key + "/temp/" + str(int(Temperatur * 10+0.5)) +
                         "/bar/" + str(int(Luftdruck*10+0.5)) +
                         "/hum/" + str(int(Luftfeuchtigkeit+0.5)) +
                         "/tempin/" + str(int(Temp_gwhs * 10+0.5)) +
                         "/humin/" + str(int(Luft_gwhs+0.5)) + last)
            print("Sending to weathercloud:"+req_line)
            p = requests.get(req_line)        
            print("Received " + str(p.status_code) + " " + str(p.text))
        else:
            print(f'Noch {600-int(TimePast)} s bis zum Senden an weathercloud...')
            
        CloudCounter+=1  
        
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
