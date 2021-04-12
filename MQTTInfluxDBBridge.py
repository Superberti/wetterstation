#!/usr/bin/python3
# -*- coding:utf-8 -*-
import re
from typing import NamedTuple

import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient
import time
import logging

INFLUXDB_ADDRESS = 'localhost'
INFLUXDB_USER = 'mqtt'
INFLUXDB_PASSWORD = 'mqtt'
INFLUXDB_DATABASE = 'wetterstation'

MQTT_ADDRESS = 'localhost'
MQTT_USER = 'Oliver'
MQTT_PASSWORD = 'Oliver'
MQTT_TOPIC = '/wetterstation/+'
MQTT_REGEX = '/wetterstation/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'MQTTInfluxDBBridge'

TopicTemp1="/wetterstation/aussen/temperatur"
TopicTemp2="/wetterstation/aussen/temperatur_top"
TopicPress1="/wetterstation/aussen/luftdruck"
TopicHum1="/wetterstation/aussen/luftfeuchtigkeit"
TopicHum2="/wetterstation/aussen/luftfeuchtigkeit_top"
TopicLight="/wetterstation/aussen/beleuchtungsstaerke"
TopicStatus="/wetterstation/aussen/status"
TopicLuefter="/wetterstation/aussen/luefterdrehzahl"
TopicError="/wetterstation/error/status"

TopicCounter=0

influxdb_client = InfluxDBClient(INFLUXDB_ADDRESS, 8086, INFLUXDB_USER, INFLUXDB_PASSWORD, None)

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
    client.subscribe(TopicError)
 
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

def _send_sensor_data_to_influxdb(sensor_data):
    json_body = [
        {
            'measurement': sensor_data.measurement,
            'tags': {
                'location': sensor_data.location
            },
            'fields': {
                'value': sensor_data.value
            }
        }
    ]
    print('JSON: '+str(json_body))
    influxdb_client.write_points(json_body)

def _send_error_log_to_influxdb(error_log):
    json_body = [
        {
            'errorlog': error_log
        }
    ]
    print('JSON: '+str(json_body))
    influxdb_client.write_points(json_body)

def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ' ' + str(msg.payload))
    try:
        sensor_data = _parse_mqtt_message(msg.topic, msg.payload.decode('utf-8'))
        if sensor_data is not None:
            _send_sensor_data_to_influxdb(sensor_data)
        else:
            if msg.topic is TopicError:
                status=msg.payload.decode('utf-8')
                _send_error_log_to_influxdb(status)
                    
    except Exception as e:
        logging.info("Fehler", e.__class__, "occurred: ",e)
        
    

def _init_influxdb_database():
    databases = influxdb_client.get_list_database()
    if len(list(filter(lambda x: x['name'] == INFLUXDB_DATABASE, databases))) == 0:
        influxdb_client.create_database(INFLUXDB_DATABASE)
    influxdb_client.switch_database(INFLUXDB_DATABASE)

def main():
    print("Warte 60 s auf den Start der Datenbank...")
    time.sleep(60)
    _init_influxdb_database()

    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    #mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()


if __name__ == '__main__':
    print('MQTT to InfluxDB bridge')
    main()
