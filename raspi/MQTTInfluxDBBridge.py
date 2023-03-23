#!/usr/bin/python3
# -*- coding:utf-8 -*-
import re
from typing import NamedTuple

import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient
import time
import logging
import urllib3
import argparse

urllib3.disable_warnings()
logging.basicConfig(format='%(asctime)s %(message)s', filename='/var/log/MQTTInfluxBridge.log', encoding='utf-8', level=logging.INFO)
#logging.basicConfig(format='%(asctime)s %(message)s', encoding='utf-8', level=logging.INFO)
INFLUXDB_ADDRESS = 'localhost'
INFLUXDB_USER = 'mqtt'
INFLUXDB_PASSWORD = 'mqtt'
INFLUXDB_DATABASE = 'wetterstation'

MQTT_ADDRESS = 'localhost'
MQTT_WETTER_REGEX = '/wetterstation/([^/]+)/([^/]+)'
MQTT_DEYE_REGEX = 'deye/([^/]+)'
MQTT_CLIENT_ID = 'MQTTInfluxDBBridge'

TopicCounter=0

influxdb_client = InfluxDBClient(INFLUXDB_ADDRESS, 8086, INFLUXDB_USER, INFLUXDB_PASSWORD, None)
# Virtueller Rechner im Netz
NeelixClient = None

class SensorData(NamedTuple):
    location: str
    measurement: str
    value: float

def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))
    
    # Fahrradschuppen
    client.subscribe("/wetterstation/aussen/temperatur")
    client.subscribe("/wetterstation/aussen/luftdruck")
    client.subscribe("/wetterstation/aussen/luftfeuchtigkeit")
    
    # Gewächshaus
    client.subscribe("/wetterstation/gwhs/temperatur")
    client.subscribe("/wetterstation/gwhs/luftfeuchtigkeit")
    client.subscribe("/wetterstation/gwhs/vbatt")
    
    # Balkonkraftwerk
    client.subscribe("deye/logger_status")
    client.subscribe("deye/day_energy")
    client.subscribe("deye/total_energy")
    client.subscribe("deye/ac/l1/power")
 
def _parse_mqtt_message(topic, payload):
    match = re.match(MQTT_WETTER_REGEX, topic)
    if match:
        location = match.group(1)
        measurement = match.group(2)
        if measurement == 'status':
            return None
        if payload == '':
            return None
        return SensorData(location, measurement, float(payload))
    
    match = re.match(MQTT_DEYE_REGEX, topic)
    if match:
        location = "deye"
        measurement = match.group(1)
        if measurement == 'ac':
            measurement = 'power'
            
        if measurement == 'logger_status':
            if payload == 'online':
                return SensorData(location, measurement, 0)
            else:
                return SensorData(location, measurement, 1)
        if payload == '':
            return None
        return SensorData(location, measurement, float(payload))
    return None

def _send_sensor_data_to_influxdb(sensor_data):
    global NeelixClient
    try:
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
        NeelixClient.write_points(json_body)
    except Exception as e:
        logging.info("Fehler", e.__class__, "occurred: ",e)

def _send_error_log_to_influxdb(error_log):
    global NeelixClient
    json_body = [
        {
            'errorlog': error_log
        }
    ]
    print('JSON: '+str(json_body))
    influxdb_client.write_points(json_body)
    NeelixCient.write_points(json_body)
    
def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ' ' + str(msg.payload))
  
    sensor_data = _parse_mqtt_message(msg.topic, msg.payload.decode('utf-8'))
    if sensor_data is not None:
        _send_sensor_data_to_influxdb(sensor_data)
    else:
        if msg.topic is TopicError:
            status=msg.payload.decode('utf-8')
            _send_error_log_to_influxdb(status)     
    
def _init_influxdb_database(password, user):
    global NeelixClient
    NeelixClient = InfluxDBClient(host='neelix.ddnss.de', port=8086, username=user, password=password, ssl=True, verify_ssl=False)
    databases = influxdb_client.get_list_database()
    print(databases)
    if len(list(filter(lambda x: x['name'] == INFLUXDB_DATABASE, databases))) == 0:
        influxdb_client.create_database(INFLUXDB_DATABASE)
    influxdb_client.switch_database(INFLUXDB_DATABASE)
    
    databases = NeelixClient.get_list_database()
    print(databases)
    if len(list(filter(lambda x: x['name'] == INFLUXDB_DATABASE, databases))) == 0:
        NeelixClient.create_database(INFLUXDB_DATABASE)
    NeelixClient.switch_database(INFLUXDB_DATABASE)
    
def main(password='test', user='test'):
    try:
        print("Warte 20 s auf den Start der Datenbank...")
        time.sleep(20)
        _init_influxdb_database(password, user)

        mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
        #mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message

        mqtt_client.connect(MQTT_ADDRESS, 1883)
        mqtt_client.loop_forever()
    except Exception as e:
        logging.info("Fehler", e.__class__, "occurred: ",e)    

def parse_args():
    """Parse the args from main."""
    parser = argparse.ArgumentParser(
        description='MQTTToInfluxDB gateway')
    parser.add_argument('--password', type=str, required=False,
                        default='test',
                        help='Password for InfluxDB.')
    parser.add_argument('--user', type=str, required=False,
                        default='test',
                        help='InfluxDB username')
    return parser.parse_args()


if __name__ == '__main__':
    print('MQTT to InfluxDB bridge')
    args = parse_args()
    main(password=args.password, user=args.user)
