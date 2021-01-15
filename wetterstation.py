#!/usr/bin/python3
# -*- coding:utf-8 -*-
import sys
import os
import time
import rrdtool
import math
import random
import subprocess
import datetime
from gpiozero import Button
import logging
import threading
import paho.mqtt.client as mqtt

# Globale Definitionen
DisplayWidth=264
DisplayHeight=176
GraphicsWidth=200

btn1=Button(5)
btn2=Button(6)
btn3=Button(13)
btn4=Button(19)
LastButton=1
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

picdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pic')
libdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib')
progdir = os.path.dirname(os.path.realpath(__file__))
db_fn = os.path.join(progdir, 'weather.rrd')
if os.path.exists(libdir):
    sys.path.append(libdir)
    
mqtt_lock = threading.Lock()

from waveshare_epd import epd2in7
from PIL import Image,ImageDraw,ImageFont,ImageOps
import traceback

epd = epd2in7.EPD()
ButtonEvent=threading.Event()


logging.basicConfig(level=logging.INFO)

def mqtt_receiver_thread
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
    global mqtt_lock
    
    with mqtt_lock:
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
        with mqtt_lock:
            try:
                rrdtool.update(
                "%s/weather.rrd" % (os.path.dirname(os.path.abspath(__file__))),
                update)
            except Exception as e:
                logging.info("Fehler", e.__class__, "occurred: ",e)

def ShowDiagram(DiaFileName, InfoText):
    global DisplayWidth
    global DisplayHeight
    global GraphicsWidth
    global LastButton
    global progdir
    global picdir
    global TempStr
    global HumStr
    global PressStr
    global BrightStr
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
    global epd
    
    print(InfoText)
    
    
    now = datetime.datetime.now()
    timetext = now.strftime('%H:%M')
    im_fn = os.path.join(progdir, DiaFileName)
    
    Himage = Image.open(im_fn)
    im_temp = Image.open(os.path.join(picdir, 'temperatur.png'))
    im_hum = Image.open(os.path.join(picdir, 'luftfeuchtigkeit.png'))
    im_press = Image.open(os.path.join(picdir, 'luftdruck.png'))
    im_bright = Image.open(os.path.join(picdir, 'helligkeit.png'))

    newImage = Image.new(Himage.mode, (DisplayWidth,DisplayHeight),(255, 255, 255))
    newImage.paste(Himage, (0,0))

    newImage.paste(im_temp, (GraphicsWidth,37))
    newImage.paste(im_press, (GraphicsWidth-5,62))
    newImage.paste(im_hum, (GraphicsWidth-5,87))
    newImage.paste(im_bright, (GraphicsWidth-5,112))

    draw = ImageDraw.Draw(newImage)
    textwidth = font24.getsize(timetext)[0]
    draw.text((DisplayWidth-2-textwidth, 0), timetext, font = font24, fill = (0, 0, 0))
    
    with mqtt_lock:
        text="{0:.1f}".format(temp1)
    textwidth = font18.getsize(text)[0]
    draw.text((DisplayWidth-2-textwidth, 40), text, font = font18, fill = (0, 0, 0))

    with mqtt_lock:
        text="{0:.0f}".format(press1)
    textwidth = font18.getsize(text)[0]
    draw.text((DisplayWidth-2-textwidth, 65), text, font = font18, fill = (0, 0, 0))

    with mqtt_lock:
        text="{0:.1f}".format(hum1)
    textwidth = font18.getsize(text)[0]
    draw.text((DisplayWidth-2-textwidth, 90), text, font = font18, fill = (0, 0, 0))

    with mqtt_lock:
        text="{0:.1f}".format(brightness)
    textwidth = font18.getsize(text)[0]
    draw.text((DisplayWidth-2-textwidth, 115), text, font = font18, fill = (0, 0, 0))
    
    im_mirror = ImageOps.mirror(newImage)
    
    epd.Init_4Gray()
    epd.display_4Gray(epd.getbuffer_4Gray(im_mirror))
    epd.sleep()

def handleBtn1Press():
    global LastButton
    global ButtonEvent 
    LastButton=1
    ButtonEvent.set()
    
def handleBtn2Press():
    global LastButton
    global ButtonEvent
    LastButton=2
    ButtonEvent.set()
    
def handleBtn3Press():
    global LastButton
    global ButtonEvent
    LastButton=3
    ButtonEvent.set()
    
def handleBtn4Press():
    global LastButton
    global ButtonEvent
    LastButton=4
    ButtonEvent.set()

try:
    btn1.when_pressed = handleBtn1Press
    btn2.when_pressed = handleBtn2Press
    btn3.when_pressed = handleBtn3Press
    btn4.when_pressed = handleBtn4Press
    logging.info("Show rrdtool last hour")    
    font24 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 24)
    font18 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 18)
    font35 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 35)
    
    mqtt_thread = threading.Thread(target=mqtt_receiver_thread, args=())
    mqtt_thread.start()
    while 1==1:
        now = datetime.datetime.now()
        s=now.second
        waittime=0
        if s>0:
            waittime=60-s
            print("Warte ",waittime," Sekunden bis zum nächsten Update")
            EventSet=ButtonEvent.wait(timeout=waittime)
            if EventSet==True:
                ButtonEvent.clear()
                print("Knopf ",LastButton," gedrückt")
                
        print("Erstelle Grafiken: ",now.strftime('%Y-%m-%d %H:%M:%S'))
        try:
            with mqtt_lock:
                # Temperatur
                im_fn = os.path.join(progdir, 'temps.png')
                result=subprocess.check_output('rrdtool graph '+im_fn+' -s "now - 24 hours" \
                    -e "now" DEF:temps1='+db_fn+':temps1:AVERAGE LINE2:temps1#000000:Außentemperatur \
                    -w '+str(GraphicsWidth)+' -h '+str(DisplayHeight)+' --full-size-mode -v "°C" -E -L 3 -A --left-axis-format "%.1lf" \
                    --border 0 --disable-rrdtool-tag -c BACK#FFFFFF',
                    shell=True, stderr=subprocess.STDOUT)
                # Luftdruck
                im_fn = os.path.join(progdir, 'press.png')
                result=subprocess.check_output('rrdtool graph '+im_fn+' -s "now - 24 hours" \
                    -e "now" DEF:press1='+db_fn+':press1:AVERAGE LINE2:press1#000000:Luftdruck \
                    -w '+str(GraphicsWidth)+' -h '+str(DisplayHeight)+' --full-size-mode -v "hPa" -E -L 5 -A --alt-y-grid \
                    --border 0 --disable-rrdtool-tag -c BACK#FFFFFF',
                    shell=True, stderr=subprocess.STDOUT)
                # Luftfeuchtigkeit
                im_fn = os.path.join(progdir, 'hums.png')
                result=subprocess.check_output('rrdtool graph '+im_fn+' -s "now - 24 hours" \
                    -e "now" DEF:hums1='+db_fn+':hums1:AVERAGE LINE2:hums1#000000:Luftfeuchtigkeit \
                    -w '+str(GraphicsWidth)+' -h '+str(DisplayHeight)+' --full-size-mode -v "%" -E -L 2 -A --alt-y-grid \
                    --border 0 --disable-rrdtool-tag -c BACK#FFFFFF',
                    shell=True, stderr=subprocess.STDOUT)
                # Beleuchtungsstärke
                im_fn = os.path.join(progdir, 'bright.png')
                result=subprocess.check_output('rrdtool graph '+im_fn+' -s "now - 24 hours" \
                    -e "now" DEF:brightness='+db_fn+':brightness:AVERAGE LINE2:brightness#000000:Beleuchtungsstärke \
                    -w '+str(GraphicsWidth)+' -h '+str(DisplayHeight)+' --full-size-mode -v "lux" -E -L 3 -A --alt-y-grid \
                    --border 0 --disable-rrdtool-tag -c BACK#FFFFFF',
                    shell=True, stderr=subprocess.STDOUT)
                
            print("fertig!")
            if LastButton==1:
                ShowDiagram("temps.png","Zeige Temperaturen...")
            elif LastButton==2:
                ShowDiagram("press.png","Zeige Luftdruck...")
            elif LastButton==3:
                ShowDiagram("hums.png","Zeige Luftfeuchtigkeit...")
            elif LastButton==4:
                ShowDiagram("bright.png","Zeige Beleuchtungsstärke...")
                
        except Exception as e:
            logging.info("Fehler", e.__class__, "occurred: ",e)
            time.sleep(1)
        
    epd.Dev_exit()

except IOError as e:
    logging.info(e)
    
except KeyboardInterrupt:    
    logging.info("ctrl + c:")
    epd2in7.epdconfig.module_exit()
    exit()
    

        

