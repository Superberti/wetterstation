#!/usr/bin/python3
# -*- coding:utf-8 -*-
import sys
import os
import time
import rrdtool
import subprocess
import datetime
from PIL import Image,ImageDraw,ImageFont,ImageOps

progdir = os.path.dirname(os.path.realpath(__file__))
db_fn = os.path.join(progdir, 'weather.rrd')
picdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pic')

 
font24 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 24)
font18 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 18)
font35 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 35)

# Temperatur Displaygroesse: 264x176
im_fn = os.path.join(progdir, 'temperatur.png')
new_image_path = os.path.join(progdir, 'temperatur_breit.png')
result=subprocess.check_output('rrdtool graph '+im_fn+' -s "now - 24 hours" \
    -e "now" DEF:temps1='+db_fn+':temps1:AVERAGE LINE2:temps1#000000:Außentemperatur \
    -w 200 -h 176 --full-size-mode -v "°C" -E -L 3 -A --left-axis-format "%.1lf" \
    --border 0 --disable-rrdtool-tag -c BACK#FFFFFF',
    shell=True, stderr=subprocess.STDOUT)
print("rrdtool result:",result)
now = datetime.datetime.now()
timetext = now.strftime('%H:%M')

Himage = Image.open(im_fn)
im_temp = Image.open(os.path.join(picdir, 'temperatur.png'))
im_hum = Image.open(os.path.join(picdir, 'luftfeuchtigkeit.png'))
im_press = Image.open(os.path.join(picdir, 'luftdruck.png'))
im_bright = Image.open(os.path.join(picdir, 'helligkeit.png'))

#im_mirror = ImageOps.mirror(Himage)
newImage = Image.new(Himage.mode, (264,176),(255, 255, 255))
newImage.paste(Himage, (0,0,Himage.width,Himage.height))

newImage.paste(im_temp, (200,37))
newImage.paste(im_press, (195,62))
newImage.paste(im_hum, (195,87))
newImage.paste(im_bright, (195,112))

draw = ImageDraw.Draw(newImage)
textwidth = font24.getsize(timetext)[0]
draw.text((262-textwidth, 0), timetext, font = font24, fill = (0, 0, 0))

text="24.5"
textwidth = font18.getsize(text)[0]
draw.text((262-textwidth, 40), text, font = font18, fill = (0, 0, 0))

text="1011"
textwidth = font18.getsize(text)[0]
draw.text((262-textwidth, 65), text, font = font18, fill = (0, 0, 0))

text="42.5"
textwidth = font18.getsize(text)[0]
draw.text((262-textwidth, 90), text, font = font18, fill = (0, 0, 0))

text="12k"
textwidth = font18.getsize(text)[0]
draw.text((262-textwidth, 115), text, font = font18, fill = (0, 0, 0))

newImage.save(new_image_path)


